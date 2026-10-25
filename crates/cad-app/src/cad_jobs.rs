//! One bounded worker. Formal documents stay on the UI thread; results require an exact input stamp.
use crate::{cam, cam_analysis, cam_sync, cam_trajectory, display, gcode, ViewerState};
use bevy::prelude::*;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, AtomicU8, Ordering},
        mpsc, Arc, Mutex,
    },
    time::Instant,
};

#[derive(Clone, PartialEq)]
struct Guard {
    session: u64,
    revision: Option<u64>,
    inputs: cam_sync::Inputs,
}
impl Guard {
    fn matches(&self, s: &ViewerState) -> bool {
        self.session == s.load_version
            && self.revision == s.drawing.as_ref().map(|d| d.revision())
            && self.inputs == s.derived.inputs
    }
    fn capture(s: &ViewerState) -> Self {
        Self {
            session: s.load_version,
            revision: s.drawing.as_ref().map(|d| d.revision()),
            inputs: s.derived.inputs,
        }
    }
}

enum Intent {
    Open(PathBuf),
    Rebuild,
    Units(Candidate, cad_core::LengthUnit),
}
pub enum Follow {
    SaveProject(bool),
    Export,
    QuickExport,
    Validate,
    View(BTreeSet<cam::UnitId>),
}
pub enum AfterSave {
    Open(Box<Prepared>),
    New,
    Close,
    Exit,
}
struct PendingFollow {
    guard: Guard,
    action: Follow,
}
struct Active {
    id: u64,
    guard: Guard,
    cancel: Arc<AtomicBool>,
    stage: Arc<AtomicU8>,
    started: Instant,
    rebuilding: bool,
}
struct Message {
    id: u64,
    work: Work,
    cancel: Arc<AtomicBool>,
    stage: Arc<AtomicU8>,
}
enum Source {
    Full(cad_core::DrawingDocument, display::DisplayData),
    Cached(display::DisplayData),
}
enum Work {
    Open {
        path: PathBuf,
        profile: gcode::GCodeProfile,
    },
    Units {
        candidate: Candidate,
        unit: cad_core::LengthUnit,
    },
    Rebuild {
        source: Source,
        cam: cam::CamModel,
        analysis: cam_analysis::CamAnalysis,
        excluded: BTreeSet<u32>,
        profile: gcode::GCodeProfile,
    },
}
pub struct Candidate {
    pub drawing: cad_core::EditableDrawing,
    pub path: PathBuf,
    pub profile: gcode::GCodeProfile,
    pub excluded: BTreeSet<u32>,
    pub setup: Option<cam_analysis::MachiningSetup>,
    pub project: Option<(Option<PathBuf>, bool)>,
    pub recovered: bool,
}
pub struct Calculated {
    pub machining: display::DisplayData,
    pub cam: cam::CamModel,
    pub analysis: cam_analysis::CamAnalysis,
    pub trajectory: cam_trajectory::MachineTrajectory,
    pub error: Option<String>,
}
pub struct Prepared {
    pub candidate: Candidate,
    pub display: display::DisplayData,
    pub calculated: Calculated,
}
enum Output {
    Units(Candidate),
    Opened(Prepared),
    Rebuilt(Calculated),
}
struct Event {
    id: u64,
    result: Result<Output, String>,
    times: BTreeMap<String, f64>,
}

pub struct Tasks {
    sender: mpsc::SyncSender<Message>,
    receiver: Mutex<mpsc::Receiver<Event>>,
    active: Option<Active>,
    intent: Option<(Guard, Intent)>,
    follow: Option<PendingFollow>,
    after_save: Option<AfterSave>,
    pending_action: Option<(Guard, AfterSave)>,
    serial: u64,
    pub status: String,
}
impl Default for Tasks {
    fn default() -> Self {
        let (sender, requests) = mpsc::sync_channel::<Message>(1);
        let (results, receiver) = mpsc::channel();
        std::thread::Builder::new()
            .name("cad-compute".into())
            .spawn(move || {
                while let Ok(message) = requests.recv() {
                    let mut run = Run {
                        cancel: message.cancel,
                        stage: message.stage,
                        times: BTreeMap::new(),
                    };
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        execute(message.work, &mut run)
                    }))
                    .unwrap_or_else(|_| Err("后台读取/计算异常，当前图纸未被替换".into()));
                    let result = if run.cancel.load(Ordering::Relaxed) {
                        Err("已取消".into())
                    } else {
                        result
                    };
                    if results
                        .send(Event {
                            id: message.id,
                            result,
                            times: run.times,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            })
            .expect("create CAD worker");
        Self {
            sender,
            receiver: Mutex::new(receiver),
            active: None,
            intent: None,
            follow: None,
            after_save: None,
            pending_action: None,
            serial: 0,
            status: String::new(),
        }
    }
}
impl Tasks {
    pub fn busy(&self) -> bool {
        self.active.is_some() || self.intent.is_some() || self.pending_action.is_some()
    }
    pub fn cancel(&mut self) {
        if let Some(active) = &self.active {
            active.cancel.store(true, Ordering::Relaxed);
        }
        self.intent = None;
        self.follow = None;
        self.after_save = None;
        self.pending_action = None;
        self.status = "已取消；当前阶段结束后释放后台资源".into();
    }
    fn request(&mut self, guard: Guard, intent: Intent) {
        self.pending_action = None;
        if !matches!(intent, Intent::Rebuild) {
            self.follow = None;
            self.after_save = None;
        }
        if matches!(intent, Intent::Rebuild)
            && self.active.as_ref().is_some_and(|a| {
                a.rebuilding && a.guard == guard && !a.cancel.load(Ordering::Relaxed)
            })
            && self.intent.is_none()
        {
            return;
        }
        if let Some(a) = &self.active {
            a.cancel.store(true, Ordering::Relaxed);
        }
        if self.follow.as_ref().is_some_and(|f| f.guard != guard) {
            self.follow = None;
            self.after_save = None;
        }
        self.intent = Some((guard, intent));
        self.status = "等待后台计算".into();
    }
    pub fn progress(&self) -> String {
        if let Some(a) = &self.active {
            let phase = match a.stage.load(Ordering::Relaxed) {
                1 => "读取 / 转换",
                2 => "显示离散",
                3 => "加工采样",
                4 => "拓扑",
                5 => "加工分析",
                6 => "四轴轨迹",
                7 => "单位解析",
                _ => "准备",
            };
            format!(
                "{} · {:.1}s{}",
                phase,
                a.started.elapsed().as_secs_f64(),
                if a.cancel.load(Ordering::Relaxed) {
                    " · 正在取消，结果不会应用"
                } else if self.intent.is_some() {
                    " · 等待更新的任务"
                } else {
                    ""
                }
            )
        } else {
            self.status.clone()
        }
    }
    fn event(&self) -> Option<Event> {
        self.receiver.lock().ok()?.try_recv().ok()
    }
    pub fn defer_after_save(&mut self, action: AfterSave) -> bool {
        if self
            .follow
            .as_ref()
            .is_some_and(|f| matches!(f.action, Follow::SaveProject(_)))
        {
            self.after_save = Some(action);
            true
        } else {
            false
        }
    }
}
struct Run {
    cancel: Arc<AtomicBool>,
    stage: Arc<AtomicU8>,
    times: BTreeMap<String, f64>,
}
impl Run {
    fn phase<T>(&mut self, code: u8, name: &str, f: impl FnOnce() -> T) -> Result<T, String> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err("已取消".into());
        }
        self.stage.store(code, Ordering::Relaxed);
        let start = Instant::now();
        let value = f();
        self.times
            .insert(name.into(), start.elapsed().as_secs_f64() * 1000.0);
        if self.cancel.load(Ordering::Relaxed) {
            return Err("已取消".into());
        }
        Ok(value)
    }
}
fn trajectory(
    run: &mut Run,
    machining: &display::DisplayData,
    cam: &cam::CamModel,
    analysis: &mut cam_analysis::CamAnalysis,
    profile: &gcode::GCodeProfile,
    error: &Option<String>,
) -> Result<cam_trajectory::MachineTrajectory, String> {
    run.phase(6, "轨迹", || {
        analysis.set_machine_start(
            machining,
            cam,
            [profile.initial_x, profile.initial_y, profile.initial_z],
        );
        let error = error
            .clone()
            .or_else(|| profile.validation_error().map(str::to_string))
            .or_else(|| {
                (!analysis.pending_arrangements.is_empty())
                    .then(|| "加工安排有失效项，请先在 CAM 核对".into())
            });
        if let Some(error) = error {
            return cam_trajectory::MachineTrajectory {
                error: Some(error),
                ..Default::default()
            };
        }
        cam_trajectory::build_trajectory(machining, cam, analysis, profile.rotary_config())
    })
}
fn calculate(
    run: &mut Run,
    source: Source,
    old: cam::CamModel,
    analysis: cam_analysis::CamAnalysis,
    excluded: &BTreeSet<u32>,
    profile: &gcode::GCodeProfile,
) -> Result<Calculated, String> {
    let (machining, error, cam, mut analysis) = match source {
        Source::Full(doc, metadata) => {
            let sampled = run.phase(3, "加工采样", || {
                display::build_machining_from_metadata(&doc, excluded, &metadata)
            })?;
            let (machining, error) = match sampled {
                Ok(data) => (data, None),
                Err(e) => (Default::default(), Some(e)),
            };
            let cam = run.phase(4, "拓扑", || cam::build_cam_model(&doc, excluded))?;
            let analysis = run.phase(5, "加工分析", || {
                analysis.reconcile(&old, &cam, &machining)
            })?;
            (machining, error, cam, analysis)
        }
        Source::Cached(machining) => (machining, None, old, analysis),
    };
    let trajectory = trajectory(run, &machining, &cam, &mut analysis, profile, &error)?;
    Ok(Calculated {
        machining,
        error,
        cam,
        analysis,
        trajectory,
    })
}
fn prepare(mut candidate: Candidate, run: &mut Run) -> Result<Output, String> {
    if candidate.drawing.document().source_units.is_none() {
        return Ok(Output::Units(candidate));
    }
    if candidate.recovered {
        candidate.drawing.mark_recovered();
    }
    let doc = candidate.drawing.document();
    let display = run.phase(2, "显示离散", || display::build_display(doc))?;
    let sampled = run.phase(3, "加工采样", || {
        display::build_machining_from_metadata(doc, &candidate.excluded, &display)
    })?;
    let (machining, error) = match sampled {
        Ok(d) => (d, None),
        Err(e) => (Default::default(), Some(e)),
    };
    let cam = run.phase(4, "拓扑", || {
        cam::build_cam_model(doc, &candidate.excluded)
    })?;
    let mut analysis = run.phase(5, "加工分析", || match candidate.setup.take() {
        Some(setup) => cam_analysis::CamAnalysis::restore_setup(setup, &machining, &cam),
        None => Ok(cam_analysis::CamAnalysis::build(&machining, &cam)),
    })??;
    let trajectory = trajectory(
        run,
        &machining,
        &cam,
        &mut analysis,
        &candidate.profile,
        &error,
    )?;
    Ok(Output::Opened(Prepared {
        candidate,
        display,
        calculated: Calculated {
            machining,
            cam,
            analysis,
            trajectory,
            error,
        },
    }))
}
fn execute(work: Work, run: &mut Run) -> Result<Output, String> {
    match work {
        Work::Open { path, profile } => {
            let project = path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("cadcam"))
                || path.file_name().is_some_and(|n| {
                    n.to_string_lossy()
                        .to_ascii_lowercase()
                        .ends_with(".cadcam.bak")
                });
            let candidate =
                run.phase(1, "读取与转换", || -> Result<Candidate, String> {
                    if project {
                        let p = crate::cad_project::read(&path)?;
                        Ok(Candidate {
                            drawing: p.drawing,
                            excluded: p.excluded,
                            profile: p.profile,
                            setup: Some(p.setup),
                            project: Some((p.source, p.migrated)),
                            recovered: false,
                            path,
                        })
                    } else {
                        let drawing =
                            cad_core::read_editable_document(&path).map_err(|e| e.to_string())?;
                        let excluded = drawing
                            .document()
                            .entities
                            .iter()
                            .filter(|e| e.reference)
                            .map(|e| e.id.0)
                            .collect();
                        let recovered = path
                            .extension()
                            .is_some_and(|e| e.eq_ignore_ascii_case("bak"));
                        Ok(Candidate {
                            drawing,
                            excluded,
                            profile,
                            setup: None,
                            project: None,
                            recovered,
                            path,
                        })
                    }
                })??;
            if candidate.project.is_none() {
                run.times
                    .insert("解析读取".into(), candidate.drawing.import_timings.parse_ms);
                run.times.insert(
                    "导入转换".into(),
                    candidate.drawing.import_timings.convert_ms,
                );
            }
            prepare(candidate, run)
        }
        Work::Units {
            mut candidate,
            unit,
        } => {
            run.phase(7, "单位解析", || candidate.drawing.resolve_units(unit))??;
            prepare(candidate, run)
        }
        Work::Rebuild {
            source,
            cam,
            analysis,
            excluded,
            profile,
        } => calculate(run, source, cam, analysis, &excluded, &profile).map(Output::Rebuilt),
    }
}

pub fn open(state: &mut ViewerState, path: PathBuf) {
    state.performance.last.clear();
    state.derived.observe_profile(&state.gcode_profile);
    let guard = Guard::capture(state);
    state.tasks.request(guard, Intent::Open(path));
}
pub fn rebuild(state: &mut ViewerState) {
    state.derived.observe_profile(&state.gcode_profile);
    if state.drawing.is_none() {
        return;
    }
    let guard = Guard::capture(state);
    state.derived.trajectory = None;
    state.trajectory = None;
    state.tasks.request(guard, Intent::Rebuild);
}
pub fn ensure(state: &mut ViewerState, action: Follow) -> bool {
    state.derived.observe_profile(&state.gcode_profile);
    if state.derived.trajectory_current() {
        return true;
    }
    if state.drawing.is_none() {
        crate::push_message(state, "请先打开图纸");
        return false;
    }
    rebuild(state);
    if !matches!(action, Follow::SaveProject(_)) {
        state.tasks.after_save = None;
    }
    state.tasks.follow = Some(PendingFollow {
        guard: Guard::capture(state),
        action,
    });
    crate::push_message(
        state,
        "正在后台重算，完成后自动继续；修改输入或取消计算将取消本次后续操作",
    );
    false
}

pub fn poll(mut state: ResMut<ViewerState>) {
    let state = &mut *state;
    state.derived.observe_profile(&state.gcode_profile);
    if state
        .tasks
        .pending_action
        .as_ref()
        .is_some_and(|(g, _)| !g.matches(state))
    {
        state.tasks.pending_action = None;
        state.performance.discarded += 1;
        state.tasks.status = "文件应用已取消：当前输入已变化".into();
    }
    if !state.cad_command.is_active() {
        if let Some((_, action)) = state.tasks.pending_action.take() {
            apply_file_action(state, action);
        }
    }

    if state
        .tasks
        .active
        .as_ref()
        .is_some_and(|a| !a.guard.matches(state))
    {
        if let Some(a) = &state.tasks.active {
            a.cancel.store(true, Ordering::Relaxed);
        }
    }
    if let Some(event) = state.tasks.event() {
        if let Some(active) = state.tasks.active.take() {
            let current = active.id == event.id
                && !active.cancel.load(Ordering::Relaxed)
                && active.guard.matches(state);
            if current {
                state.performance.last.extend(event.times);
                match event.result {
                    Ok(Output::Units(candidate)) => {
                        let choice = rfd::MessageDialog::new()
                            .set_title("请选择无单位图纸的实际单位")
                            .set_description(
                                "是：毫米；否：英寸；取消：保留当前工作。图纸尚未安装。",
                            )
                            .set_buttons(rfd::MessageButtons::YesNoCancel)
                            .show();
                        let unit = match choice {
                            rfd::MessageDialogResult::Yes => {
                                Some(cad_core::LengthUnit::Millimeters)
                            }
                            rfd::MessageDialogResult::No => Some(cad_core::LengthUnit::Inches),
                            _ => None,
                        };
                        if let Some(unit) = unit {
                            state.tasks.intent =
                                Some((Guard::capture(state), Intent::Units(candidate, unit)));
                        } else {
                            state.tasks.status = "已取消打开".into();
                        }
                    }
                    Ok(Output::Opened(prepared)) => {
                        apply_file_action(state, AfterSave::Open(Box::new(prepared)))
                    }
                    Ok(Output::Rebuilt(result)) => {
                        state.machining = result.machining;
                        state.machining_error = result.error;
                        state.cam = result.cam;
                        if state.analysis.tube_section != result.analysis.tube_section {
                            if let Some(tube) = &result.analysis.tube_section {
                                state.manual_tube = tube.clone();
                            }
                        }
                        state.analysis = result.analysis;
                        state.trajectory = Some(result.trajectory);
                        state.derived.geometry_ready();
                        state.derived.trajectory = Some(state.derived.inputs);
                        for slot in &mut state.manual_pair {
                            *slot = slot.filter(|id| {
                                state
                                    .analysis
                                    .machining_sections
                                    .iter()
                                    .any(|s| s.unit_id == *id)
                            });
                        }
                        state.performance.paths(&state.display, &state.machining);
                        state.tasks.status = "重算完成".into();
                        if let Some(follow) =
                            state.tasks.follow.take().filter(|f| f.guard.matches(state))
                        {
                            match follow.action {
                                Follow::SaveProject(save_as) => {
                                    let after = state.tasks.after_save.take();
                                    if crate::cad_project::save(state, save_as) {
                                        if let Some(after) = after {
                                            apply_file_action(state, after);
                                        }
                                    }
                                }
                                Follow::Export => crate::export_gcode_file(state),
                                Follow::QuickExport => crate::quick_export_gcode_file(state),
                                Follow::Validate => {
                                    let result = crate::gcode_config_action(
                                        state,
                                        crate::GCodeConfigAction::Validate,
                                    );
                                    match result {
                                        Ok(Some(message)) => {
                                            state.gcode_settings_message = message.clone();
                                            state.gcode_settings_error = false;
                                            crate::push_message(state, message);
                                        }
                                        Err(error) => {
                                            state.gcode_settings_message = error.clone();
                                            state.gcode_settings_error = true;
                                            crate::push_message(state, error);
                                        }
                                        _ => {}
                                    }
                                }
                                Follow::View(selected) => {
                                    crate::open_trajectory_view(state, &selected)
                                }
                            }
                        }
                    }
                    Err(error) => {
                        state.tasks.status = error.clone();
                        state.tasks.follow = None;
                        state.tasks.after_save = None;
                        crate::push_message(state, format!("后台任务失败：{error}"));
                    }
                }
            } else {
                state.performance.discarded += 1;
                state.tasks.status = "旧结果已丢弃".into();
            }
        }
    }
    if state.tasks.active.is_none() {
        if let Some((guard, intent)) = state.tasks.intent.take() {
            if !guard.matches(state) {
                state.performance.discarded += 1;
                state.tasks.follow = None;
                state.tasks.after_save = None;
                return;
            }
            let started = Instant::now();
            let work = match intent {
                Intent::Open(path) => Work::Open {
                    path,
                    profile: state.gcode_profile.clone(),
                },
                Intent::Units(candidate, unit) => Work::Units { candidate, unit },
                Intent::Rebuild => {
                    let Some(drawing) = &state.drawing else {
                        return;
                    };
                    let source =
                        if state.derived.sources_current() && state.machining_error.is_none() {
                            Source::Cached(state.machining.clone())
                        } else {
                            Source::Full(
                                drawing.document().clone(),
                                display::metadata(&state.display),
                            )
                        };
                    Work::Rebuild {
                        source,
                        cam: state.cam.clone(),
                        analysis: state.analysis.clone(),
                        excluded: state.excluded.clone(),
                        profile: state.gcode_profile.clone(),
                    }
                }
            };
            state.performance.record("后台快照", started);
            state.tasks.serial += 1;
            let id = state.tasks.serial;
            let cancel = Arc::new(AtomicBool::new(false));
            let stage = Arc::new(AtomicU8::new(0));
            let rebuilding = matches!(work, Work::Rebuild { .. });
            match state.tasks.sender.send(Message {
                id,
                work,
                cancel: cancel.clone(),
                stage: stage.clone(),
            }) {
                Ok(()) => {
                    state.tasks.active = Some(Active {
                        id,
                        guard,
                        cancel,
                        stage,
                        started: Instant::now(),
                        rebuilding,
                    })
                }
                Err(_) => {
                    state.tasks.status = "后台线程已退出".into();
                    state.tasks.follow = None;
                }
            }
        }
    }
}

fn apply_file_action(state: &mut ViewerState, action: AfterSave) {
    if state.cad_command.is_active() {
        state.tasks.pending_action = Some((Guard::capture(state), action));
        state.tasks.status =
            "文件已就绪，等待当前 CAD 命令完成或取消；提交新几何会丢弃旧结果".into();
        return;
    }
    match action {
        AfterSave::Open(prepared) => {
            if crate::cad_file::confirm_replace(state) {
                crate::cad_file::install_prepared(state, *prepared);
                state.tasks.status = "加载完成".into();
            } else if !state.tasks.defer_after_save(AfterSave::Open(prepared)) {
                state.tasks.status = "已取消替换，当前工作保留".into();
            }
        }
        AfterSave::New => crate::cad_file::new_drawing(state),
        AfterSave::Close => crate::cad_file::close_drawing(state),
        AfterSave::Exit => state.file_ui.exit_requested = true,
    }
}
