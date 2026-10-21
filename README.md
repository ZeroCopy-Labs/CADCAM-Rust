# CAD/CAM Rust Viewer

当前版本是旧 Qt Quick / CXX-Qt 方向的重构版本：保留纯 Rust 的 DXF/DWG 核心，
界面和渲染换成 **Bevy + wgpu**，先达到原查看器的功能点，再继续 CAD/CAM 功能。

## 当前功能

- 顶部 `文件` 菜单：打开文件、最近打开；
- `视图` 菜单：适应视图、等轴测、俯视图；
- 三维正交（平行投影）轨道相机：`Shift + 中键拖动` 旋转视角，`中键拖动` 平移，滚轮缩放；
- 右上角可点击视角魔方，与当前相机方向同步；
- 顶部 `CAD / CAM` 滑动开关，左侧 CAD、右侧 CAM；当前空间高亮，切换时保留相机和选择语义映射；
- CAD 空间按图元选择；CAM 空间按加工单元选择；
- CAM 显示保留每个图元原来的 CAD 颜色，只有选中的加工单元用黄色高亮；
- CAM 加工单元按端点连续自动分组；完整圆、完整椭圆、闭合多段线和闭合样条各自单独成单元；
- 左键点选、`Shift + 左键` 追加选择、左/右框选在 CAM 空间都以整个加工单元为选择单位；
- 左侧“特征”面板：
  - CAD 模式显示图元基本信息与该类型几何信息；
  - CAM 模式显示加工单元类型、成员数和成员图元；显示颜色跟随成员图元；
  - 多选时收进可展开的下拉项；
  - 面板与命令栏捕获鼠标，操作不会透传到三维画布；
- 三维坐标轴辅助定位（正半轴实线、负半轴虚线）；
- 打开文件使用原生文件对话框，默认桌面，并记住上次目录；
- 最近打开列表启动时从 `%APPDATA%\CADCAM-Rust\recent.txt` 加载；
- 窗口顶部居中显示当前文件名；
- 支持命令行参数直接打开 DXF / DWG；
- 解析后显示模型空间中的 LINE、POLYLINE、CIRCLE、ARC、ELLIPSE、SPLINE；
- 三维几何：LINE/POLYLINE 保留端点 Z，圆/弧/椭圆使用其平面基，样条按 3D 控制点求值；
- 按图层/ACI/RGB 颜色绘制线段；
- 底部命令栏默认显示最新消息；点击整个命令栏展开历史（最多 4 行历史 + 1 行输入），输入框自动聚焦，滚轮滚动，点击其它区域收起；
- 命令输入当前只做回显打印，不再要求 `!` 前缀；
- 3D 正交相机自适应图纸范围；
- 深色界面。

## 构建

```powershell
cd D:\projects\CADCAM-Rust
.\build.ps1
```

构建产物：

```text
build\cargo\debug\cad-app.exe
```

Cargo 的 target 目录由 `.cargo/config.toml` 固定到 `build/cargo`。

## 运行

```powershell
.\run.ps1
.\run.ps1 "D:\CAD\sample.dxf"
```

也可以直接运行：

```powershell
build\cargo\debug\cad-app.exe "D:\CAD\sample.dxf"
```

## 核心开发检查

```powershell
cargo fmt --all
cargo check -p cad-core
cargo check -p cad-app
```

项目要求暂不添加自动化测试；构建成功由脚本保证，功能由人工验证。

## 目录

```text
crates/
  cad-core/             纯 Rust：DXF/DWG 解析 + 内部 CAD 模型
    src/
      import/           私有：格式检测、failsafe 解析
      convert.rs        私有：解析结果 -> DrawingDocument
      model/            公开：文档/实体/六种几何
  cad-app/              Bevy + wgpu 查看器
    src/
      main.rs           Bevy App、相机、空间切换、选择、egui 界面
      display.rs        内部模型 -> 三维显示线段
      cam.rs            端点连续性 -> 加工单元拓扑
技术栈.md               技术栈说明
架构.md                 架构说明
build.ps1               一键构建
run.ps1                 一键运行
```

## 当前限制

- 只读取模型空间；块定义、INSERT、图纸空间尚未展开；
- CAM 目前只生成加工单元拓扑，不包含加工顺序、参数、刀具轨迹或 NC；
- 显示适配保留三维点（Z 高度）并映射到 Bevy 世界坐标；
- 没有编辑功能；
- 大图纸的全量显示数据仍是一次性生成，后续需要改成增量/GPU 缓冲；
- egui 只是低成本的桌面 UI 壳，后续可以替换为 Bevy 原生 UI；
- Cargo.lock 由构建生成且不跟踪，依赖按 Cargo.toml 解析。

## 技术方向

- Rust：业务核心和界面同语言，降低 Qt/C++/Rust 桥接成本；
- Bevy：ECS、窗口、输入、渲染基础能力；
- wgpu：Bevy 的 GPU 后端，Windows 上无需单独的 Qt RHI 层；
- acadrust：仅存在于 cad-core 的解析/转换边界；
- egui：当前阶段的轻量桌面 UI，不作为核心业务依赖。
