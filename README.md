# CAD/CAM Rust Viewer

当前版本是旧 Qt Quick / CXX-Qt 方向的重构版本：保留纯 Rust 的 DXF/DWG 核心，
界面和渲染换成 **Bevy + wgpu**，先达到原查看器的功能点，再继续 CAD/CAM 功能。

## 当前功能

- 顶部 `文件` 菜单：打开文件、最近打开；
- `视图` 菜单：适应视图、旋转；
- 打开文件使用原生文件对话框，默认桌面，并记住上次目录；
- 最近打开列表启动时从 `%APPDATA%\CADCAM-Rust\recent.txt` 加载；
- 窗口顶部居中显示当前文件名；
- 支持命令行参数直接打开 DXF / DWG；
- 解析后显示模型空间中的 LINE、POLYLINE、CIRCLE、ARC、ELLIPSE、SPLINE；
- 按图层/ACI/RGB 颜色绘制二维线段；
- 底部命令栏默认显示最新消息；点击整个命令栏展开历史（最多 4 行历史 + 1 行输入），输入框自动聚焦，滚轮滚动，点击其它区域收起；
- 命令输入当前只做回显打印，不再要求 `!` 前缀；
- 视图自适应图纸范围，支持旋转角度；
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
      main.rs           Bevy App、相机、egui 界面
      display.rs        内部模型 -> 二维显示线段
技术栈.md               技术栈说明
架构.md                 架构说明
build.ps1               一键构建
run.ps1                 一键运行
```

## 当前限制

- 只读取模型空间；块定义、INSERT、图纸空间尚未展开；
- 只做二维 XY 投影，没有三维相机、拾取、编辑、CAM；
- 大图纸的全量显示数据仍是一次性生成，后续需要改成增量/GPU 缓冲；
- egui 只是低成本的桌面 UI 壳，后续可以替换为 Bevy 原生 UI；
- Cargo.lock 由构建生成且不跟踪，依赖按 Cargo.toml 解析。

## 技术方向

- Rust：业务核心和界面同语言，降低 Qt/C++/Rust 桥接成本；
- Bevy：ECS、窗口、输入、渲染基础能力；
- wgpu：Bevy 的 GPU 后端，Windows 上无需单独的 Qt RHI 层；
- acadrust：仅存在于 cad-core 的解析/转换边界；
- egui：当前阶段的轻量桌面 UI，不作为核心业务依赖。
