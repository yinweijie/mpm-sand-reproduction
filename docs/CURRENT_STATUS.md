# Klár 2016 Current Status and Runbook

## 1. Theory Overview

当前工程以 `Klár et al. 2016, Drucker-Prager Elastoplasticity for Sand Animation`
为主线，目标是先复现单相干砂的基本行为，再逐步向论文案例靠近。

当前实现采用的理论框架：

- `3D explicit APIC/MPM`：
  粒子携带质量、速度、仿射速度项 `B`、弹性形变梯度 `F_e` 等状态；
  每个时间步执行 `P2G -> grid update -> G2P`。
- `Hencky/log-strain elastic response`：
  通过 `F_e` 的谱分解和 log-strain 构造弹性应力。
- `Drucker-Prager plastic projection`：
  用砂土屈服面约束弹性预测状态，避免材料表现得像普通弹性体。
- `Hardening`：
  用论文中的 `alpha / q` 风格内部变量更新摩擦相关响应。
- `Boundary contact with Coulomb friction`：
  当前支持域边界、有限平面碰撞体、解析 hourglass 壳体和圆柱壳碰撞体。

当前实现边界：

- 已做：单相 dry sand、显式时间推进、APIC、Drucker-Prager、hardening、发射器、PLY 导出。
- 未做：论文中的 implicit velocity solve、水砂耦合、2017 volume correction、OpenVDB 后端。

## 2. Algorithm Implementation

核心代码：

- [simulation.hpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/include/klar2016/core/simulation.hpp)
- [simulation.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation.cpp)
- [simulation_config.hpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/include/klar2016/core/simulation_config.hpp)
- [simulation_config.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation_config.cpp)
- [kernel.hpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/include/klar2016/core/kernel.hpp)

当前求解器的实现结构：

1. 读取场景、材料、求解器、发射器、导出配置。
2. 初始化粒子：
   - 支持盒状 seed；
   - 支持规则采样；
   - 支持圆柱喷口发射；
   - 当前 `pile_lab` 默认使用 `stratified cylindrical emitter`。
3. 每步仿真：
   - 清空活跃网格；
   - `P2G` 传质量、动量和 APIC 仿射项；
   - 网格上加重力并做碰撞/摩擦处理；
   - `G2P` 回写粒子速度和位置；
   - 更新 `F_e`；
   - 做 Drucker-Prager 投影与 hardening 更新。
4. 导出：
   - 可按步长导出 `.ply` 粒子帧；
   - 统计粒子数、总质量、`det(F_e)`、`alpha`、活跃网格节点数、包围盒体积等。

当前工程里和案例直接相关的附加实现：

- 稀疏活跃网格：
  目前使用 `std::unordered_map<int, GridNode>`，避免 3D dense grid 直接炸内存。
- 几何：
  - `PlaneColliderConfig`
  - `HourglassShellConfig`
  - `CylinderShellConfig`
- CLI：
  [main.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/main.cpp)
  支持 `--scene`、`--steps`、`--export`、`--output-dir`、
  `--friction-angle`、`--sweep-friction-angles`。

### 2.1 Gap to the 2016 Supplementary Technical Document

当前实现已经覆盖了 2016 主论文里最关键的显式砂求解主干，
但和补充技术文档相比，仍然是一个 `explicit subset`，
不是“技术文档已经基本做完”的状态。

对照资料：

- 主论文：
  [MinerU_html_KGPSJT16_2032719086785196032.html](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_KGPSJT16_2032719086785196032.html)
- 补充技术文档：
  [MinerU_html_tech-doc_2032868345404772352.html](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_tech-doc_2032868345404772352.html)

当前未实现部分及其影响如下。

#### A. Implicit solve 未实现：高影响

主论文 `5.6 Implicit velocity update` 和技术文档 `Algorithm 2 Implicit solve`
都给出了 Newton、GMRES、`HESSIAN_TIMES` 这套流程。
当前代码仍然固定在显式路线：
[simulation_config.hpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/include/klar2016/core/simulation_config.hpp#L32)。

具体影响：

- 不能声称当前结果与 2016 技术文档“基本对齐”；
- 很难严格复现论文中标为 `Implicit` 的案例；
- 对高粒子数、强接触、较硬材料和更激进时间步长的鲁棒性不足；
- 当前 `pile_lab` 更适合作为显式近似原型，不等同于论文正式 `Pile from spout` 求解流程。

直接相关的论文证据：

- 主论文里 `Hourglass` 标为 `Explicit`，
  `Pile from spout` 标为 `Implicit`：
  [MinerU_html_KGPSJT16_2032719086785196032.html#L1679](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_KGPSJT16_2032719086785196032.html#L1679)
  [MinerU_html_KGPSJT16_2032719086785196032.html#L1744](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_KGPSJT16_2032719086785196032.html#L1744)
- 技术文档的 `Algorithm 2 Implicit solve`：
  [MinerU_html_tech-doc_2032868345404772352.html#L362](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_tech-doc_2032868345404772352.html#L362)

#### B. 碰撞/摩擦模型未完全对齐：中高影响

技术文档和主论文把碰撞区分为 `sticky / slipping / separating`，
并在隐式流程中记录碰撞集合后再施加约束。
当前实现使用的是统一的单边约束 + 摩擦投影，
主要代码在：
[simulation.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation.cpp#L326)
[simulation.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation.cpp#L344)

具体影响：

- 沙子落地后的滑移、停滞和分离过渡不够像论文；
- 堆体更容易偏扁，边界附近行为更容易失真；
- 对喷口、沙漏颈口、斜面和工具接触这类案例影响明显；
- 这是当前“堆形不像论文”的重要原因之一。

#### C. 初始化方法未完全对齐：中等影响

主论文 `5.7 Initialization` 明确写了 `Poisson disk sampling`。
当前工程主要使用规则采样、盒状 seed 和圆柱分层采样：
[simulation.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation.cpp#L685)
[simulation.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/core/simulation.cpp#L813)

具体影响：

- 粒子分布更容易出现条带、列状和局部对称性偏强；
- 自由表面和喷流的颗粒观感不如论文自然；
- 会放大发射器和接触区域中的离散化伪影；
- 对“看起来像不像沙子”有直接影响，但通常不如碰撞模型和隐式求解那么根本。

相关论文证据：

- 主论文初始化部分：
  [MinerU_html_KGPSJT16_2032719086785196032.html#L1115](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/references/MinerU_html_KGPSJT16_2032719086785196032.html#L1115)

#### D. 隐式导数链未实现：当前显式结果直接影响小，但对补齐论文实现影响大

技术文档中用于隐式求解的 `Y(F)`、`M = dY/dF`、
`HESSIAN_TIMES` 这套导数链当前没有实现。

具体影响：

- 对现在正在跑的显式案例，短期内不会直接改变画面；
- 但这是补齐 `Algorithm 2` 的前提，不做就无法进入论文完整隐式实现；
- 因此它更像“结构性缺口”，而不是当前显式效果的第一主因。

#### E. 对当前工作的实际判断

如果当前目标是继续把 `hourglass`、`pile_lab` 这类显式案例做得更像样，
影响最大的是：

- 碰撞/摩擦模型；
- 初始化与发射采样质量。

如果当前目标是说“已经基本实现了 2016 技术文档”，
最大阻塞项则是：

- `Algorithm 2` 的隐式求解；
- 与之配套的导数链和碰撞处理。

一句话概括：

- 最影响“堆得像不像”的，是碰撞和初始化；
- 最影响“是不是 2016 技术文档同一套算法”的，是隐式求解。

## 3. Third-Party Tools

编译和依赖管理：

- `CMake`
- `Conan 2`
- `Ninja`
- `C++20 compiler`

由 Conan 管理的核心库：

- `Eigen 3.4.0`
- `fmt 11.2.0`
- `nlohmann_json 3.12.0`
- `gtest 1.15.0`

可视化和后处理：

- `Python 3`
- `matplotlib`
- `ffmpeg`
- `Blender`

当前更可靠的最终渲染链使用：

- 官方 Blender 二进制：
  `/home/ywj22/blender-5.0.1-linux-x64/blender`

说明：

- `tools/render_ply_sequence.py` 适合快速预览。
- `tools/render_ply_sequence_blender.py` 适合最终交付渲染。

## 4. Case Files and Scripts

当前已经整理好的案例配置：

- [box_case.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/box_case.json)
- [hourglass.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/hourglass.json)
- [friction_angle.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/friction_angle.json)
- [pile_from_spout.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/pile_from_spout.json)
- [pile_lab.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/pile_lab.json)

和当前论文式堆积案例最相关的文件：

- 场景配置：
  [pile_lab.json](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scenes/pile_lab.json)
- 仿真脚本：
  [run_pile_lab_case.sh](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scripts/run_pile_lab_case.sh)
- Blender 最终渲染脚本：
  [render_pile_lab_blender.sh](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scripts/render_pile_lab_blender.sh)
- matplotlib 预览渲染：
  [render_ply_sequence.py](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/tools/render_ply_sequence.py)
- Blender 渲染主逻辑：
  [render_ply_sequence_blender.py](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/tools/render_ply_sequence_blender.py)

当前已生成、可直接查看的论文风格颗粒渲染结果：

- 视频：
  [pile_lab_paper_like_v1.mp4](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1.mp4)
- 末帧：
  [pile_lab_paper_like_v1_last.png](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1_last.png)

## 5. Build and Run

### 5.1 One-time environment check

```bash
ninja --version
conan --version
cmake --version
```

### 5.2 Build

当前工程用 Conan + CMake preset，`build.sh` 已固定 `C++20`。

脚本：

- [build.sh](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/build.sh)

参数说明：

- 第 1 个位置参数 `mode`
  - 可选：`release` 或 `debug`
  - 默认：`release`
- 第 2 个位置参数 `scene`
  - 要运行的场景配置 JSON
  - 默认：`scenes/hourglass.json`

脚本行为：

- 先执行 `conan install`
- 再执行 `cmake preset + build`
- 构建完成后立即运行一次二进制：
  - release 对应 `build/Release/klar2016_sand`
  - debug 对应 `build/Debug/klar2016_sand`
- 同时把 `compile_commands.json` 拷到 `build/`

构建并默认运行一个场景：

```bash
bash ./build.sh release scenes/pile_lab.json
```

调试版：

```bash
bash ./build.sh debug scenes/pile_lab.json
```

也可以手动构建：

```bash
conan install . --build=missing -s="compiler.cppstd=20"
cmake --preset=conan-release
cmake --build --preset=conan-release
```

### 5.3 Run simulation with frame export

二进制入口：

- [main.cpp](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/src/main.cpp)

当前常用 CLI 参数：

- `--scene <path>`
  - 选择场景 JSON
- `--steps <int>`
  - 覆盖场景里的 `preview_steps`
- `--export`
  - 开启 PLY 导出
- `--no-export`
  - 关闭 PLY 导出
- `--output-dir <path>`
  - 指定导出目录
- `--friction-angle <deg>`
  - 覆盖单次运行的摩擦角
- `--sweep-friction-angles a,b,c`
  - 批量扫描多个摩擦角

说明：

- 当使用 `--export` 时，程序会把 PLY 直接写到 `--output-dir` 指向的目录。
- 导出文件名格式为 `frame_00000.ply`、`frame_00016.ply` 这类。

关于“扫描摩擦角”的说明：

- 它的目的不是生成一个特殊案例，而是做参数扫描。
- 做法是：固定同一个场景、同一套求解器设置，只改变材料里的 `friction_angle_degrees`，连续跑多次仿真。
- 用途主要有两个：
  - 调参：
    快速判断哪一组摩擦角更接近论文里的堆积形状。
  - 验证实现：
    检查材料响应是否合理。正常情况下，摩擦角越大，砂堆通常越高、越陡；摩擦角越小，砂堆通常越扁、更容易摊开。
- 什么时候建议使用：
  - 当你已经确认仿真基本稳定，但还不知道该把摩擦角定在多少时。
  - 当你想做论文结果对比，需要观察堆角随材料参数变化的趋势时。
- 什么时候不必使用：
  - 如果当前只想先盯住一个固定案例反复调其它参数，可以先直接用单个 `--friction-angle`。
- 当前实现里的输出方式：
  - 如果同时开启 `--export`，程序会为每个角度创建一个单独子目录，例如：
    - `outputs/friction_sweep/friction_20.0`
    - `outputs/friction_sweep/friction_25.0`
    - `outputs/friction_sweep/friction_30.0`
  - 这样便于逐组查看 PLY、预览视频或最终渲染结果。

直接运行二进制并导出 PLY：

```bash
./build/Release/klar2016_sand \
  --scene scenes/pile_lab.json \
  --friction-angle 50 \
  --steps 2400 \
  --export \
  --output-dir outputs/pile_lab_dense_check_v2
```

或者直接用脚本：

```bash
bash ./scripts/run_pile_lab_case.sh release 2400 outputs/pile_lab_case 50
```

脚本：

- [run_pile_lab_case.sh](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scripts/run_pile_lab_case.sh)

参数说明：

- 第 1 个位置参数 `mode`
  - 可选：`release` 或 `debug`
  - 默认：`release`
- 第 2 个位置参数 `steps`
  - 仿真步数
  - 默认：`2400`
- 第 3 个位置参数 `output_root`
  - 输出根目录
  - 默认：`outputs/pile_lab_case`
- 第 4 个位置参数 `friction_angle`
  - 覆盖 `pile_lab` 的摩擦角
  - 默认：`50`

脚本行为：

- 先调用 `build.sh`
- 再运行 `klar2016_sand` 导出 PLY
- 最后调用 `tools/render_ply_sequence.py` 生成一条快速预览视频

输出结构：

- `${output_root}/frames`
  - PLY 帧目录
- `${output_root}/pile_lab.mp4`
  - matplotlib 预览视频

示例：

```bash
bash ./scripts/run_pile_lab_case.sh release 2400 outputs/pile_lab_case 50
```

这条命令的含义：

- 用 release 版运行
- 仿真 2400 步
- 输出到 `outputs/pile_lab_case`
- 用 `50` 度摩擦角

当前 pile-lab 相关常用命令：

- 只跑仿真并导出：

```bash
./build/Release/klar2016_sand \
  --scene scenes/pile_lab.json \
  --steps 2400 \
  --export \
  --output-dir outputs/pile_lab_dense_check_v2
```

- 摩擦角 sweep：

```bash
./build/Release/klar2016_sand \
  --scene scenes/friction_angle.json \
  --steps 128 \
  --sweep-friction-angles 20,25,30,35,40 \
  --export \
  --output-dir outputs/friction_sweep
```

这条命令的含义：

- 固定使用 `friction_angle.json` 这个场景；
- 依次把摩擦角设为 `20`、`25`、`30`、`35`、`40`；
- 每个角度都独立跑 `128` 步；
- 把结果分别写到 `outputs/friction_sweep/` 下的不同子目录。

### 5.4 Tests

```bash
ctest --test-dir build/Release --output-on-failure
```

## 6. Visualization

### 6.1 Fast preview

适合快速检查粒子是否下落、堆体是否形成。

```bash
python3 tools/render_ply_sequence.py \
  --input-dir outputs/pile_lab_dense_check_v2 \
  --scene scenes/pile_lab.json \
  --fit-mode hybrid \
  --camera-preset pile_lab \
  --fps 20 \
  --point-size 4.0 \
  --output outputs/pile_lab_dense_check_v2/pile_lab_preview.mp4 \
  --title "klar2016 pile lab preview"
```

说明：

- 这条链快，但只是预览。
- 不建议把 matplotlib 输出当最终论文风格结果。

关键参数说明：

- `--input-dir`
  - PLY 帧目录
- `--scene`
  - 对应的场景 JSON，用于相机和辅助几何
- `--fit-mode hybrid`
  - 当前 pile-lab 预览常用模式，兼顾沙堆与地面
- `--camera-preset pile_lab`
  - 使用专门给 pile-lab 准备的构图
- `--point-size`
  - 预览点大小，只影响 matplotlib 预览，不影响仿真
- `--output`
  - 预览视频输出路径

### 6.2 Final render with Blender

当前推荐的最终渲染方法：

```bash
export BLENDER_BIN=/home/ywj22/blender-5.0.1-linux-x64/blender

bash ./scripts/render_pile_lab_blender.sh \
  outputs/pile_lab_dense_check_v2 \
  outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1.mp4 \
  scenes/pile_lab.json
```

脚本：

- [render_pile_lab_blender.sh](/home/ywj22/wokdir/mpm/mpm_SIGGRAPH_2017/scripts/render_pile_lab_blender.sh)

参数说明：

- 第 1 个位置参数 `frames_dir`
  - PLY 帧目录
  - 默认：`outputs/pile_lab_cyl_phi50/frames`
- 第 2 个位置参数 `output_path`
  - 最终 MP4 路径
  - 默认：`outputs/pile_lab_cyl_phi50/pile_lab_blender.mp4`
- 第 3 个位置参数 `scene_path`
  - 场景 JSON
  - 默认：`scenes/pile_lab.json`
- 第 4 个位置参数 `shading_mode`
  - 当前常用：`studio`
  - 也支持诊断用 `flat`
- 第 5 个位置参数 `render_mode`
  - 当前常用：`points`
  - 也支持 `surface`

脚本行为：

- 自动寻找 Blender：
  - 优先使用环境变量 `BLENDER_BIN`
  - 否则尝试 `/home/ywj22/blender-5.0.1-linux-x64/blender`
  - 再否则退回系统里的 `blender`
- 调用 `tools/render_ply_sequence_blender.py`
- 默认使用 `Cycles`、`960x960`、`64 samples`

示例：

```bash
export BLENDER_BIN=/home/ywj22/blender-5.0.1-linux-x64/blender

bash ./scripts/render_pile_lab_blender.sh \
  outputs/pile_lab_dense_check_v2 \
  outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1.mp4 \
  scenes/pile_lab.json \
  studio \
  points
```

这条渲染链的当前特点：

- `Cycles`
- 自动按粒子间距估计渲染半径，避免“大白球互穿”
- 颗粒随机尺寸扰动
- 哑光砂粒材质
- 暗背景 + 暗地面 + 棚拍光源
- 轻微景深和运动模糊

如果只想渲一张检查图：

```bash
export BLENDER_BIN=/home/ywj22/blender-5.0.1-linux-x64/blender

"$BLENDER_BIN" -b -P tools/render_ply_sequence_blender.py -- \
  --input-dir outputs/pile_lab_dense_check_v2 \
  --scene scenes/pile_lab.json \
  --output outputs/pile_lab_dense_check_v2/check.png \
  --output-format PNG \
  --engine CYCLES \
  --samples 48 \
  --resolution 960 \
  --render-mode points \
  --shading-mode studio \
  --still-frame 50
```

说明：

- `--still-frame 50` 表示只渲第 50 帧。
- `--render-mode points` 是当前推荐的论文颗粒风格。
- `--shading-mode studio` 是当前推荐的最终交付灯光。

### 6.3 Suggested workflow

当前建议的完整链路：

1. 用 `build.sh` 构建。
2. 用 `klar2016_sand` 或 `run_pile_lab_case.sh` 导出 PLY。
3. 先用 `render_ply_sequence.py` 做快速预览。
4. 确认仿真阶段没有明显 bug 后，再用 Blender 出最终视频。

## Current Assessment

当前阶段已经具备：

- 从论文 2016 单相砂理论到最小可跑 3D 求解器；
- 从场景配置到 PLY 导出；
- 从快速预览到 Blender 最终渲染；
- 一个可直接查看的论文风格堆积案例输出。

当前最主要的剩余问题已经不是渲染链，而是模拟参数和堆体形状本身：

- `pile_lab` 的堆还偏扁；
- `hourglass` 仍与论文容器形状有差距；
- 后续应优先回到材料、发射、碰撞和几何参数调优。
