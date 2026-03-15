# Klár 2016 初始化清单

目标：基于 *Drucker-Prager Elastoplasticity for Sand Animation*，先完成一个可运行的 `3D single-species dry sand` 原型，范围限定为 `explicit APIC/MPM + Drucker-Prager + hardening + 边界碰撞`。

## 1. 输入资料

- [ ] 保存并确认以下论文材料可直接查阅：
  - `references/MinerU_html_KGPSJT16_2032719086785196032.html`
  - `references/MinerU_KGPSJT16__20260314072304.json`
- [ ] 尽量补齐 `Klár et al. 2016 supplementary technical document`
- [ ] 从论文表格中抄出首批复现场景参数：
  - `Hourglass`
  - `Friction angle`
  - `Pile from spout`

## 2. 开发环境

- [ ] 安装 `CMake 3.26+`
- [ ] 安装支持 `C++20` 的编译器：
  - `clang++ 16+` 或 `g++ 13+`
- [ ] 安装 `Ninja`
- [ ] 准备依赖获取方式，二选一：
  - `Conan 2.x`
  - 系统包管理器 + 手工 `find_package`
- [ ] 打开 `compile_commands.json` 支持

## 3. 三方库

首版必需：

- [ ] `Eigen 3.4+`
- [ ] `nlohmann/json`
- [ ] `GoogleTest`
- [ ] `tinyply` 或自写简单 `PLY` 导出器

首版建议：

- [ ] `fmt`
- [ ] `spdlog`

可以延后：

- [ ] `oneTBB`
- [ ] `OpenVDB`

说明：

- 首版不做 2017 多相水砂，也不做稀疏 OpenVDB 网格，因此 `OpenVDB` 不是起步依赖。
- 如果先做单线程版本，`TBB` 也可以后补。

## 4. 目录结构

- [ ] 建立以下基础结构：

```text
.
├── CMakeLists.txt
├── cmake/
├── external/                # 如果不用 Conan
├── include/
│   ├── mpm/
│   ├── math/
│   ├── io/
│   └── scene/
├── src/
│   ├── main.cpp
│   ├── mpm/
│   ├── math/
│   ├── io/
│   └── scene/
├── tests/
├── scenes/
│   ├── hourglass.json
│   ├── friction_angle.json
│   └── pile_from_spout.json
├── output/
└── docs/
```

- [ ] 将论文参数表单独整理成 `docs/klar2016_parameters.md`

## 5. 首版构建配置

- [ ] `CMakeLists.txt` 中先只生成一个主程序和一个测试程序
- [ ] 打开编译选项：
  - `-Wall -Wextra -Wpedantic`
  - Debug 下保留符号
  - Release 下打开 `-O3`
- [ ] 统一浮点精度：
  - 首版固定 `double`
- [ ] 先不引入平台专用优化和 SIMD

## 6. 首批核心模块

### 6.1 数学模块

- [ ] `Vec3 / Mat3` 类型别名
- [ ] `SVD`
- [ ] `polar decomposition`
- [ ] `det / trace / inverse / clamp`
- [ ] Hencky strain 相关辅助函数

### 6.2 核心数据结构

- [ ] `Particle`
  - `x`
  - `v`
  - `B` 或等价 APIC affine 项
  - `F_e`
  - `alpha`
  - `q`
  - `mass`
  - `volume0`
  - `material_id`
- [ ] `GridNode`
  - `mass`
  - `velocity`
  - `force`
  - `active`

### 6.3 求解器模块

- [ ] cubic B-spline 权重与梯度
- [ ] P2G 质量传输
- [ ] APIC 动量传输
- [ ] 网格显式力更新
- [ ] G2P 速度回传
- [ ] 位置更新
- [ ] `F_e` 预测更新
- [ ] Drucker-Prager 投影
- [ ] hardening 更新
- [ ] 边界碰撞与摩擦

## 7. 配置接口

- [ ] 定义 `SceneConfig`
  - 域尺寸
  - 重力
  - 边界条件
  - 发射器几何
  - 发射速率
  - 仿真总时长
  - 输出步长
- [ ] 定义 `MaterialConfig`
  - `rho`
  - `E`
  - `nu`
  - `friction_angle`
  - `h0/h1/h2/h3`
- [ ] 定义 `SolverConfig`
  - `dx`
  - `dt_max`
  - `cfl`
  - `particles_per_cell`
  - `explicit_only`

## 8. 首批参数录入

先录入论文表格中闭合度最高的 3 个场景：

- [ ] `Hourglass`
  - `dt = 1e-4`
  - `dx = 0.0025`
  - `particles = 4.60e5`
  - `particles_per_cell = 8`
  - `rho = 2200`
  - `E = 3.537e5`
  - `nu = 0.3`
  - `h0/h1/h2/h3 = 35/9/0.3/10`
- [ ] `Pile from spout`
  - `dt_max = 1.5e-4`
  - `dx = 8.3e-4`
  - `particles = 9.94e5`
  - `particles_per_cell = 32`
  - `rho = 2200`
  - `E = 3.537e5`
  - `nu = 0.3`
  - `friction_angle = 30`
- [ ] `Friction angle`
  - 建立多个摩擦角版本：`20/25/30/35/40`

## 9. 开发顺序

- [ ] 第 1 步：只做 `P2G + G2P`，验证质量守恒
- [ ] 第 2 步：加入显式网格受力更新
- [ ] 第 3 步：加入 APIC affine 项
- [ ] 第 4 步：加入 `F_e` 更新和弹性应力
- [ ] 第 5 步：加入 Drucker-Prager 投影
- [ ] 第 6 步：加入 hardening
- [ ] 第 7 步：加入边界摩擦
- [ ] 第 8 步：复现 `Hourglass`
- [ ] 第 9 步：复现 `Pile from spout`

## 10. 测试清单

单元测试：

- [ ] B-spline 权重和为 `1`
- [ ] B-spline 梯度在对称点行为正确
- [ ] 无外力单步质量守恒
- [ ] APIC 单步动量守恒
- [ ] `det(F_e) > 0`
- [ ] SVD / polar 在极端形变下不产生 `NaN`
- [ ] 投影后状态不越出 Drucker-Prager 屈服面
- [ ] hardening 更新后 `alpha` 在合理范围内

场景回归：

- [ ] `Hourglass` 形成稳定砂流和堆积
- [ ] `Friction angle` 变化能明显改变堆积角
- [ ] `Pile from spout` 能形成连续落砂和稳定砂堆

## 11. 输出与检查

- [ ] 每帧导出粒子 `PLY`
- [ ] 输出基础统计：
  - 粒子数
  - 活跃网格节点数
  - 当前 `dt`
  - frame time
- [ ] 保存最终参数快照到输出目录
- [ ] 记录每个场景与论文表格的差异

## 12. 首版完成标准

- [ ] 可以从命令行读取 `scene json`
- [ ] 可以稳定跑完至少一个 3D 场景
- [ ] `Hourglass` 或 `Pile from spout` 结果形态合理
- [ ] 单元测试全部通过
- [ ] 输出粒子可被 Blender / ParaView 正常读取

## 13. 明确不做

- [ ] 不实现 2017 多相水砂耦合
- [ ] 不实现 `OpenVDB` 稀疏网格
- [ ] 不实现隐式求解首版
- [ ] 不实现体渲染或项目内可视化 GUI

## 14. 做完 2016 后的下一步

- [ ] 先不要直接跳到 2017
- [ ] 先对照 `Klár 2016 supplementary technical document` 补齐当前 2016 缺口

当前实现相对技术文档仍有的主要差距：

- [ ] `implicit solve` 尚未实现
  - 技术文档有 `Algorithm 2`：
    `Newton + GMRES + Hessian-times`
  - 当前代码仍是 `explicit_only=true`
- [ ] 碰撞模型尚未对齐技术文档
  - 技术文档区分：
    `sticky / slipping / separating`
  - 当前代码是统一的平面/壳体单边约束 + 摩擦投影
  - 还没有技术文档里的 `SDF constraint recording` 和与隐式求解耦合的那一套
- [ ] 初始化尚未完全对齐技术文档
  - 技术文档说明：
    `Poisson disk sampling`
    `B_p` 根据初始速度梯度初始化
    `q_p^n = 0`
    `F_p^{E,n} = I`
  - 当前代码里：
    `F_e = I`、`hardening_state = 0` 已基本符合
    但粒子采样仍是 `random / regular / stratified cylinder`
    `B_p` 只覆盖零初速度这种最小情形
- [ ] 论文/技术文档中的完整隐式力导数链尚未实现
  - 包括 `Y(F)`、`M = ∂Y/∂F`、`HESSIAN_TIMES`
  - 当前只实现了显式用到的弹性能量导数和塑性投影

建议的下一步顺序：

- [ ] 第一步：把当前实现明确定位为
  `Klár 2016 explicit subset`
- [ ] 第二步：补 `Poisson disk seeding`
- [ ] 第三步：补更贴近技术文档的碰撞接口
  - `sticky`
  - `slipping`
  - `separating`
- [ ] 第四步：如果目标是“和 2016 技术文档基本对齐”，实现 `Algorithm 2`
  - `Newton`
  - `GMRES`
  - `Hessian-times`
- [ ] 第五步：只有在上述缺口补齐后，再考虑 2017 的
  `volume correction scalar v_c`

进入 2017 的前置判断：

- [ ] 如果目标只是继续改 `pile_lab`、`hourglass`、`friction angle` 这些显式场景，
  可以先不做隐式，继续调 2016 显式子集
- [ ] 如果目标是声称“2016 技术文档已基本做完”，则必须先补上隐式和碰撞差距
- [ ] 只有在 2016 差距关闭后，再把 2017 `Fig. 9 volume correction` 作为下一阶段
