# 大视场鱼眼支持：编译与回归验证任务书

> 交给执行验证的智能体（下称"执行者"）。
> 被验证的提交：`6c23542`（分支 `claude/affectionate-einstein-6xq4zp`），"Support fisheye lenses wider than 180 degrees in spherical mode"。
> 该提交的 `src/vio.cpp`、`src/LIVMapper.cpp` 在开发环境中**没有编译过**（没有 ROS）。只有 `include/wide_fisheye_camera.h` 和 `test/test_wide_fisheye_camera.cpp` 在纯 Eigen 环境下编译并通过了测试。本任务就是补上这一步。

---

## 0. 背景：这次改了什么

- 新增 `include/wide_fisheye_camera.h`：类 `sphere::WideFisheyeCamera`，Kannala-Brandt（等距 k1–k4）模型，投影用 `θ = atan2(√(x²+y²), z)`，可以表示相机后方的视线；配置项 `cam_model: WideFisheyeCamera` 加 `max_incidence_deg`。
- `src/vio.cpp`：球面补丁路径中 9 处 `z() <= 0.2` 改为 `sphericalHidden()`，1 处 `z() > .2` 改为 `sphericalVisible()`；插入补丁时增加视锥检查；投影雅可比在新相机下改用 `jacobianPoint()`；补丁切向基在新相机下改用最不平行的坐标轴。
- `src/LIVMapper.cpp`：读取和校验新配置；点云着色的可见性判断。
- `CMakeLists.txt`：新增测试程序 `test_wide_fisheye_camera`（在 `CATKIN_ENABLE_TESTING` 打开时构建，catkin 默认打开）。

**设计约定**：配置为 `EquidistantCamera` 时，所有新分支都不会执行，结果应与改动前**逐位一致**。

---

## 1. 约束

1. 在**独立的工作空间**里编译，不要覆盖正在跑阶段 1 实验所用的二进制。建议路径 `~/sphere_ws_widefov`。
2. 不修改本提交的任何代码。发现编译错误时，把完整错误信息和你建议的最小修复写进汇报，**先不要自行提交修复**；若错误很明显（拼写、缺少头文件），可以在本地修好继续验证，但要在汇报中逐行列出改动。
3. 每次运行使用新的 `seq_name` 和新的输出目录，保存配置、日志、轨迹和二进制 SHA256。
4. 回放速率统一 0.5×，与阶段 0、阶段 1 保持一致。

---

## 2. 步骤

### 步骤 1：获取代码并编译

```bash
mkdir -p ~/sphere_ws_widefov/src
cd ~/sphere_ws_widefov/src
git clone https://github.com/mxxxOrz12/SPHERE-LIVO.git
git -C SPHERE-LIVO checkout 6c23542
```

`rpg_vikit` 使用与当前实验工作空间相同的版本（README 指定提交 `6c886c8e5d83997806e00294826d528cea3581dd`），可以复制或软链接过来。

按 README 的方式编译（Sophus 路径按你们的实际安装调整）：

```bash
source /opt/ros/noetic/setup.bash
cd ~/sphere_ws_widefov
catkin_make -j1 -l2 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSophus_LIBRARIES=/usr/local/lib/libSophus.so 2>&1 | tee build.log
```

检查项：
- 编译是否成功；
- `build.log` 中与本提交改动文件相关的**新增**警告（`wide_fisheye_camera.h`、`vio.cpp`、`vio.h`、`LIVMapper.cpp`、`LIVMapper.h`、`test_wide_fisheye_camera.cpp`），逐条列出。可用 `git diff 6c23542~1 6c23542 --stat` 确认改动文件范围。

### 步骤 2：运行单元测试

```bash
./devel/lib/fast_livo/test_wide_fisheye_camera
```

期望输出一行 `all wide fisheye camera checks passed (...)`，退出码 0。记录括号里的三个数字。

若可执行文件不在该路径，用 `find devel build -name test_wide_fisheye_camera` 查找。

### 步骤 3：回归检查（必须逐位一致）

用**改动前**的二进制（阶段 0 使用的 2026-11-10 归档版本，或在另一个工作空间编译提交 `6c23542~1`）和**本提交**的二进制，使用完全相同的 SPHERE Hilti 配置（`cam_model: EquidistantCamera`，即阶段 0 的配置），各跑一次：

- exp04（完整序列）
- exp09（完整序列，用来覆盖长时间运行）

比较：
1. `trajectory.tum`：逐行完全相同（`cmp` 或 `diff` 无输出）；
2. `*_timing.csv`：除 `vio_ms` 列（耗时，会有波动）外逐列相同；
3. 精度指标与阶段 0 记录一致。

如果不一致：
- 先在同一二进制上重复跑一次，确认改动前的二进制本身在当前环境下是否确定（阶段 0 中 SPHERE 在 0.5× 下是确定的）；
- 若确认是本提交导致的差异，找出第一条不同的轨迹行和对应的 timing CSV 行，记录在汇报中。

### 步骤 4：新模型的功能检查（Hilti 上的近似一致性）

Hilti 相机视场小于 180°，因此可以用新模型跑同一数据，结果应当**非常接近**但**不要求逐位一致**。原因：可见性判断从"深度 z > 0.2 m"变为"距离 > 0.2 m 且在视锥内"，雅可比的计算方式不同，浮点结果会有微小差别。

做法：复制阶段 0 的 SPHERE Hilti 配置，只把三个相机（cam0、cam3、cam4）改为：

```yaml
cam_model: WideFisheyeCamera
max_incidence_deg: 80
```

其他所有参数不变。`max_incidence_deg: 80` 大于 Hilti 相机对角半视场（约 80–82°）的大部分范围，基本不额外裁剪；如果启动报"polynomial is not monotonic"，改为 75 再试并在汇报中说明。

在 exp04 和 exp16 上各跑一次，与步骤 3 中本提交的 `EquidistantCamera` 结果比较：
- 精度（exp04 稠密 ATE、exp16 稠密 ATE 和 1 s RPE）；
- timing CSV 中 `accepted_patches`、`selected_patches` 的中位数，`guard_status` 中 `committed` 的占比；
- 两条轨迹逐时刻的位置差最大值和均方根。

期望：精度差在 2 mm 以内，补丁数和 committed 占比相差在 5% 以内。超出时如实报告，并给出两次运行中第一次明显分叉的时刻。

### 步骤 5：启动校验

各用一个最小改动的配置启动 `mapping_multifisheye.launch`（或你们的 Hilti launch 文件），只需确认启动阶段的行为，不必回放数据：

| 用例 | 配置改动 | 期望 |
|---|---|---|
| 5a | `WideFisheyeCamera` 但不写 `max_incidence_deg` | 启动报错 "WideFisheyeCamera requires max_incidence_deg in (0, 180)" |
| 5b | `WideFisheyeCamera` + `innovations/spherical_patch: false` | 启动报错 "WideFisheyeCamera is supported only with innovations/spherical_patch" |
| 5c | `WideFisheyeCamera`，`k1: -0.3`，其余 k 为 0，`max_incidence_deg: 100` | 启动报错 "polynomial is not monotonic" |
| 5d | `WideFisheyeCamera`，`max_incidence_deg: 80`，其余正常 | 正常启动 |

### 步骤 6（可选）：真实大于 180° 的数据

如果手头有视场大于 180° 的鱼眼数据（标定为 Kalibr `pinhole-equi`），在其上：
1. 先用 `max_incidence_deg` 等于 85 跑一次（相当于把视场裁到 170°）；
2. 再用标定能覆盖的最大入射角减去 3–5°（例如 200° 镜头用 95–97）跑一次；
3. 对比精度、被接受补丁数，以及补丁中心在图像上的分布（可在 `insertSphericalMapPoints` 处临时打印，不提交）。

没有这类数据就跳过，并在汇报中说明。

---

## 3. 汇报格式

```markdown
## 大视场鱼眼支持验证（日期，提交 6c23542，二进制 SHA256）

### 1. 编译
- 结果：成功 / 失败
- 本提交相关的新增警告：（逐条列出，或"无"）
- 若有本地修复：文件、行号、改动内容

### 2. 单元测试
- 输出原文：
- 退出码：

### 3. 回归（EquidistantCamera）
| 序列 | 轨迹逐位一致 | timing CSV（除 vio_ms）一致 | 指标 |

### 4. 新模型功能检查（WideFisheyeCamera, max_incidence_deg = 80）
| 序列 | 指标（Equidistant） | 指标（Wide） | 差 | accepted_patches 中位数（E / W） | committed 占比（E / W） | 轨迹位置差 RMS / max |

### 5. 启动校验
| 用例 | 期望 | 实际 |

### 6. 大于 180° 数据（如有）

### 结论
- 是否可以合并
- 需要修改的问题
```

---

## 4. 完成判定

1. 编译成功且无本提交相关的新增错误；
2. 单元测试通过；
3. 步骤 3 在 exp04 和 exp09 上轨迹逐位一致；
4. 步骤 4 的差异在期望范围内，或已给出超出原因的定位信息；
5. 步骤 5 四个用例的行为都与期望一致。

不满足时如实说明，不要为了通过而修改阈值或配置。
