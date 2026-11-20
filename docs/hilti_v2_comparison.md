# SPHERE-LIVO v2 与 Omni-LIVO：Hilti 2022 九序列对比任务书

> 交给执行实验的智能体（下称"执行者"）。
> 代码版本：分支 `claude/affectionate-einstein-6xq4zp` 上包含本文件的提交（下称"v2 提交"）。**必须从该提交编译**，不要叠加你们本地为 E2b 做的 `spherical_patch_angular_scale` 改动。
> 配置：`config/hilti_v2_overlay.yaml`（在现有 Hilti 预设之上覆盖的键）。
> 本轮目标：在九条 Hilti 序列上，用多个种子公平比较 SPHERE-LIVO 与 Omni-LIVO，并分清 LiDAR 地图修正和视觉改动各自的贡献。

---

## 1. 这一版改了什么（相对阶段 1）

| 改动 | 原因 | 默认值（不配置时） |
|---|---|---|
| `local_map/half_map_size_m`：以米为单位的局部地图半径 | 原 `half_map_size` 按**体素个数**计算。体素 0.5 m、`half_map_size: 30` 时，SPHERE 只保留 ±15 m，而 Omni 把同样的 30 当作米，保留 ±30 m。两边 LiDAR 地图大小一直不一致 | -1（关闭，沿用旧语义） |
| `innovations/spherical_shortlist_per_camera` | 原来写死 120 | 120 |
| `innovations/spherical_pool_size` | 原来写死 360 | 360 |
| `innovations/spherical_min_patches_per_camera`：每台相机保底名额 | Omni 每台相机至少 50 个点 | 0（关闭） |
| `innovations/spherical_chi2_gate`、`spherical_max_correction_m`、`spherical_max_correction_deg`：视觉修正合理性门限 | exp04、exp21 曾出现缺口后发散；代价下降不代表修正合理 | 0（关闭） |
| `multi_fisheye/frame_phase`：每 `frame_stride` 组中取第几组 | 作为种子，得到不同但同样合理的图像子集 | 0（原行为） |
| timing CSV 末尾新增两列 `correction_chi2`、`chi2_rejections` | 记录门限的作用 | — |

所有新参数取默认值时，算法行为与改动前相同。`include/spherical_information_selector.h` 在保底为 0 时与旧实现逐位一致（随机候选集 200/200 一致）；新增单元测试 `test/test_information_selector.cpp`。

**主程序在开发环境中未编译过**（没有 ROS）。编译和回归检查是本任务的第一步。

---

## 2. 约束

1. 回放速率统一 **0.5×**；不开 RViz；同一时间只运行一个实验；分析脚本在实验间隙运行。
2. 不修改标定、真值、评测对齐方式（SE(3)、不拟合尺度、不调时间偏移），沿用阶段 0/1 的评测脚本。
3. 每次运行新目录，保存：配置（合成后的完整 YAML）、二进制 SHA256、`git rev-parse HEAD`、`trajectory.tum`、`*_timing.csv`、日志、`metrics.json`、`mat_pre.txt` / `mat_out.txt`。
4. 内存采样改为**每 1 秒**一次（mapper 进程 RSS），并保存 `vmstat 1` 的输出。
5. 所有配置在所有序列上共用，不按序列调参。
6. Omni-LIVO 的代码和配置保持阶段 1 的 E5-U 设置不变。

---

## 3. 配置档

| 档位 | 程序 | 内容 |
|---|---|---|
| **S-LIO** | SPHERE v2 | 现有 Hilti 预设 + `half_map_size_m: 30` + `img_en: 0` |
| **O-LIO** | Omni | E5-U 配置 + `img_en: 0` |
| **V1-map** | SPHERE v2 | 现有 Hilti 预设 + `half_map_size_m: 30`（视觉参数全部不变） |
| **V2** | SPHERE v2 | V1-map + `config/hilti_v2_overlay.yaml` 中 `innovations` 下的全部键 |
| **Omni** | Omni | E5-U 配置 |

生成方式：从阶段 1 实际使用的 SPHERE 预设出发，只改上表列出的键，把合成后的完整 YAML 保存为 `config/hilti_v2_V1map.yaml`、`config/hilti_v2_V2.yaml`，提交到你们的实验记录中，并在汇报中列出它们与旧预设的 diff。

---

## 4. 种子（图像相位）

Hilti 原始图像组约 40 Hz，两方法都只用其中每 4 组的 1 组（10 Hz）。用 4 个相位 0、1、2、3 作为种子，每个相位对应一份不同的图像子集。

**两方法在同一相位下必须拿到完全相同的图像组。** 先确认你们现在是怎么降到 10 Hz 的：

- 若 SPHERE 用内部 `multi_fisheye/frame_stride: 4`（或 `preprocess/hilti_en`），而 Omni 由外部节点降采样：在 SPHERE 中设 `multi_fisheye/frame_phase: p`，并让 Omni 的外部节点保留同一组。
- 若两方法都由同一个外部节点降采样：在该节点中实现相位 p，SPHERE 保持 `frame_stride: 1`、`frame_phase: 0`。

**校验**：每个相位、每条序列，导出两方法实际使用的图像组时间戳列表（SPHERE 可在 `enqueueMultiFisheyeImage` 处或外部节点处记录），两份列表必须完全一致；相位 0 的列表必须与阶段 1 一致。在汇报中给出校验结果。

---

## 5. 序列

exp04、exp05、exp06、exp07、exp09、exp10、exp15、exp16、exp21（九条，完整 bag）。

真值与指标沿用阶段 0：exp04/05/06/16 报稠密 ATE 和 1 s RPE；有稀疏真值的报稀疏位置 RMSE（exp04/05/06 两者都有）。

---

## 6. 步骤

### 步骤 0：编译与回归（必须先通过）

1. 编译 v2 提交，记录与本次改动文件相关的新增警告。
2. 运行两个单元测试，均应输出 "all ... checks passed"：

```bash
./devel/lib/fast_livo/test_wide_fisheye_camera
```

```bash
./devel/lib/fast_livo/test_information_selector
```

3. **回归**：用 v2 二进制加**阶段 1 的 E2-1000 配置**（不加任何新键，相位 0），跑 exp04 和 exp09。与阶段 1 归档二进制的 E2-1000 轨迹比较，`trajectory.tum` 必须逐字节相同；timing CSV 除 `vio_ms` 和末尾两列新增列外逐列相同。
   - 若不同：在同一个 v2 二进制上再跑一次，判断是否本身不确定；找出第一条不同的轨迹行，与该时刻前后的 timing CSV、日志、`vmstat` 一起汇报。**回归不通过时停止，不进入后续步骤。**
4. 澄清阶段 1 的遗留问题：E2b-4x4 的两次运行是用 E2b 新二进制重新跑的，还是复用了 E2-10000 的结果？请在汇报中说明。

### 步骤 1：LIO-only（每档每序列 1 次）

S-LIO 和 O-LIO，九条序列。两者都是确定性的，各跑 1 次；S-LIO 在 exp07 上跑 2 次确认确定性。

**判读**：若 S-LIO 在 exp07、exp21 上接近 O-LIO（阶段 1 中 O-LIO 为 59.5、62.4 mm，S-LIO 为 81.9、95.4 mm），说明之前的 LIO 差距来自地图大小。

### 步骤 2：LIVO（每档每序列 4 个相位）

V1-map、V2、Omni，九条序列，每条 4 个相位，共 108 次。

执行顺序（每完成一批先出一次中间报告）：
1. 相位 0：V1-map、V2、Omni 各九条（27 次）。
2. 相位 1（27 次）。
3. 相位 2、3（54 次）。

若时间不够，最低要求是相位 0、1 全部完成。

---

## 7. 汇报内容

### 7.1 编译、单元测试、回归
- 编译结果与新增警告；两个单元测试输出原文；回归是否逐字节一致；E2b-4x4 的澄清。

### 7.2 LIO-only

| 序列 | S-LIO | O-LIO | 差 / mm | 阶段 1 的 S-LIO（±15 m 地图） |

### 7.3 LIVO 主表（每格为 4 个相位的均值 ± 样本标准差，另列最小–最大）

| 序列 | 指标 | S-LIO | V1-map | V2 | O-LIO | Omni |

### 7.4 配对比较（同一相位的图像子集相同，按相位配对）

| 序列 | V2 − Omni（4 个相位逐个列出） | 均值 | V2 更好的相位数 / 4 |

同样给出 V2 − V1-map、V1-map − Omni 两组配对表。

### 7.5 分组汇总
- 4 条稠密序列 ATE 等权均值、8 条稀疏序列 RMSE 等权均值，分别按 V1-map / V2 / Omni 给出（先对每条序列取 4 个相位均值，再跨序列平均）。
- 视觉贡献：每档 LIVO 均值减去对应 LIO-only。

### 7.6 运行诊断（V1-map 与 V2）
- `guard_status` 各取值占比，其中 `chi2_rejected` 单列；`correction_chi2` 的 P50/P95/最大值。
- `accepted_patches`、`selected_patches` 的 P10/P50/P90；按相机的 `camera_quotas` 均值。
- `vio_ms` 的 P50/P95/最大值；最大位姿间隔；覆盖检查结果。
- mapper RSS 峰值（1 s 采样），三档对比。注意：V1-map 的 LiDAR 地图比阶段 0/1 大，RSS 不能与之前的数字直接比较。

### 7.7 结论
- 地图修正对 LIO 和 LIVO 的影响；
- V2 相对 V1-map 是否有效（配对比较中 4 个相位多数一致才算）；
- V2 相对 Omni 的结论，按序列和分组分别给出；
- 是否出现发散或覆盖失败，若有，给出时刻、`vio_ms`、`chi2` 和日志。

---

## 8. 判定原则

- 单个相位的毫米级差异不作结论；配对比较中至少 3/4 个相位方向一致，且均值差大于相位间标准差，才写"更好"或"更差"。
- V2 同时改了多项（预算、短名单、候选池、保底、金字塔、迭代、门限）。本轮只回答"整体是否有效"；若有效，下一轮再做逐项消融。
- 结果不满足预期时如实报告，不调整阈值或配置去迎合。
