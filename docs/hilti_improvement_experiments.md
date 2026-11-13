# SPHERE-LIVO Hilti 2022 改进实验任务书

> 本文件交给执行实验的智能体（下称"执行者"）。请按阶段顺序执行，每阶段结束后按第 7 节的格式汇报，再进入下一阶段。
> 代码仓库：`SPHERE-LIVO`（ROS 包名 `fast_livo`，基于 FAST-LIVO2）。对比基线：`Omni-LIVO`。
> 上一轮结果：`E:\FastLIVO2_GS\results\Hilti_comparison_20261110\report_zh.md`（下称"基线报告"）。

---

## 1. 背景与目标

基线报告（2026-11-10，三鱼眼 cam0/cam3/cam4，10 Hz，0.8× 回放）结论：

| 分组（等权均值） | SPHERE-LIVO / m | Omni-LIVO / m |
|---|---:|---:|
| 4 条稠密 IMU ATE（exp04/05/06/16） | 0.046 | 0.040 |
| 8 条稀疏位置 RMSE | 0.087 | 0.071 |

- 施工区域 exp04/05/06：两者几乎相同（LiDAR 主导）。
- 落后集中在**视角变化大、长时间、回访多**的序列：exp07（长走廊）、exp09/10（穹顶）、exp15/16（阁楼→画廊）。
- exp21（室外）0.8× 首轮：SPHERE 出现 1.9 s 位姿缺口、稀疏 RMSE 114 m；Omni 正常。

**本轮目标**：
1. 先定位视觉模块是"没起作用"还是"起反作用"；
2. 实施并消融下列代码改进，使 SPHERE-LIVO 在 8 条稀疏序列均值上**不劣于** Omni-LIVO，exp21 在 0.8× 下不再发散；
3. 每项改动都必须有独立开关和消融结果，不允许只报告最好组合。

---

## 2. 硬性约束（每阶段都必须遵守）

1. **不修改**：标定、真值、评测脚本的对齐方式（SE(3)、不拟合尺度、不调时间偏移）、Omni-LIVO 代码与配置（除第 6 节"同预算对照"明确要求外）。
2. 所有新功能都加 `innovations/*` 开关，**默认值 = 旧行为**，确保关闭开关后与当前二进制结果一致（在 exp04 上验证 ATE 差 < 1 mm）。
3. 回放速率在同一阶段内固定。建议本轮全部使用 **0.5×**；稳定性测试（阶段 4）额外跑 0.8× 和 1.0×。
4. 每个运行使用新的输出目录和新的 `seq_name`，保留失败运行，不覆盖。
5. 每个运行保存：`config.yaml`、二进制 SHA256、`git rev-parse HEAD` + `git diff` 补丁、`trajectory.tum`、`*_timing.csv`、日志、`metrics.json`。
6. 只跑一次的结果，毫米级差异不得写成"优于"。每个关键配置至少在 exp07、exp09、exp16 上**重复 3 次**，报告均值 ± 标准差。
7. 不要因为某条序列变差就为该序列单独调参；所有序列共用一套配置。

---

## 3. 阶段 0：诊断（不改算法代码）

### 3.1 LIO-only 对照

在 exp07、exp09、exp10、exp15、exp16、exp21 上，用 SPHERE-LIVO 的同一 Hilti 配置，仅设置 `common/img_en: 0`，跑 LIO-only。Omni-LIVO 同样跑一次 LIO-only。

产出表格：

| 序列 | LIO-only(SPHERE) | SPHERE | LIO-only(Omni) | Omni |
|---|---:|---:|---:|---:|

判读：
- SPHERE ≈ LIO-only（差 < 3 mm）→ 视觉基本没贡献，重点看 3.2 中的拒绝率和补丁数。
- SPHERE 明显差于 LIO-only → 视觉在引入错误约束，优先做阶段 2 的 4.3（卡方门限）与 4.4（自适应噪声）。
- Omni 优于 Omni-LIVO 的 LIO-only → Omni 的视觉确实有效，对比目标成立。

### 3.2 解析 `*_timing.csv`

列名（逗号分隔，列内多相机值用 `;` 分隔）：

```
timestamp,mode,cameras,selected_patches,vio_ms,map_points,active_ids,lidar_weak_ratio,camera_scores,accepted_patches,camera_quotas,refreshed_patches,temporal_selected_patches,guard_accepted_steps,guard_rejected_steps,guard_backtracks,guard_initial_cost,guard_final_cost,guard_status,incremental_pool_size,unique_world_patches,geometry_fine_rejections,geometry_coarse_rejections,brightness_gains,brightness_biases,brightness_fit_rejections,brightness_bound_fits
```

写一个 Python 脚本 `scripts/analyze_vio_timing.py`，对每条序列输出：
- `guard_status` 各取值占比（`committed`、`photo_only_committed`、`prior_kept`、`initial_support_rejected`、`support_rejected`、`final_support_rejected`、`final_cost_rejected`）；
- `accepted_patches`、`selected_patches`、`cameras` 的 P10/P50/P90；
- `vio_ms` 的 P50/P95/max，以及随时间的曲线；
- `lidar_weak_ratio` 的分布（P10/P50/P90）；
- `map_points` 随时间曲线；
- 按 30 s 时间窗统计 `committed` 占比与 `accepted_patches` 中位数（看是否随时间衰减——衰减说明参考外观过期，对应 4.1）。

对 exp21 的 0.8× 首轮失败运行同样分析，定位 1.9 s 缺口前后 10 s 的 `vio_ms`、`map_points`、日志中的 "queue overflow" / "Dropping" 警告。

**阶段 0 判读重点**：若 `committed` < 50% 或 `accepted_patches` P50 < 60，或 committed 率/补丁数随时间明显下降，则 4.1 为第一优先。

---

## 4. 阶段 1–2：代码改进（每项一个开关）

以下行号基于提交 `804e82c`，实施前请先 `grep` 确认位置。

### 4.1 多视角参考外观快照（预计收益最大）

**问题**：补丁外观只在插入时记录一次。当前 Hilti 预设开启了 `spherical_geometry_support` 和 `spherical_shared_brightness`，而 `src/LIVMapper.cpp` 约第 95 行和第 103 行的校验禁止这两者与 `spherical_reference_refresh` 同时开启，因此外观永久冻结；`insertSphericalMapPoints` 遇到已有补丁的体素直接跳过，补丁也不会被替换。长序列与回访时视角/曝光变化大，ZNCC 与光度误差变差，有效视觉约束减少。Omni-LIVO 每点保留最多 15 个观测，用 `getCloseViewObs` 取视角最近的参考，并持续 `updateReferencePatch`。

**开关**：`innovations/spherical_multi_view_appearance`（bool，默认 false），`innovations/spherical_appearance_slots`（int，默认 4）。

**实现要求**：
1. `include/vio.h` 的 `SphericalPatchPoint` 增加：
   ```cpp
   struct SphericalAppearance {
     V3D origin_w; int camera_id; int frame_id;
     double gain, bias;                       // 写入时的相机亮度（归一化用）
     std::vector<float> intensity;            // 与 patch_world 一一对应，已归一化到规范亮度
     std::vector<std::vector<float>> coarse_intensity;
   };
   std::vector<SphericalAppearance> appearances;  // 最多 slots 个
   ```
   插入时把原 `intensity` 作为第 0 个快照。
2. 新增 `selectAppearance(const SphericalPatchPoint&, const V3D& current_origin_w)`：返回与当前视线方向 `(center_w - origin)` 夹角最小的快照。`linearizeSpherical`、`evaluateSphericalCost`、`prepareSphericalFrame` 中所有读取 `patch.intensity` / `coarse_intensity` 的位置，开关开启时改为读所选快照。**同一次更新事务内（含 descent guard 的 fixed support）所用快照必须固定**，在 `SphericalCostPatch` 中记录快照索引。
3. 新增快照写入（可复用 `refreshSphericalReferences` 的验证逻辑，但**不修改 3D 采样点**）：仅当本帧视觉更新状态为 `committed`，该补丁在本帧被选中且 ZNCC ≥ 0.9、残差 MSE ≤ 100、与已有所有快照的视线夹角 > 15°（或最旧快照年龄 ≥ 30 帧）时写入；满额时替换与当前视角最近的那个（保持视角多样性）。强度按当前相机已拟合的 gain/bias 归一化后存储。
4. 开关开启时，放开 `LIVMapper.cpp` 中"geometry support / shared brightness 与 refresh 互斥"的校验（只针对新路径；旧 `spherical_reference_refresh` 仍保持原互斥）。
5. timing CSV 追加列：`appearance_writes`、`mean_appearance_count`、`selected_appearance_age_p50`。

**验证**：exp04 上开关关闭时与旧二进制一致；开启后在 exp07/09/10/15/16 上对比 `accepted_patches` 随时间曲线与 RPE/ATE。

### 4.2 候选评分随退化程度自适应 + 加大短名单 + 每相机保底

**问题**：`src/vio.cpp` 约第 499 行评分为 `zncc² · (wᵀ H_t w + 0.01·tr H_t)`，w 是 LiDAR 最弱平移方向，且只用了平移信息。Hilti 多数时刻 LiDAR 不退化，此时 w 近似随机，短名单被偏置。约第 528 行每相机短名单硬编码 120；`LIVMapper.cpp` 的 incremental 路径候选池硬编码 360，且无每相机保底。

**开关**：
- `innovations/spherical_adaptive_weak_score`（bool，默认 false）
- `innovations/spherical_shortlist_per_camera`（int，默认 120）
- `innovations/spherical_pool_size`（int，默认 360）
- `innovations/spherical_min_patches_per_camera`（int，默认 0）

**实现要求**：
1. 评分改为 `zncc² · [ α·wᵀH_t w / λ_ref + (1-α)·log det(I + Lᵀ H_6 L) ]`，其中 `H_6` 为含旋转的 6×6 有效信息（该候选在 posterior 路径已计算，见 `candidate_information`），L 为后验因子；`α = clamp((τ - r)/τ, 0, 1)`，`r = last_lidar_weak_ratio`，`τ = 0.3`（可配置 `spherical_degeneracy_ratio`）。r 无效（-1）时 α = 0。`last_lidar_weak_ratio` 需通过参数传入 `prepareSphericalFrame`。
2. 短名单与候选池大小改为参数；建议实验值：短名单 = 预算，候选池 = 3×预算。
3. `selectSphericalInformation`（`include/spherical_information_selector.h`）增加 `min_per_camera` 参数：先对每个有候选的相机（受 `camera_limit` 约束）用同一 logdet 增益准则各贪心选 `min_per_camera` 个，再进入全局贪心。

### 4.3 视觉更新合理性门限 + 积压时降级（针对 exp21）

**问题**：descent guard 只保证代价下降，错误匹配也可能使代价下降并被提交；mapper 处理慢时会卡住流水线（1.9 s 缺口）。

**开关**：`innovations/spherical_chi2_gate`（double，默认 0 = 关闭；建议 22.46，即 6 自由度 99.9%），`innovations/spherical_max_correction_m`（默认 0 = 关闭；建议 0.2），`innovations/spherical_max_correction_deg`（默认 0；建议 2.0），`innovations/vio_skip_backlog`（int，默认 0 = 关闭；建议 3），`innovations/vio_skip_ms`（double，默认 0；建议 80）。

**实现要求**：
1. 在 `runSphericalVIO` 的 `update_from_cameras` 最终提交前（写 `_state.cov` 之前），计算 `δ = (_state - prior_state).head<6>()`，`d² = δᵀ (P_prior[0:6,0:6])⁻¹ δ`；若 `d² > chi2_gate` 或平移修正 > max_correction_m 或旋转修正 > max_correction_deg，则 `_state = prior_state`，`last_visual_guard_status = "chi2_rejected"`。
2. 在 `handleVIO` 中：若 `img_group_buffer.size() > vio_skip_backlog`，或上一帧 `vio_ms > vio_skip_ms`，本帧跳过视觉更新（仍执行地图点插入的低成本部分可选，默认也跳过），CSV 的 `guard_status` 记 `skipped_backlog`。
3. 在 timing CSV 追加分阶段耗时列：`prepare_ms,select_ms,update_ms,insert_ms`，以及 `img_backlog`。

### 4.4 按相机自适应视觉噪声

**问题**：`IMG_POINT_COV` 全程固定（Hilti 预设 1000），仅靠 Huber 与 ZNCC 门限抵御退化。

**开关**：`innovations/spherical_adaptive_noise`（bool，默认 false），`spherical_noise_scale_min`（默认 0.5），`spherical_noise_scale_max`（默认 3.0），`spherical_noise_smoothing`（默认 0.6）。

**实现要求**：每个相机在上一次 `committed` 更新后，用已接受像素残差计算稳健方差 `σ² = (1.4826·MAD)²`；尺度 `s_c = clamp(σ² / IMG_POINT_COV, min, max)`，按 `s ← α·s_new + (1-α)·s_old` 平滑。在 `linearizeSpherical` 累加信息与梯度时，按相机除以 `s_c`；`evaluateSphericalCost` 中同步使用同一 `s_c`（同一事务内固定）。CSV 追加 `noise_scales` 列（`;` 分隔）。

### 4.5 收敛范围（仅改配置）

无需改代码。消融值：`spherical_pyramid_levels ∈ {1, 2, 3}`，`joint_max_iterations ∈ {3, 5}`。

---

## 5. 实验矩阵

序列：exp04、exp05、exp06、exp07、exp09、exp10、exp15、exp16、exp21（全部 9 条），0.5×。

| ID | 配置 | 说明 |
|---|---|---|
| B0 | 当前 bounded_geometry_safe | 基线复现（0.5×） |
| B0-LIO | `img_en: 0` | 阶段 0 |
| C1 | B0 + pyramid 2, iter 5 | 4.5 |
| C2 | B0 + pyramid 3, iter 5 | 4.5 |
| A1 | B0 + 4.1 | 多视角外观 |
| A2 | B0 + 4.2 | 评分与预算 |
| A3 | B0 + 4.3 | 门限与降级 |
| A4 | B0 + 4.4 | 自适应噪声 |
| F1 | B0 + 4.1 + 4.2 + 4.3 + 4.4 + C1/C2 中较好者 | 完整组合 |
| F1-x | F1 分别去掉 4.1 / 4.2 / 4.3 / 4.4 | 留一消融 |

执行顺序：B0、B0-LIO → C1、C2 → A3（最先做，保证后续运行不发散）→ A1、A2、A4 → F1 → F1-x。

若计算资源有限，先只跑 exp07、exp09、exp16、exp21 四条筛选，筛出有效项后再补全 9 条。

---

## 6. 阶段 3–4：公平对照与稳定性

### 6.1 同预算对照

当前两方法视觉预算不同（SPHERE：3 相机、180 patch；Omni：总视觉点 300、视觉 voxel 上限 3000），属于系统对比。补一组：
- SPHERE F1：`total_patch_budget: 300`，`max_active_cameras: 3`；
- Omni：保持 `max_total_points: 300`。

在 9 条序列上各跑 3 次，报告均值 ± 标准差。

### 6.2 回放速率稳定性

F1 与 Omni 在 exp09、exp21 上分别以 0.5×、0.8×、1.0× 各跑 3 次。报告：完整回放率、时序覆盖检查通过率（首位姿延迟 < 2 s、最大位姿间隔 < 0.25 s、位姿数 ≥ 预期 10 Hz 的 95%）、`vio_ms` P95、mapper 峰值 RSS（建议采样周期改为 1 s）。

---

## 7. 每阶段汇报格式

```markdown
## 阶段 X 结果（日期，提交 SHA）

### 改动
- 文件与函数列表，开关名与默认值
- 关闭开关时 exp04 回归检查：ATE 旧/新 = …

### 精度
| 序列 | 配置 | 稠密 ATE / m | 1 s RPE / m | 稀疏 RMSE / m | 重复次数 | 标准差 |

### 视觉统计（来自 timing CSV）
| 序列 | 配置 | committed 率 | accepted_patches P50 | vio_ms P95 | 其他新增列 |

### 结论
- 有效 / 无效 / 有害，依据是什么
- 下一步
```

---

## 8. 成功标准

1. F1 在 8 条稀疏序列等权均值 RMSE ≤ Omni-LIVO（0.5×，3 次均值）。
2. F1 在 4 条稠密序列 ATE 均值 ≤ Omni-LIVO + 2 mm。
3. exp21 在 0.8× 下 3/3 次通过时序覆盖检查，且无 > 1 m 的稀疏误差。
4. F1 不劣于 B0-LIO（视觉贡献为正）。
5. 每个保留在 F1 中的模块，在留一消融中去掉后误差上升（至少在走廊/穹顶/阁楼序列中的两条上）。

不满足时如实报告，不要为满足标准而按序列调参。
