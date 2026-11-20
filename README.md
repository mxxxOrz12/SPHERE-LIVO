# SPHERE-LIVO

Multi-fisheye LiDAR–inertial–visual odometry built on
[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2).

SPHERE-LIVO combines synchronized fisheye cameras with LiDAR and IMU measurements.
It adds shared spherical patches, information-based observation selection and a
joint visual update with a configurable patch budget. The ROS package remains
`fast_livo` for compatibility with the upstream implementation.

## Features

- Calibrated equidistant fisheye cameras with raw or JPEG-compressed ROS images.
- World-anchored spherical patches shared across cameras using local LiDAR planes.
- Camera and patch selection informed by LiDAR geometric constraints.
- Per-camera affine brightness to a canonical scale and a guarded joint update.
- One method configuration for every dataset: nine parameters (four of them
  ablation switches), with all fixed constants in one header.
- TUM trajectories, visual timing logs and reconstructed PCD point clouds.

This is research software. Current experiments do **not** establish a consistent
accuracy advantage over Omni-LIVO. Results depend on the sequence, calibration,
configuration and replay rate. A bounded patch count does not guarantee a bounded
total processing time. This repository provides odometry and mapping; it does not
implement loop closure or 3D Gaussian reconstruction.

## Build

Validated locally on Ubuntu 20.04 / ROS Noetic, including WSL. Install ROS Noetic,
PCL, Eigen, OpenCV and the non-templated Sophus version described in
[the upstream instructions](README.upstream.md). For evaluation, install Python
NumPy, SciPy and PyYAML.

```bash
mkdir -p ~/sphere_ws/src
cd ~/sphere_ws/src
git clone https://github.com/mxxxOrz12/SPHERE-LIVO.git
git clone https://github.com/xuankuzcr/rpg_vikit.git
git -C rpg_vikit checkout 6c886c8e5d83997806e00294826d528cea3581dd
source /opt/ros/noetic/setup.bash
cd ~/sphere_ws
catkin_make -j1 -l2 -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSophus_LIBRARIES=/usr/local/lib/libSophus.so
source devel/setup.bash
```

The Sophus library path above assumes the upstream `/usr/local` installation.
Adjust it to your installation. Serial compilation is recommended on machines
with limited memory. Dataset files and compiled dependencies are not included.

## Configuration

Each run loads two files:

- a **sensor configuration** per dataset or rig: topics, LiDAR/IMU settings,
  camera calibration and outputs (for example `config/sphere_oxford.yaml`);
- the **method configuration** `config/sphere_method.yaml`, identical for every
  dataset. It holds the nine SPHERE-LIVO parameters and the three visual-update
  settings (`img_point_cov`, `patch_size`, `max_iterations`).

Fixed constants, and the quantities derived from the method parameters, are
listed in `include/sphere_constants.h`. An unknown key under `innovations` is
rejected at start-up, so configurations from earlier versions fail loudly
instead of silently running a different method.

Ablations overlay one file from `config/ablation` on the method configuration:
`fastlivo2_baseline`, `no_shared_map`, `no_geometry_support`,
`no_brightness_model` and `texture_selection`.

```bash
roslaunch fast_livo mapping_sphere.launch \
  sensor_config:=/absolute/path/to/dataset.yaml seq_name:=run01 ablation:=none
```

## Oxford Spires example

`config/sphere_oxford.yaml` is the Oxford Spires sensor configuration.
`mapping_sphere_oxford.launch` loads it with the shared method configuration
and enables point cloud saving.

First terminal (add `ablation:=<name>` for an ablation):

```bash
source ~/sphere_ws/devel/setup.bash
roslaunch fast_livo mapping_sphere_oxford.launch seq_name:=observatory_test
```

Second terminal:

```bash
source /opt/ros/noetic/setup.bash
rosbag play --clock -r 0.4 --duration 30 /path/to/observatory-quarter-01.bag
```

Use a fresh `seq_name` for every run. Allow the mapper to finish processing after
bag playback, then stop it with Ctrl+C so point clouds are flushed. On shutdown,
the default outputs are inside this source package:

- `Log/result/<seq_name>.txt`: IMU trajectory in TUM format.
- `Log/result/<seq_name>_timing.csv`: visual update timing and counts.
- `Log/pcd/`: reconstructed point clouds.

The preset consumes the original compressed bag images. Official separately
distributed white-balanced images require timestamp-matched input preparation;
they are not substituted automatically.

For another rig, see [camera and method notes](MULTI_FISHEYE.md) and
`config/multi_fisheye.template.yaml`. The template contains intentionally invalid
calibration values and must be filled before use.

## Evaluate a trajectory

Oxford reference poses use a different body frame from the estimated IMU poses.
The supplied evaluator applies the published frame conversion and rigid SE(3)
alignment, without fitting scale:

```bash
python3 scripts/evaluate_oxford_tum.py \
  Log/result/observatory_test.txt /path/to/gt-tum.txt \
  --config config/sphere_oxford.yaml
```

The algorithm of commit 804e82c was compiled and replayed on 30-second prefixes
of Hilti exp04 and Oxford Observatory-01. These are functional checks of
trajectory and point-cloud output, not full-sequence accuracy claims. The
parameter consolidation that followed removed configuration paths not used by
the Oxford preset; on a synthetic three-camera scene it reproduces the earlier
preset bit for bit, but it has not yet been replayed on the datasets.
Full datasets, experiment outputs and the Omni-LIVO comparison implementation are
managed separately.

## Limitations

Complete camera groups are required; camera dropout can stall the pipeline.
The current fisheye model expects a calibrated field of view below 180 degrees.
Spherical patches assume locally planar surfaces. Camera clocks, image sizes and
extrinsics must match the configuration. Fisheye COLMAP export is not supported
by the inherited pinhole exporter; keep `colmap_output_en: false`.

## Acknowledgements and license

This work modifies FAST-LIVO2 and uses rpg_vikit camera models. Original history,
copyright notices and the upstream [GPL-2.0 license](LICENSE) are retained.
See [README.upstream.md](README.upstream.md) for original authors, papers,
citations and additional upstream examples.

## 中文说明

SPHERE-LIVO 是基于 FAST-LIVO2 的多鱼眼激光–惯性–视觉里程计研究实现，
包含共享球面补丁、信息驱动的观测选择以及联合视觉更新。
ROS 包名仍为 `fast_livo`。每次运行加载两份配置：数据集各自的传感器配置
（话题、标定、LiDAR/IMU），以及所有数据集共用的方法配置
`config/sphere_method.yaml`；固定常数集中在 `include/sphere_constants.h`。
消融实验通过 `ablation:=<config/ablation 中的文件名>` 切换。
Oxford 使用 `mapping_sphere_oxford.launch`，轨迹和点云保存在源码目录的 `Log/` 下。
现有实验尚未证明相对 Omni-LIVO 存在稳定的精度优势；请根据序列实测评估。
