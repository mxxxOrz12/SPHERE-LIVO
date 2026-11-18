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
- Configurable camera, patch, candidate-probe and visual-map budgets.
- Optional brightness models, geometry support and guarded visual updates.
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

## Oxford Spires example

`config/sphere_oxford.yaml` is the complete preset used in the successful
migration replay check. It enables the current SPHERE-LIVO profile and point
cloud saving. It differs from the older experimental presets retained in
`config/oxford_spires_observatory_01.yaml`.

First terminal:

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

The current algorithm source was compiled and replayed on 30-second prefixes of
Hilti exp04 and Oxford Observatory-01 before publication. These are functional
checks of trajectory and point-cloud output, not full-sequence accuracy claims.
Full datasets, experiment outputs and the Omni-LIVO comparison implementation are
managed separately.

## Limitations

Complete camera groups are required; camera dropout can stall the pipeline.
`EquidistantCamera` expects a calibrated field of view below 180 degrees. For
wider lenses use `cam_model: WideFisheyeCamera` with `max_incidence_deg`
(spherical patch mode only; see [camera notes](MULTI_FISHEYE.md)).
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
ROS 包名仍为 `fast_livo`。建议使用上面的 `mapping_sphere_oxford.launch`
运行当前配置，轨迹和点云保存在源码目录的 `Log/` 下。
现有实验尚未证明相对 Omni-LIVO 存在稳定的精度优势；请根据序列实测评估。
