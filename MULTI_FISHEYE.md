# Multi-fisheye and spherical patch notes

SPHERE-LIVO extends FAST-LIVO2 with separate camera models, image frames and
extrinsics for synchronized equidistant fisheye cameras. It supports raw and
JPEG-compressed ROS images. `multi_fisheye/frame_stride` samples complete rig
groups, rather than sampling each camera independently.

## Calibration

Copy `config/multi_fisheye.template.yaml` and fill in the topics, image sizes,
intrinsic/distortion parameters and extrinsics. The template is intentionally
invalid. `Rcl` is a row-major rotation and `Pcl` is a translation mapping LiDAR
coordinates into camera coordinates. These are not IMU-to-camera transforms.

The model expects a calibrated field of view below 180 degrees. Camera timestamps
must share a clock; the single global image offset cannot correct independent
camera clock offsets. Fixed camera exposure and synchronized global-shutter
images are recommended. The inherited single exposure state is disabled in
multi-fisheye mode.

```bash
roslaunch fast_livo mapping_multifisheye.launch \
  lidar_config:=/absolute/path/to/lidar_imu.yaml \
  rig_config:=/absolute/path/to/rig.yaml
```

The launch file loads the LiDAR and rig files, then the shared method
configuration `config/sphere_method.yaml`. Keep method parameters out of the rig
file; pass `ablation:=<name>` to overlay a file from `config/ablation`.

## Method parameters

`config/sphere_method.yaml` is the same for every dataset:

- `spherical_patch`: the spherical joint update (false: per-camera FAST-LIVO2).
- `shared_spherical_map`, `spherical_geometry_support`,
  `spherical_shared_brightness`, `information_selection`: one switch per
  contribution, all true for SPHERE-LIVO and used only for ablations.
- `spherical_min_zncc`, `max_active_cameras`, `total_patch_budget`,
  `spherical_map_max_points`: correspondence gate, cameras and observations per
  update, and map size.
- `vio/img_point_cov`, `vio/patch_size`, `vio/max_iterations`: photometric noise,
  patch size and iterations of the joint update.

Everything else is fixed in `include/sphere_constants.h` or derived from these
values: the Huber threshold is two photometric standard deviations, the
brightness bias prior has the photometric variance, the selection shortlist is
three candidates per budgeted observation and each camera inserts a quarter of
the budget into the map per frame.

Patch rays intersect local LiDAR planes to form surface samples. Candidate
matching and photometric gates reject unsuitable observations. The budgets limit
selected work; decoding, probing and LiDAR processing still contribute to total
runtime. For implementation details see `src/vio.cpp` and `include/spherical_*.h`.

## Operational limits

Strict timestamp grouping requires all configured cameras. A missing camera can
stall the mapping loop. Queues are bounded and can drop old images under load.
The debug image shows camera 0. The inherited COLMAP exporter assumes pinhole
images and must remain disabled for fisheye configurations. This implementation
does not add loop closure. See README.md for build, output and evaluation steps.
