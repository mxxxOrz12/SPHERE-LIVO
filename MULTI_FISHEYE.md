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

This generic launch loads the supplied configuration. Enable
`innovations/spherical_patch` explicitly for SPHERE-LIVO. For Oxford Spires,
`mapping_sphere_oxford.launch` loads the complete current preset directly.

## Method switches

- `spherical_patch`: enables the spherical visual update.
- `information_selection`: uses the weak LiDAR translation direction to score
  visual candidates; disabling it uses texture-based ranking.
- `shared_spherical_map`: allows cameras to observe common world-anchored patches.
- `joint_fixed_budget`: accumulates selected camera observations in one visual
  update, with configurable active-camera and patch limits.
- `spherical_map_max_points`, `spherical_probe_budget`, `total_patch_budget`:
  control map size, candidate probing and selected observations.
- Brightness, geometry-support and update-guard options are set explicitly in
  `config/sphere_oxford.yaml`; other combinations are experimental ablations.

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
