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

`cam_model: EquidistantCamera` uses the vikit model and expects a calibrated field
of view below 180 degrees. Camera timestamps
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

## Lenses wider than 180 degrees

Set `cam_model: WideFisheyeCamera` and keep the Kannala-Brandt (equidistant
k1..k4) calibration, for example from Kalibr `pinhole-equi`. Add
`max_incidence_deg`, the largest angle between a ray and the optical axis that
lies inside the valid image circle (half the usable field of view, e.g. 100 for
200 degrees). Choose it a few degrees inside the image circle so patch gradients
do not touch the black border.

This model (`include/wide_fisheye_camera.h`) projects with
`theta = atan2(sqrt(x^2 + y^2), z)` and back-projects to unit bearings, so rays
behind the image plane are valid. In the spherical path, visibility becomes
"range > 0.2 m and inside the incidence cone" instead of "depth z > 0.2 m", and
the projection Jacobian is taken with respect to the 3D point directly. For rays
in front of the camera it produces the same pixels as `EquidistantCamera`; with
`EquidistantCamera` the code path is unchanged.

Limits: spherical patch mode only (the legacy affine-warp path is rejected at
startup); the polynomial must be strictly increasing up to `max_incidence_deg`
(checked at startup); there is no vignetting model, which matters more at the
edge of very wide lenses. `test/test_wide_fisheye_camera.cpp` checks agreement
with the legacy model, round trips and the Jacobian up to 105 degrees.

## Method switches

- `spherical_patch`: enables the spherical visual update.
- `information_selection`: uses the weak LiDAR translation direction to score
  visual candidates; disabling it uses texture-based ranking.
- `shared_spherical_map`: allows cameras to observe common world-anchored patches.
- `joint_fixed_budget`: accumulates selected camera observations in one visual
  update, with configurable active-camera and patch limits.
- `spherical_map_max_points`, `spherical_probe_budget`, `total_patch_budget`:
  control map size, candidate probing and selected observations.
- `spherical_shortlist_per_camera`, `spherical_pool_size`: candidates kept per
  camera and in the global pool before incremental selection (default 120, 360).
- `spherical_min_patches_per_camera`: per-camera minimum granted before the
  global greedy selection (default 0).
- `spherical_chi2_gate`, `spherical_max_correction_m`,
  `spherical_max_correction_deg`: reject a visual correction that is implausible
  under the LIO prior (default 0, disabled).
- `multi_fisheye/frame_phase`: which group of every `frame_stride` groups is
  used (default 0); different phases serve as seeds.
- `local_map/half_map_size_m`: local LiDAR map half size in meters. The legacy
  `half_map_size` counts voxels (FAST-LIVO2 convention).
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
