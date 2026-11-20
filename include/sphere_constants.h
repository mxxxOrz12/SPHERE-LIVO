#pragma once

// Every fixed SPHERE-LIVO constant is defined here. Quantities that follow
// from configured parameters are derived where they are used instead:
//   photometric Huber threshold  = 2 * sqrt(vio/img_point_cov)
//   brightness bias prior variance = vio/img_point_cov
//   map-insertion texture threshold = vio/img_point_cov (squared gradient)
//   selection shortlist = 3 * total_patch_budget, map insertions = budget / 4
//   update iterations = vio/max_iterations
namespace sphere
{
// Geometric validity of a patch sample.
constexpr double kMinDepth = 0.2;           // m in front of the camera
constexpr double kMinRayPlaneCosine = 0.15; // reject grazing ray-plane intersections
constexpr double kMapVoxelSize = 0.5;       // m, one shared patch per voxel

// Camera brightness prior: frame-to-frame gain changes of about 15 %, and a
// gain that stays within a factor of four of the canonical intensity scale.
constexpr double kGainStd = 0.15;
constexpr double kGainVariance = kGainStd * kGainStd;
constexpr double kMaxGainRatio = 4.0;
constexpr int kBrightnessIterations = 6; // IRLS iterations of the brightness fit

// Guarded visual update.
constexpr double kMaxStepRotation = 0.03;    // rad per iteration
constexpr double kMaxStepTranslation = 0.08; // m per iteration
constexpr int kMaxBacktracks = 6;            // step halvings before rejecting a step
constexpr int kMinSupportPatches = 5;

// LiDAR points probed per camera and frame for map lookup and insertion.
constexpr int kProbeBudget = 2048;
} // namespace sphere
