#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct SphericalInformationCandidate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 6, 6> whitened = Eigen::Matrix<double, 6, 6>::Zero();
  size_t camera = 0;
  std::uintptr_t world_patch = 0;
  Eigen::Matrix<double, 6, 2> brightness_coupling = Eigen::Matrix<double, 6, 2>::Zero();
  Eigen::Matrix2d brightness_information = Eigen::Matrix2d::Zero();
};

// Caller caps the shortlist independently of map size. This is greedy
// complementary information, not an exact globally optimal subset search.
// Select at most one camera observation per shared reference patch.
// min_per_camera > 0 first grants each camera (up to camera_limit cameras,
// in order of first appearance) that many picks by the same criterion
// restricted to its own candidates; min_per_camera == 0 is the original rule.
inline std::vector<size_t> selectSphericalInformation(
    const std::vector<SphericalInformationCandidate> &candidates, size_t budget, size_t camera_limit,
    size_t min_per_camera = 0)
{
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  std::vector<size_t> selected;
  if (!budget || !camera_limit || candidates.empty()) return selected;
  selected.reserve(std::min(budget, candidates.size()));
  std::vector<Matrix6> information(candidates.size());
  std::vector<Eigen::Matrix<double, 6, 2>> coupling(candidates.size());
  std::vector<Eigen::Matrix2d> brightness(candidates.size());
  std::vector<bool> available(candidates.size(), false);
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    const auto &raw = candidates[i].whitened;
    if (!raw.allFinite() || !candidates[i].brightness_coupling.allFinite() ||
        !candidates[i].brightness_information.allFinite()) continue;
    Eigen::Matrix<double, 8, 8> joint = Eigen::Matrix<double, 8, 8>::Zero();
    joint.topLeftCorner<6,6>() = raw;
    joint.topRightCorner<6,2>() = candidates[i].brightness_coupling;
    joint.bottomLeftCorner<2,6>() = candidates[i].brightness_coupling.transpose();
    joint.bottomRightCorner<2,2>() = candidates[i].brightness_information;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,8,8>> factor(.5*(joint+joint.transpose()).eval());
    if (factor.info() != Eigen::Success || factor.eigenvalues().minCoeff() < -1e-8) continue;
    joint = factor.eigenvectors()*factor.eigenvalues().cwiseMax(0).asDiagonal()*factor.eigenvectors().transpose();
    information[i] = joint.topLeftCorner<6,6>();
    coupling[i] = joint.topRightCorner<6,2>();
    brightness[i] = joint.bottomRightCorner<2,2>();
    available[i] = true;
  }
  Matrix6 accumulated = Matrix6::Identity();
  std::unordered_set<std::uintptr_t> patches;
  std::unordered_set<size_t> cameras;
  double current_logdet = 0;
  struct BrightnessState
  {
    Eigen::Matrix<double,6,2> cross = Eigen::Matrix<double,6,2>::Zero();
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  };
  std::unordered_map<size_t,BrightnessState> radiometry;
  auto trial_information = [&](size_t i)
  {
    Matrix6 trial = accumulated+information[i];
    if (coupling[i].squaredNorm() == 0 && brightness[i].squaredNorm() == 0) return trial;
    const auto found = radiometry.find(candidates[i].camera);
    BrightnessState old;
    if (found != radiometry.end()) old = found->second;
    const auto combined_cross = (old.cross+coupling[i]).eval();
    const auto combined_information = (old.information+brightness[i]).eval();
    trial.noalias() += old.cross*old.information.ldlt().solve(old.cross.transpose());
    trial.noalias() -= combined_cross*combined_information.ldlt().solve(combined_cross.transpose());
    return (.5*(trial+trial.transpose())).eval();
  };
  // Returns false when no candidate (optionally of only_camera) adds information.
  auto pick = [&](bool restrict_camera, size_t only_camera)
  {
    size_t best = candidates.size();
    double best_gain = 1e-12, best_logdet = 0;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
      const auto &candidate = candidates[i];
      if (restrict_camera && candidate.camera != only_camera) continue;
      if (!available[i] || patches.count(candidate.world_patch) ||
          (cameras.size() >= camera_limit && !cameras.count(candidate.camera))) continue;
      Eigen::LLT<Matrix6> factor(trial_information(i));
      if (factor.info() != Eigen::Success) continue;
      const Matrix6 lower = factor.matrixL();
      const double logdet = 2*lower.diagonal().array().log().sum();
      const double gain = logdet - current_logdet;
      if (std::isfinite(gain) && gain > best_gain)
      { best = i; best_gain = gain; best_logdet = logdet; }
    }
    if (best == candidates.size()) return false;
    accumulated = trial_information(best);
    auto &camera_brightness = radiometry[candidates[best].camera];
    camera_brightness.cross += coupling[best];
    camera_brightness.information += brightness[best];
    current_logdet = best_logdet;
    selected.push_back(best);
    available[best] = false;
    cameras.insert(candidates[best].camera);
    patches.insert(candidates[best].world_patch);
    return true;
  };
  if (min_per_camera > 0)
  {
    std::vector<size_t> order;
    for (const auto &candidate : candidates)
      if (std::find(order.begin(), order.end(), candidate.camera) == order.end()) order.push_back(candidate.camera);
    for (size_t camera : order)
    {
      if (cameras.size() >= camera_limit && !cameras.count(camera)) continue;
      for (size_t k = 0; k < min_per_camera && selected.size() < budget; ++k)
        if (!pick(true, camera)) break;
    }
  }
  while (selected.size() < budget)
    if (!pick(false, 0)) break;
  return selected;
}
