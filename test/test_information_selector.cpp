// Checks for include/spherical_information_selector.h (Eigen only).
// Built by catkin when testing is enabled, or directly (one line):
//   g++ -std=c++14 -O2 -I include -I /usr/include/eigen3 test/test_information_selector.cpp
#include "spherical_information_selector.h"

#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>

int main()
{
  std::mt19937 rng(7);
  std::normal_distribution<double> normal(0, 1);
  int failures = 0;
  for (int trial = 0; trial < 200; ++trial)
  {
    const int count = 30 + static_cast<int>(rng() % 120);
    std::vector<SphericalInformationCandidate> candidates(count);
    for (int i = 0; i < count; ++i)
    {
      Eigen::Matrix<double, 8, 3> factor;
      for (int r = 0; r < 8; ++r)
        for (int k = 0; k < 3; ++k) factor(r, k) = normal(rng) * (i % 3 == 0 ? 3 : 1);
      const Eigen::Matrix<double, 8, 8> joint = 0.1 * factor * factor.transpose();
      candidates[i].whitened = joint.topLeftCorner<6, 6>();
      candidates[i].brightness_coupling = joint.topRightCorner<6, 2>();
      candidates[i].brightness_information = joint.bottomRightCorner<2, 2>();
      candidates[i].camera = (i * 7 + trial) % 3;
      candidates[i].world_patch = rng() % (count * 2) + 1;
    }
    const size_t budget = 10 + rng() % 60, minimum = 8;
    for (size_t camera_minimum : {size_t(0), minimum})
    {
      const auto selected = selectSphericalInformation(candidates, budget, 3, camera_minimum);
      std::set<size_t> indices(selected.begin(), selected.end()), patches;
      for (size_t i : selected) patches.insert(candidates[i].world_patch);
      if (selected.size() > budget || indices.size() != selected.size() || patches.size() != selected.size())
      { ++failures; std::printf("FAIL: budget or one-observation-per-patch violated\n"); }
      if (!camera_minimum || selected.size() == budget) continue;
      // With budget left, each camera reaches the minimum unless its remaining
      // candidates only observe patches already taken by another camera.
      for (size_t camera = 0; camera < 3; ++camera)
      {
        std::set<size_t> taken, own;
        size_t picked = 0;
        for (size_t i : selected)
          if (candidates[i].camera == camera) ++picked; else taken.insert(candidates[i].world_patch);
        for (const auto &candidate : candidates)
          if (candidate.camera == camera && !taken.count(candidate.world_patch)) own.insert(candidate.world_patch);
        if (picked < std::min(minimum, own.size()))
        { ++failures; std::printf("FAIL: camera %zu got %zu < minimum\n", camera, picked); }
      }
    }
  }
  if (failures) { std::printf("%d check(s) failed\n", failures); return EXIT_FAILURE; }
  std::printf("all information selector checks passed\n");
  return EXIT_SUCCESS;
}
