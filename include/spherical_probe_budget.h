#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>

struct SphericalProbePlan
{
  std::size_t temporal_count;
  std::size_t lidar_count;
};

inline SphericalProbePlan makeSphericalProbePlan(std::size_t previous_count,
                                               std::size_t lidar_points,
                                               std::size_t budget)
{
  if (!budget) throw std::invalid_argument("Spherical probe budget must be positive");
  const std::size_t temporal = std::min(previous_count, std::max<std::size_t>(1, budget / 4));
  return {temporal, std::min(lidar_points, budget - temporal)};
}

inline std::size_t balancedSphericalProbeIndex(std::size_t ordinal, std::size_t points,
                                              std::size_t count)
{
  if (!count || count > points || ordinal >= count)
    throw std::invalid_argument("Invalid balanced probe index");
  return ordinal * points / count;
}
