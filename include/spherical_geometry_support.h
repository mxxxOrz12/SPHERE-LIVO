#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

// Use the same three-radius tangent support as the LiDAR plane association.
// A ray-plane intersection being finite does not establish surface support.
inline bool sphericalPlaneSupports(const Eigen::Vector3d &sample, const Eigen::Vector3d &center,
                                    const Eigen::Vector3d &normal, double radius)
{
  if (!sample.allFinite() || !center.allFinite() || !normal.allFinite() ||
      !std::isfinite(radius) || radius <= 0 || std::abs(normal.norm()-1) > 1e-5) return false;
  const Eigen::Vector3d offset = sample-center;
  return (offset-normal*normal.dot(offset)).norm() <= 3*radius;
}

inline bool sphericalPlaneRangeVariance(const Eigen::Vector3d &sample, const Eigen::Vector3d &center,
                                         const Eigen::Vector3d &normal, const Eigen::Vector3d &unit_ray,
                                         const Eigen::Matrix<double, 6, 6> &plane_covariance,
                                         double &variance)
{
  if (!sample.allFinite() || !center.allFinite() || !normal.allFinite() || !unit_ray.allFinite() ||
      !plane_covariance.allFinite() || std::abs(unit_ray.norm()-1) > 1e-5) return false;
  const double denominator = normal.dot(unit_ray);
  if (std::abs(denominator) < .15) return false;
  Eigen::Matrix<double, 1, 6> derivative;
  derivative << -(sample-center).transpose()/denominator, normal.transpose()/denominator;
  variance = derivative*plane_covariance*derivative.transpose();
  if (!std::isfinite(variance) || variance < -1e-10) return false;
  variance = std::max(0.0, variance);
  return true;
}
