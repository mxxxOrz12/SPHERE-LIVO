#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

// Twice the conventional Huber loss: its half-gradient is w(r)*r,
// matching the frontend's IRLS normal equations (not w(r)*r*r).
inline double sphericalHuberCost(double residual, double threshold)
{
  if (!std::isfinite(residual) || !std::isfinite(threshold) || threshold <= 0)
    return std::numeric_limits<double>::infinity();
  const double magnitude = std::abs(residual);
  return magnitude <= threshold ? residual * residual
                                : threshold * (2.0 * magnitude - threshold);
}

inline bool acceptSphericalDescent(double before, double after,
                                   double predicted_half_decrease, double step)
{
  if (!std::isfinite(before) || !std::isfinite(after) ||
      !std::isfinite(predicted_half_decrease) || !std::isfinite(step) ||
      before < 0 || after < 0 || predicted_half_decrease <= 0 || step <= 0 || step > 1)
    return false;
  // Standard Armijo sufficient decrease (c1 = 1e-4). The cost is twice the
  // half-objective, hence the factor two.
  constexpr double armijo = 1e-4;
  return after < before && after <= before - 2.0 * armijo * step * predicted_half_decrease;
}
