#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

// A bounded number of camera quotas, not a second pass over image pixels.
// Keep a small minimum per visible camera; allocate the remainder according
// to directional information, with a square root to limit concentration.
inline std::vector<int> allocateVisualBudget(const std::vector<double> &scores,
                                           const std::vector<int> &capacities,
                                           int budget)
{
  std::vector<int> quota(capacities.size(), 0);
  if (budget <= 0 || scores.size() != capacities.size() || capacities.empty()) return quota;
  const int minimum = budget / (4 * static_cast<int>(capacities.size()));
  int remaining = budget;
  for (size_t i = 0; i < capacities.size(); ++i)
  {
    quota[i] = std::min(std::max(0, capacities[i]), minimum);
    remaining -= quota[i];
  }
  std::vector<double> weight(scores.size(), 0.0), granted(scores.size(), 0.0);
  for (size_t i = 0; i < scores.size(); ++i)
    weight[i] = std::isfinite(scores[i]) ? std::sqrt(std::max(0.0, scores[i])) : 0.0;
  // Weighted fair queuing: exactly B grants at most, with depleted cameras
  // skipped and zero-information cameras used only when no useful capacity
  // remains. Three cameras make this O(B), independent of map size.
  while (remaining-- > 0)
  {
    int best = -1;
    double best_priority = -1.0;
    double live_weight = 0.0;
    for (size_t i = 0; i < capacities.size(); ++i)
      if (quota[i] < capacities[i]) live_weight += weight[i];
    for (size_t i = 0; i < capacities.size(); ++i)
    {
      if (quota[i] >= capacities[i]) continue;
      const double priority = (live_weight > 1e-12 ? weight[i] : 1.0) / (granted[i] + 1.0);
      if (priority > best_priority) { best = static_cast<int>(i); best_priority = priority; }
    }
    if (best < 0) break;
    ++quota[best];
    granted[best] += 1.0;
  }
  return quota;
}
