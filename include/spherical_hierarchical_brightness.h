#pragma once
#include "spherical_brightness_model.h"

struct SphericalBrightnessBlock
{
  std::vector<std::pair<double,double>> samples;
  std::vector<double> weights;
};

struct SphericalHierarchicalFit
{
  Eigen::Vector2d global = Eigen::Vector2d(1,0);
  std::vector<Eigen::Vector2d> local;
  bool valid = false;
};

// Camera-wide temporal exposure plus zero-mean, regularized local shading.
// All local 2x2 blocks are eliminated before the single global 2x2 solve.
inline SphericalHierarchicalFit fitSphericalHierarchicalBrightness(
    const std::vector<SphericalBrightnessBlock> &blocks, const Eigen::Vector2d &prior,
    double image_variance)
{
  SphericalHierarchicalFit fit;
  fit.global = prior;
  fit.local.assign(blocks.size(),Eigen::Vector2d::Zero());
  if (blocks.empty() || !prior.allFinite() || !std::isfinite(image_variance) || image_variance <= 0) return fit;
  Eigen::Matrix2d global_prior = Eigen::Matrix2d::Zero(), local_prior = Eigen::Matrix2d::Zero();
  global_prior.diagonal() << image_variance/.0225,image_variance/100;
  local_prior.diagonal() << image_variance/.01,image_variance/25;
  for (const auto &block : blocks)
  {
    if (block.samples.empty() || block.samples.size() != block.weights.size()) return fit;
    for (size_t j = 0; j < block.samples.size(); ++j)
      if (!std::isfinite(block.samples[j].first) || !std::isfinite(block.samples[j].second) ||
          !std::isfinite(block.weights[j]) || block.weights[j] < 0) return fit;
  }
  std::vector<Eigen::Matrix2d> data(blocks.size()), local_information(blocks.size());
  std::vector<Eigen::Vector2d> data_rhs(blocks.size());
  for (int iteration = 0; iteration < 8; ++iteration)
  {
    Eigen::Matrix2d information = global_prior;
    Eigen::Vector2d rhs = global_prior*prior;
    for (size_t i = 0; i < blocks.size(); ++i)
    {
      data[i].setZero(); data_rhs[i].setZero();
      const auto parameters = (fit.global+fit.local[i]).eval();
      for (size_t j = 0; j < blocks[i].samples.size(); ++j)
      {
        const auto &sample = blocks[i].samples[j];
        const double residual = parameters(0)*sample.first+parameters(1)-sample.second;
        const double weight = blocks[i].weights[j]*std::min(1.0,20.0/std::max(std::abs(residual),1e-9));
        const Eigen::Vector2d design(sample.first,1);
        data[i].noalias() += weight*design*design.transpose();
        data_rhs[i].noalias() += weight*design*sample.second;
      }
      local_information[i] = data[i]+local_prior;
      const auto factor = local_information[i].ldlt();
      information += data[i]-data[i]*factor.solve(data[i]);
      rhs += data_rhs[i]-data[i]*factor.solve(data_rhs[i]);
    }
    const auto factor = information.ldlt();
    if (factor.info() != Eigen::Success || !factor.isPositive()) return fit;
    fit.global = factor.solve(rhs);
    if (!fit.global.allFinite() || fit.global(0) < .25 || fit.global(0) > 4) return fit;
    for (size_t i = 0; i < blocks.size(); ++i)
    {
      fit.local[i] = local_information[i].ldlt().solve(data_rhs[i]-data[i]*fit.global);
      const auto effective = (fit.global+fit.local[i]).eval();
      if (!effective.allFinite() || effective(0) < .25 || effective(0) > 4) return fit;
    }
  }
  fit.valid = true;
  return fit;
}

inline double sphericalLocalBrightnessPriorCost(const Eigen::Vector2d &delta)
{
  return delta(0)*delta(0)/.01+delta(1)*delta(1)/25;
}
