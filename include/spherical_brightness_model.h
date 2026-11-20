#pragma once

#include "sphere_constants.h"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

struct SphericalBrightnessFit
{
  double gain = 1, bias = 0, mean = 0;
  Eigen::Matrix2d centered_prior = Eigen::Matrix2d::Zero();
  Eigen::Vector2d centered_prior_gradient = Eigen::Vector2d::Zero();
  bool valid = false;
  bool gain_constrained = false;
};

// Camera-wide affine brightness, current = gain*raw + bias, fitted against
// references in canonical intensity units. The temporal prior on gain uses
// sphere::kGainVariance; the bias prior has the photometric noise variance,
// so it carries unit information. It is counted once per camera and frame.
inline SphericalBrightnessFit fitSphericalBrightness(
    const std::vector<std::pair<double, double>> &samples, const std::vector<double> &sample_weights,
    double prior_gain, double prior_bias, double image_variance, double huber_threshold)
{
  SphericalBrightnessFit result;
  result.gain = prior_gain; result.bias = prior_bias;
  if (samples.empty() || sample_weights.size() != samples.size() ||
      !std::isfinite(prior_gain) || !std::isfinite(prior_bias) ||
      !std::isfinite(image_variance) || image_variance <= 0 ||
      !std::isfinite(huber_threshold) || huber_threshold <= 0) return result;
  if (std::any_of(sample_weights.begin(),sample_weights.end(),[](double w){return !std::isfinite(w) || w<0;}))
    return result;
  double mean = 0;
  for (const auto &sample : samples)
  {
    if (!std::isfinite(sample.first) || !std::isfinite(sample.second)) return result;
    mean += sample.first;
  }
  result.mean = mean/samples.size();
  Eigen::Matrix2d from_centered;
  from_centered << 1,0,-result.mean,1;
  Eigen::Matrix2d raw_prior = Eigen::Matrix2d::Zero();
  raw_prior(0,0) = image_variance/sphere::kGainVariance;
  raw_prior(1,1) = 1.0;
  result.centered_prior = from_centered.transpose()*raw_prior*from_centered;
  const Eigen::Vector2d prior(prior_gain, prior_bias);
  const Eigen::Vector2d prior_rhs = from_centered.transpose()*raw_prior*prior;
  const double min_gain = 1.0/sphere::kMaxGainRatio, max_gain = sphere::kMaxGainRatio;
  for (int iteration = 0; iteration < sphere::kBrightnessIterations; ++iteration)
  {
    Eigen::Matrix2d information = result.centered_prior;
    Eigen::Vector2d rhs = prior_rhs;
    for (size_t i = 0; i < samples.size(); ++i)
    {
      const auto &sample = samples[i];
      const double residual = result.gain*sample.first+result.bias-sample.second;
      const double weight = sample_weights[i] *
                            std::min(1.0,huber_threshold/std::max(std::abs(residual),1e-9));
      const Eigen::Vector2d design(sample.first-result.mean,1);
      information.noalias() += weight*design*design.transpose();
      rhs.noalias() += weight*design*sample.second;
    }
    Eigen::LDLT<Eigen::Matrix2d> factor(information);
    if (factor.info() != Eigen::Success || !factor.isPositive()) return result;
    Eigen::Vector2d centered = factor.solve(rhs);
    if (!centered.allFinite()) return result;
    result.gain_constrained = centered(0) < min_gain || centered(0) > max_gain;
    if (result.gain_constrained)
    {
      // Exact box-constrained solution of this IRLS quadratic: hold gain
      // at the active bound and re-solve bias, rather than clamp both or
      // discard the whole camera. Centered parameter #1 is b+mean*g.
      centered(0) = std::max(min_gain,std::min(max_gain,centered(0)));
      centered(1) = (rhs(1)-information(1,0)*centered(0))/information(1,1);
    }
    const Eigen::Vector2d raw = from_centered*centered;
    if (!raw.allFinite()) return result;
    result.gain = raw(0); result.bias = raw(1);
  }
  const Eigen::Vector2d fitted(result.gain,result.bias);
  result.centered_prior_gradient = from_centered.transpose()*raw_prior*(fitted-prior);
  result.valid = true;
  return result;
}

// Temporal prior cost in units of the photometric noise variance.
inline double sphericalBrightnessPriorCost(double gain, double bias, double prior_gain, double prior_bias,
                                           double image_variance)
{
  return (gain-prior_gain)*(gain-prior_gain)/sphere::kGainVariance +
         (bias-prior_bias)*(bias-prior_bias)/image_variance;
}
