#pragma once

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

// Shared camera/frame parameters. References are already in canonical
// intensity units. The fixed temporal prior is counted once, not per patch.
inline SphericalBrightnessFit fitSphericalBrightness(
    const std::vector<std::pair<double, double>> &samples,
    double prior_gain, double prior_bias, double image_variance,
    double gain_variance = .0225, double bias_variance = 100,
    const std::vector<double> *sample_weights = nullptr, bool bounded_gain = false)
{
  SphericalBrightnessFit result;
  result.gain = prior_gain; result.bias = prior_bias;
  if (samples.size() < 16 || !std::isfinite(prior_gain) || !std::isfinite(prior_bias) ||
      !std::isfinite(image_variance) || !std::isfinite(gain_variance) || !std::isfinite(bias_variance) ||
      image_variance <= 0 || gain_variance <= 0 || bias_variance <= 0) return result;
  if (sample_weights && (sample_weights->size() != samples.size() ||
      std::any_of(sample_weights->begin(),sample_weights->end(),[](double w){return !std::isfinite(w) || w<0;})))
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
  raw_prior(0,0) = image_variance/gain_variance;
  raw_prior(1,1) = image_variance/bias_variance;
  result.centered_prior = from_centered.transpose()*raw_prior*from_centered;
  const Eigen::Vector2d prior(prior_gain, prior_bias);
  const Eigen::Vector2d prior_rhs = from_centered.transpose()*raw_prior*prior;
  for (int iteration = 0; iteration < 6; ++iteration)
  {
    Eigen::Matrix2d information = result.centered_prior;
    Eigen::Vector2d rhs = prior_rhs;
    for (size_t i = 0; i < samples.size(); ++i)
    {
      const auto &sample = samples[i];
      const double residual = result.gain*sample.first+result.bias-sample.second;
      const double weight = (sample_weights ? (*sample_weights)[i] : 1.0) *
                            std::min(1.0,20.0/std::max(std::abs(residual),1e-9));
      const Eigen::Vector2d design(sample.first-result.mean,1);
      information.noalias() += weight*design*design.transpose();
      rhs.noalias() += weight*design*sample.second;
    }
    Eigen::LDLT<Eigen::Matrix2d> factor(information);
    if (factor.info() != Eigen::Success || !factor.isPositive()) return result;
    Eigen::Vector2d centered = factor.solve(rhs);
    if (!centered.allFinite()) return result;
    result.gain_constrained = centered(0) < .25 || centered(0) > 4;
    if (result.gain_constrained)
    {
      if (!bounded_gain) return result;
      // Exact box-constrained solution of this IRLS quadratic: hold gain
      // at the active bound and re-solve bias, rather than clamp both or
      // discard the whole camera. Centered parameter #1 is b+mean*g.
      centered(0) = std::max(.25,std::min(4.,centered(0)));
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

inline double sphericalBrightnessPriorCost(double gain, double bias, double prior_gain, double prior_bias,
                                           double gain_variance = .0225, double bias_variance = 100)
{
  return (gain-prior_gain)*(gain-prior_gain)/gain_variance +
         (bias-prior_bias)*(bias-prior_bias)/bias_variance;
}
