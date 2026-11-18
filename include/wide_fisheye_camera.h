#pragma once

// Kannala-Brandt (equidistant k1..k4) fisheye model expressed on unit bearings.
//
// vk::EquidistantCamera projects through the normalized plane (x/z, y/z) and
// uses theta = atan(r), so it cannot represent rays at or beyond 90 degrees
// from the optical axis. This model uses theta = atan2(sqrt(x^2+y^2), z) and
// returns true unit bearings from cam2world, so lenses wider than 180 degrees
// are supported up to the calibrated maximum incidence angle. For rays in
// front of the camera it gives the same pixels as vk::EquidistantCamera.

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vikit/abstract_camera.h>

namespace sphere
{

class WideFisheyeCamera : public vk::AbstractCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Intrinsics are given at the calibrated resolution and multiplied by scale,
  // matching vk::EquidistantCamera. max_incidence_deg is the largest angle
  // between a ray and the optical axis that maps inside the valid image circle.
  WideFisheyeCamera(int width, int height, double scale, double fx, double fy, double cx, double cy,
                    double k1, double k2, double k3, double k4, double max_incidence_deg)
    : vk::AbstractCamera(static_cast<int>(width * scale), static_cast<int>(height * scale), scale),
      fx_(fx * scale), fy_(fy * scale), cx_(cx * scale), cy_(cy * scale),
      k1_(k1), k2_(k2), k3_(k3), k4_(k4),
      max_incidence_(max_incidence_deg * M_PI / 180.0), cos_max_incidence_(std::cos(max_incidence_))
  {
    valid_ = std::isfinite(fx_) && std::isfinite(fy_) && fx_ > 0 && fy_ > 0 &&
             std::isfinite(max_incidence_) && max_incidence_ > 0 && max_incidence_ < M_PI;
    // The polynomial must be strictly increasing on the calibrated range,
    // otherwise two incidence angles map to the same image radius.
    const int samples = 2000;
    for (int i = 0; valid_ && i <= samples; ++i)
      if (!(dThetadDTheta(max_incidence_ * i / samples) > 1e-6)) valid_ = false;
  }

  bool valid() const { return valid_; }
  double maxIncidence() const { return max_incidence_; }
  double cosMaxIncidence() const { return cos_max_incidence_; }

  double thetad(double theta) const
  {
    const double t2 = theta * theta;
    return theta * (1.0 + t2 * (k1_ + t2 * (k2_ + t2 * (k3_ + t2 * k4_))));
  }

  double dThetadDTheta(double theta) const
  {
    const double t2 = theta * theta;
    return 1.0 + t2 * (3 * k1_ + t2 * (5 * k2_ + t2 * (7 * k3_ + t2 * 9 * k4_)));
  }

  // Incidence angle of a camera-frame point, valid for any direction.
  static double incidence(const Eigen::Vector3d &p)
  {
    return std::atan2(std::hypot(p.x(), p.y()), p.z());
  }

  // True when p is at least min_range away and inside the calibrated cone.
  bool inCone(const Eigen::Vector3d &p, double min_range) const
  {
    const double range = p.norm();
    return std::isfinite(range) && range > min_range && p.z() >= cos_max_incidence_ * range;
  }

  Eigen::Vector2d world2cam(const Eigen::Vector3d &p) const override
  {
    const double r = std::hypot(p.x(), p.y());
    if (r < 1e-12 * std::max(1.0, std::abs(p.z())))
    {
      if (p.z() > 0) return Eigen::Vector2d(cx_ + fx_ * p.x() / p.z(), cy_ + fy_ * p.y() / p.z());
      return Eigen::Vector2d(cx_, cy_); // Undefined azimuth exactly behind the camera.
    }
    const double s = thetad(std::atan2(r, p.z())) / r;
    return Eigen::Vector2d(cx_ + fx_ * s * p.x(), cy_ + fy_ * s * p.y());
  }

  // Normalized-plane input (x/z, y/z) implies a point in front of the camera.
  Eigen::Vector2d world2cam(const Eigen::Vector2d &uv) const override
  {
    return world2cam(Eigen::Vector3d(uv.x(), uv.y(), 1.0));
  }

  // Returns a unit bearing, which may point behind the image plane (z < 0).
  Eigen::Vector3d cam2world(const double &u, const double &v) const override
  {
    const double mx = (u - cx_) / fx_, my = (v - cy_) / fy_;
    const double td = std::hypot(mx, my);
    if (td < 1e-12) return Eigen::Vector3d::UnitZ();
    double theta = std::min(td, max_incidence_);
    for (int i = 0; i < 20; ++i)
    {
      const double derivative = dThetadDTheta(theta);
      if (!(std::abs(derivative) > 1e-9)) break;
      const double step = (thetad(theta) - td) / derivative;
      theta = std::max(0.0, std::min(M_PI, theta - step));
      if (std::abs(step) < 1e-13) break;
    }
    const double sin_theta = std::sin(theta);
    return Eigen::Vector3d(sin_theta * mx / td, sin_theta * my / td, std::cos(theta));
  }

  Eigen::Vector3d cam2world(const Eigen::Vector2d &px) const override { return cam2world(px.x(), px.y()); }

  // d(pixel)/d(point) in the camera frame, without a 1/z singularity at 90 deg.
  Eigen::Matrix<double, 2, 3> jacobianPoint(const Eigen::Vector3d &p) const
  {
    Eigen::Matrix<double, 2, 3> J;
    const double x = p.x(), y = p.y(), z = p.z();
    const double r2 = x * x + y * y, r = std::sqrt(r2);
    const double rho2 = r2 + z * z;
    if (r < 1e-9 * std::sqrt(std::max(rho2, 1e-300)))
    {
      // On the optical axis g(theta) ~ theta, i.e. the pinhole limit.
      const double z_inv = 1.0 / z;
      J << fx_ * z_inv, 0, -fx_ * x * z_inv * z_inv,
           0, fy_ * z_inv, -fy_ * y * z_inv * z_inv;
      return J;
    }
    const double theta = std::atan2(r, z);
    const double g = thetad(theta), dg = dThetadDTheta(theta);
    const Eigen::Vector3d dtheta(z * x / (r * rho2), z * y / (r * rho2), -r / rho2);
    const Eigen::Vector3d dr(x / r, y / r, 0);
    const double s = g / r;
    const Eigen::Vector3d ds = (dg * dtheta * r - g * dr) / r2;
    J.row(0) = fx_ * (x * ds.transpose() + s * Eigen::RowVector3d(1, 0, 0));
    J.row(1) = fy_ * (y * ds.transpose() + s * Eigen::RowVector3d(0, 1, 0));
    return J;
  }

  double errorMultiplier2() const override { return std::abs(fx_); }
  double errorMultiplier() const override { return std::abs(4.0 * fx_ * fy_); }
  double fx() const override { return fx_; }
  double fy() const override { return fy_; }
  double cx() const override { return cx_; }
  double cy() const override { return cy_; }

private:
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_, k4_;
  double max_incidence_, cos_max_incidence_;
  bool valid_ = false;
};

} // namespace sphere
