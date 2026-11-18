// Standalone checks for include/wide_fisheye_camera.h (Eigen + vikit headers only).
// Built by catkin when testing is enabled, or directly (one line):
//   g++ -std=c++14 -O2 -I include -I <rpg_vikit>/vikit_common/include -I /usr/include/eigen3
//       test/test_wide_fisheye_camera.cpp -o test_wide_fisheye_camera
#include "wide_fisheye_camera.h"

#include <cstdio>
#include <cstdlib>

namespace
{
int failures = 0;

void check(bool condition, const char *what, double value = 0)
{
  if (!condition)
  {
    ++failures;
    std::printf("FAIL: %s (%.3e)\n", what, value);
  }
}

// vk::EquidistantCamera::world2cam(Vector3d), copied so the test needs no OpenCV.
Eigen::Vector2d legacyWorld2cam(const Eigen::Vector3d &p, double fx, double fy, double cx, double cy,
                                double k1, double k2, double k3, double k4)
{
  const Eigen::Vector2d uv(p.x() / p.z(), p.y() / p.z());
  const double r = uv.norm();
  if (r < 1e-8) return uv; // Matches the legacy early return (normalized coordinates).
  const double theta = std::atan(r), t2 = theta * theta, t4 = t2 * t2;
  const double thetad = theta * (1 + k1 * t2 + k2 * t4 + k3 * t4 * t2 + k4 * t4 * t4);
  const double scaling = thetad / r;
  return Eigen::Vector2d(uv.x() * scaling * fx + cx, uv.y() * scaling * fy + cy);
}

Eigen::Vector3d ray(double incidence_deg, double azimuth_deg, double range)
{
  const double t = incidence_deg * M_PI / 180, a = azimuth_deg * M_PI / 180;
  return range * Eigen::Vector3d(std::sin(t) * std::cos(a), std::sin(t) * std::sin(a), std::cos(t));
}
} // namespace

int main()
{
  // 1. Same pixels as the legacy model in front of the camera (Hilti cam0, scale 0.5).
  const double s = 0.5, fx = 351.314, fy = 351.491, cx = 367.852, cy = 253.840;
  const double k1 = -0.036967, k2 = -0.008918, k3 = 0.008913, k4 = -0.003769;
  sphere::WideFisheyeCamera hilti(720, 540, s, fx, fy, cx, cy, k1, k2, k3, k4, 80);
  check(hilti.valid(), "Hilti calibration accepted");
  check(hilti.width() == 360 && hilti.height() == 270, "scaled image size");
  double legacy_error = 0;
  for (double t = 0.5; t < 85; t += 2.5)
    for (double a = 0; a < 360; a += 17)
    {
      const Eigen::Vector3d p = ray(t, a, 3.7);
      const Eigen::Vector2d legacy = legacyWorld2cam(p, fx * s, fy * s, cx * s, cy * s, k1, k2, k3, k4);
      legacy_error = std::max(legacy_error, (legacy - hilti.world2cam(p)).norm());
    }
  check(legacy_error < 1e-7, "matches vk::EquidistantCamera for incidence < 85 deg", legacy_error);

  // 2. A 210-degree lens: projection, back-projection and Jacobian beyond 90 deg.
  sphere::WideFisheyeCamera wide(1280, 1280, 1.0, 380, 381, 640.5, 639.5, 0.01, -0.004, 0.0008, -0.00005, 105);
  check(wide.valid(), "210-degree calibration accepted");
  double roundtrip_error = 0, jacobian_error = 0;
  for (double t = 0; t <= 105; t += 1.5)
    for (double a = 0; a < 360; a += 23)
    {
      const Eigen::Vector3d p = ray(t, a, 2.3);
      const Eigen::Vector2d px = wide.world2cam(p);
      check(px.allFinite(), "finite projection");
      const Eigen::Vector3d bearing = wide.cam2world(px);
      check(std::abs(bearing.norm() - 1) < 1e-12, "unit bearing");
      if (t > 0) roundtrip_error = std::max(roundtrip_error, (bearing - p.normalized()).norm());
      const Eigen::Matrix<double, 2, 3> J = wide.jacobianPoint(p);
      Eigen::Matrix<double, 2, 3> numeric;
      for (int k = 0; k < 3; ++k)
      {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        d[k] = 1e-6;
        numeric.col(k) = (wide.world2cam((p + d).eval()) - wide.world2cam((p - d).eval())) / 2e-6;
      }
      jacobian_error = std::max(jacobian_error, (J - numeric).norm() / std::max(1.0, numeric.norm()));
    }
  check(roundtrip_error < 1e-9, "world2cam/cam2world round trip up to 105 deg", roundtrip_error);
  check(jacobian_error < 1e-5, "analytic Jacobian matches finite differences", jacobian_error);

  // Rays behind the image plane land on the far side of the 90-degree circle.
  const double r90 = wide.thetad(M_PI / 2) * 380, r100 = (wide.world2cam(ray(100, 0, 1)) - Eigen::Vector2d(640.5, 639.5)).norm();
  check(r100 > r90, "100 deg projects outside the 90 deg circle", r100 - r90);

  // 3. Visibility cone and range.
  check(wide.inCone(ray(104, 40, 1.0), 0.2), "inside 105 deg cone");
  check(!wide.inCone(ray(106, 40, 1.0), 0.2), "outside 105 deg cone");
  check(!wide.inCone(ray(10, 40, 0.1), 0.2), "closer than minimum range");

  // 4. A polynomial that folds back inside the claimed range is rejected.
  sphere::WideFisheyeCamera folded(1280, 1280, 1.0, 380, 380, 640, 640, -0.3, 0, 0, 0, 100);
  check(!folded.valid(), "non-monotonic polynomial rejected");

  if (failures) { std::printf("%d check(s) failed\n", failures); return EXIT_FAILURE; }
  std::printf("all wide fisheye camera checks passed (legacy %.1e, roundtrip %.1e, jacobian %.1e)\n",
              legacy_error, roundtrip_error, jacobian_error);
  return EXIT_SUCCESS;
}
