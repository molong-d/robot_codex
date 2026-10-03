#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace robot_core {
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
struct Pose {
  double x{0.0}, y{0.0}, z{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};
struct PoseTolerance {
  double position_m{0.01};
  double orientation_rad{0.05};
};
inline bool valid_pose(const Pose& p) {
  for (double v : {p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw})
    if (!std::isfinite(v)) return false;
  const double n = p.qx*p.qx + p.qy*p.qy + p.qz*p.qz + p.qw*p.qw;
  return std::abs(n - 1.0) <= 1e-3;
}
// T_A_C = T_A_B * T_B_C. Quaternions use x,y,z,w; translations use metres.
inline Pose compose_pose(Pose a, Pose b) {
  if (!valid_pose(a) || !valid_pose(b)) throw std::invalid_argument("invalid rigid transform");
  const auto normalize = [](Pose& p) {
    const double n = std::sqrt(p.qx*p.qx+p.qy*p.qy+p.qz*p.qz+p.qw*p.qw);
    p.qx /= n; p.qy /= n; p.qz /= n; p.qw /= n;
  };
  normalize(a); normalize(b);
  const double tx = 2.0*(a.qy*b.z-a.qz*b.y);
  const double ty = 2.0*(a.qz*b.x-a.qx*b.z);
  const double tz = 2.0*(a.qx*b.y-a.qy*b.x);
  Pose p{a.x+b.x+a.qw*tx+a.qy*tz-a.qz*ty,
         a.y+b.y+a.qw*ty+a.qz*tx-a.qx*tz,
         a.z+b.z+a.qw*tz+a.qx*ty-a.qy*tx,
         a.qw*b.qx+a.qx*b.qw+a.qy*b.qz-a.qz*b.qy,
         a.qw*b.qy-a.qx*b.qz+a.qy*b.qw+a.qz*b.qx,
         a.qw*b.qz+a.qx*b.qy-a.qy*b.qx+a.qz*b.qw,
         a.qw*b.qw-a.qx*b.qx-a.qy*b.qy-a.qz*b.qz};
  normalize(p);
  if (!valid_pose(p)) throw std::invalid_argument("rigid transform overflow");
  return p;
}
inline bool valid_tolerance(const PoseTolerance& t) {
  return std::isfinite(t.position_m) && t.position_m > 0.0 &&
         std::isfinite(t.orientation_rad) && t.orientation_rad > 0.0 && t.orientation_rad <= 3.141593;
}
inline bool pose_near(const Pose& actual, const Pose& target, const PoseTolerance& t) {
  if (!valid_pose(actual) || !valid_pose(target) || !valid_tolerance(t)) return false;
  const double dx = actual.x-target.x, dy = actual.y-target.y, dz = actual.z-target.z;
  const double an = std::sqrt(actual.qx*actual.qx + actual.qy*actual.qy + actual.qz*actual.qz + actual.qw*actual.qw);
  const double tn = std::sqrt(target.qx*target.qx + target.qy*target.qy + target.qz*target.qz + target.qw*target.qw);
  const double dot = std::abs((actual.qx*target.qx + actual.qy*target.qy + actual.qz*target.qz + actual.qw*target.qw)/(an*tn));
  return std::sqrt(dx*dx+dy*dy+dz*dz) <= t.position_m &&
         2.0*std::acos(std::clamp(dot, 0.0, 1.0)) <= t.orientation_rad;
}
inline bool fresh(Time stamp, Time now, std::chrono::milliseconds max_age) {
  return stamp != Time{} && max_age.count() > 0 && stamp <= now && now-stamp <= max_age;
}
}  // namespace robot_core
