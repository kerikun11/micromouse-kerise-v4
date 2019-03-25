#pragma once

#include "Position.h"

namespace trajectory {
using namespace coordinate;

class TrajectoryTracker {
public:
  struct Result {
    float v;
    float w;
    float dv;
    float dw;
  };
  constexpr static const float Ts = 0.001f;
  constexpr static const float xi_threshold = 180.0f;

public:
  TrajectoryTracker(const float vs = 0) { reset(vs); }
  void reset(const float vs = 0) { xi = vs; }
  const struct Result update(const Polar &est_v, const Polar &est_a,
                             const Position &est_q, const Position &ref_q,
                             const Position &ref_dq, const Position &ref_ddq) {
    const float x = est_q.x;
    const float y = est_q.y;
    const float theta = est_q.theta;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float dx = est_v.tra * cos_theta;
    const float dy = est_v.tra * sin_theta;
    const float ddx = est_a.tra * cos_theta;
    const float ddy = est_a.tra * sin_theta;
    const float zeta = 1.0f;
    const float omega_n = 1;
    const float kx = omega_n * omega_n;
    const float kdx = 2 * zeta * omega_n;
    const float ky = kx;
    const float kdy = kdx;
    const float dddx_r = 0; //< assume this
    const float dddy_r = 0; //< assume this
    const float ddx_r = ref_ddq.x;
    const float ddy_r = ref_ddq.y;
    const float dx_r = ref_dq.x;
    const float dy_r = ref_dq.y;
    const float x_r = ref_q.x;
    const float y_r = ref_q.y;
    const float u1 = ddx_r + kx * (x_r - x) + kdx * (dx_r - dx);
    const float u2 = ddy_r + ky * (y_r - y) + kdy * (dy_r - dy);
    const float du1 = dddx_r + kdx * (ddx_r - ddx) + kx * (dx_r - dx);
    const float du2 = dddy_r + kdy * (ddy_r - ddy) + ky * (dy_r - dy);
    const float d_xi = u1 * cos_theta + u2 * sin_theta;
    /* integral the state(s) */
    xi += d_xi * Ts;
    /* determine the output signal */
    struct Result res;
    if (xi < xi_threshold) {
      res.v = ref_dq.getNorm();
      res.w = 0;
      res.dv = ref_ddq.getNorm();
      res.dw = 0;
    } else {
      res.v = xi;
      res.dv = d_xi;
      res.w = (u2 * cos_theta - u1 * sin_theta) / xi;
      res.dw = -(4 * d_xi * res.w + du1 * sin_theta - du2 * cos_theta) / xi;
    }
    return res;
  }

private:
  float xi;
};

} // namespace trajectory
