/**
 * @file TrajectoryTracker.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 独立2輪車の線形化フィードバック軌道追従コントローラ
 * @version 0.1
 * @date 2019-03-31
 *
 * @copyright Copyright (c) 2019
 *
 */
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
                             const Position &ref_dq, const Position &ref_ddq,
                             const Position &ref_dddq) {
    /* Prepare Variable */
    const float x = est_q.x;
    const float y = est_q.y;
    const float theta = est_q.theta;
    // const float theta = ref_q.theta;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float dx = est_v.tra * cos_theta;
    const float dy = est_v.tra * sin_theta;
    const float ddx = est_a.tra * cos_theta;
    const float ddy = est_a.tra * sin_theta;
    /* Feedback Gain Design */
    const float zeta = 1.0f;
    const float omega_n = 5;
    const float kx = omega_n * omega_n;
    const float kdx = 2 * zeta * omega_n;
    const float ky = kx;
    const float kdy = kdx;
    /* Determine Reference */
    const float dddx_r = ref_dddq.x;
    const float dddy_r = ref_dddq.y;
    const float ddx_r = ref_ddq.x;
    const float ddy_r = ref_ddq.y;
    const float dx_r = ref_dq.x;
    const float dy_r = ref_dq.y;
    const float x_r = ref_q.x;
    const float y_r = ref_q.y;
    const float th_r = ref_q.theta;
    const float cos_th_r = std::cos(th_r);
    const float sin_th_r = std::sin(th_r);
    const float u1 = ddx_r + kdx * (dx_r - dx) + kx * (x_r - x);
    const float u2 = ddy_r + kdy * (dy_r - dy) + ky * (y_r - y);
    const float du1 = dddx_r + kdx * (ddx_r - ddx) + kx * (dx_r - dx);
    const float du2 = dddy_r + kdy * (ddy_r - ddy) + ky * (dy_r - dy);
    const float d_xi = u1 * cos_th_r + u2 * sin_th_r;
    /* integral the state(s) */
    xi += d_xi * Ts;
    /* determine the output signal */
    struct Result res;
    if (xi < xi_threshold) {
      res.v = ref_dq.x * cos_th_r + ref_dq.y * sin_th_r;
      res.w = 0;
      res.dv = ref_ddq.x * cos_th_r + ref_ddq.y * sin_th_r;
      res.dw = 0;
    } else {
      res.v = xi;
      res.dv = d_xi;
      res.w = (u2 * cos_th_r - u1 * sin_th_r) / xi;
      res.dw = -(2 * d_xi * res.w + du1 * sin_th_r - du2 * cos_th_r) / xi;
    }
    return res;
  }

private:
  float xi;
};

} // namespace trajectory
