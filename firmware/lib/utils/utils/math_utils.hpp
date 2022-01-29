/**
 * @file math_utils.hpp
 * @brief Math Utilities
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <cmath>

namespace math_utils {

static float round2(float value, float div) {
  return std::floor((value + div / 2) / div) * div;
}
static float saturate(float src, float sat) {
  return std::max(std::min(src, sat), -sat);
}
static float sum_of_square(float v1, float v2) { //
  return v1 * v1 + v2 * v2;
}

} // namespace math_utils
