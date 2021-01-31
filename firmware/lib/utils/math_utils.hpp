#pragma once

#include <cmath>

namespace math_utils {

static auto round2(auto value, auto div) {
  return std::floor((value + div / 2) / div) * div;
}
static auto saturate(auto src, auto sat) {
  return std::max(std::min(src, sat), -sat);
}
static auto sum_of_square(auto v1, auto v2) { return v1 * v1 + v2 * v2; }

} // namespace math_utils
