/**
 * @file logger.h
 * @brief Logger
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include <vector>

class Logger {
public:
  Logger() {}
  auto clear() { return buf.clear(); }
  auto push(const std::vector<float> &data) { return buf.push_back(data); }
  auto size() const { return buf.size(); }
  auto reserve(std::size_t n) { return buf.reserve(n); }
  void print(std::ostream &os = std::cout) const {
    os << "# KERISE v" << KERISE_SELECT << std::endl;
    for (const auto &data : buf) {
      bool first = true;
      for (const auto &value : data) {
        if (!first)
          os << "\t";
        if (first)
          first = false;
        os << value;
      }
      os << std::endl;
      vPortYield();
    }
  }

private:
  std::vector<std::vector<float>> buf;
};
