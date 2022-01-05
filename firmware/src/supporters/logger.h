/**
 * @file logger.h
 * @brief Logger
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

class Logger {
public:
  Logger() {}
  auto clear() { return buf.clear(); }
  void init(const std::vector<std::string> &labels) {
    clear();
    this->labels = labels;
  }
  void push(const std::vector<float> &data) { buf.push_back(data); }
  void print(std::ostream &os = std::cout) const {
    // header
    os << "# KERISE v" << KERISE_SELECT << std::endl;
    // labels
    os << "# ";
    for (int i = 0; i < labels.size(); ++i)
      os << labels[i] << (i < labels.size() - 1 ? "\t" : "");
    os << std::endl;
    // data
    for (const auto &data : buf) {
      for (int i = 0; i < data.size(); ++i)
        os << data[i] << (i < data.size() - 1 ? "\t" : "");
      os << std::endl;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

private:
  std::vector<std::vector<float>> buf;
  std::vector<std::string> labels;
};
