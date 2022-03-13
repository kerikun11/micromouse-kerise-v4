/**
 * @file logger.h
 * @brief Logger
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <string>
#include <vector>

class Logger {
 public:
  Logger() {}
  void clear() { buf.clear(); }
  void init(const std::vector<std::string>& labels) {
    clear();
    this->labels = labels;
  }
  void push(const std::vector<float>& data) { buf.push_back(data); }
  void print() const {
    // header
    std::printf("# KERISE v%d\n", KERISE_SELECT);
    // labels
    std::printf("# ");
    for (int i = 0; i < labels.size(); ++i) {
      std::printf("%s", labels[i].c_str());
      i < labels.size() - 1 && std::printf("\t");
    }
    std::printf("\n");
    // data
    for (const auto& data : buf) {
      for (int i = 0; i < data.size(); ++i) {
        std::printf("%.3e", (double)data[i]);  //< printf supports only double
        i < data.size() - 1 && std::printf("\t");
      }
      std::printf("\n");
      // vTaskDelay(pdMS_TO_TICKS(1));
      taskYIELD();
    }
  }

 private:
  std::vector<std::vector<float>> buf;
  std::vector<std::string> labels;
};
