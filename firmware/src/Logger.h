#pragma once

#include <iostream>
#include <ostream>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Logger {
public:
  Logger() {}
  auto clear() { return buf.clear(); }
  auto push(const std::vector<float> &data) { return buf.push_back(data); }
  auto size() const { return buf.size(); }
  void print(std::ostream &os = std::cout) const {
    for (const auto data : buf) {
      bool first = true;
      for (const auto value : data) {
        if (!first)
          os << "\t";
        if (first)
          first = false;
        os << value;
      }
      taskYIELD();
      os << std::endl;
    }
  }
  friend std::ostream &operator<<(std::ostream &os, const Logger &obj) {
    obj.print(os);
    return os;
  }

private:
  std::vector<std::vector<float>> buf;
};
