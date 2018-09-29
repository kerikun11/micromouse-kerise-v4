#pragma once

#include <iostream>
#include <ostream>
#include <queue>

class Logger {
public:
  Logger() {}
  void reset(int size) {
    this->size = size;
    while (!buf.empty())
      buf.pop();
  }
  void push(const float *data) {
    for (int i = 0; i < size; ++i) {
      buf.push(data[i]);
    }
  }
  void print(std::ostream &os = std::cout) {
    int length = buf.size() / size;
    for (int i = 0; i < length; ++i) {
      os << buf.front();
      buf.push(buf.front());
      buf.pop();
      for (int j = 1; j < size; ++j) {
        os << "\t";
        os << buf.front();
        buf.push(buf.front());
        buf.pop();
      }
      os << std::endl;
      vTaskDelay(0);
    }
  }

private:
  int index;
  int size;
  std::queue<float> buf;
};