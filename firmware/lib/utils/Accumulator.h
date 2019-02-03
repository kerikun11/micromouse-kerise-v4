/**
 * @file Accumulator.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief リングバッファにより一定数のデータを蓄積するクラスを定義
 * @version 0.1
 * @date 2019-02-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#pragma once

#include <new>

template <typename T, size_t _size> class Accumulator {
public:
  Accumulator(const T &value = T()) {
    buffer = new T[_size];
    head = 0;
    clear(value);
  }
  ~Accumulator() { delete buffer; }
  void clear(const T &value = T()) {
    for (int i = 0; i < _size; i++)
      buffer[i] = value;
  }
  void push(const T &value) {
    head = (head + 1) % _size;
    buffer[head] = value;
  }
  const T &operator[](const size_t index) const {
    return buffer[((int)_size + head - index) % _size];
  }
  const T average(const int num = _size) const {
    T sum = T();
    for (int i = 0; i < num; i++) {
      sum += buffer[((int)_size + head - i) % _size];
    }
    return sum / num;
  }
  size_t size() const { return _size; }

private:
  T *buffer;
  size_t head;
};
