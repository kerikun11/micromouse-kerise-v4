/**
 * @file concurrent_queue.hpp
 * @brief C++ STL thread-safe queue
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-06
 */
#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

namespace utils {

template <typename T, typename Container = std::deque<T>>
class concurrent_queue {
public:
  typedef typename std::queue<T>::size_type size_type;
  typedef typename std::queue<T>::reference reference;
  typedef typename std::queue<T>::const_reference const_reference;
  void push(T const &e) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    queue_.push(e);
    condition_variable_.notify_one();
  }
  template <typename... _Args> void emplace(_Args &&...__args) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    queue_.emplace(std::forward<_Args>(__args)...);
    condition_variable_.notify_one();
  }
  bool empty() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return queue_.empty();
  }
  void pop() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!queue_.empty())
      queue_.pop();
  }
  void front_pop(T &ret) {
    std::unique_lock<std::mutex> lock(this->mutex_);
    wait(lock);
    ret = queue_.front();
    queue_.pop();
  }
  size_type size() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return queue_.size();
  }
  reference front() {
    std::unique_lock<std::mutex> lock(this->mutex_);
    wait(lock);
    return queue_.front();
  }
  const_reference front() const {
    std::unique_lock<std::mutex> lock(this->mutex_);
    wait(lock);
    return queue_.front();
  }
  reference back() {
    std::unique_lock<std::mutex> lock(this->mutex_);
    wait(lock);
    return queue_.back();
  }
  const_reference back() const {
    std::unique_lock<std::mutex> lock(this->mutex_);
    wait(lock);
    return queue_.back();
  }

protected:
  std::queue<T, Container> queue_;
  std::mutex mutex_;
  std::condition_variable condition_variable_;

private:
  void wait(std::unique_lock<std::mutex> &lock) {
    while (queue_.empty()) {
      condition_variable_.wait(lock);
    }
  }
};

} // namespace utils
