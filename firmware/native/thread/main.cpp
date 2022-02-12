#include "log.h"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

int main(int argc, char const *argv[]) {
  LOGI("Build: %s %s", __DATE__, __TIME__);

  std::mutex mtx;
  std::condition_variable cv;
  int data_index = 0;

  std::thread thread_sampling([&] {
    //
    while (1) {
      LOGI("sampling wait");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      LOGI("sampling start");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      LOGI("sampling end");
      {
        std::lock_guard<std::mutex> lock(mtx);
        data_index++;
        cv.notify_all();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  std::thread thread_control1([&] {
    int got_index = 0;
    while (1) {
      std::unique_lock<std::mutex> uniq_lk(mtx);
      cv.wait(uniq_lk, [&] { return data_index >= got_index; });
      LOGI("control 1");
      got_index++;
    }
  });
  std::thread thread_control2([&] {
    int got_index = 0;
    while (1) {
      std::unique_lock<std::mutex> uniq_lk(mtx);
      cv.wait(uniq_lk, [&] { return data_index >= got_index; });
      LOGI("control 2");
      got_index++;
    }
  });

  thread_sampling.join();
  thread_control1.join();
  thread_control2.join();

  return 0;
}
