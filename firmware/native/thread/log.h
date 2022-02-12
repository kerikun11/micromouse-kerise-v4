#pragma once
#include <cstdio>
#define __STRINGIFY__(n) #n
#define __TO_STR__(n) __STRINGIFY__(n)
#define LOG_COMMON(l, c, f, ...)                                               \
  std::printf(c "[" l "][" __FILE__ ":" __TO_STR__(__LINE__) "]\x1b[0m\t" f    \
                                                             "\n",             \
              ##__VA_ARGS__)
#define LOGD(fmt, ...) LOG_COMMON("D", "\x1b[34m", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_COMMON("I", "\x1b[32m", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_COMMON("W", "\x1b[33m", fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) LOG_COMMON("E", "\x1b[31m", fmt, ##__VA_ARGS__)
