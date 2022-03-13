/**
 * @file app_log.h
 * @brief this provides logging formats
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-12-18
 */
#pragma once

#include <iostream>

#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)
#define APP_LOG_OSTREAM_COMMON(c) \
  (std::cout << "[" c "][" __FILE__ ":" APP_TOSTRING(__LINE__) "] ")

#ifndef app_logd
#if 0
#define app_logd APP_LOG_OSTREAM_COMMON("D")
#else
#define app_logd std::ostream(0)
#endif
#endif

#ifndef app_logi
#if 1
#define app_logi APP_LOG_OSTREAM_COMMON("I")
#else
#define app_logi std::ostream(0)
#endif
#endif

#ifndef app_logw
#if 1
#define app_logw APP_LOG_OSTREAM_COMMON("W")
#else
#define app_logw std::ostream(0)
#endif
#endif

#ifndef app_loge
#if 1
#define app_loge APP_LOG_OSTREAM_COMMON("E")
#else
#define app_loge std::ostream(0)
#endif
#endif

#include <cstdio>
#define __STRINGIFY__(n) #n
#define __TO_STR__(n) __STRINGIFY__(n)
#define LOG_COMMON(l, c, f, ...)                                            \
  std::printf(c "[" l "][" __FILE__ ":" __TO_STR__(__LINE__) "]\x1b[0m\t" f \
                                                             "\n",          \
              ##__VA_ARGS__)
#define LOGD(fmt, ...) LOG_COMMON("D", "\x1b[34m", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_COMMON("I", "\x1b[32m", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_COMMON("W", "\x1b[33m", fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) LOG_COMMON("E", "\x1b[31m", fmt, ##__VA_ARGS__)
