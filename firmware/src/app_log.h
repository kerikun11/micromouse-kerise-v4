/**
 * @file app_log.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this provides logging formats
 * @version 0.1
 * @date 2018-12-18
 *
 * @copyright Copyright (c) 2018
 *
 */
#pragma once

#include <iostream>

#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)
#define APP_LOG_OSTREAM_COMMON(c)                                              \
  (std::cout << "[" c "][" __FILE__ ":" APP_TOSTRING(__LINE__) "] ")

#ifndef logd
#if 1
#define logd APP_LOG_OSTREAM_COMMON("D")
#else
#define logd std::ostream(0)
#endif
#endif

#ifndef logi
#if 1
#define logi APP_LOG_OSTREAM_COMMON("I")
#else
#define logi std::ostream(0)
#endif
#endif

#ifndef logw
#if 1
#define logw APP_LOG_OSTREAM_COMMON("W")
#else
#define logw std::ostream(0)
#endif
#endif

#ifndef loge
#if 1
#define loge APP_LOG_OSTREAM_COMMON("E")
#else
#define loge std::ostream(0)
#endif
#endif
