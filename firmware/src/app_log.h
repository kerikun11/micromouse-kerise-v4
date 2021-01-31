/**
 * @file app_log.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this provides logging formats
 * @date 2018-12-18
 */
#pragma once

#include <iostream>

#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)
#define APP_LOG_OSTREAM_COMMON(c)                                              \
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
