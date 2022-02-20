# main component of ESP-IDF

FILE(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

idf_component_register(SRCS ${app_sources})

# c.f. gcc warnings: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
component_compile_options(
  -std=gnu++14 # for use of freertos macro
  # -fconcepts # for use of 'auto' in parameter declaration
  -Wall # This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid (or modify to prevent the warning), even in conjunction with macros. 
  -Wextra # This enables some extra warning flags that are not enabled by -Wall. 
  -Wcast-align # Warn whenever a pointer is cast such that the required alignment of the target is increased.
  -Wdouble-promotion # Give a warning when a value of type float is implicitly promoted to double.
  -Wfloat-equal # Warn if floating-point values are used in equality comparisons.
  # -Wconversion # Warn for implicit conversions that may alter a value.
)
