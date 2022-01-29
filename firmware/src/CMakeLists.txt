# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

idf_component_register(SRCS ${app_sources})

component_compile_options(
  -std=gnu++14 # for use of freertos macro
  # -fconcepts # for use of 'auto' in parameter declaration
  -Wall # This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid (or modify to prevent the warning), even in conjunction with macros. 
  -Wcast-align # Warn whenever a pointer is cast such that the required alignment of the target is increased.
  -Wdouble-promotion # Give a warning when a value of type float is implicitly promoted to double.
  -Wextra # This enables some extra warning flags that are not enabled by -Wall. 
  -Wfloat-equal # Warn if floating-point values are used in equality comparisons.
)
