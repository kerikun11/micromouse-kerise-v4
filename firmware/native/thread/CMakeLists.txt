cmake_minimum_required(VERSION 3.13)
project("test" CXX)

## global build options
set(CMAKE_CXX_STANDARD 17) # enable option -std=c++17
set(CMAKE_CXX_EXTENSIONS OFF) # without compiler extensions like gnu++17

set(TARGET_NAME "app")
file(GLOB SRC_FILES *.cpp)
add_executable(${TARGET_NAME} ${SRC_FILES})
find_package(Threads REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE Threads::Threads)
