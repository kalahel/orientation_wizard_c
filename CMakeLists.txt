cmake_minimum_required(VERSION 3.13)
project(orientation_wizard_c C)

set(CMAKE_C_STANDARD 99)

add_executable(orientation_wizard_c main.c serial_gps.c serial_gps.h vision.cpp vision.h)



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -pedantic -Wextra")

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(. util)

target_link_libraries(orientation_wizard_c C ${OpenCV_LIBS})
target_link_libraries(orientation_wizard_c C ${CMAKE_THREAD_LIBS_INIT})