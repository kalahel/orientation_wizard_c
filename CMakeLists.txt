cmake_minimum_required(VERSION 3.13)
project(orientation_wizard_c C)

set(CMAKE_C_STANDARD 99)

add_executable(orientation_wizard_c main.c serial_gps.c serial_gps.h vision.cpp vision.h pathfinding.c pathfinding.h)