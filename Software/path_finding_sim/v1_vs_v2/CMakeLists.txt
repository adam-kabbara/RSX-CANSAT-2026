cmake_minimum_required(VERSION 3.15)
project(pathguidance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Desktop debug module only. The flight build just compiles PathGuidance.cpp
# into the STM32 project directly -- it has no pybind11 / desktop dependency.
#
# Get pybind11's cmake dir from the pip package:
#   pip install pybind11
#   cmake -B build -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
#   cmake --build build
find_package(pybind11 REQUIRED)

pybind11_add_module(pathguidance bindings.cpp PathGuidance.cpp)
target_compile_options(pathguidance PRIVATE -O2 -Wall)
