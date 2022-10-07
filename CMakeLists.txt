cmake_minimum_required(VERSION 3.12)
project(pygpg)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package (Python3 COMPONENTS Interpreter Development)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add gpg DEPENDENCIES:
find_library(GENERATOR_LIB grasp_candidates_generator)
find_path(GENERATOR_LIB_INCLUDE_DIR gpg/grasp.h)
include_directories(${GENERATOR_LIB_INCLUDE_DIR})

set(PYBIND11_PYTHON_VERSION 3)
# Check if pybind11 exists as a subdirectory
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pybind11)
    add_subdirectory(pybind11)
else()
    find_package(pybind11 2.6 REQUIRED)
endif()
pybind11_add_module(pygpg MODULE src/py_generate_grasp_from_gpg.cpp)
target_link_libraries(pygpg PRIVATE ${GENERATOR_LIB})
