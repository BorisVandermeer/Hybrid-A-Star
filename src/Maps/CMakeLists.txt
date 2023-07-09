cmake_minimum_required(VERSION 3.0.2)
project(boris_maps)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BORIS_MATH_INCLUDE_DIRS})
include_directories(${VEHICLE_MODELS_INCLUDE_DIRS})
include_directories(${PLANNING_UTILS_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(. BORIS_MAP_SRC_FILES)

add_library(BORIS_MAP_LIB ${BORIS_MAP_SRC_FILES})

SET(BORIS_MAP_LIBRARIES BORIS_MAP_LIB PARENT_SCOPE)
SET(BORIS_MAP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)