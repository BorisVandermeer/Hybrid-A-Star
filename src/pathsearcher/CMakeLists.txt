cmake_minimum_required(VERSION 3.0.2)
project(path_searchers)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BORIS_MATH_INCLUDE_DIRS})
include_directories(${BORIS_MAP_INCLUDE_DIRS})
include_directories(${VEHICLE_MODELS_INCLUDE_DIRS})
include_directories(${PLANNING_UTILS_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(. PATH_SEARCHER_SRC_FILES)

add_library(PATH_SEARCHER_LIB ${PATH_SEARCHER_SRC_FILES})
target_link_libraries(PATH_SEARCHER_LIB 
                    ${BORIS_MATH_LIBRARIES} 
        ${BORIS_MAP_LIBRARIES} ${VEHICLE_MODELS_LIBRARIES})

SET(PATH_SEARCHER_LIBRARIES PATH_SEARCHER_LIB PARENT_SCOPE)
SET(PATH_SEARCHER_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)