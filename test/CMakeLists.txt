cmake_minimum_required(VERSION 3.0.2)
project(test)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(OpenCV)

include_directories(${PLANNING_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_map_gridmap_dilate maps/test_map_Dilate.cpp)
target_link_libraries(test_map_gridmap_dilate ${OpenCV_LIBS} ${PLANNING_LIBRARIES})

add_executable(test_models_move_radius models/test_models_move.cpp)
target_link_libraries(test_models_move_radius ${OpenCV_LIBS} ${PLANNING_LIBRARIES})

add_executable(test_searcher_HAS_dilate pathsearcher/test_HAS_global_dilate.cpp)
target_link_libraries(test_searcher_HAS_dilate ${OpenCV_LIBS} ${PLANNING_LIBRARIES})

add_executable(test_searcher_HAS_functions pathsearcher/test_HAS_functions.cpp)
target_link_libraries(test_searcher_HAS_functions ${OpenCV_LIBS} ${PLANNING_LIBRARIES})

add_executable(test_searcher_HAS_search pathsearcher/test_HAS_search.cpp)
target_link_libraries(test_searcher_HAS_search ${OpenCV_LIBS} ${PLANNING_LIBRARIES})

