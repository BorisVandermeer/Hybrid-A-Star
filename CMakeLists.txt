cmake_minimum_required(VERSION 3.21.0)
project(boris_planner)

# find_package(catkin REQUIRED)
set(CMAKE_CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug")

# Enable GDB Debugger
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_subdirectory(SecondParty/BorisMathLib)

message(${BORIS_MATH_INCLUDE_DIRS})

add_subdirectory(src)


# Build Tests
# add_subdirectory(test)

find_package(OpenCV 4 REQUIRED)

include_directories(${PLANNING_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

aux_source_directory(app NODE_SRC)
add_executable(node ${NODE_SRC} )
target_link_libraries(node ${PLANNING_LIBRARIES} ${OpenCV_LIBS})