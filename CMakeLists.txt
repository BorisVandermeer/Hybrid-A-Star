cmake_minimum_required(VERSION 3.21.0)
project(BorisPlanner)

# Top Level Check
if( BorisPlanner_IS_TOP_LEVEL)
    SET(IS_TOP_DIR YES)
    SET(ENABLE_TEST YES)
else()
    SET(IS_TOP_DIR NO)
    SET(ENABLE_TEST NO)
endif()

# Build Config
if(IS_TOP_DIR)
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
endif()

# <<<<< Find Dependencies <<<<< #

if(NOT BORIS_MATH_FOUND)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
        execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty/BorisMathLib)
        execute_process(COMMAND git clone git@github.com:BorisVandermeer/BorisMathLib.git --depth=1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()

    add_subdirectory(SecondParty/BorisMathLib)
endif()
# >>>>> Find Dependencies >>>>> #

SET(BORIS_PLANNER_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)

# Build Tests
if(ENABLE_TEST)
    add_subdirectory(test)
endif()

find_package(OpenCV 4 REQUIRED)
aux_source_directory(app NODE_SRC)
add_executable(node ${NODE_SRC} )
target_link_libraries(node ${PLANNING_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(node PUBLIC ${PLANNING_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
