cmake_minimum_required(VERSION 3.21.0)
project(BorisMathLib VERSION 0.0.4)

if( BorisMathLib_IS_TOP_LEVEL)
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

include_directories(.)

aux_source_directory(Points    SRC_FILES_1)
aux_source_directory(Curves    SRC_FILES_2)
aux_source_directory(Interplot SRC_FILES_3)
aux_source_directory(Vectors   SRC_FILES_4)
aux_source_directory(Geometry  SRC_FILES_5)

SET(BORIS_MATH_SRC_FILES ${SRC_FILES_1} ${SRC_FILES_2}  ${SRC_FILES_3}  ${SRC_FILES_4} ${SRC_FILES_5})

add_library(BORIS_MATH_LIB ${BORIS_MATH_SRC_FILES})

if(IS_TOP_DIR)
    SET(BORIS_MATH_VERSION BorisMathLib_VERSION)
    SET(BORIS_MATH_FOUND YES PARENT_SCOPE)
    SET(BORIS_MATH_LIBRARIES BORIS_MATH_LIB)
    SET(BORIS_MATH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
else()
    SET(BORIS_MATH_VERSION BorisMathLib_VERSION PARENT_SCOPE)
    SET(BORIS_MATH_FOUND YES PARENT_SCOPE)
    SET(BORIS_MATH_LIBRARIES BORIS_MATH_LIB PARENT_SCOPE)
    SET(BORIS_MATH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
endif()

if(ENABLE_TEST) 
    add_subdirectory(test)
endif()