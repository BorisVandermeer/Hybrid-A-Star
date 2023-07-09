cmake_minimum_required(VERSION 3.0.2)
project(math)

include_directories(.)

aux_source_directory(Points RefPoints_SRC_FILES)
aux_source_directory(Curves Curves_SRC_FILES)
aux_source_directory(Interplot Interpolot_SRC_FILES)

add_library(MATH_LIBS ${RefPoints_SRC_FILES} ${Curves_SRC_FILES} ${Interpolot_SRC_FILES})

SET(BORIS_MATH_LIBRARIES MATH_LIBS PARENT_SCOPE)
SET(BORIS_MATH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
