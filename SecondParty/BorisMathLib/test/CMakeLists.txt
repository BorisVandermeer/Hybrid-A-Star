cmake_minimum_required(VERSION 3.0.2)

find_package(OpenCV REQUIRED)

include_directories(${BORIS_MATH_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_spline test_Spline.cpp)
target_link_libraries(test_spline ${OpenCV_LIBS} ${BORIS_MATH_LIBRARIES})

add_executable(test_spline_curve test_SplineCurve.cpp)
target_link_libraries(test_spline_curve ${OpenCV_LIBS} ${BORIS_MATH_LIBRARIES})

add_executable(test_rscurve test_RS.cpp)
target_link_libraries(test_rscurve ${OpenCV_LIBS} ${BORIS_MATH_LIBRARIES})

add_executable(test_geometry_fuctions test_GeometryFuc.cpp)
target_link_libraries(test_geometry_fuctions ${OpenCV_LIBS} ${BORIS_MATH_LIBRARIES})