cmake_minimum_required(VERSION 3.16)
project(plane_segmentation_tool VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(PLANE_SEG_BUILD_TESTS "Build unit tests" ON)
option(PLANE_SEG_WITH_VIEWER "Build the --view option (needs pcl_visualization)" ON)

set(PLANE_SEG_PCL_COMPONENTS common io filters search kdtree sample_consensus segmentation surface)
if(PLANE_SEG_WITH_VIEWER)
  list(APPEND PLANE_SEG_PCL_COMPONENTS visualization)
endif()
find_package(PCL 1.10 REQUIRED COMPONENTS ${PLANE_SEG_PCL_COMPONENTS})
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(plane_seg
  src/plane_segmenter.cpp
  src/plane_pose.cpp
  src/config.cpp
  src/synthetic_scene.cpp
)
target_include_directories(plane_seg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(plane_seg PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(plane_seg PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)
target_compile_options(plane_seg PRIVATE -Wall -Wextra -Wpedantic)

add_executable(segment_plane apps/segment_plane.cpp)
target_link_libraries(segment_plane PRIVATE plane_seg)
if(PLANE_SEG_WITH_VIEWER)
  target_compile_definitions(segment_plane PRIVATE PLANE_SEG_WITH_VIEWER)
endif()

add_executable(benchmark apps/benchmark.cpp)
target_link_libraries(benchmark PRIVATE plane_seg)

add_executable(evaluate apps/evaluate.cpp)
target_link_libraries(evaluate PRIVATE plane_seg)

if(PLANE_SEG_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(plane_seg_tests tests/test_plane_segmenter.cpp)
  target_link_libraries(plane_seg_tests PRIVATE plane_seg GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(plane_seg_tests)
endif()
