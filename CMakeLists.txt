cmake_minimum_required(VERSION 3.10)
project(plane_segmenter)

# Include your own headers
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters segmentation visualization surface)

# Force PCL include path if variable is not working
include_directories(
    ${PCL_INCLUDE_DIRS}
    /usr/include/pcl-1.12
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(plane_segmenter
    src/main.cpp
    src/PlaneSegmenter.cpp
)

target_link_libraries(plane_segmenter ${PCL_LIBRARIES})

