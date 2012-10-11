cmake_minimum_required(VERSION 2.6)
project(april_tags)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME april_tags)
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.cc")
include_directories(AprilTags .)
include_directories(AprilTags /usr/include/eigen3)
add_library(apriltags ${SOURCE_FILES})

pods_use_pkg_config_packages(apriltags opencv)

pods_install_libraries(apriltags)

file(GLOB header_files "AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

pods_install_pkg_config_file(april_tags
    LIBS -lapriltags
    REQUIRES eigen3 opencv
    VERSION 1.0.0)

add_subdirectory(example)
