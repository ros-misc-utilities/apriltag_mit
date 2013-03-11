cmake_minimum_required(VERSION 2.6)
project(apriltags)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME apriltags)
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.cc")
include_directories(AprilTags .)
add_library(apriltags ${SOURCE_FILES})

# working around on annoying OpenCV 2.4 bug that renders pkgconfig useless (reason: malformed --libs)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIR})
target_link_libraries(apriltags ${OpenCV_LIBS})

pods_use_pkg_config_packages(apriltags eigen3)

pods_install_libraries(apriltags)

file(GLOB header_files "AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

pods_install_pkg_config_file(apriltags
    LIBS -lapriltags
    REQUIRES eigen3 opencv
    VERSION 1.0.0)

add_subdirectory(example)
