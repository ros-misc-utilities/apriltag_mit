cmake_minimum_required(VERSION 2.6)
project(apriltags)

#add_definitions(-pg) #"-fopenmp)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME apriltags)
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.cc")
include_directories(AprilTags . /opt/local/include)
add_library(apriltags ${SOURCE_FILES})

pods_use_pkg_config_packages(apriltags eigen3 opencv)
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # v4l not available on Mac
  pods_use_pkg_config_packages(apriltags libv4l2)
endif ()

pods_install_libraries(apriltags)

file(GLOB header_files "AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

pods_install_pkg_config_file(apriltags
    LIBS -lapriltags
    REQUIRES eigen3 opencv
    VERSION 1.0.0)

add_subdirectory(example)
