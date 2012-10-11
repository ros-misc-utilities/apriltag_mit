cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

add_executable(apriltags_demo apriltags_demo.cpp)
pods_install_executables(apriltags_demo)
