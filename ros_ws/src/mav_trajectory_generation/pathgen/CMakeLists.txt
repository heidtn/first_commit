cmake_minimum_required(VERSION 2.8.3)
project(pathgen)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(pathgen_node
  src/pathgen_node.cpp
  src/main.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
