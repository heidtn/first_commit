cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(FirstCommitExternal VERSION 0.1.0 LANGUAGES CXX)

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

# ------------------------------------------------------------------------------
# Set vars
set(CMAKE_CXX_STANDARD 14)

get_filename_component(workspace_root ".."
  REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

set(FIRST_COMMIT_OUT_DIR "${workspace_root}/out" CACHE STRING "Output directory" FORCE)
set(FIRST_COMMIT_EXTERNAL_SRC_DIR "${FIRST_COMMIT_OUT_DIR}/external/src" CACHE STRING
  "Location to download external project sources" FORCE)
set(FIRST_COMMIT_EXTERNAL_INSTALL_DIR "${FIRST_COMMIT_OUT_DIR}/external/install" CACHE STRING
  "Location to install external project artifacts" FORCE)

set(external_cmake_args
  "-DCMAKE_INSTALL_PREFIX=${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  "-DCMAKE_PREFIX_PATH=${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}")

set(external_configure_args
  "--prefix=${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  "CXXFLAGS=-std=c++11")

# ------------------------------------------------------------------------------
# External projects
ExternalProject_Add("protobuf"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/protobuf"
  URL "https://github.com/google/protobuf/archive/v3.3.0.tar.gz"
  URL_HASH SHA256=94c414775f275d876e5e0e4a276527d155ab2d0da45eed6b7734301c330be36e
  CONFIGURE_COMMAND ./autogen.sh
  COMMAND ./configure ${external_configure_args}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
  BUILD_IN_SOURCE 1)

ExternalProject_Add("zmq"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/zmq"
  URL "https://github.com/zeromq/libzmq/archive/v4.2.2.tar.gz"
  URL_HASH SHA256=ea5eecbb9cc231a750535a7b74f887b18656014cc92601e8ab34734db799b796
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS ${external_cmake_args})

ExternalProject_Add("cppzmq"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/cppzmq"
  URL "https://github.com/zeromq/cppzmq/archive/v4.2.1.tar.gz"
  URL_HASH SHA256=11c699001659336c7d46779f714f3e9d15d63343cd2ae7c1905e4bf58907cef9
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS "${external_cmake_args}"
  DEPENDS "zmq")

ExternalProject_Add("util_linux"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/util_linux"
  URL "https://github.com/karelzak/util-linux/archive/v2.30.tar.gz"
  URL_HASH SHA256=ec6416bfecd07d10939350f515ea9628e37c31d03f74be9113eb3b7d008262f1
  CONFIGURE_COMMAND ./autogen.sh
  COMMAND ./configure ${external_configure_args}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
  BUILD_IN_SOURCE 1)

ExternalProject_Add("ignition_math"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/ignition_math"
  URL "https://bitbucket.org/ignitionrobotics/ign-math/get/ignition-math3_3.2.0.tar.gz"
  URL_HASH SHA256=1948c1610fa4403bce7ba2a262a29662990ee66aab00882411a0868afe0e5309
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS "${external_cmake_args}")

# This builds and installs the ign command-line tool.
ExternalProject_Add("ignition_tools"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/ignition_tools"
  # NOTE: ign-tools does not have any release tags.
  # But there haven't been any new commits since 2016-08-08 (as of 2017-06-22).
  URL "https://bitbucket.org/ignitionrobotics/ign-tools/get/75a248c4c899.tar.gz"
  URL_HASH SHA256=61b041ee347fecb47ddab7973c9a8e4999faeaa857f49848aa180b9864885cda
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS "${external_cmake_args}")

ExternalProject_Add("ignition_msgs"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/ignition_msgs"
  URL "https://bitbucket.org/ignitionrobotics/ign-msgs/get/ignition-msgs_0.7.0.tar.gz"
  URL_HASH SHA256=a3bd439c15df1dbde7f9e8c6fb272c7bfb2dcd34059d6b672f6805918e455e72
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS "${external_cmake_args}"
  DEPENDS "ignition_math" "protobuf")

ExternalProject_Add("ignition_transport"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/ignition_transport"
  URL "https://bitbucket.org/ignitionrobotics/ign-transport/get/ignition-transport3_3.0.1.tar.gz"
  URL_HASH SHA256=bcc9d7ad640d9ad3a38f8186b48408eef8195d4a04d768e0a5769b7f501d10ca
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS
    "${external_cmake_args}"
    "-DPKG_CONFIG_PATH=${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}/lib/pkgconfig"
  DEPENDS "ignition_math" "ignition_msgs" "protobuf" "zmq" "cppzmq" "util_linux")

ExternalProject_Add("eigen"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/eigen"
  URL "https://bitbucket.org/eigen/eigen/get/3.3.tar.gz"
  URL_HASH SHA256=045efcfae7fc2f243abb2e80181ae7c6952dcf6cd66623f9395f37452c70e45a
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS "${external_cmake_args}")

ExternalProject_Add("opencv"
  PREFIX "${FIRST_COMMIT_EXTERNAL_SRC_DIR}/opencv"
  URL "https://github.com/opencv/opencv/archive/3.2.0.tar.gz"
  URL_HASH SHA256=b9d62dfffb8130d59d587627703d5f3e6252dce4a94c1955784998da7a39dd35
  INSTALL_DIR "${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}"
  CMAKE_ARGS
    "${external_cmake_args}"
    "-DPKG_CONFIG_PATH=${FIRST_COMMIT_EXTERNAL_INSTALL_DIR}/lib/pkgconfig")
