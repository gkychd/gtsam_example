cmake_minimum_required(VERSION 2.8)
enable_testing()
project(gtsamexamples CXX C)

set(CMAKE_BUILD_TYPE "Debug")

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

find_package(Boost 1.50 REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

add_subdirectory(examples)