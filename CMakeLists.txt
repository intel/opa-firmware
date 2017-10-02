# COPYRIGHT (c) 2017 Intel Corporation
#
# Run cmake as:
#  cmake
#

cmake_minimum_required(VERSION 2.6)
project(opa-firmware C)

set(PACKAGE_VERSION "1")

configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

include_directories ("${PROJECT_BINARY_DIR}")
add_executable(hfi1_eprom hfi1_eprom.c)

