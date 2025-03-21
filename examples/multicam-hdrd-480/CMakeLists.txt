# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2024 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.8)

project(RealsenseExamplesMulticamHdrd480)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-multicam-hdrd-480 rs-multicam-hdrd-480.cpp ../example.hpp)
    set_property(TARGET rs-multicam-hdrd-480 PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-multicam-hdrd-480 ${DEPENDENCIES})
    set_target_properties (rs-multicam-hdrd-480 PROPERTIES FOLDER Examples)
    install(TARGETS rs-multicam-hdrd-480 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
