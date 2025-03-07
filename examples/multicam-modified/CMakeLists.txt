# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2024 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.8)

project(RealsenseExamplesMulticamModified)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-multicam-modified rs-multicam-modified.cpp ../example.hpp)
    set_property(TARGET rs-multicam-modified PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-multicam-modified ${DEPENDENCIES})
    set_target_properties (rs-multicam-modified PROPERTIES FOLDER Examples)
    install(TARGETS rs-multicam-modified RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
