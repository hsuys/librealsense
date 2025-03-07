# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2024 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.8)

project(RealsenseExamplesMulticamFramedrop)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-multicam-framedrop rs-multicam-framedrop.cpp ../example.hpp)
    set_property(TARGET rs-multicam-framedrop PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-multicam-framedrop ${DEPENDENCIES})
    set_target_properties (rs-multicam-framedrop PROPERTIES FOLDER Examples)
    install(TARGETS rs-multicam-framedrop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
