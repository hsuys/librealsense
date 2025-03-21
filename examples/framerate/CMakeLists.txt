# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2024 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.8)

project(RealsenseExamplesFramerate)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-framerate rs-framerate.cpp ../example.hpp)
    set_property(TARGET rs-framerate PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-framerate ${DEPENDENCIES})
    set_target_properties (rs-framerate PROPERTIES FOLDER Examples)
    install(TARGETS rs-framerate RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
