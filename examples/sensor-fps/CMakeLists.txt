# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2024 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.8)

project(RealsenseExamplesSensorFps)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-sensor-fps rs-sensor-fps.cpp ../example.hpp)
    set_property(TARGET rs-sensor-fps PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-sensor-fps ${DEPENDENCIES})
    set_target_properties (rs-sensor-fps PROPERTIES FOLDER Examples)
    install(TARGETS rs-sensor-fps RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
