# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

cmake_minimum_required(VERSION 3.1)
project(FingersPositionControl_test-package)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(FingersPositionControl REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} FingersPositionControl::FingersPositionControl)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()
