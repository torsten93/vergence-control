# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

find_package(YARP QUIET)
if(YARP_FOUND)
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
  set(ICUB_MODULE_NAME iCubVergenceControl)

  add_executable(${ICUB_MODULE_NAME} main.cpp)
  target_link_libraries(${ICUB_MODULE_NAME} ${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
else()
  message(STATUS "YARP not found, skipping compilation of ${ICUB_MODULE_NAME}")
endif()
