# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

add_executable(iCubVergenceControl main.cpp)
target_link_libraries(iCubVergenceControl ${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
