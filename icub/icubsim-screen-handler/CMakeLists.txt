# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

add_definitions(-D_USE_MATH_DEFINES)
add_executable(iCubSimScreenHandler main.cpp)
target_link_libraries(iCubSimScreenHandler ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
