# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

set(TEST_TARGET_NAME ${PROJECT_NAME}_test)

add_executable(${TEST_TARGET_NAME} main.cpp)
target_link_libraries(${TEST_TARGET_NAME} ${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})
add_test(NAME ${TEST_TARGET_NAME} COMMAND ${TEST_TARGET_NAME})
