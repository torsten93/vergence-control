# Copyright (C) 2017 PSPC-lab - Department of Informatics, Bioengineering,
#                    Robotics and Systems Engineering - University of Genoa
# All Rights Reserved.
# Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini

set(${PROJECT_NAME}_SRC src/VergenceControl.cpp)
set(${PROJECT_NAME}_HDR include/VergenceControl/VergenceControl.h)

add_definitions(-D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                                      PUBLIC_HEADER "${${PROJECT_NAME}_HDR}")

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                  ${OpenCV_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                     COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
