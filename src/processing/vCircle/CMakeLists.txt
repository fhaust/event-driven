# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME vCircle)
project(${PROJECTNAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${EMORPHLIB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} eMorphLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
