CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "LKOptFlowViewer")
project(${PROJECT_NAME})

FIND_PACKAGE(YARP)
#FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})


FILE (GLOB source_folder *.cpp)
FILE (GLOB header_folder *.h)

add_executable(${PROJECT_NAME} ${source_folder} ${header_folder})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES})

#INSTALL(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)
INSTALL_TARGETS(/bin ${PROJECT_NAME})

