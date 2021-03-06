# Copyright: (C) 2011 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "logSort")
PROJECT(${KEYWORD})

#set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}    
    ${ICUB_INCLUDE_DIRS}    
    ${EMORPHLIB_INCLUDE_DIRS}
)


# Search for source code.
FILE(GLOB folder_source 
  src/convert.cpp 
  src/logFrameConverter.cpp
  src/logSortModule.cpp
  src/logSortThread.cpp
  src/logUnmask.cpp
  src/main.cpp
  src/plotterThread.cpp
  src/processingThread.cpp
  )

FILE(GLOB folder_header 
  include/iCub/convert.h	  
  include/iCub/logFrameConverter.h
  include/iCub/logSortModule.h
  include/iCub/logSortThread.h
  include/iCub/logUnmask.h
  include/iCub/plotterThread.h
  include/iCub/processingThread.h
  )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}
      eMorphLib
      ${YARP_LIBRARIES}
      ${OpenCV_LIBRARIES}
      )	

    install(TARGETS ${KEYWORD} DESTINATION bin)
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
