#-----------------------------------
# Bhaskar Bhattacharya - backbone taken from Nik Karpinsky
# Iowa State University 
# CMake file for compiling HandTracker
#-----------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(HandTracker)

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_SOURCE_DIR}/CMakeModules")
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${PROJECT_NAME}")
SET(SRC_DIR "${PROJECT_SOURCE_DIR}/src")

#------
# QT
#------

FIND_PACKAGE(Qt4 4.8.4)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
SET(LIBS ${LIBS} ${QT_LIBRARIES})

#------
# OpenGL (use glut32)
#------
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})

#------
# OpenCV (2.4)
#------
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
SET(LIBS ${LIBS} ${OpenCV_LIBS})

#------
# OSG (3.0.1)
#------
FIND_PACKAGE(OpenSceneGraph REQUIRED osgAnimation osgDB osgFX osgGA osgManipulator osgParticle osgPresentation osgShadow osgSim osgTerrain osgText osgUtil osgViewer osgVolume osgWidget)
INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR})
SET(LIBS ${LIBS} ${OPENSCENEGRAPH_LIBRARIES})


#------
# PCL (1.6.1)
#------

FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${PCL_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})


SET(SOURCES
	${SRC_DIR}/main.cpp
	${SRC_DIR}/Recorder.cpp
	${SRC_DIR}/Analyzer.cpp
	${SRC_DIR}/HandModel.cpp
	${SRC_DIR}/PointCloudViewer.cpp
	${SRC_DIR}/Background.cpp
	${SRC_DIR}/Controller.cpp
	${SRC_DIR}/skinfunction.cpp


	${SRC_DIR}/Recorder.h
	${SRC_DIR}/Analyzer.h
	${SRC_DIR}/HandModel.h
	${SRC_DIR}/PointCloudViewer.h
	${SRC_DIR}/Background.h
	${SRC_DIR}/Controller.h
	${SRC_DIR}/skinfunction.h
	
	
)

SET(UIS
	
	)
SET(MOCS
	)

QT4_WRAP_UI(GENERATED_SOURCES ${UIS})
QT4_WRAP_cPP(GENERATED_SOURCES ${MOCS})

SOURCE_GROUP("Generated Sources - Do Not Edit" FILES ${GENERATED_SOURCES})

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}") 

ADD_EXECUTABLE(${PROJECT_NAME}
	${SOURCES}
	${GENERATED_SOURCES}
	${HEADERS}
	${UIS}
	${MOCS}
	)


TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIBS})