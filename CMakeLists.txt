CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  

PROJECT(DirectOdometry)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
message(${CMAKE_MODULE_PATH})

SET(CMAKE_BUILD_TYPE Release)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)


INCLUDE_DIRECTORIES(
  include
  outer
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${OPENGL_gl_LIBRARY}
  glfw
)

LIST(APPEND SOURCEFILES
    main.cpp
    Detector.cpp
    Frame.cpp
    parameter.cpp
    Tracker.cpp
    outer/PixelSelector2.cpp
    outer/GLFWViewer.cpp
)

ADD_EXECUTABLE(DirectOdometry ${SOURCEFILES})
TARGET_LINK_LIBRARIES(DirectOdometry ${LINK_LIBS})