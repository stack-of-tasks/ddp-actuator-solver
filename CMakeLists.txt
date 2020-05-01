CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME ddp-actuator-solver)
SET(PROJECT_DESCRIPTION "DDP/iLQR solver for robotics actuators command")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES eigen3)
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/qpOASES")
FIND_PACKAGE("qpOASES" REQUIRED)


# Set the headers to be installed
SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/costfunction.hh
  include/${CUSTOM_HEADER_DIR}/dynamicmodel.hh
  include/${CUSTOM_HEADER_DIR}/ddpsolver.hh
  include/${CUSTOM_HEADER_DIR}/pyrene_actuator/pyreneCostFunction.hh
  include/${CUSTOM_HEADER_DIR}/pyrene_actuator/pyreneActuator.hh
  include/${CUSTOM_HEADER_DIR}/romeo_actuator/costfunctionromeoactuator.hh
  include/${CUSTOM_HEADER_DIR}/romeo_actuator/romeosimpleactuator.hh
  include/${CUSTOM_HEADER_DIR}/romeo_actuator/romeotorqueactuator.hh
  include/${CUSTOM_HEADER_DIR}/temperature_control/costtemp.hh
  include/${CUSTOM_HEADER_DIR}/temperature_control/dctemp.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/pyrene_actuator/pyreneCostFunction.cpp
  src/pyrene_actuator/pyreneActuator.cpp
  src/romeo_actuator/costfunctionromeoactuator.cpp
  src/romeo_actuator/romeosimpleactuator.cpp
  src/romeo_actuator/romeotorqueactuator.cpp
  src/temperature_control/costtemp.cpp
  src/temperature_control/dctemp.cpp
  src/linear/costLinear.cpp
  src/linear/modelLinear.cpp
  src/inverse_pendulum/costIP.cpp
  src/inverse_pendulum/modelIP.cpp
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR} ${qpOASES_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${qpOASES_LIBRARIES})

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
