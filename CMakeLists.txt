cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME ddp-actuator-solver)
set(PROJECT_DESCRIPTION "DDP/iLQR solver for robotics actuators command")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
set(CXX_DISABLE_WERROR TRUE)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(14 ENFORCE)

# Project dependencies
add_project_dependency(Eigen3 REQUIRED)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/qpOASES")
find_package("qpOASES" REQUIRED)

# Set the headers to be installed
set(${PROJECT_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/costfunction.hh
    include/${CUSTOM_HEADER_DIR}/dynamicmodel.hh
    include/${CUSTOM_HEADER_DIR}/ddpsolver.hh
    include/${CUSTOM_HEADER_DIR}/pyrene_actuator/pyreneCostFunction.hh
    include/${CUSTOM_HEADER_DIR}/pyrene_actuator/pyreneActuator.hh
    include/${CUSTOM_HEADER_DIR}/romeo_actuator/costfunctionromeoactuator.hh
    include/${CUSTOM_HEADER_DIR}/romeo_actuator/romeosimpleactuator.hh
    include/${CUSTOM_HEADER_DIR}/romeo_actuator/romeotorqueactuator.hh
    include/${CUSTOM_HEADER_DIR}/temperature_control/costtemp.hh
    include/${CUSTOM_HEADER_DIR}/temperature_control/dctemp.hh)

set(${PROJECT_NAME}_SOURCES
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
    src/inverse_pendulum/modelIP.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} SYSTEM
                           PUBLIC ${EIGEN3_INCLUDE_DIR} ${qpOASES_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${qpOASES_LIBRARIES})

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
add_subdirectory(tests)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
