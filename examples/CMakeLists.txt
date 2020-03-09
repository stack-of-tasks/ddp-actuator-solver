macro(_local_test_ddp)
  set(list_var "${ARGN}")
  foreach(ex IN LISTS list_var)
    ADD_EXECUTABLE(${ex} ${ex}.cpp)
    TARGET_LINK_LIBRARIES(${ex} ${PROJECT_NAME}-examples ${qpOASES_LIBRARIES})
  endforeach()
endmacro()

SET(${PROJECT_NAME}_EXAMPLE_SOURCES
  pyrene_actuator/pyreneCostFunction.cpp
  pyrene_actuator/pyreneActuator.cpp
  romeo_actuator/costfunctionromeoactuator.cpp
  romeo_actuator/romeosimpleactuator.cpp
  romeo_actuator/romeotorqueactuator.cpp
  temperature_control/costtemp.cpp
  temperature_control/dctemp.cpp
  linear/costLinear.cpp
  linear/modelLinear.cpp
  inverse_pendulum/costIP.cpp
  inverse_pendulum/modelIP.cpp
  )

SET(${PROJECT_NAME}_EXAMPLE_HEADERS
  pyrene_actuator/pyreneCostFunction.hh
  pyrene_actuator/pyreneActuator.hh
  romeo_actuator/costfunctionromeoactuator.hh
  romeo_actuator/romeosimpleactuator.hh
  romeo_actuator/romeotorqueactuator.hh
  temperature_control/costtemp.hh
  temperature_control/dctemp.hh
  )

ADD_LIBRARY(${PROJECT_NAME}-examples SHARED
  ${${PROJECT_NAME}_EXAMPLE_SOURCES} ${${PROJECT_NAME}_EXAMPLE_HEADERS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}-examples ${qpOASES_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}-examples SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})

INSTALL(TARGETS ${PROJECT_NAME}-examples EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(pyrene_actuator)
ADD_SUBDIRECTORY(romeo_actuator)
ADD_SUBDIRECTORY(temperature_control)
ADD_SUBDIRECTORY(linear)
ADD_SUBDIRECTORY(inverse_pendulum)
