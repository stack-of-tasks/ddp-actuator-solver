macro(_local_test_ddp)
  set(list_var "${ARGN}")
  foreach(ex IN LISTS list_var)
    add_executable(${ex} ${ex}.cpp)
    target_link_libraries(${ex} ${PROJECT_NAME} ${qpOASES_LIBRARIES})
  endforeach()
endmacro()

add_subdirectory(pyrene_actuator)
add_subdirectory(romeo_actuator)
add_subdirectory(temperature_control)
add_subdirectory(linear)
add_subdirectory(inverse_pendulum)
