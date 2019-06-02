macro(generate_message_properties_map)
set(
  CPP_AND_HPP
  "${CMAKE_CURRENT_SOURCE_DIR}/src/message_properties_map.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/message_properties_map.hpp"
)
set(GENERATOR_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_message_properties_map.py")
add_custom_command(
  OUTPUT ${CPP_AND_HPP}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENERATOR_SCRIPT} ${ARGN}
  COMMENT "Generating C++ code for message_properties_map using packages ${ARG_PACKAGES}"
  DEPENDS ${GENERATOR_SCRIPT}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
endmacro()