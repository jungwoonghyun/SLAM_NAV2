# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_safeway_robot_controll_packet_RS485_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED safeway_robot_controll_packet_RS485_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(safeway_robot_controll_packet_RS485_FOUND FALSE)
  elseif(NOT safeway_robot_controll_packet_RS485_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(safeway_robot_controll_packet_RS485_FOUND FALSE)
  endif()
  return()
endif()
set(_safeway_robot_controll_packet_RS485_CONFIG_INCLUDED TRUE)

# output package information
if(NOT safeway_robot_controll_packet_RS485_FIND_QUIETLY)
  message(STATUS "Found safeway_robot_controll_packet_RS485: 0.0.0 (${safeway_robot_controll_packet_RS485_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'safeway_robot_controll_packet_RS485' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${safeway_robot_controll_packet_RS485_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(safeway_robot_controll_packet_RS485_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${safeway_robot_controll_packet_RS485_DIR}/${_extra}")
endforeach()
