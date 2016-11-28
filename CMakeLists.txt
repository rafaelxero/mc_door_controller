cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)

set(PROJECT_NAME mc_door_controller)
set(PROJECT_DESCRIPTION "Door controller")
set(PROJECT_URL "")

setup_project()

if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

add_subdirectory(src)
setup_project_finalize()