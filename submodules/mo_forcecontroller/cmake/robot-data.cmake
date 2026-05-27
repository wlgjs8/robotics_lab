find_package(robot-data QUIET)

if(NOT robot-data_FOUND)
  message(STATUS "robot-data does not exist")
else()
  message(STATUS "robot-data found: ${robot-data_VERSION}")
endif()
