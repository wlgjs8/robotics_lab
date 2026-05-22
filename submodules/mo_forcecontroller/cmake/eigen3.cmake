find_package(Eigen3 QUIET)

if(NOT Eigen3_FOUND)
  # Run the shell script to check and install dependencies
  execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/script/eigen3.sh RESULT_VARIABLE RESULT)
  if(NOT RESULT EQUAL 0)
    message(FATAL_ERROR "Dependency installation failed.")
  endif()
endif()
