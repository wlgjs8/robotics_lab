set(CMAKE_TARGET_NAME "moforcecontroller")

add_library(
  ${CMAKE_TARGET_NAME} SHARED
  src/cpp/plaif/moforcecontroller/force_control.cpp src/cpp/plaif/moforcecontroller/algorithm.cpp
  src/cpp/plaif/moforcecontroller/controller_utils.cpp src/cpp/plaif/moforcecontroller/data_analysis.cpp
)

add_library(
  plaif::${CMAKE_TARGET_NAME} ALIAS ${CMAKE_TARGET_NAME}
)

target_include_directories(
  ${CMAKE_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp> $<INSTALL_INTERFACE:include>
         ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${CMAKE_TARGET_NAME} PUBLIC ${PINOCCHIO_LIBRARIES} plaif::plogger)

set_target_properties(
  ${CMAKE_TARGET_NAME}
  PROPERTIES CXX_STANDARD 17
             CXX_STANDARD_REQUIRED ON
             POSITION_INDEPENDENT_CODE ON
             SOVERSION ${PROJECT_VERSION_MAJOR}
             EXPORT_NAME ${CMAKE_TARGET_NAME}
)

install(
  TARGETS ${CMAKE_TARGET_NAME}
  EXPORT ${CMAKE_TARGET_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include)

install(
  EXPORT ${CMAKE_TARGET_NAME}Targets
  FILE ${CMAKE_TARGET_NAME}Targets.cmake
  NAMESPACE plaif::
  DESTINATION lib/cmake/moforcecontroller
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}ConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/in/config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}Config.cmake" @ONLY
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}ConfigVersion.cmake"
        DESTINATION lib/cmake/${CMAKE_TARGET_NAME}
)
