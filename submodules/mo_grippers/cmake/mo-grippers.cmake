set(CMAKE_TARGET_NAME "mo-grippers")

find_package(Threads REQUIRED)

find_library(
  MODBUS_LIBRARY
  NAMES modbus
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(
    ${CMAKE_TARGET_NAME} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/grippers/dh-gripper.cpp
)

add_library(
  plaif::${CMAKE_TARGET_NAME} ALIAS ${CMAKE_TARGET_NAME}
)

target_include_directories(${CMAKE_TARGET_NAME} PRIVATE Threads::Threads ${LIBMODBUS_INCLUDE_DIRS})

target_link_libraries(${CMAKE_TARGET_NAME} PRIVATE ${LIBMODBUS_LIBRARIES})

target_compile_options(${CMAKE_TARGET_NAME} PRIVATE -fPIC)

# Enable _DEBUG when in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_CONFIGURATION_TYPES MATCHES "Debug")
  target_compile_definitions(${CMAKE_TARGET_NAME} PRIVATE _DEBUG)
endif()

set_target_properties(${CMAKE_TARGET_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME ${CMAKE_TARGET_NAME}
)

install(TARGETS ${CMAKE_TARGET_NAME}
    EXPORT ${CMAKE_TARGET_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
)

install(
    EXPORT ${CMAKE_TARGET_NAME}Targets
    FILE ${CMAKE_TARGET_NAME}Targets.cmake
    NAMESPACE plaif::
    DESTINATION lib/cmake/${CMAKE_TARGET_NAME}
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/in/config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}Config.cmake"
    @ONLY
)

install(
    FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_TARGET_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${CMAKE_TARGET_NAME}
)

# Example executable
#add_executable(dh_example examples/dh_test.cpp)
#target_link_libraries(dh_example PRIVATE mo-grippers ${LIBMODBUS_LIBRARIES})