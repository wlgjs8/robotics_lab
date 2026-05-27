cmake_minimum_required(VERSION 3.10)

# Set default options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set plugin version
include(cmake/version.cmake)
include(cmake/dependency.cmake)
project(
	mo-grippers
	VERSION "${CMAKE_PROJECT_VERSION}"
	LANGUAGES CXX
)

# Add your modular target
include(cmake/mo-grippers.cmake)

include(cmake/modbus.cmake)

# Uninstall target
include(cmake/uninstall.cmake)

# Add CPack for packaging
include(cmake/cpack.cmake)

message("")
message("= CMAKE STATUS =============================================")
message("    Version      : ${PROJECT_VERSION_DETAIL}")
message("    Compiler     : ${CMAKE_CXX_COMPILER}")
message("    C++ standard : ${CMAKE_CXX_STANDARD}")
message("    Build type   : ${CMAKE_BUILD_TYPE}")
message("    Source path  : ${CMAKE_SOURCE_DIR}")
message("    Install path : ${CMAKE_INSTALL_PREFIX}")
message("============================================================")
message("")