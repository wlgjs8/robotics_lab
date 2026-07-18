cmake_minimum_required(VERSION 3.16)
project(camera_server VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAMERA_SERVER_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(CAMERA_SERVER_BUILD_TESTS "Build camera_server tests" ON)
option(CAMERA_SERVER_FORCE_MOCK_CAMERA "Do not compile RealSense backend" OFF)
option(CAMERA_SERVER_FORCE_ZMQ_STUB "Do not compile ZeroMQ publisher backend" OFF)
option(CAMERA_SERVER_FORCE_NO_OPENCV "Do not compile the UVC/OpenCV fisheye backend" OFF)
set(CAMERA_SERVER_REALSENSE_BACKEND "system" CACHE STRING "RealSense transport backend label (system, native, or rsusb)")
set_property(CACHE CAMERA_SERVER_REALSENSE_BACKEND PROPERTY STRINGS system native rsusb)
if(NOT CAMERA_SERVER_REALSENSE_BACKEND MATCHES "^(system|native|rsusb)$")
  message(FATAL_ERROR "CAMERA_SERVER_REALSENSE_BACKEND must be system, native, or rsusb")
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig QUIET)

set(CAMERA_SERVER_HAVE_REALSENSE 0)
if(NOT CAMERA_SERVER_FORCE_MOCK_CAMERA)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    set(CAMERA_SERVER_HAVE_REALSENSE 1)
  elseif(PkgConfig_FOUND)
    pkg_check_modules(REALSENSE2 QUIET realsense2)
    if(REALSENSE2_FOUND)
      set(CAMERA_SERVER_HAVE_REALSENSE 1)
    endif()
  endif()
endif()

set(CAMERA_SERVER_HAVE_ZMQ 0)
if(NOT CAMERA_SERVER_FORCE_ZMQ_STUB AND PkgConfig_FOUND)
  pkg_check_modules(LIBZMQ QUIET libzmq)
  if(LIBZMQ_FOUND)
    set(CAMERA_SERVER_HAVE_ZMQ 1)
  endif()
endif()

set(CAMERA_SERVER_HAVE_OPENCV 0)
if(NOT CAMERA_SERVER_FORCE_NO_OPENCV)
  find_package(OpenCV QUIET COMPONENTS imgproc videoio)
  if(OpenCV_FOUND)
    set(CAMERA_SERVER_HAVE_OPENCV 1)
  endif()
endif()

add_library(camera_server_lib
  src/core/clock.cpp
  src/config/config.cpp
  src/shm/shared_memory_ring.cpp
  src/sync/frame_synchronizer.cpp
  src/publish/metadata_publisher.cpp
  src/recording/recorder.cpp
  src/health/health_monitor.cpp
  src/camera/camera_manager.cpp
  src/camera/realsense_device.cpp
  src/camera/uvc_device.cpp
)

target_include_directories(camera_server_lib PUBLIC include)
target_link_libraries(camera_server_lib PUBLIC yaml-cpp Threads::Threads rt)
target_compile_definitions(camera_server_lib PUBLIC
  CAMERA_SERVER_HAVE_REALSENSE=$<BOOL:${CAMERA_SERVER_HAVE_REALSENSE}>
  CAMERA_SERVER_HAVE_ZMQ=$<BOOL:${CAMERA_SERVER_HAVE_ZMQ}>
  CAMERA_SERVER_HAVE_OPENCV=$<BOOL:${CAMERA_SERVER_HAVE_OPENCV}>
  CAMERA_SERVER_REALSENSE_BACKEND="${CAMERA_SERVER_REALSENSE_BACKEND}"
)

if(CAMERA_SERVER_HAVE_OPENCV)
  target_link_libraries(camera_server_lib PUBLIC ${OpenCV_LIBS})
  target_include_directories(camera_server_lib PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

if(CAMERA_SERVER_HAVE_REALSENSE)
  if(realsense2_FOUND)
    target_link_libraries(camera_server_lib PUBLIC realsense2::realsense2)
  else()
    target_include_directories(camera_server_lib PUBLIC ${REALSENSE2_INCLUDE_DIRS})
    target_link_libraries(camera_server_lib PUBLIC ${REALSENSE2_LIBRARIES})
    target_link_directories(camera_server_lib PUBLIC ${REALSENSE2_LIBRARY_DIRS})
  endif()
endif()

if(CAMERA_SERVER_HAVE_ZMQ)
  target_include_directories(camera_server_lib PUBLIC ${LIBZMQ_INCLUDE_DIRS})
  target_link_libraries(camera_server_lib PUBLIC ${LIBZMQ_LIBRARIES})
  target_link_directories(camera_server_lib PUBLIC ${LIBZMQ_LIBRARY_DIRS})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_server_lib PRIVATE -Wall -Wextra -Wpedantic)
  if(CAMERA_SERVER_WARNINGS_AS_ERRORS)
    target_compile_options(camera_server_lib PRIVATE -Werror)
  endif()
endif()

add_executable(camera_server src/main.cpp)
target_link_libraries(camera_server PRIVATE camera_server_lib)

if(CAMERA_SERVER_BUILD_TESTS)
  enable_testing()
  add_executable(camera_server_tests tests/test_camera_server.cpp)
  target_link_libraries(camera_server_tests PRIVATE camera_server_lib)
  # The test harness uses assert() for both execution and verification, so keep
  # assertions active even when the container image is built in Release mode.
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(camera_server_tests PRIVATE -UNDEBUG)
  endif()
  add_test(NAME camera_server_tests COMMAND camera_server_tests)
  set_tests_properties(camera_server_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

message(STATUS "camera_server: RealSense backend=${CAMERA_SERVER_HAVE_REALSENSE}, ZeroMQ backend=${CAMERA_SERVER_HAVE_ZMQ}, OpenCV/UVC backend=${CAMERA_SERVER_HAVE_OPENCV}")
