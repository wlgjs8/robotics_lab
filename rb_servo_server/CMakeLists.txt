cmake_minimum_required(VERSION 3.16)
project(rb_servo_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RB_SERVO_ENABLE_RBPODO "Enable rbpodo backend" OFF)
option(RB_SERVO_ALLOW_FETCHCONTENT "Allow CMake to download missing build dependencies" ON)

add_library(rb_servo_core
    src/core/types.cpp
    src/core/clock.cpp
    src/core/realtime.cpp
    src/config/config.cpp
    src/robot/mock_backend.cpp
    src/robot/rbpodo_backend.cpp
    src/robot/rbsim_backend.cpp
    src/robot/backend_factory.cpp
    src/control/command_buffer.cpp
    src/control/trajectory_filter.cpp
    src/control/safety_filter.cpp
    src/control/cartesian_controller.cpp
    src/control/force_controller.cpp
    src/control/dual_arm_servo_loop.cpp
    src/network/command_server.cpp
    src/network/state_publisher.cpp
    src/sensor/mock_force_torque_sensor.cpp
    src/logging/servo_logger.cpp
)

target_include_directories(rb_servo_core PUBLIC include)

target_compile_definitions(rb_servo_core PRIVATE
    $<$<BOOL:${RB_SERVO_ENABLE_RBPODO}>:RB_SERVO_ENABLE_RBPODO=1>
)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    if(NOT RB_SERVO_ALLOW_FETCHCONTENT)
        message(FATAL_ERROR "nlohmann_json was not found and RB_SERVO_ALLOW_FETCHCONTENT=OFF. Install nlohmann_json or set CMAKE_PREFIX_PATH to a local package prefix.")
    endif()
    include(FetchContent)
    FetchContent_Declare(
        json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(json)
endif()

find_package(Threads REQUIRED)
target_link_libraries(rb_servo_core PUBLIC Threads::Threads nlohmann_json::nlohmann_json)

add_executable(rb_servo_server src/main.cpp)
target_link_libraries(rb_servo_server PRIVATE rb_servo_core)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_safety_policy tests/test_safety_policy.cpp)
    target_link_libraries(test_safety_policy PRIVATE rb_servo_core)
    add_test(NAME safety_policy COMMAND test_safety_policy)

    set(RB_GUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rb_gui")
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        if(EXISTS "${RB_GUI_DIR}/tests")
            add_test(
                NAME rb_servo_gui_unittest
                COMMAND
                    ${Python3_EXECUTABLE}
                    -m unittest discover
                    -s ${RB_GUI_DIR}/tests
                    -p test_*.py
            )
            set_tests_properties(
                rb_servo_gui_unittest
                PROPERTIES
                    ENVIRONMENT "PYTHONPATH=${RB_GUI_DIR}"
            )
        else()
            message(WARNING "rb_gui tests not found at ${RB_GUI_DIR}/tests; rb_servo_gui_unittest will not be registered")
        endif()
        add_test(
            NAME rb_servo_tools_unittest
            COMMAND
                ${Python3_EXECUTABLE}
                -m unittest discover
                -s ${CMAKE_CURRENT_SOURCE_DIR}/tests
                -p test_*.py
        )
        add_test(
            NAME rbsim_hardware_free_gate
            COMMAND
                ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_rbsim_hardware_free_gate.py
        )
        set_tests_properties(
            rbsim_hardware_free_gate
            PROPERTIES
                ENVIRONMENT
                    "RB_SERVO_SERVER_BIN=$<TARGET_FILE:rb_servo_server>;PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../rb_simulator/src:${CMAKE_CURRENT_SOURCE_DIR}/tools"
        )
    else()
        message(WARNING "Python3 interpreter not found; rb_servo Python unittests will not be registered")
    endif()
endif()
