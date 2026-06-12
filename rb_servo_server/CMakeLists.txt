cmake_minimum_required(VERSION 3.16)
project(rb_servo_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RB_SERVO_ENABLE_RBPODO "Enable rbpodo backend" OFF)
option(RB_SERVO_ENABLE_PINOCCHIO "Enable Pinocchio FK/IK support" ON)
option(RB_SERVO_ALLOW_FETCHCONTENT "Allow CMake to download missing build dependencies" ON)

if(NOT RB_SERVO_ENABLE_PINOCCHIO)
    message(FATAL_ERROR "Disabling Pinocchio is no longer supported. rb_servo_server Cartesian math requires Eigen3 and Pinocchio.")
endif()

add_library(rb_servo_core
    src/core/types.cpp
    src/core/clock.cpp
    src/core/realtime.cpp
    src/core/shutdown.cpp
    src/robot/backend_result.cpp
    src/config/config.cpp
    src/kinematics/ik_solver.cpp
    src/kinematics/pinocchio_kinematics.cpp
    src/robot/mock_backend.cpp
    src/robot/rbpodo_backend.cpp
    src/robot/rbsim_backend.cpp
    src/robot/backend_factory.cpp
    src/control/command_buffer.cpp
    src/control/trajectory_filter.cpp
    src/control/safety_filter.cpp
    src/control/fault_classifier.cpp
    src/control/servo_dispatcher.cpp
    src/control/arm_worker.cpp
    src/control/cartesian_controller.cpp
    src/control/cartesian_servo_controller.cpp
    src/control/cartesian_trajectory_planner.cpp
    src/control/smd_pose_tracker.cpp
    src/math/se3.cpp
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

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
if(IS_DIRECTORY "/opt/openrobots")
    list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
target_link_libraries(
    rb_servo_core
    PUBLIC
        Threads::Threads
        nlohmann_json::nlohmann_json
        yaml-cpp
        Eigen3::Eigen
        pinocchio::pinocchio
)
target_compile_definitions(rb_servo_core PUBLIC RB_SERVO_ENABLE_PINOCCHIO=1)

if(RB_SERVO_ENABLE_RBPODO)
    find_package(rbpodo REQUIRED)
    target_link_libraries(rb_servo_core PUBLIC rbpodo::rbpodo)
endif()

add_executable(rb_servo_server src/main.cpp)
target_link_libraries(rb_servo_server PRIVATE rb_servo_core)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_backend_result tests/test_backend_result.cpp)
    target_link_libraries(test_backend_result PRIVATE rb_servo_core)
    add_test(NAME backend_result COMMAND test_backend_result)

    add_executable(test_fault_classifier tests/test_fault_classifier.cpp)
    target_link_libraries(test_fault_classifier PRIVATE rb_servo_core)
    add_test(NAME fault_classifier COMMAND test_fault_classifier)

    add_executable(test_servo_dispatcher tests/test_servo_dispatcher.cpp)
    target_link_libraries(test_servo_dispatcher PRIVATE rb_servo_core)
    add_test(NAME servo_dispatcher COMMAND test_servo_dispatcher)

    add_executable(test_arm_worker tests/test_arm_worker.cpp)
    target_link_libraries(test_arm_worker PRIVATE rb_servo_core)
    add_test(NAME arm_worker COMMAND test_arm_worker)

    add_executable(test_safety_policy tests/test_safety_policy.cpp)
    target_link_libraries(test_safety_policy PRIVATE rb_servo_core)
    add_test(NAME safety_policy COMMAND test_safety_policy)

    add_executable(test_config_loader tests/test_config_loader.cpp)
    target_link_libraries(test_config_loader PRIVATE rb_servo_core)
    add_test(NAME config_loader COMMAND test_config_loader)

    add_executable(test_config tests/test_config.cpp)
    target_link_libraries(test_config PRIVATE rb_servo_core)
    add_test(NAME config COMMAND test_config)

    add_executable(test_rbpodo_backend tests/test_rbpodo_backend.cpp)
    target_link_libraries(test_rbpodo_backend PRIVATE rb_servo_core)
    add_test(NAME rbpodo_backend COMMAND test_rbpodo_backend)

    add_executable(test_kinematics_fk tests/test_kinematics_fk.cpp)
    target_link_libraries(test_kinematics_fk PRIVATE rb_servo_core)
    add_test(NAME kinematics_fk COMMAND test_kinematics_fk)

    add_executable(test_kinematics_ik tests/test_kinematics_ik.cpp)
    target_link_libraries(test_kinematics_ik PRIVATE rb_servo_core)
    add_test(NAME kinematics_ik COMMAND test_kinematics_ik)

    add_executable(test_se3_math tests/test_se3_math.cpp)
    target_link_libraries(test_se3_math PRIVATE rb_servo_core)
    add_test(NAME se3_math COMMAND test_se3_math)

    add_executable(test_capsule_distance tests/test_capsule_distance.cpp)
    target_link_libraries(test_capsule_distance PRIVATE rb_servo_core)
    add_test(NAME capsule_distance COMMAND test_capsule_distance)

    add_executable(test_self_collision tests/test_self_collision.cpp)
    target_link_libraries(test_self_collision PRIVATE rb_servo_core)
    add_test(NAME self_collision COMMAND test_self_collision)

    add_executable(test_floor_constraint tests/test_floor_constraint.cpp)
    target_link_libraries(test_floor_constraint PRIVATE rb_servo_core)
    add_test(NAME floor_constraint COMMAND test_floor_constraint)

    add_executable(test_cartesian_planner tests/test_cartesian_planner.cpp)
    target_link_libraries(test_cartesian_planner PRIVATE rb_servo_core)
    add_test(NAME cartesian_planner COMMAND test_cartesian_planner)

    add_executable(test_cartesian_servo_controller tests/test_cartesian_servo_controller.cpp)
    target_link_libraries(test_cartesian_servo_controller PRIVATE rb_servo_core)
    add_test(NAME cartesian_servo_controller COMMAND test_cartesian_servo_controller)

    add_executable(test_command_source_lease tests/test_command_source_lease.cpp)
    target_link_libraries(test_command_source_lease PRIVATE rb_servo_core)
    add_test(NAME command_source_lease COMMAND test_command_source_lease)

    add_executable(test_smd_pose_tracker tests/test_smd_pose_tracker.cpp)
    target_link_libraries(test_smd_pose_tracker PRIVATE rb_servo_core)
    add_test(NAME smd_pose_tracker COMMAND test_smd_pose_tracker)

    add_executable(test_joint_smd_tracker tests/test_joint_smd_tracker.cpp)
    target_link_libraries(test_joint_smd_tracker PRIVATE rb_servo_core)
    add_test(NAME joint_smd_tracker COMMAND test_joint_smd_tracker)

    add_executable(test_joint_moving_average tests/test_joint_moving_average.cpp)
    target_link_libraries(test_joint_moving_average PRIVATE rb_servo_core)
    add_test(NAME joint_moving_average COMMAND test_joint_moving_average)

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
