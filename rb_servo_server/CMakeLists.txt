cmake_minimum_required(VERSION 3.16)
project(rb_servo_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RB_SERVO_ENABLE_RBPODO "Enable rbpodo backend" ON)
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
    src/kinematics/dh_calibration.cpp
    src/kinematics/box_dh_calibration.cpp
    src/robot/mock_backend.cpp
    src/robot/rbpodo_backend.cpp
    src/robot/backend_factory.cpp
    src/control/command_buffer.cpp
    src/control/trajectory_filter.cpp
    src/control/safety_filter.cpp
    src/control/fault_classifier.cpp
    src/control/servo_dispatcher.cpp
    src/control/arm_worker.cpp
    src/control/cartesian_controller.cpp
    src/control/queue_sync_controller.cpp
    src/control/admittance_overlay.cpp
    src/sensor/ft_pipeline.cpp
    src/control/cartesian_servo_controller.cpp
    src/control/cartesian_trajectory_planner.cpp
    src/control/smd_pose_tracker.cpp
    src/control/follower_output_smd.cpp
    src/control/chunk_window.cpp
    src/control/cartesian_chunk_follower.cpp
    src/control/delta_twist_follower.cpp
    src/math/se3.cpp
    src/control/collision_monitor.cpp
    src/control/init_motion_planner.cpp
    src/control/dual_arm_servo_loop.cpp
    src/network/chunk_frame_receiver.cpp
    src/network/command_server.cpp
    src/network/state_publisher.cpp
    src/network/scope_publisher.cpp
    src/network/gripper_bridge.cpp
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
# ruckig (online jerk-limited OTG) for the receding-horizon chunk-follower.
# Community edition, exports the ruckig::ruckig target. The system install ships
# with ROS (/opt/ros/*); add those prefixes so find_package resolves it there.
foreach(_ros_prefix /opt/ros/$ENV{ROS_DISTRO} /opt/ros/humble)
    if(IS_DIRECTORY "${_ros_prefix}")
        list(APPEND CMAKE_PREFIX_PATH "${_ros_prefix}")
    endif()
endforeach()
find_package(ruckig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
# coal (hpp-fcl successor) + pinocchio collision component: URDF mesh/box
# self-collision in the async CollisionMonitor. pinocchio targets export
# PINOCCHIO_WITH_HPP_FCL via INTERFACE_COMPILE_DEFINITIONS.
find_package(coal REQUIRED)
target_link_libraries(
    rb_servo_core
    PUBLIC
        Threads::Threads
        nlohmann_json::nlohmann_json
        yaml-cpp
        Eigen3::Eigen
        pinocchio::pinocchio
        pinocchio::pinocchio_collision
        coal::coal
        ruckig::ruckig
)
target_compile_definitions(rb_servo_core PUBLIC RB_SERVO_ENABLE_PINOCCHIO=1)

if(RB_SERVO_ENABLE_RBPODO)
    find_package(rbpodo REQUIRED)
    target_link_libraries(rb_servo_core PUBLIC rbpodo::rbpodo)
endif()

add_executable(rb_servo_server src/main.cpp)
target_link_libraries(rb_servo_server PRIVATE rb_servo_core)

add_executable(rb_isaac_bridge src/isaac_bridge_main.cpp)
target_link_libraries(rb_isaac_bridge PRIVATE rb_servo_core)

# config.hpp contains layout-sensitive runtime/safety structs shared across the
# server. Make every shipped server object depend on it explicitly so an
# incremental build recompiles the complete ABI surface without deleting the
# build tree. tools/build_stack.sh keeps a post-link timestamp guard as a
# fail-closed backstop.
set(RB_SERVO_LAYOUT_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/rb_servo/config/config.hpp"
)
get_target_property(RB_SERVO_CORE_SOURCES rb_servo_core SOURCES)
set_property(
    SOURCE ${RB_SERVO_CORE_SOURCES} src/main.cpp
    APPEND PROPERTY OBJECT_DEPENDS "${RB_SERVO_LAYOUT_HEADERS}"
)

# Offline IK-feasibility grid sampler (feeds tools/ik_infeasible_region.py, which
# turns the reachability grid into the viser "IK 불가 영역" overlay mesh). Reuses
# the real Pinocchio IK solver in rb_servo_core for fidelity.
add_executable(ik_feasibility_grid tools/ik_feasibility_grid.cpp)
target_link_libraries(ik_feasibility_grid PRIVATE rb_servo_core)

# Offline chunk-follower replay + limit sweep. Feeds a recorded absolute knot
# stream through the real CartesianChunkFollower so the Ruckig segment-duration
# budget can be characterised without hardware.
add_executable(follower_replay tools/follower_replay.cpp)
target_link_libraries(follower_replay PRIVATE rb_servo_core)

if(RB_SERVO_ENABLE_RBPODO)
    # READ-ONLY controller state reader, for hand-parked geometry checks (stand and
    # mount verification) performed with a person inside the workspace.
    #
    # Links rbpodo::rbpodo ONLY -- deliberately NOT rb_servo_core. The rbpodo SDK keeps
    # every command entry point on rb::podo::Cobot and the data port on
    # rb::podo::CobotData; this target names only the latter, and by leaving the servo
    # core out of the link line the backend/dispatcher are not in the binary either. The
    # property is checkable:
    #     nm -C rbpodo_read_state | grep -E 'move_j|servo_j|Cobot<'   # must be empty
    # Do not link rb_servo_core here and do not add motion helpers to this target;
    # scripts/send_*.py is where anything that moves belongs.
    add_executable(rbpodo_read_state tools/rbpodo_read_state.cpp)
    target_link_libraries(rbpodo_read_state PRIVATE rbpodo::rbpodo)
endif()

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

    # tests/test_safety_filter.cpp existed but was never registered, so the joint
    # clamp stack (joint-limit / velocity / acceleration) had no test in ctest.
    add_executable(test_safety_filter tests/test_safety_filter.cpp)
    target_link_libraries(test_safety_filter PRIVATE rb_servo_core)
    add_test(NAME safety_filter COMMAND test_safety_filter)

    add_executable(test_config_loader tests/test_config_loader.cpp)
    target_link_libraries(test_config_loader PRIVATE rb_servo_core)
    add_test(NAME config_loader COMMAND test_config_loader)

    add_executable(test_config tests/test_config.cpp)
    target_link_libraries(test_config PRIVATE rb_servo_core)
    add_test(NAME config COMMAND test_config)

    add_executable(test_gripper_bridge tests/test_gripper_bridge.cpp)
    target_link_libraries(test_gripper_bridge PRIVATE rb_servo_core)
    add_test(NAME gripper_bridge COMMAND test_gripper_bridge)

    add_executable(test_scope_publisher tests/test_scope_publisher.cpp)
    target_link_libraries(test_scope_publisher PRIVATE rb_servo_core)
    add_test(NAME scope_publisher COMMAND test_scope_publisher)

    add_executable(test_state_publisher tests/test_state_publisher.cpp)
    target_link_libraries(test_state_publisher PRIVATE rb_servo_core)
    add_test(NAME state_publisher COMMAND test_state_publisher)

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

    add_executable(test_collision_monitor tests/test_collision_monitor.cpp)
    target_link_libraries(test_collision_monitor PRIVATE rb_servo_core)
    add_test(NAME collision_monitor COMMAND test_collision_monitor)

    add_executable(test_init_motion_planner tests/test_init_motion_planner.cpp)
    target_link_libraries(test_init_motion_planner PRIVATE rb_servo_core)
    add_test(NAME init_motion_planner COMMAND test_init_motion_planner)

    add_executable(test_init_motion_pursuit tests/test_init_motion_pursuit.cpp)
    target_link_libraries(test_init_motion_pursuit PRIVATE rb_servo_core)
    add_test(NAME init_motion_pursuit COMMAND test_init_motion_pursuit)

    add_executable(test_floor_constraint tests/test_floor_constraint.cpp)
    target_link_libraries(test_floor_constraint PRIVATE rb_servo_core)
    add_test(NAME floor_constraint COMMAND test_floor_constraint)

    add_executable(test_roi_box tests/test_roi_box.cpp)
    target_link_libraries(test_roi_box PRIVATE rb_servo_core)
    add_test(NAME roi_box COMMAND test_roi_box)

    add_executable(test_reach_constraint tests/test_reach_constraint.cpp)
    target_link_libraries(test_reach_constraint PRIVATE rb_servo_core)
    add_test(NAME reach_constraint COMMAND test_reach_constraint)

    add_executable(test_plan_gate tests/test_plan_gate.cpp)
    target_link_libraries(test_plan_gate PRIVATE rb_servo_core)
    add_test(NAME plan_gate COMMAND test_plan_gate)

    add_executable(test_projection_release tests/test_projection_release.cpp)
    target_link_libraries(test_projection_release PRIVATE rb_servo_core)
    add_test(NAME projection_release COMMAND test_projection_release)

    add_executable(test_user_floor_constraint tests/test_user_floor_constraint.cpp)
    target_link_libraries(test_user_floor_constraint PRIVATE rb_servo_core)
    add_test(NAME user_floor_constraint COMMAND test_user_floor_constraint)

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

    add_executable(test_follower_output_smd tests/test_follower_output_smd.cpp)
    target_link_libraries(test_follower_output_smd PRIVATE rb_servo_core)
    add_test(NAME follower_output_smd COMMAND test_follower_output_smd)

    add_executable(test_joint_smd_tracker tests/test_joint_smd_tracker.cpp)
    target_link_libraries(test_joint_smd_tracker PRIVATE rb_servo_core)
    add_test(NAME joint_smd_tracker COMMAND test_joint_smd_tracker)

    add_executable(test_chunk_follower_core tests/test_chunk_follower_core.cpp)
    target_link_libraries(test_chunk_follower_core PRIVATE rb_servo_core)
    add_test(NAME chunk_follower_core COMMAND test_chunk_follower_core)

    add_executable(test_chunk_window tests/test_chunk_window.cpp)
    target_link_libraries(test_chunk_window PRIVATE rb_servo_core)
    add_test(NAME chunk_window COMMAND test_chunk_window)

    add_executable(test_cartesian_chunk_follower tests/test_cartesian_chunk_follower.cpp)
    target_link_libraries(test_cartesian_chunk_follower PRIVATE rb_servo_core)
    add_test(NAME cartesian_chunk_follower COMMAND test_cartesian_chunk_follower)

    add_executable(test_delta_twist_follower tests/test_delta_twist_follower.cpp)
    target_link_libraries(test_delta_twist_follower PRIVATE rb_servo_core)
    add_test(NAME delta_twist_follower COMMAND test_delta_twist_follower)

    add_executable(test_chunk_frame_receiver tests/test_chunk_frame_receiver.cpp)
    target_link_libraries(test_chunk_frame_receiver PRIVATE rb_servo_core)
    add_test(NAME chunk_frame_receiver COMMAND test_chunk_frame_receiver)

    add_executable(test_joint_moving_average tests/test_joint_moving_average.cpp)
    target_link_libraries(test_joint_moving_average PRIVATE rb_servo_core)
    add_test(NAME joint_moving_average COMMAND test_joint_moving_average)

    add_executable(test_servo_logger_columns tests/test_servo_logger_columns.cpp)
    target_link_libraries(test_servo_logger_columns PRIVATE rb_servo_core)
    add_test(NAME servo_logger_columns COMMAND test_servo_logger_columns)

    add_executable(test_queue_sync_controller tests/test_queue_sync_controller.cpp)
    target_link_libraries(test_queue_sync_controller PRIVATE rb_servo_core)
    add_test(NAME queue_sync_controller COMMAND test_queue_sync_controller)

    add_executable(test_setpoint_interpolator tests/test_setpoint_interpolator.cpp)
    target_link_libraries(test_setpoint_interpolator PRIVATE rb_servo_core)
    add_test(NAME setpoint_interpolator COMMAND test_setpoint_interpolator)

    add_executable(test_force_control tests/test_force_control.cpp)
    target_link_libraries(test_force_control PRIVATE rb_servo_core)
    add_test(NAME force_control COMMAND test_force_control)

    add_executable(test_dh_calibration tests/test_dh_calibration.cpp)
    target_link_libraries(test_dh_calibration PRIVATE rb_servo_core)
    target_compile_definitions(test_dh_calibration PRIVATE RB_SERVO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME dh_calibration COMMAND test_dh_calibration)

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
    else()
        message(WARNING "Python3 interpreter not found; rb_servo Python unittests will not be registered")
    endif()
endif()
