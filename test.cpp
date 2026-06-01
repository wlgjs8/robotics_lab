#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <array>
#include <cmath>
#include <iomanip>
#include <algorithm> // std::max 사용을 위해 추가
#include <rbpodo/rbpodo.hpp> 

using namespace rb::podo;

// ==========================================
// 1. 안전 및 설정 파라미터 
// ==========================================
const std::string ROBOT_IP = "172.28.60.100";
const double DELTA_THETA_DEG = 50.0;    // 목표 변위

// [안전 설정] V_MAX를 높여도 내부적으로 안전하게 재계산됩니다.
const double V_MAX_DEG_S = 20.0;        // 설정 최고 속도
const double A_MAX_DEG_S2 = 15.0;       // 설정 가속도

// ==========================================
// 2. 동적 제어 주기(Frequency) 설정
// ==========================================
const double LOGGING_CYCLE_SEC = 0.002; // 메인 루프 및 로깅 주기 (고정 2ms / 500Hz)

// 사용자가 원하는 제어 주기를 자유롭게 입력하십시오. (예: 0.008, 0.010, 0.015 등)
const double TARGET_COMMAND_CYCLE_SEC = 0.010;  

// [안전장치] 입력된 제어 주기를 2ms(LOGGING_CYCLE_SEC)의 정수배로 자동 반올림 보정합니다.
// 이렇게 하면 t1 파라미터와 실제 루프 타이밍 간의 오차가 발생하여 M568 에러가 발생하는 것을 방지합니다.
const int COMMAND_INTERVAL_STEPS = std::max(1, static_cast<int>(std::round(TARGET_COMMAND_CYCLE_SEC / LOGGING_CYCLE_SEC))); 
const double ACTUAL_COMMAND_CYCLE_SEC = COMMAND_INTERVAL_STEPS * LOGGING_CYCLE_SEC; // 실제 로봇에 인가되는 t1 시간

int main() {
    try {
        Cobot control_channel(ROBOT_IP);
        CobotData data_channel(ROBOT_IP);
        ResponseCollector rc; 
        
        std::ofstream outfile("joint_ft_safe_speed_log_2ms.csv");
        if (!outfile.is_open()) {
            std::cerr << "로그 파일을 생성할 수 없습니다." << std::endl;
            return -1;
        }
        
        outfile << std::fixed << std::setprecision(7);
        
        // CSV 헤더 작성
        outfile << "Time,Command_Flag,";
        for(int i=0; i<6; ++i) outfile << "Ref_J" << i << ",";
        for(int i=0; i<6; ++i) outfile << "Ang_J" << i << ",";
        outfile << "TCP_Pos_0,TCP_Pos_1,TCP_Pos_2,Fx,Fy,Fz,Mx,My,Mz,Robot_State,Init_State\n";

        std::cout << "초기 상태를 수신 중입니다..." << std::endl;
        
        SystemState initial_state;
        bool state_received = false;
        for (int retry = 0; retry < 5; ++retry) {
            auto state_opt = data_channel.request_data(0.5);
            if (state_opt.has_value()) {
                initial_state = state_opt.value();
                state_received = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!state_received) {
            std::cerr << "초기 로봇 상태를 가져오는데 실패했습니다." << std::endl;
            return -1;
        }

        // ==========================================================
        // 3. Target A / B 판별 로직
        // ==========================================================
        std::array<double, 6> start_joints;
        for(int i=0; i<6; ++i) start_joints[i] = initial_state.sdata.jnt_ang[i];
        
        std::array<double, 6> target_A = {-317.84, 3.33, -93.37, 46.14, -30.45, -9.05};
        std::array<double, 6> target_B = target_A;
        target_B[0] += DELTA_THETA_DEG; 

        double current_j0 = start_joints[0];
        std::array<double, 6> final_target = (std::abs(current_j0 - target_A[0]) > std::abs(current_j0 - target_B[0])) ? target_A : target_B;

        // ==========================================================
        // 4. 사다리꼴/삼각형 궤적 동적 계산
        // ==========================================================
        double D = final_target[0] - start_joints[0]; 
        double sign = (D > 0) ? 1.0 : -1.0;
        double abs_D = std::abs(D);
        
        double t_accel, t_decel, t_cruise, cruise_dist, accel_dist;
        double actual_v_max = V_MAX_DEG_S;

        double required_dist_for_max_v = (V_MAX_DEG_S * V_MAX_DEG_S) / A_MAX_DEG_S2;
        
        if (abs_D < required_dist_for_max_v) {
            actual_v_max = std::sqrt(abs_D * A_MAX_DEG_S2);
            t_accel = actual_v_max / A_MAX_DEG_S2;
            t_decel = t_accel;
            accel_dist = 0.5 * A_MAX_DEG_S2 * t_accel * t_accel;
            t_cruise = 0.0;
            cruise_dist = 0.0;
            std::cout << "\n[알림] 설정한 V_MAX(" << V_MAX_DEG_S << ")에 도달하기엔 거리가 짧아 속도가 " << actual_v_max << " deg/s 로 자동 하향 조정됩니다.\n" << std::endl;
        } else {
            t_accel = V_MAX_DEG_S / A_MAX_DEG_S2; 
            t_decel = t_accel;
            accel_dist = 0.5 * A_MAX_DEG_S2 * t_accel * t_accel;
            cruise_dist = abs_D - (2.0 * accel_dist);
            t_cruise = cruise_dist / V_MAX_DEG_S;
        }
        
        double MOTION_TIME_SEC = t_accel + t_cruise + t_decel; 
        double LOGGING_TIME_SEC = MOTION_TIME_SEC + 2.0; 
        const int TOTAL_STEPS = static_cast<int>(LOGGING_TIME_SEC / LOGGING_CYCLE_SEC);
        
        std::cout << "--- 궤적 및 제어 파라미터 요약 ---" << std::endl;
        std::cout << "목표 이동 거리: " << D << " 도" << std::endl;
        std::cout << "실제 도달 속도: " << actual_v_max << " deg/s" << std::endl;
        std::cout << "총 예상 시간  : " << MOTION_TIME_SEC << " 초" << std::endl;
        std::cout << "목표 제어 주기: " << TARGET_COMMAND_CYCLE_SEC << " 초" << std::endl;
        std::cout << "실제 제어 주기: " << ACTUAL_COMMAND_CYCLE_SEC << " 초 (" << (1.0/ACTUAL_COMMAND_CYCLE_SEC) << " Hz 적용)" << std::endl;
        std::cout << "----------------------------------" << std::endl;

        control_channel.set_operation_mode(rc, OperationMode::Real);

        std::cout << "\n동적 주기 분할 제어를 시작합니다..." << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        auto next_wake_time = start_time;

        // ==========================================================
        // 5. 메인 루프 (500Hz 블로킹 대기 보장)
        // ==========================================================
        for (int step = 0; step <= TOTAL_STEPS; ++step) {
            next_wake_time += std::chrono::microseconds(2000); 
            double t = step * LOGGING_CYCLE_SEC;
            int command_flag = 0; 

            // [A] 보정된 명령 주기에 따른 실행 블록
            if (step % COMMAND_INTERVAL_STEPS == 0) {
                command_flag = 1; 
                std::array<double, 6> current_target = start_joints; 

                // 궤적 연산 (시간 t에 따른 완벽한 위치 도출)
                if (t <= t_accel) {
                    current_target[0] = start_joints[0] + sign * (0.5 * A_MAX_DEG_S2 * t * t);
                } 
                else if (t <= t_accel + t_cruise) {
                    double dt = t - t_accel;
                    current_target[0] = start_joints[0] + sign * (accel_dist + actual_v_max * dt);
                } 
                else if (t <= t_accel + t_cruise + t_decel) {
                    double dt = t - (t_accel + t_cruise);
                    current_target[0] = start_joints[0] + sign * (accel_dist + cruise_dist + actual_v_max * dt - 0.5 * A_MAX_DEG_S2 * dt * dt);
                } 
                else {
                    current_target[0] = final_target[0];
                }

                // [중요] 사용자가 입력한 TARGET 주기가 아닌, 수학적으로 동기화된 ACTUAL 주기를 전송합니다.
                control_channel.move_servo_j(rc, current_target, ACTUAL_COMMAND_CYCLE_SEC, 0.05, 0.5, 0.5);
            }

            // [B] 데이터 수신 및 로깅 블록: 매 2ms 실행
            auto state_opt = data_channel.request_data(0.001); 
            
            if (state_opt.has_value()) {
                SystemState state = state_opt.value(); 
                auto now = std::chrono::steady_clock::now();
                double log_time = std::chrono::duration<double>(now - start_time).count();

                outfile << log_time << "," << command_flag << ","; 
                for(int j = 0; j < 6; ++j) outfile << state.sdata.jnt_ref[j] << ","; 
                for(int j = 0; j < 6; ++j) outfile << state.sdata.jnt_ang[j] << ","; 
                
                outfile << state.sdata.tcp_pos[0] << "," << state.sdata.tcp_pos[1] << "," << state.sdata.tcp_pos[2] << ","
                        << state.sdata.eft_fx << "," << state.sdata.eft_fy << "," << state.sdata.eft_fz << ","
                        << state.sdata.eft_mx << "," << state.sdata.eft_my << "," << state.sdata.eft_mz << ","
                        << state.sdata.robot_state << "," << state.sdata.init_state_info << "\n";
            }

            std::this_thread::sleep_until(next_wake_time);
        }

        outfile.close();
        std::cout << "안전 모션 및 데이터 로깅 프로세스가 정상 종료되었습니다." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "런타임 에러 발생: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}