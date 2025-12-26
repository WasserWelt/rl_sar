// g++ -std=c++17 -O2 -I src/rl_sar/library/thirdparty/robot_sdk/wheel_leg/wheel_leg -lpthread -o check_serial check_serial.cpp
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include the SDK
#include "wheel_leg_sdk.hpp"

volatile std::sig_atomic_t g_signal_status = 0;

void signal_handler(int signal) {
    g_signal_status = signal;
}

// 辅助函数：四元数转欧拉角
struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(double q0, double q1, double q2, double q3) {
    EulerAngles angles;
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp);
    else
        angles.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

int main(int argc, char** argv) {
    // 默认端口，可根据实际情况修改
    std::string port = "/dev/ttyACM0"; 
    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "Starting Serial Check on " << port << "..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    std::signal(SIGINT, signal_handler);

    WheelLegSDK sdk;
    LowCmd cmd;
    LowState state;

    sdk.InitCmdData(cmd);

    try {
        sdk.InitSerial(port.c_str(), 460800); 
    } catch (...) {
        std::cerr << "Failed to initialize serial port." << std::endl;
        return 1;
    }

    int valid_packets = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    std::vector<unsigned char> last_raw_data;

    while (g_signal_status == 0) {
        // 周期性发送指令防止断连
        static int send_ctr = 0;
        if (send_ctr++ % 100 == 0) { 
             sdk.InitCmdData(cmd);
             sdk.Send(cmd);
        }

        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(sdk.fd, &rfd);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0; 

        int ret = select(sdk.fd + 1, &rfd, NULL, NULL, &timeout);
        
        if (ret > 0) {
            int received = sdk.Recv();
            if (received > 0) {
                last_raw_data.assign(sdk.recv_buff, sdk.recv_buff + received);
                sdk.AnalyzeData(sdk.recv_buff, state);
                valid_packets++;
                
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count() > 100) { // 10Hz刷新
                    last_print_time = now;

                    xRockerBtnDataStruct* rc = (xRockerBtnDataStruct*)state.wirelessRemote;
                    EulerAngles calc_rpy = ToEulerAngles(
                        state.imu.quaternion[0], state.imu.quaternion[1], 
                        state.imu.quaternion[2], state.imu.quaternion[3]
                    ); //wxyz

                    std::cout << "\033[2J\033[1;1H"; // 清屏
                    std::cout << "================ ROBOT STATE MONITOR ================" << std::endl;
                    std::cout << "Port: " << port << " | Valid Packets: " << valid_packets << std::endl;
                    
                    // 1. 遥控器全数据显示
                    std::cout << "-------------------- REMOTE -------------------------" << std::endl;
                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << "Joysticks:" << std::endl;
                    std::cout << "  LX: " << std::setw(6) << rc->lx << "  RX: " << std::setw(6) << rc->rx << std::endl;
                    std::cout << "  LY: " << std::setw(6) << rc->ly << "  RY: " << std::setw(6) << rc->ry << std::endl;
                    std::cout << "  L2 (Analog): " << std::setw(6) << rc->L2 << std::endl;
                    
                    std::cout << "Buttons:" << std::endl;
                    std::cout << "  [A]: " << (rc->btn.components.A ? "ON " : "OFF") << "  [B]: " << (rc->btn.components.B ? "ON " : "OFF") 
                              << "  [X]: " << (rc->btn.components.X ? "ON " : "OFF") << "  [Y]: " << (rc->btn.components.Y ? "ON " : "OFF") << std::endl;
                    std::cout << "  [L1]:" << (rc->btn.components.L1 ? "ON " : "OFF") << "  [R1]:" << (rc->btn.components.R1 ? "ON " : "OFF")
                              << "  [L2]:" << (rc->btn.components.L2 ? "ON " : "OFF") << "  [R2]:" << (rc->btn.components.R2 ? "ON " : "OFF") << std::endl;
                    std::cout << "  [UP]:" << (rc->btn.components.up ? "ON " : "OFF") << "  [DW]:" << (rc->btn.components.down ? "ON " : "OFF")
                              << "  [LE]:" << (rc->btn.components.left ? "ON " : "OFF") << "  [RI]:" << (rc->btn.components.right ? "ON " : "OFF") << std::endl;
                    std::cout << "  [F1]:" << (rc->btn.components.F1 ? "ON " : "OFF") << "  [F2]:" << (rc->btn.components.F2 ? "ON " : "OFF")
                              << "  [ST]:" << (rc->btn.components.start ? "ON " : "OFF") << "  [SE]:" << (rc->btn.components.select ? "ON " : "OFF") << std::endl;

                    // 2. IMU 和 电机简要信息
                    std::cout << "---------------------- IMU --------------------------" << std::endl;
                    std::cout << "Quat: " << state.imu.quaternion[0] << ", " << state.imu.quaternion[1] << ", " 
                              << state.imu.quaternion[2] << ", " << state.imu.quaternion[3] << std::endl;
                    std::cout << "RPY (Calc): R=" << calc_rpy.roll << " P=" << calc_rpy.pitch << " Y=" << calc_rpy.yaw << std::endl;
                    
                    std::cout << "-------------------- MOTORS -------------------------" << std::endl;
                    std::cout << "Pos: ";
                    for(int i=0; i<4; i++) std::cout << std::setw(7) << state.motorState[i].q << " ";
                    std::cout << std::endl;

                    std::cout << "\nRaw Hex (First 16 bytes): ";
                    std::cout << std::hex << std::setfill('0');
                    for (size_t i = 0; i < std::min((size_t)16, last_raw_data.size()); i++) {
                        std::cout << std::setw(2) << (int)last_raw_data[i] << " ";
                    }
                    std::cout << std::dec << std::setfill(' ') << std::endl;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return 0;
}