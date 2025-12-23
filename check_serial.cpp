// g++ -std=c++17 -O2  -I src/rl_sar/library/thirdparty/robot_sdk/wheel_leg/wheel_leg -lpthread -o check_serial
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include the SDK
#include "wheel_leg_sdk.hpp"

volatile std::sig_atomic_t g_signal_status = 0;

void signal_handler(int signal) {
    g_signal_status = signal;
}

int main(int argc, char** argv) {
    // std::string port = "/dev/ttyUSB0";
    std::string port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_206C326B4D31-if00";
    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "Starting Serial Check on " << port << "..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    std::signal(SIGINT, signal_handler);

    WheelLegSDK sdk;
    LowCmd cmd;
    LowState state;

    // Initialize command data
    sdk.InitCmdData(cmd);

    // Try to initialize serial port
    try {
        sdk.InitSerial(port.c_str(), 460800); 
    } catch (...) {
        std::cerr << "Failed to initialize serial port." << std::endl;
        return 1;
    }

    int valid_packets = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    while (g_signal_status == 0) {
        // Send a command periodically
        static int send_ctr = 0;
        if (send_ctr++ % 100 == 0) { 
             sdk.InitCmdData(cmd);
             sdk.Send(cmd);
             std::cout << "Sent command packet " << (send_ctr / 100) << std::endl; 
        }

        // Use select to check if data is available to read
        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(sdk.fd, &rfd);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0; 

        int ret = select(sdk.fd + 1, &rfd, NULL, NULL, &timeout);
        
        // if (ret > 0) {
        //     int received = sdk.Recv();
        //     if (received > 0) {
        //         // Parse locally to check validity and print stats
        //         sdk.AnalyzeData(sdk.recv_buff, state);
        //         valid_packets++;
        //         // Debug: Print Raw Hex
        //         std::cout << "\n[Packet " << valid_packets << "] Raw Hex (" << received << " bytes):" << std::endl;
        //         for (int i = 0; i < received; i++) {
        //             std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sdk.recv_buff[i] << " ";
        //             if ((i + 1) % 16 == 0) std::cout << std::endl;
        //         }
        //         std::cout << std::dec << std::endl;
        //         // Debug: Print Parsed Values
        //         std::cout << "  Time: " << state.motorState[0].q_raw << std::endl; // Assuming temp usage of q_raw for time debug? No, AnalyzeData maps time to tmp_time_from_mcu private var.
        //         // Let's print something we can access
        //         std::cout << "  Motors[0].q: " << state.motorState[0].q << std::endl;
        //         std::cout << "  Joy LX: " << (float)sdk.recv_buff[49] << " (Raw byte check needed)" << std::endl; // Hard to print struct internals from here without getters
        //         // We can't easily access private members of WheelLegSDK (like tmp_time_from_mcu)
        //         // but we can look at state.
        //     } else {
        //          // Print what we have in buffer if not a full packet (requires modifying SDK to peek, but standard read just returns 0)
        //          // If Recv returns 0, it means no *full valid* packet was found yet.
        //          // To debug raw stream, we would need to bypass Recv or add a debug print inside Recv.
        //     }
        // }
        if (ret > 0) {
            int received = sdk.Recv();
            if (received > 0) {
                sdk.AnalyzeData(sdk.recv_buff, state);
                valid_packets++;
                
                // 获取当前时间，检查是否需要刷新显示 (例如每 50ms 刷新一次)
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count() > 5) {
                    last_print_time = now;

                    float mcu_time = sdk.GetMCUTime();
                    xRockerBtnDataStruct* rc = (xRockerBtnDataStruct*)state.wirelessRemote;

                    // ================= 屏幕刷新与格式化的核心部分 =================
                    
                    // \033[2J 清屏, \033[1;1H 光标移动到左上角
                    std::cout << "\033[2J\033[1;1H"; 

                    std::cout << "================ ROBOT STATE MONITOR ================" << std::endl;
                    std::cout << "Port: " << port << " | Valid Packets: " << valid_packets << std::endl;
                    std::cout << "MCU Time: " << std::fixed << std::setprecision(4) << mcu_time << " s" << std::endl;
                    std::cout << "-----------------------------------------------------" << std::endl;

                    // 设置统一的数字格式：固定宽度(7)，保留3位小数，显示正负号
                    std::cout << std::fixed << std::setprecision(3) << std::showpos;

                    // 1. 电机数据 (表格化)
                    std::cout << "[Motors]     M0(FL)    M1(FR)    M2(RL)    M3(RR)" << std::endl;
                    std::cout << "  Pos (rad): ";
                    for(int i=0; i<4; i++) std::cout << std::setw(9) << state.motorState[i].q << " ";
                    std::cout << std::endl;

                    std::cout << "  Vel (r/s): ";
                    for(int i=0; i<4; i++) std::cout << std::setw(9) << state.motorState[i].dq << " ";
                    std::cout << std::endl;

                    std::cout << "-----------------------------------------------------" << std::endl;

                    // 2. IMU 数据
                    std::cout << "[IMU]" << std::endl;
                    std::cout << "  Quat (wxyz): " 
                              << std::setw(8) << state.imu.quaternion[0] 
                              << std::setw(8) << state.imu.quaternion[1] 
                              << std::setw(8) << state.imu.quaternion[2] 
                              << std::setw(8) << state.imu.quaternion[3] << std::endl;
                    
                    std::cout << "  Gyro (r/s) : " 
                              << std::setw(8) << state.imu.gyroscope[0] 
                              << std::setw(8) << state.imu.gyroscope[1] 
                              << std::setw(8) << state.imu.gyroscope[2] << std::endl;

                    std::cout << "  Accel(m/s2): " 
                              << std::setw(8) << state.imu.accelerometer[0] 
                              << std::setw(8) << state.imu.accelerometer[1] 
                              << std::setw(8) << state.imu.accelerometer[2] << std::endl;

                    std::cout << "-----------------------------------------------------" << std::endl;

                    // 3. 遥控器数据
                    std::cout << "[Remote]" << std::endl;
                    std::cout << "  Joy LX: " << std::setw(8) << rc->lx << std::endl;
                    
                    // 恢复默认显示设置 (取消 showpos)
                    std::cout << std::noshowpos << std::endl;
                    std::cout << "Press Ctrl+C to exit." << std::endl;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "\nStopped. Total valid packets: " << valid_packets << std::endl;

    return 0;
}

