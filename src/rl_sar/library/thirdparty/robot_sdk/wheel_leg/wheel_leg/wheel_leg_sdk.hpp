/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef WHEEL_LEG_SDK_HPP
#define WHEEL_LEG_SDK_HPP

#include <iostream>
#include <vector>
#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <cmath>
#include "joystick.h"
#include "comm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class WheelLegSDK
{
private:
    std::vector<unsigned char> internal_buffer;
    float tmp_time_from_mcu = -1;
    int show_rc = 0;
    int ctr = 0;
    float t0 = -1;
    int n_cpu = 0;
    int n_run = 0;

    float read_float(unsigned char *buff, int *idx);
    float read_from1byte(unsigned char *buff, int *idx, float v_min, float v_max);
    float read_from2bytes(unsigned char *buff, int *idx, float v_min, float v_max);
    void write_into2bytes(float value, unsigned char *buff, int *idx, float v_min, float v_max);
    float read_byte(unsigned char *buff, int *idx);
    float Vector3_Dot(Vector3 v1, Vector3 v2);
    Vector3 Vector3_Cross(Vector3 u, Vector3 v);
    float Sign(float value);
    float Vector3_invSqrt(Vector3 v);
    Vector3 Vector3_Direction(Vector3 v);
    float Clamp(float value, float min, float max);
    float Angle_vA_2_vB(Vector3 vA, Vector3 vB, Vector3 axle);
    void Vector3_Normalize(Vector3 *v);
    Vector3 Quaternion_Transform(float vx, float vy, float vz, float qx, float qy, float qz, float qw);

public:
    WheelLegSDK() {};
    ~WheelLegSDK() {};
    int fd;
    int recv_len = 0;
    int ex_send_recv = -1;
    unsigned char sent_buff[256];
    unsigned char recv_buff[512];
    void InitSerial(const char *port_name, int baud_rate);
    int Recv();
    void Send(LowCmd &lowCmd);
    void AnalyzeData(unsigned char *recv_buff, LowState &lowState);
    void PrintMCU(int running_state);
    void InitCmdData(LowCmd &cmd);
    float GetMCUTime() { return tmp_time_from_mcu; } // 添加这个函数

private:
    // CRC8 lookup table could be used for speed, but simple implementation is fine for now
    unsigned char CalcCRC8(unsigned char *data, int len)
    {
        unsigned char crc = 0x00;
        unsigned char poly = 0x31; // CRC-8-Maxim
        
        for (int i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
};

void WheelLegSDK::InitCmdData(LowCmd &cmd)
{
    for (int i = 0; i < 6; ++i)
    {
        cmd.motorCmd[i].mode = 0;
        cmd.motorCmd[i].q = 0.0;
        cmd.motorCmd[i].dq = 0.0;
        cmd.motorCmd[i].tau = 0.0;
        cmd.motorCmd[i].Kp = 0.0;
        cmd.motorCmd[i].Kd = 0.0;
    }
    memset(cmd.wirelessRemote, 0, sizeof(cmd.wirelessRemote));
}

void WheelLegSDK::PrintMCU(int running_state)
{
    float t1 = tmp_time_from_mcu;
    if (t1 < t0)
        t0 = t1;

    float ms = (t1 - t0) * 1000.0;
    n_cpu++;
    n_run++;

    if (ms > 1000)
    {
        std::cout << "mcu time = " << tmp_time_from_mcu << ",   n_cpu = " << n_cpu << ", runningState= " << running_state << std::endl;
        t0 = t1;
        n_cpu = 0;
        n_run = 0;
    }
    else
    {
        if (n_run > 50)
        {
            std::cout << "  dull ms= " << ms << ",  n_cpu = " << n_cpu << ", runningState= " << running_state << std::endl;
            n_run = 0;
        }
    }
}


void WheelLegSDK::Send(LowCmd &lowCmd)
{
    int idx = 0;
    
    // [MODIFIED] Add Header 0xFE at the beginning
    sent_buff[idx++] = 0xFE;

    for (int i = 0; i < 6; i++)
    {
        // [MODIFIED] Wheel Motors are 0 and 1 (as requested)
        // Protocol: 
        // Wheels (0,1): Send [dq, Kp, Kd]
        // Joints (2-5): Send [q, Kp, Kd]
        // Total bytes per motor: 6 bytes. Total payload: 36 bytes.
        
        if (i == 0 || i == 1)
        {
            // Wheel: Speed Control
            write_into2bytes(lowCmd.motorCmd[i].dq, sent_buff, &idx, -200, 200);
        }
        else
        {
            // Joint: Position Control
            write_into2bytes(lowCmd.motorCmd[i].q, sent_buff, &idx, -3.2, 3.2);
        }
        
        write_into2bytes(lowCmd.motorCmd[i].Kp, sent_buff, &idx, 0, 1000);
        write_into2bytes(lowCmd.motorCmd[i].Kd, sent_buff, &idx, 0, 1000);
        
        // Removed tau as requested to strictly match "use dq kp kd / q kp kd"
    }
    
    // [MODIFIED] Calculate CRC
    unsigned char crc = CalcCRC8(sent_buff, idx);
    sent_buff[idx++] = crc;

    int ret = write(fd, sent_buff, idx);
    if (ret < 0) {
        perror("write serial error");
    }
}

void WheelLegSDK::InitSerial(const char *port_name, int baud_rate)
{
    fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        std::cout << "open_port: Unable to open " << port_name << std::endl;
        perror("open_port");
        exit(1);
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    struct termios options;
    tcgetattr(fd, &options);

    speed_t baud;
    switch (baud_rate)
    {
    case 115200: baud = B115200; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default: baud = B460800; break;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    std::cout << "Serial port initialized: " << port_name << " @ " << baud_rate << std::endl;
}

int WheelLegSDK::Recv()
{
    unsigned char tmp[512];
    int n = read(fd, tmp, sizeof(tmp));
    if (n > 0)
    {
        internal_buffer.insert(internal_buffer.end(), tmp, tmp + n);
    }

    // Limit buffer size to avoid memory leak if no header found
    if (internal_buffer.size() > 4096)
    {
        internal_buffer.erase(internal_buffer.begin(), internal_buffer.begin() + 2048);
    }

    // [MODIFIED] New Packet Size = 94 Bytes
    // Header(1) + Data(92) + CRC(1) = 94
    while (internal_buffer.size() >= 94)
    {
        // [MODIFIED] Check Single Header 0xFE
        if (internal_buffer[0] == 0xFE)
        {
            // Verify CRC
            // Calculate CRC of the first 93 bytes
            unsigned char received_crc = internal_buffer[93];
            unsigned char calculated_crc = CalcCRC8(internal_buffer.data(), 93);

            if (received_crc == calculated_crc) 
            {
                memcpy(recv_buff, internal_buffer.data(), 94);
                recv_len = 94;
                internal_buffer.erase(internal_buffer.begin(), internal_buffer.begin() + 94);
                return 94;
            }
            else 
            {
                // Header found but CRC failed
                internal_buffer.erase(internal_buffer.begin());
            }
        }
        else
        {
            internal_buffer.erase(internal_buffer.begin());
        }
    }
    
    return 0;
}


float WheelLegSDK::read_float(unsigned char *buff, int *idx)
{
    float v = 0;
    *((unsigned char *)&v + 0) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 1) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 2) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 3) = buff[*idx];
    (*idx)++;

    return v;
}

float WheelLegSDK::read_from1byte(unsigned char *buff, int *idx, float v_min, float v_max)
{
    float v = (float)buff[*idx] / 255.0 * (v_max - v_min) + v_min;
    (*idx)++;
    return v;
}

float WheelLegSDK::read_from2bytes(unsigned char *buff, int *idx, float v_min, float v_max)
{
    unsigned int v = 0;
    *((unsigned char *)&v + 0) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 1) = buff[*idx];
    (*idx)++;

    return v / 65535.0 * (v_max - v_min) + v_min;
}

void WheelLegSDK::write_into2bytes(float value, unsigned char *buff, int *idx, float v_min, float v_max)
{
    ushort v16 = (value - v_min) / (v_max - v_min) * 65535;
    buff[*idx] = (v16 >> 8) & 0xff;
    (*idx)++;
    buff[*idx] = v16 & 0xff;
    (*idx)++;
}

float WheelLegSDK::read_byte(unsigned char *buff, int *idx)
{
    float v = (float)buff[*idx];
    (*idx)++;
    return v;
}

float WheelLegSDK::Vector3_Dot(Vector3 v1, Vector3 v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 WheelLegSDK::Vector3_Cross(Vector3 u, Vector3 v)
{
    Vector3 vn;
    vn.x = u.y * v.z - u.z * v.y;
    vn.y = u.z * v.x - u.x * v.z;
    vn.z = u.x * v.y - u.y * v.x;
    return vn;
}

float WheelLegSDK::Sign(float value)
{
    if (value >= 0)
        return 1;
    else
        return -1;
}

float WheelLegSDK::Vector3_invSqrt(Vector3 v)
{
    return 1.0 / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 WheelLegSDK::Vector3_Direction(Vector3 v)
{
    float invSqrt = Vector3_invSqrt(v);
    Vector3 dv = v;
    dv.x *= invSqrt;
    dv.y *= invSqrt;
    dv.z *= invSqrt;

    return dv;
}

float WheelLegSDK::Clamp(float value, float min, float max)
{
    if (min > max)
    {
        float tmp = min;
        min = max;
        max = tmp;
    }

    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

float WheelLegSDK::Angle_vA_2_vB(Vector3 vA, Vector3 vB, Vector3 axle)
{
    Vector3 vA_dir = Vector3_Direction(vA);
    Vector3 vB_dir = Vector3_Direction(vB);
    Vector3 axle_dir = Vector3_Direction(axle);

    float sinA = Clamp(Vector3_Dot(axle_dir, Vector3_Cross(vA_dir, vB_dir)), -1, 1);
    float angle_raw = asinf(sinA);

    float angle = Vector3_Dot(vA_dir, vB_dir) > 0 ? angle_raw : Sign(angle_raw) * M_PI - angle_raw;

    return angle;
}

void WheelLegSDK::Vector3_Normalize(Vector3 *v)
{
    float invSqrt = Vector3_invSqrt(*v);
    v->x *= invSqrt;
    v->y *= invSqrt;
    v->z *= invSqrt;
}
Vector3 WheelLegSDK::Quaternion_Transform(float vx, float vy, float vz, float qx, float qy, float qz, float qw)
{
    float dot_u_v = qx * vx + qy * vy + qz * vz;
    float dot_u_u = qx * qx + qy * qy + qz * qz;
    float x_of_uXv = qy * vz - qz * vy;
    float y_of_uXv = qz * vx - qx * vz;
    float z_of_uXv = qx * vy - qy * vx;
    float k1 = 2.0 * dot_u_v;
    float k2 = qw * qw - dot_u_u;
    float k3 = 2.0 * qw;

    Vector3 vout;
    vout.x = k1 * qx + k2 * vx + k3 * x_of_uXv;
    vout.y = k1 * qy + k2 * vy + k3 * y_of_uXv;
    vout.z = k1 * qz + k2 * vz + k3 * z_of_uXv;
    return vout;
}

// void WheelLegSDK::AnalyzeData(unsigned char *recv_buff, LowState &lowState)
// {
//     // int idx = 2;
//     int idx = 2;
//     tmp_time_from_mcu = read_float(recv_buff, &idx);
//     float BatteryVoltage = read_from1byte(recv_buff, &idx, 20, 60);
//     float MCUTemperature = read_byte(recv_buff, &idx);

//     float acc_lx = read_float(recv_buff, &idx);
//     float acc_ly = read_float(recv_buff, &idx);
//     float acc_lz = read_float(recv_buff, &idx);
//     float omega_lx = read_float(recv_buff, &idx);
//     float omega_ly = read_float(recv_buff, &idx);
//     float omega_lz = read_float(recv_buff, &idx);
//     float orientation_x = read_float(recv_buff, &idx);
//     float orientation_y = read_float(recv_buff, &idx);
//     float orientation_z = read_float(recv_buff, &idx);
//     float orientation_w = read_float(recv_buff, &idx);

//     Vector3 robot_up_w = Quaternion_Transform(0, 1, 0, orientation_x, orientation_y, orientation_z, orientation_w);
//     Vector3 current_trunk_front = Quaternion_Transform(1, 0, 0, orientation_x, orientation_y, orientation_z, orientation_w);
//     Vector3 current_trunk_right = Quaternion_Transform(0, 0, 1, orientation_x, orientation_y, orientation_z, orientation_w);
//     Vector3 trunk_hori_front = current_trunk_front;
//     trunk_hori_front.y = 0;
//     Vector3_Normalize(&trunk_hori_front);
//     Vector3 trunk_hori_right = Vector3_Cross(trunk_hori_front, {0, 1, 0});
//     float r2d = 180.0f / M_PI;

//     float robot_yaw_deg = Angle_vA_2_vB({1, 0, 0}, trunk_hori_front, {0, 1, 0}) * r2d;
//     float robot_pitch_deg = Angle_vA_2_vB(trunk_hori_front, current_trunk_front, trunk_hori_right) * r2d;
//     float robot_roll_deg = Angle_vA_2_vB(trunk_hori_right, current_trunk_right, current_trunk_front) * r2d;

//     read_byte(recv_buff, &idx); // Key1
//     read_byte(recv_buff, &idx); // Key2

//     xRockerBtnDataStruct *rockerBtn = (xRockerBtnDataStruct *)(&(lowState.wirelessRemote));

//     // [MODIFIED] Re-parsing keys from the buffer we just read
//     unsigned char key1 = recv_buff[idx-2];
//     unsigned char key2 = recv_buff[idx-1];

//     rockerBtn->btn.components.R1 = (key1 & 0x80) >> 7;
//     rockerBtn->btn.components.L1 = (key1 & 0x40) >> 6;
//     rockerBtn->btn.components.start = (key1 & 0x20) >> 5;
//     rockerBtn->btn.components.select = (key1 & 0x10) >> 4;
//     rockerBtn->btn.components.R2 = (key1 & 0x08) >> 3;
//     rockerBtn->btn.components.L2 = (key1 & 0x04) >> 2;
//     rockerBtn->btn.components.F1 = (key1 & 0x02) >> 1;
//     rockerBtn->btn.components.F2 = (key1 & 0x01) >> 0;

//     rockerBtn->btn.components.A = (key2 & 0x80) >> 7;
//     rockerBtn->btn.components.B = (key2 & 0x40) >> 6;
//     rockerBtn->btn.components.X = (key2 & 0x20) >> 5;
//     rockerBtn->btn.components.Y = (key2 & 0x10) >> 4;
//     rockerBtn->btn.components.up = (key2 & 0x08) >> 3;
//     rockerBtn->btn.components.right = (key2 & 0x04) >> 2;
//     rockerBtn->btn.components.down = (key2 & 0x02) >> 1;
//     rockerBtn->btn.components.left = (key2 & 0x01) >> 0;

//     // [MODIFIED] Read Joystick as Floats (match L4W4SDK)
//     // 5 floats: lx, rx, ly, L2, ry
//     rockerBtn->lx = read_float(recv_buff, &idx);
//     rockerBtn->rx = read_float(recv_buff, &idx);
//     rockerBtn->ly = read_float(recv_buff, &idx);
//     rockerBtn->L2 = read_float(recv_buff, &idx); // Note: L4W4 uses float for L2 trigger
//     rockerBtn->ry = read_float(recv_buff, &idx);

//     if (rockerBtn->btn.components.F1)
//         show_rc = 1;
//     else
//         show_rc = 0;

//     for (int i = 0; i < 6; i++)
//     {
//         lowState.motorState[i].q = lowState.motorState[i].q_raw = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
//         lowState.motorState[i].dq = lowState.motorState[i].dq_raw = read_from2bytes(recv_buff, &idx, -33, 33);
//         lowState.motorState[i].ddq = lowState.motorState[i].ddq_raw = 0;
//     }
    
//     lowState.imu.quaternion[0] = orientation_w;
//     lowState.imu.quaternion[1] = orientation_x;
//     lowState.imu.quaternion[2] = -orientation_z;
//     lowState.imu.quaternion[3] = orientation_y;

//     lowState.imu.gyroscope[0] = omega_lx;
//     lowState.imu.gyroscope[1] = -omega_lz;
//     lowState.imu.gyroscope[2] = omega_ly;
//     lowState.imu.accelerometer[0] = acc_lx;
//     lowState.imu.accelerometer[1] = -acc_lz;
//     lowState.imu.accelerometer[2] = acc_ly;


//     // ctr++;
//     // if (ctr >= 100)
//     // {
//     //     // float r2d = 180.0/M_PI;
//     //     // std::cout<<"mcu time = "<<tmp_time_from_mcu<<",\tbattery = "<< BatteryVoltage<<std::endl;
//     //     // std::cout<<"lx = "<<rockerBtn->lx<<",\try = "<<rockerBtn->ry<<",\trx = "<<rockerBtn->rx<<std::endl;
//     //     // if(show_rc)
//     //     if (0)
//     //     {
//     //         std::cout << "   rc: " << ((int)(rockerBtn->ly * 100) / 100.0) << "\t" << ((int)(rockerBtn->lx * 100) / 100.0);
//     //         std::cout << "\t" << ((int)(rockerBtn->ry * 100) / 100.0) << "\t" << ((int)(rockerBtn->rx * 100) / 100.0) << std::endl;
//     //     }

//     //     {
//     //         std::cout<<"                    roll, roll_offset.        vz = "<<robot_roll_deg<<",\t"<< current_roll_offset<<std::endl;
//     //     }

//     //     ctr = 0;
//     // }
// }
void WheelLegSDK::AnalyzeData(unsigned char *recv_buff, LowState &lowState)
{
    // int idx = 2; // 错误：从字节2开始，会跳过一个数据字节
    int idx = 1;    // 正确：只跳过1个字节的Header (0xFE)

    tmp_time_from_mcu = read_float(recv_buff, &idx);
    float BatteryVoltage = read_from1byte(recv_buff, &idx, 20, 60);
    float MCUTemperature = read_byte(recv_buff, &idx);

    float acc_lx = read_float(recv_buff, &idx);
    float acc_ly = read_float(recv_buff, &idx);
    float acc_lz = read_float(recv_buff, &idx);
    float omega_lx = read_float(recv_buff, &idx);
    float omega_ly = read_float(recv_buff, &idx);
    float omega_lz = read_float(recv_buff, &idx);
    float orientation_x = read_float(recv_buff, &idx);
    float orientation_y = read_float(recv_buff, &idx);
    float orientation_z = read_float(recv_buff, &idx);
    float orientation_w = read_float(recv_buff, &idx);

    Vector3 robot_up_w = Quaternion_Transform(0, 1, 0, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 current_trunk_front = Quaternion_Transform(1, 0, 0, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 current_trunk_right = Quaternion_Transform(0, 0, 1, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 trunk_hori_front = current_trunk_front;
    trunk_hori_front.y = 0;
    Vector3_Normalize(&trunk_hori_front);
    Vector3 trunk_hori_right = Vector3_Cross(trunk_hori_front, {0, 1, 0});
    float r2d = 180.0f / M_PI;

    float robot_yaw_deg = Angle_vA_2_vB({1, 0, 0}, trunk_hori_front, {0, 1, 0}) * r2d;
    float robot_pitch_deg = Angle_vA_2_vB(trunk_hori_front, current_trunk_front, trunk_hori_right) * r2d;
    float robot_roll_deg = Angle_vA_2_vB(trunk_hori_right, current_trunk_right, current_trunk_front) * r2d;
    
    read_byte(recv_buff, &idx); // Key1
    read_byte(recv_buff, &idx); // Key2

    xRockerBtnDataStruct *rockerBtn = (xRockerBtnDataStruct *)(&(lowState.wirelessRemote));

    unsigned char key1 = recv_buff[idx-2];
    unsigned char key2 = recv_buff[idx-1];

    rockerBtn->btn.components.R1 = (key1 & 0x80) >> 7;
    rockerBtn->btn.components.L1 = (key1 & 0x40) >> 6;
    rockerBtn->btn.components.start = (key1 & 0x20) >> 5;
    rockerBtn->btn.components.select = (key1 & 0x10) >> 4;
    rockerBtn->btn.components.R2 = (key1 & 0x08) >> 3;
    rockerBtn->btn.components.L2 = (key1 & 0x04) >> 2;
    rockerBtn->btn.components.F1 = (key1 & 0x02) >> 1;
    rockerBtn->btn.components.F2 = (key1 & 0x01) >> 0;

    rockerBtn->btn.components.A = (key2 & 0x80) >> 7;
    rockerBtn->btn.components.B = (key2 & 0x40) >> 6;
    rockerBtn->btn.components.X = (key2 & 0x20) >> 5;
    rockerBtn->btn.components.Y = (key2 & 0x10) >> 4;
    rockerBtn->btn.components.up = (key2 & 0x08) >> 3;
    rockerBtn->btn.components.right = (key2 & 0x04) >> 2;
    rockerBtn->btn.components.down = (key2 & 0x02) >> 1;
    rockerBtn->btn.components.left = (key2 & 0x01) >> 0;

    rockerBtn->lx = read_float(recv_buff, &idx);
    rockerBtn->rx = read_float(recv_buff, &idx);
    rockerBtn->ly = read_float(recv_buff, &idx);
    rockerBtn->L2 = read_float(recv_buff, &idx);
    rockerBtn->ry = read_float(recv_buff, &idx);

    if (rockerBtn->btn.components.F1)
        show_rc = 1;
    else
        show_rc = 0;

    for (int i = 0; i < 6; i++)
    {
        lowState.motorState[i].q = lowState.motorState[i].q_raw = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
        lowState.motorState[i].dq = lowState.motorState[i].dq_raw = read_from2bytes(recv_buff, &idx, -33, 33);
        lowState.motorState[i].ddq = lowState.motorState[i].ddq_raw = 0;
    }

    lowState.imu.quaternion[0] = orientation_w;
    lowState.imu.quaternion[1] = orientation_x;
    lowState.imu.quaternion[2] = -orientation_z;
    lowState.imu.quaternion[3] = orientation_y;

    lowState.imu.gyroscope[0] = omega_lx;
    lowState.imu.gyroscope[1] = -omega_lz;
    lowState.imu.gyroscope[2] = omega_ly;
    lowState.imu.accelerometer[0] = acc_lx;
    lowState.imu.accelerometer[1] = -acc_lz;
    lowState.imu.accelerometer[2] = acc_ly;
}


#endif 