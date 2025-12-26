/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#include "rl_real_wheel_leg.hpp"
#include <iomanip> // 用于 std::setw, std::setprecision

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "wheel_leg";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->wheel_leg_sdk.InitSerial("/dev/ttyACM0", 460800);
    this->wheel_leg_sdk.InitCmdData(this->wheel_leg_low_command);
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_imu.resize(3); // R, P, Y
    this->plot_cmd.resize(3); // vx, vy, wz
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_imu) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_cmd) { vector = std::vector<float>(this->plot_size, 0); }

    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state)
{
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(this->wheel_leg_sdk.fd, &rfd);
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    int ret = select(this->wheel_leg_sdk.fd + 1, &rfd, NULL, NULL, &timeout);

    if (ret > 0)
    {
        this->wheel_leg_sdk.Recv();
        // this->wheel_leg_sdk.ex_send_recv++;

        if (this->wheel_leg_sdk.recv_len < 32)
        {
            // std::cout << " serial recv_len = " << this->wheel_leg_sdk.recv_len << std::endl;
            return;
        }
        this->wheel_leg_sdk.AnalyzeData(this->wheel_leg_sdk.recv_buff, this->wheel_leg_low_state);

        // ======================= 调试信息 (Raw Data Monitor) =======================
        // static int serial_dbg_cnt = 0;
        // if (serial_dbg_cnt++ % 50 == 0) // 500Hz 循环下，约每 0.1s 刷新一次
        // {
        //     // 1. 获取遥控器指针 (直接转换，方便访问)
        //     xRockerBtnDataStruct* rc = (xRockerBtnDataStruct*)(&this->wheel_leg_low_state.wirelessRemote);
        //     // 2. 计算 RPY (欧拉角)
        //     double q0 = this->wheel_leg_low_state.imu.quaternion[0]; // w
        //     double q1 = this->wheel_leg_low_state.imu.quaternion[1]; // x
        //     double q2 = this->wheel_leg_low_state.imu.quaternion[2]; // y
        //     double q3 = this->wheel_leg_low_state.imu.quaternion[3]; // z
        //     double roll, pitch, yaw;
        //     // Roll (x-axis rotation)
        //     double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
        //     double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        //     roll = std::atan2(sinr_cosp, cosr_cosp);
        //     // Pitch (y-axis rotation)
        //     double sinp = 2 * (q0 * q2 - q3 * q1);
        //     if (std::abs(sinp) >= 1)
        //         pitch = std::copysign(M_PI / 2, sinp);
        //     else
        //         pitch = std::asin(sinp);
        //     // Yaw (z-axis rotation)
        //     double siny_cosp = 2 * (q0 * q3 + q1 * q2);
        //     double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        //     yaw = std::atan2(siny_cosp, cosy_cosp);
        //     // 3. 打印详细信息
        //     std::cout << "\033[2J\033[1;1H"; // 清屏
        //     std::cout << "================ ROBOT STATE MONITOR ================" << std::endl;
        //     // --- 遥控器 ---
        //     std::cout << "-------------------- REMOTE -------------------------" << std::endl;
        //     std::cout << std::fixed << std::setprecision(3);
        //     std::cout << "Joysticks:" << std::endl;
        //     std::cout << "  LX: " << std::setw(6) << rc->lx << "  RX: " << std::setw(6) << rc->rx << std::endl;
        //     std::cout << "  LY: " << std::setw(6) << rc->ly << "  RY: " << std::setw(6) << rc->ry << std::endl;
        //     std::cout << "  L2 (Analog): " << std::setw(6) << rc->L2 << std::endl;
        //     std::cout << "Buttons:" << std::endl;
        //     std::cout << "  [A]: " << (rc->btn.components.A ? "ON " : "OFF") << "  [B]: " << (rc->btn.components.B ? "ON " : "OFF") 
        //               << "  [X]: " << (rc->btn.components.X ? "ON " : "OFF") << "  [Y]: " << (rc->btn.components.Y ? "ON " : "OFF") << std::endl;
        //     std::cout << "  [L1]:" << (rc->btn.components.L1 ? "ON " : "OFF") << "  [R1]:" << (rc->btn.components.R1 ? "ON " : "OFF")
        //               << "  [L2]:" << (rc->btn.components.L2 ? "ON " : "OFF") << "  [R2]:" << (rc->btn.components.R2 ? "ON " : "OFF") << std::endl;
        //     std::cout << "  [UP]:" << (rc->btn.components.up ? "ON " : "OFF") << "  [DW]:" << (rc->btn.components.down ? "ON " : "OFF")
        //               << "  [LE]:" << (rc->btn.components.left ? "ON " : "OFF") << "  [RI]:" << (rc->btn.components.right ? "ON " : "OFF") << std::endl;
        //     std::cout << "  [F1]:" << (rc->btn.components.F1 ? "ON " : "OFF") << "  [F2]:" << (rc->btn.components.F2 ? "ON " : "OFF")
        //               << "  [ST]:" << (rc->btn.components.start ? "ON " : "OFF") << "  [SE]:" << (rc->btn.components.select ? "ON " : "OFF") << std::endl;
        //     // --- IMU ---
        //     std::cout << "---------------------- IMU --------------------------" << std::endl;
        //     std::cout << "Quat: " << q0 << ", " << q1 << ", " << q2 << ", " << q3 << std::endl;
        //     std::cout << "RPY (Calc): R=" << roll << " P=" << pitch << " Y=" << yaw << std::endl;
        //     // --- Motors ---
        //     std::cout << "-------------------- MOTORS -------------------------" << std::endl;
        //     std::cout << "Pos: ";
        //     for(int i=0; i<4; i++) std::cout << std::setw(7) << this->wheel_leg_low_state.motorState[i].q << " ";
        //     std::cout << std::endl;
        //     // --- Raw Hex ---
        //     std::cout << "\nRaw Hex (First 16 bytes): ";
        //     std::cout << std::hex << std::setfill('0');
        //     for (int i = 0; i < 16; i++) {
        //         std::cout << std::setw(2) << (int)this->wheel_leg_sdk.recv_buff[i] << " ";
        //     }
        //     std::cout << std::dec << std::setfill(' ') << std::endl;
        //     std::cout << "=====================================================" << std::endl;
        // }
        // // ========================================================================

        memcpy(&this->wheel_leg_joy, this->wheel_leg_low_state.wirelessRemote, 40);

        if (this->wheel_leg_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::A);
        if (this->wheel_leg_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::B);
        if (this->wheel_leg_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::X);
        if (this->wheel_leg_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
        if (this->wheel_leg_joy.btn.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
        if (this->wheel_leg_joy.btn.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
        if (this->wheel_leg_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
        if (this->wheel_leg_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
        if (this->wheel_leg_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
        if (this->wheel_leg_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (this->wheel_leg_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
        if (this->wheel_leg_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
        if (this->wheel_leg_joy.btn.components.R1 && this->wheel_leg_joy.btn.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
        if (this->wheel_leg_joy.btn.components.L1 && this->wheel_leg_joy.btn.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

        this->control.x = -((float)this->wheel_leg_joy.ly - 128.0f) / 128.0f * 1.0f;
        this->control.y = -((float)this->wheel_leg_joy.lx - 128.0f) / 128.0f * 0.0f;
        this->control.yaw = -((float)this->wheel_leg_joy.rx - 128.0f) / 128.0f * 1.5f;

        state->imu.quaternion[0] = this->wheel_leg_low_state.imu.quaternion[0]; // w
        state->imu.quaternion[1] = this->wheel_leg_low_state.imu.quaternion[1]; // x
        state->imu.quaternion[2] = this->wheel_leg_low_state.imu.quaternion[2]; // y
        state->imu.quaternion[3] = this->wheel_leg_low_state.imu.quaternion[3]; // z

        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = this->wheel_leg_low_state.imu.gyroscope[i];
        }

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = this->wheel_leg_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
            state->motor_state.dq[i] = this->wheel_leg_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
            state->motor_state.tau_est[i] = this->wheel_leg_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].tauEst;
        }
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode = 0x0A;
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kp = command->motor_command.kp[i];
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kd = command->motor_command.kd[i];
        this->wheel_leg_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
    }

    this->wheel_leg_sdk.Send(this->wheel_leg_low_command);
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        // ========================== [新增仪表盘代码开始] ==========================
        // 使用静态变量控制刷新频率，防止闪屏
        static int dashboard_cnt = 0;
        const int refresh_rate = 10; // 每10次推理刷新一次 (视dt而定，约0.1-0.2秒一次)

        if (dashboard_cnt++ % refresh_rate == 0)
        {
            // ANSI 转义序列说明：
            // \033[2J : 清除整个屏幕
            // \033[H  : 将光标移动到左上角 (1,1) 位置
            std::cout << "\033[2J\033[H"; 

            std::cout << "################# RL INFERENCE DASHBOARD #################" << std::endl;
            std::cout << "Motion Time: " << this->motiontime << " | Episode Len: " << this->episode_length_buf << std::endl;
            std::cout << "----------------------------------------------------------" << std::endl;
            
            // 1. 显示 Action (神经网络输出)
            std::cout << "Raw Actions (Net Output):" << std::endl;
            std::cout << std::fixed << std::setprecision(4); // 4位小数

            int n_dofs = this->obs.actions.size();
            for (int i = 0; i < n_dofs; ++i)
            {
                // 格式：[索引]: 数值
                std::cout << " [" << std::setw(2) << i << "]: " << std::setw(8) << (this->obs.actions[i] >= 0 ? "+" : "") << this->obs.actions[i];
                
                // 这里的 3 表示每行打印3个关节数据，你可以根据你的机器人腿部结构修改
                // 比如轮足如果是 (Hip, Knee, Wheel) 一组，设为 3 比较直观
                if ((i + 1) % 3 == 0) std::cout << std::endl;
                else std::cout << "  "; // 列间距
            }
            if (n_dofs % 3 != 0) std::cout << std::endl; // 补换行

            std::cout << "----------------------------------------------------------" << std::endl;
            
            // 2. 显示计算后的力矩 (可选，方便对比 Action 和最终 Tau 的关系)
            // 注意：这里使用的是上一帧计算出的 output_dof_tau，或者如果你想看当前的，
            // 需要把这段打印代码移到下方的 ComputeOutput 之后。
            std::cout << "Computed Torque (Target):" << std::endl;
            std::cout << std::setprecision(2); 
            
            int n_tau = this->output_dof_tau.size(); // 确保 ComputeOutput 已经运行过至少一次
            if (n_tau > 0)
            {
                for (int i = 0; i < n_tau; ++i)
                {
                    std::cout << " [" << std::setw(2) << i << "]: " << std::setw(6) << (this->output_dof_tau[i] >= 0 ? "+" : "") << this->output_dof_tau[i];
                    if ((i + 1) % 3 == 0) std::cout << std::endl;
                    else std::cout << "  ";
                }
                if (n_tau % 3 != 0) std::cout << std::endl;
            }
            else
            {
                std::cout << " (Waiting for first computation...)" << std::endl;
            }

            std::cout << "##########################################################" << std::endl;
        }
        // ========================== [新增仪表盘代码结束] ==========================

        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    // 1. Calculate Euler Angles from Quaternion (Order: usually Z-Y-X for vehicles)
    // Quaternion w, x, y, z
    float qw = this->wheel_leg_low_state.imu.quaternion[0];
    float qx = this->wheel_leg_low_state.imu.quaternion[1];
    float qy = this->wheel_leg_low_state.imu.quaternion[2];
    float qz = this->wheel_leg_low_state.imu.quaternion[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    float pitch = 0;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    // 2. Update Buffers (Remove old, Add new)
    for (int i = 0; i < 3; ++i) {
        this->plot_imu[i].erase(this->plot_imu[i].begin());
        this->plot_cmd[i].erase(this->plot_cmd[i].begin());
    }
    
    // IMU Data
    this->plot_imu[0].push_back(roll);
    this->plot_imu[1].push_back(pitch);
    this->plot_imu[2].push_back(yaw);

    // Command Data (Vx, Vy, Wz)
    this->plot_cmd[0].push_back(this->control.x);
    this->plot_cmd[1].push_back(this->control.y);
    this->plot_cmd[2].push_back(this->control.yaw);
    // 3. Update Joint Buffers
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->wheel_leg_low_state.motorState[i].q);
        this->plot_target_joint_pos[i].push_back(this->wheel_leg_low_command.motorCmd[i].q);
    }
    // 4. Drawing
    plt::cla();
    plt::clf();
    // --- 窗口 1: 关节电机 (Joints) ---
    plt::figure(1); // 切换到图表 ID 1
    plt::clf();     // 清空当前窗口
    // 这里的 subplot 是在这个窗口内部切分布局
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_q", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_q", this->plot_t, this->plot_target_joint_pos[i], "b--");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
        // 只在第一个子图显示标题，避免混乱
        if(i==0) plt::title("Joint Positions");
    }

    // --- 窗口 2: IMU 数据 (Roll/Pitch/Yaw) ---
    plt::figure(2); // 切换到图表 ID 2 (会弹出一个新窗口)
    plt::clf();
    
    plt::subplot(3, 1, 1);
    plt::named_plot("Roll", this->plot_t, this->plot_imu[0], "r");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::named_plot("Pitch", this->plot_t, this->plot_imu[1], "g");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::named_plot("Yaw", this->plot_t, this->plot_imu[2], "b");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();


    // --- 窗口 3: 控制指令 (Vx/Vy/Wz) ---
    plt::figure(3); // 切换到图表 ID 3 (会弹出一个新窗口)
    plt::clf();

    plt::subplot(3, 1, 1);
    plt::named_plot("Vx (Linear X)", this->plot_t, this->plot_cmd[0], "r");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::named_plot("Vy (Linear Y)", this->plot_t, this->plot_cmd[1], "g");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::named_plot("Wz (Angular Z)", this->plot_t, this->plot_cmd[2], "b");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
    plt::legend();

    // 暂停以刷新所有窗口
    plt::pause(0.0001);
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(argc, argv);
    while (1) { sleep(10); }
#endif
    return 0;
}