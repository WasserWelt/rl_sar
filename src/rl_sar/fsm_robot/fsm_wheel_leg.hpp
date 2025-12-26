/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEEL_LEG_FSM_HPP
#define WHEEL_LEG_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace wheel_leg_fsm
{

// 1. 被动状态：高阻尼，电机不输出力矩，用于保护和待机
class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        // 遍历所有自由度 (wheel_leg 通常是 6 个: 2轮 + 4关节)
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8.0; // 设置一定的阻尼让关节不松垮
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // 按下 '0' 或手柄 'A' 进入起立状态
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

// 2. 起立状态：从当前姿态插值移动到 default_dof_pos
class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_getup = 0.0f;
    
    void Enter() override
    {
        percent_getup = 0.0f;
        // 记录进入起立状态时的瞬间姿态，作为插值的起点
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        std::cout << LOGGER::INFO << "Getting Up..." << std::endl;
    }

    void Run() override
    {
        // 线性插值：从当前位置移动到配置文件中的 default_dof_pos，耗时 2.0秒
        // "Getting up" 是调试打印的标签
        if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // 紧急停止回到 Passive
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        
        // 动作完成后允许切换
        if (percent_getup >= 1.0f)
        {
            // 按 '1' 或手柄 'RB + Up' 进入 RL 控制
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            // 按 '9' 或手柄 'B' 趴下
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

// 3. 趴下/复位状态：缓慢回到起立前的状态 (start_state)
class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
        std::cout << LOGGER::INFO << "Getting Down..." << std::endl;
    }

    void Run() override
    {
        // 插值回到最初的 start_state
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // 紧急停止或动作完成后回到 Passive
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        // 中途如果反悔，可以按 '0' 重新起立
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

// 4. RL 运动控制状态
class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // 设置模型配置名，对应文件路径: robot_name/config_name
        // 这里假设配置文件在 wheel_leg/legged_gym 下
        rl.config_name = "flat"; 
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        
        try
        {
            // 初始化 RL 模型
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
            rl.rl_init_done = true;
            std::cout << LOGGER::INFO << "RL Inferring..." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) return;

        // 打印当前控制指令
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        
        // 执行 RL 推理和控制输出
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        // 各种按键切换逻辑
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            // 重置当前状态 (重新 InitRL)
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace wheel_leg_fsm

// 工厂类定义
class WheelLegFSMFactory : public FSMFactory
{
public:
    // 构造函数接收初始状态名
    WheelLegFSMFactory(const std::string& initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<wheel_leg_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<wheel_leg_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<wheel_leg_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<wheel_leg_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }

    // 必须返回 "wheel_leg"，因为 rl_real_wheel_leg.cpp 中使用的是这个名字
    std::string GetType() const override { return "wheel_leg"; }
    
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }
    
    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

// 注册工厂，设置默认初始状态为 Passive
REGISTER_FSM_FACTORY(WheelLegFSMFactory, "RLFSMStatePassive")

#endif // WHEEL_LEG_FSM_HPP