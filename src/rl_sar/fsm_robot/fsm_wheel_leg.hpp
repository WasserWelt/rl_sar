#ifndef WHEEL_LEG_FSM_HPP
#define WHEEL_LEG_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace wheel_leg_fsm {
// 1. 被动状态（安全模式，电机零力矩或阻尼）
class RLFSMStatePassive : public RLFSMState {
public:
  RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}
  void Enter() override { std::cout << "Entered Passive Mode" << std::endl; }
  void Run() override {
    // 让电机输出 0 力矩，或者微小的阻尼
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
      fsm_command->motor_command.tau[i] = 0;
      fsm_command->motor_command.kd[i] = 1.0; // 轻微阻尼
    }
  }
  void Exit() override {}

  std::string CheckChange() override {
    // 这里写切换条件，比如按下键盘 '0' 切换到 RL 模式
    if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
        rl.control.current_gamepad == Input::Gamepad::A) {
      return "RLFSMStateGetUp";
    }
    return state_name_;
  }
};

// 2. RL 运行状态（加载模型并推理）
class RLFSMStateRLLocomotion : public RLFSMState {
public:
  RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}
  void Enter() override {
    rl.InitRL(rl.robot_name + "/params_file"); // 初始化 RL 环境
    rl.rl_init_done = true;
  }
  void Run() override {
    // position transition from last default_dof_pos to current default_dof_pos
    // if (Interpolate(percent_transition, rl.now_state.motor_state.q,
    // rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy
    // transition", true)) return;

    if (!rl.rl_init_done)
      rl.rl_init_done = true;

    std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller ["
              << rl.config_name << "] x:" << rl.control.x
              << " y:" << rl.control.y << " yaw:" << rl.control.yaw
              << std::flush;
    RLControl();
  }
  void Exit() override { rl.rl_init_done = false; }
  std::string CheckChange() override {
    if (rl.control.current_keyboard == Input::Keyboard::P ||
        rl.control.current_gamepad == Input::Gamepad::LB_X)
      return "RLFSMStatePassive";
    return state_name_;
  }
};
} // namespace fivebar_fsm

// 注册工厂类
class FivebarFSMFactory : public FSMFactory {
public:
  FivebarFSMFactory() {}
  std::shared_ptr<FSMState>
  CreateState(void *context, const std::string &state_name) override {
    RL *rl = static_cast<RL *>(context);
    if (state_name == "RLFSMStatePassive")
      return std::make_shared<fivebar_fsm::RLFSMStatePassive>(rl);
    if (state_name == "RLFSMStateRLLocomotion")
      return std::make_shared<fivebar_fsm::RLFSMStateRLLocomotion>(rl);
    return nullptr;
  }
  std::string GetType() const override { return "fivebar"; } // 对应 robot_name
  std::vector<std::string> GetSupportedStates() const override {
    return {"RLFSMStatePassive", "RLFSMStateRLLocomotion"};
  }
  std::string GetInitialState() const override { return "RLFSMStatePassive"; }
};

REGISTER_FSM_FACTORY(FivebarFSMFactory, "RLFSMStatePassive")

#endif