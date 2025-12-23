# 适配需要的操作

下面使用 \<ROBOT>/\<CONFIG> 代替表示你的机器人环境。

## 你的机器人description

- [X] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/CMakeLists.txt
- [X] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/package.ros1.xml
- [X] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/package.ros2.xml
- [ ] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/xacro/robot.xacro
- [ ] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/xacro/gazebo.xacro
- [ ] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/config/robot_control.yaml
- [ ] rl_sar/src/rl_sar_zoo/\<ROBOT>_description/config/robot_control_ros2.yaml

## 你训练的policy

- [X] policy/\<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
- [X] policy/\<ROBOT>/\<CONFIG>/config.yaml
  - [ ] observer stack problem
  - [ ] in `rl_sim` and `rl_real_xxx`
- [X] policy/\<ROBOT>/\<CONFIG>/\<POLICY>.pt  # libtorch使用，注意导出jit
- [X] policy/\<ROBOT>/\<CONFIG>/\<POLICY>.onnx  # onnxruntime使用

## 机器人的fsm

- [ ] src/rl_sar/fsm_robot/fsm_`<ROBOT>`.hpp
- [ ] src/rl_sar/fsm_robot/fsm_all.hpp

## 你实物机器人的代码

- [ ] rl_sar/src/rl_sar/src/rl_real_\<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy

---

要实现类似训练时的 `stack obs`（堆叠历史观测）和 `nonstack obs`（非堆叠当前观测）混合排列，可以通过修改 `RL::ComputeObservation` 函数使其支持读取不同的配置键，然后在 `Forward` 函数中分别获取这两部分观测并拼接。

这种方法对代码的侵入性很小，且兼容原有的配置方式。

以下是具体的修改步骤：

### 1. 修改头文件 `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`

修改 `ComputeObservation` 的声明，增加一个带默认值的参数 `key`。

```215:215:src/rl_sar/library/core/rl_sdk/rl_sdk.hpp
    std::vector<float> ComputeObservation(const std::string& key = "observations");
```

### 2. 修改源文件 `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`

更新 `ComputeObservation` 的实现，使其根据传入的 `key` 读取配置，并处理键不存在的情况。

```64:68:src/rl_sar/library/core/rl_sdk/rl_sdk.cpp
std::vector<float> RL::ComputeObservation(const std::string& key)
{
    std::vector<std::vector<float>> obs_list;

    // 如果配置文件中没有这个key，直接返回空
    if (!this->params.Has(key)) return {};

    for (const std::string &observation : this->params.Get<std::vector<std::string>>(key))
    {
```

### 3. 修改 `src/rl_sar/src/rl_sim.cpp` 中的 `Forward` 函数

在 `Forward` 函数中，分别计算堆叠观测和非堆叠观测，然后将它们拼接起来传给模型。

```513:536:src/rl_sar/src/rl_sim.cpp
    // 1. 获取需要堆叠的观测 (使用默认 key "observations")
    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    std::vector<float> model_input;

    // 处理历史堆叠 (Stack Policy)
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        model_input = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
    }
    else
    {
        model_input = clamped_obs;
    }

    // 2. 获取不需要堆叠的观测 (使用 key "observations_nonstack") 并拼接到后面
    // 对应你截图中的 none_stack_policy
    if (this->params.Has("observations_nonstack"))
    {
        std::vector<float> nonstack_obs = this->ComputeObservation("observations_nonstack");
        model_input.insert(model_input.end(), nonstack_obs.begin(), nonstack_obs.end());
    }

    actions = this->model->forward({model_input});

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}
```

### 4. 修改配置文件 `config.yaml`

在你的机器人配置文件中（例如 `policy/fivebar/flat/config.yaml`），取消 `observations_nonstack` 的注释或添加它，填入不需要堆叠的观测项：

```yaml
  observations: ["dof_pos", "dof_vel", "wheel_joint_vel", "ang_vel", "gravity_vec", "actions"]
  observations_nonstack: ["commands"] # 这里填入不需要堆叠的观测，例如 commands
  observations_history: [0,1,2]
```

**注意：** 如果你是使用真机运行，请记得同样修改对应的 `rl_real_*.cpp` 文件（例如 `src/rl_sar/src/rl_real_a1.cpp` 等）中的 `Forward` 函数，逻辑与上面 `rl_sim.cpp` 的修改完全一致。

[5 tools called]

部署自制机器人到现实环境是一个系统工程，特别是没有现成SDK的情况下。你需要自己充当“中间人”，把硬件（电机、手柄）和软件（RL模型）连接起来。

因为你的机器人是自制的，没有宇树的SDK，我们需要**移除原代码中所有Unitree相关的依赖**，并用你自己的硬件接口和标准Linux手柄驱动替代。

以下是详细的分步指南：

### 第一步：理解核心逻辑

部署的核心就是填空题，你需要在一个循环中完成三件事：

1. **Read (读)**：读取电机角度、速度，读取手柄命令。
2. **Forward (算)**：把数据喂给RL模型，算出目标关节位置或力矩。
3. **Write (写)**：把计算出的力矩发送给电机。

### 第二步：编写硬件接口代码 (`rl_real_fivebar.cpp`)

我为你重新编写了 `rl_real_fivebar.cpp` 的模板。这个模板移除了宇树的SDK，接入了通用的Linux手柄驱动，并留出了你填写入电机通信代码的位置。

你需要做的是：

1. 把这个文件覆盖到 `src/rl_sar/src/rl_real_fivebar.cpp`。
2. 在注释标注 **"TODO"** 的地方，填入你自己的电机通信代码（CAN或串口发送指令）。

```cpp:src/rl_sar/src/rl_real_fivebar.cpp
#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "joystick.hh" // 使用项目自带的通用手柄库
#include <vector>
#include <string>
#include <thread>
#include <mutex>

// 如果使用ROS2
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

class RL_Real_Fivebar : public RL
{
public:
    RL_Real_Fivebar(int argc, char **argv)
    {
        // 1. 初始化手柄 (假设是 /dev/input/js0)
        // 确保你的手柄已经通过蓝牙连上，并且 ls /dev/input/js* 能看到
        joystick = std::make_shared<Joystick>("/dev/input/js0");
        if (!joystick->isFound()) {
            std::cout << "Warning: Joystick not found on /dev/input/js0" << std::endl;
        }

        // 2. 读取配置
        this->robot_name = "fivebar"; // 确保这里名字对应
        this->ReadYaml(this->robot_name, "base.yaml");
      
        // 3. 初始化电机通信 (TODO: 在这里初始化你的CAN卡或串口)
        InitMotors();

        // 4. 初始化RL相关
        this->InitJointNum(this->params.Get<int>("num_of_dofs"));
        this->InitOutputs();
        this->InitControl();

        // 5. 启动控制循环
        // 这里的频率很重要，实机通常需要较高的控制频率 (如 500Hz - 0.002s)
        this->loop_control = std::make_shared<LoopFunc>("loop_control", 0.002, std::bind(&RL_Real_Fivebar::RobotControl, this));
        this->loop_rl = std::make_shared<LoopFunc>("loop_rl", 0.02, std::bind(&RL_Real_Fivebar::RunModel, this)); // RL通常频率低一些
        this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.05, std::bind(&RL_Real_Fivebar::ReadJoystick, this));

        this->loop_control->start();
        this->loop_rl->start();
        this->loop_joystick->start();
    }

    ~RL_Real_Fivebar()
    {
        this->loop_control->shutdown();
        this->loop_rl->shutdown();
        this->loop_joystick->shutdown();
    }

private:
    std::shared_ptr<Joystick> joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_joystick;

    std::vector<float> motor_q;   // 关节位置
    std::vector<float> motor_dq;  // 关节速度
  
    // 初始化你的电机
    void InitMotors() {
        motor_q.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
        motor_dq.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
        // TODO: 打开串口或CAN设备
        std::cout << "Init Motors..." << std::endl;
    }

    // 读取手柄输入
    void ReadJoystick() {
        JoystickEvent event;
        if (joystick->isFound()) {
            while (joystick->sample(&event)) {
                if (event.isAxis()) {
                    // 假设 轴1是前进后退(X)，轴0是左右(Y)，轴3是旋转(Yaw)
                    // 你可以使用 jstest /dev/input/js0 来测试你的手柄轴号
                    if (event.number == 1) this->control.x = -event.value / 32767.0f; 
                    if (event.number == 0) this->control.y = -event.value / 32767.0f;
                    if (event.number == 3) this->control.yaw = -event.value / 32767.0f;
                }
            }
        }
    }

    // 核心控制循环：读传感器 -> 写电机
    void RobotControl()
    {
        // --- 1. 读取硬件状态 (READ) ---
        // TODO: 替换为你的电机读取代码
        // my_motor_driver.read(motor_q, motor_dq); 
        // 暂时模拟数据：
        // motor_q = ...; 

        // 更新 robot_state 供 RL 使用
        this->GetState(&this->robot_state);

        // --- 2. 设置控制命令 (WRITE) ---
        // RL模型计算出的结果会放在 this->robot_command 中
        // TODO: 发送力矩给电机
        for(int i=0; i<this->params.Get<int>("num_of_dofs"); ++i) {
             float target_torque = this->robot_command.motor_command.tau[i];
             float target_q = this->robot_command.motor_command.q[i];
             float target_kp = this->robot_command.motor_command.kp[i];
             float target_kd = this->robot_command.motor_command.kd[i];
           
             // 简单的 PD + 前馈力矩 公式
             float send_torque = target_torque + target_kp * (target_q - motor_q[i]) + target_kd * (0 - motor_dq[i]);
           
             // 安全限幅！！！非常重要
             if (send_torque > 5.0f) send_torque = 5.0f;
             if (send_torque < -5.0f) send_torque = -5.0f;

             // my_motor_driver.sendTorque(i, send_torque);
        }
    }

    // 适配 RL 接口
    void GetState(RobotState<float> *state) override
    {
        // 填充 IMU 数据 (TODO: 如果你有真实IMU，在这里读取)
        state->imu.quaternion[0] = 1.0; // w
        state->imu.quaternion[1] = 0.0; // x
        state->imu.quaternion[2] = 0.0; // y
        state->imu.quaternion[3] = 0.0; // z
        state->imu.gyroscope[0] = 0.0;
        state->imu.gyroscope[1] = 0.0;
        state->imu.gyroscope[2] = 0.0;

        // 填充电机数据
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = motor_q[i];
            state->motor_state.dq[i] = motor_dq[i];
        }
    }

    void SetCommand(const RobotCommand<float> *command) override {
        // 这个函数在 LoopControl 里直接处理了，这里可以留空或者做日志
    }

    void RunModel()
    {
        if (this->rl_init_done)
        {
            // 收集观测值
            this->obs.ang_vel = this->robot_state.imu.gyroscope;
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
            this->obs.base_quat = this->robot_state.imu.quaternion;
            this->obs.dof_pos = this->robot_state.motor_state.q;
            this->obs.dof_vel = this->robot_state.motor_state.dq;

            // 推理
            this->obs.actions = this->Forward();
          
            // 转换输出 (Actions -> Torques/Positions)
            this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
        }
    }

    // 标准 Forward 函数 (不需要修改，除非你有特殊的观测处理)
    std::vector<float> Forward() override
    {
        // ... (保持与 rl_real_a1.cpp 类似的逻辑，使用 obs 缓冲区)
        std::vector<float> clamped_obs = this->ComputeObservation();
        return this->model->forward({clamped_obs});
    }
};

int main(int argc, char **argv)
{
    // 简单的程序入口
    RL_Real_Fivebar robot(argc, argv);
    while(1) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    return 0;
}
```

### 第三步：如何测试安全性 (Safety Test)

在你把上面的代码跑起来之前，**绝对不要直接接上RL模型**。RL模型输出的力矩可能非常大且剧烈。

你应该编写一个独立的、简单的测试程序（例如 `test_hardware.cpp`），不涉及任何 ROS 或 RL，只做以下三件事：

1. **通信测试**：只读取，不写入。用手掰动机器人关节，看屏幕打印的角度是否变化正确。
   * *检查点*：顺时针掰动，数值是变大还是变小？这必须和仿真里的定义一致（通常右手定则）。如果不一致，你的机器人会直接飞出去。
2. **零力矩测试**：发送 0 力矩。
   * *检查点*：电机应该没有力，可以自由转动。
3. **微小力矩/位置测试**：发送一个极小的力矩（如 0.5 Nm）或者使用 PD 控制让它保持在一个位置。
   * *检查点*：看电机是否有力矩抵抗。

**测试代码示例逻辑：**

```cpp
// 伪代码
int main() {
    MotorDriver motors;
    motors.init();
    while(true) {
        auto q = motors.readPos();
        std::cout << "Joint 0: " << q[0] << std::endl; // 确认读数正确
      
        // 安全测试：发送 0 力矩
        motors.sendTorque(0, 0.0); 
    }
}
```

### 第四步：怎么接入遥控器（手柄）

你在Linux下可以直接使用蓝牙手柄。

1. **连接手柄**：在 Ubuntu 蓝牙设置里连上你的游戏手柄。
2. **测试手柄**：打开终端运行：
   ```bash
   sudo apt install joystick
   ls /dev/input/js*  # 应该能看到 js0
   jstest /dev/input/js0
   ```

   按动手柄按键，看屏幕数值变化。记住哪个轴对应前后（通常是轴1），哪个对应左右。
3. **代码集成**：上面的 `rl_real_fivebar.cpp` 中已经集成了 `Joystick` 类。它会自动读取 `/dev/input/js0` 并把数值写入 `this->control.x/y/yaw`，RL模型会自动读取这些变量作为 `commands` 观测值。

### 第五步：运行命令

当你完成代码填空（TODO部分）并编译通过后，运行步骤如下：

1. **编译**：
   在工作空间根目录（`rl_sar`）：

   ```bash
   colcon build --packages-select rl_sar
   source install/setup.bash
   ```
2. **给权限**：
   确保你的程序有权限读写串口/USB和手柄：

   ```bash
   sudo chmod 777 /dev/ttyUSB0  # 假设你的电机是USB串口
   sudo chmod 777 /dev/input/js0
   ```
3. **运行**：
   因为我们写的 `main` 函数比较简单，可以直接作为可执行文件运行，或者通过 ROS2 运行（如果在 CMakeLists.txt 里注册了节点）。

   如果是直接运行二进制（假设编译生成在 `install/rl_sar/lib/rl_sar/rl_real_fivebar`）：

   ```bash
   ./install/rl_sar/lib/rl_sar/rl_real_fivebar
   ```

### 总结 Checklist

1. [ ] **确认方向**：手动掰动关节，确认 `rl_real_fivebar.cpp` 读到的角度正负方向与 MuJoCo 仿真里一致（这最关键）。
2. [ ] **确认零点**：机器人在“零位”姿态时，读到的角度是否接近 0（或者你设定的初始偏置）。
3. [ ] **限幅保护**：在代码里死写一行 `if (torque > 5.0) torque = 5.0;`，防止电机过热或失控。
4. [ ] **急停开关**：如果没有硬件急停，准备好随时 `Ctrl+C` 或者把机器人架空测试。

你需要根据你具体的电机驱动协议（例如 CAN总线协议），去填充代码中的 `TODO` 部分。

---

# 测试代码

要测试 `fivebar` 的相关配置，你需要启用并编译 `test_inference_runtime.cpp` 测试工具，然后使用正确的参数运行它。

根据你的 `policy/fivebar/flat/config.yaml` 配置文件，模型的输入维度（`num_observations`）为 **105**。

请按照以下步骤操作：

### 1. 修改 CMakeLists.txt 以启用测试工具

`src/rl_sar/CMakeLists.txt` 中默认注释掉了测试工具的编译配置，且文件名旧版可能为 `test_inference_stability`。你需要取消注释并修改为正确的文件名 `test_inference_runtime`。

打开 `src/rl_sar/CMakeLists.txt`，找到文件末尾附近的 `only for test` 部分，修改如下：

```cmake:src/rl_sar/CMakeLists.txt
// ... existing code ...
# ========================
# only for test
# ========================
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
# add_executable(test_vector_math test/test_vector_math.cpp)
# install(TARGETS test_vector_math DESTINATION lib/${PROJECT_NAME})

# 启用 test_inference_runtime
add_executable(test_inference_runtime test/test_inference_runtime.cpp)
target_link_libraries(test_inference_runtime
    inference_runtime
    ${TORCH_LIBRARIES}
    ${ONNX_RUNTIME_LIB}
)
install(TARGETS test_inference_runtime DESTINATION lib/${PROJECT_NAME})

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
// ... existing code ...
```

### 2. 重新编译项目

在终端中执行以下命令重新编译：

```bash
cd /home/wyw/my_tasks/rl_sar/cmake_build
cmake ..
make -j4
```

编译完成后，你应该能在 `cmake_build/bin/` 目录下看到 `test_inference_runtime` 可执行文件。

### 3. 运行测试

使用以下命令测试 `fivebar` 的模型。我们使用 `num_observations: 105` 作为输入大小。

**测试 PyTorch 模型 (.pt):**

```bash
# 参数说明: <模型路径> <框架> <是否使用抽象层> <输入大小> <迭代次数>
./bin/test_inference_runtime ../policy/fivebar/flat/policy.pt torch 1 105 100
```

**测试 ONNX 模型 (.onnx) (如果已导出):**

```bash
./bin/test_inference_runtime ../policy/fivebar/flat/policy.onnx onnx 1 105 100
```

### 结果说明

* **Status**: 如果显示 `✓ STABLE`，说明模型推理结果稳定（每次给定相同输入输出一致）。
* **Time**: 关注 `Avg Time`，这代表了模型推理的耗时。对于实时控制，这个时间通常需要在 1ms - 2ms 以内。
* 如果提示 `Input size mismatch` 或程序崩溃，请尝试将输入大小改为 **126** (即 `21 * 6`，根据 `config.yaml` 中的观测历史长度推算的另一种可能性)，但通常以 `config.yaml` 中显式写的 `num_observations: 105` 为准。


# 串口通信

**版本**: v1.0 (Verified)
**更新日期**: 2025-01-01

## 1. 物理层配置 (Physical Layer)

* **波特率 (Baud Rate)**: 460800
* **数据位**: 8
* **停止位**: 1
* **校验位**: None (无)
* **流控**: None (无)
* **字节序 (Endianness)**: **Little Endian (小端序)** —— *低字节在前，高字节在后*

## 2. 帧结构总览 (Frame Structure)

所有数据包（发送和接收）均遵循以下结构：

| 偏移 (Offset) | 长度 (Length) | 字段 (Field)      | 描述 (Description)              |
| :------------ | :------------ | :---------------- | :------------------------------ |
| 0             | 1 Byte        | **Header**  | 固定帧头**0xFE**          |
| 1             | N Bytes       | **Payload** | 有效数据载荷                    |
| N+1           | 1 Byte        | **CRC8**    | 校验和 (CRC-8-Maxim, Poly=0x31) |

---

## 3. 上行数据：MCU -> PC (反馈状态)

**总长度**: 94 Bytes
**功能**: 反馈机器人的传感器、电机和遥控器状态。

| 字节偏移 | 数据类型  | 原始类型 | 压缩方式 | 内容描述                   | 备注                     |
| :------- | :-------- | :------- | :------- | :------------------------- | :----------------------- |
| 0        | uint8     | -        | -        | **Header (0xFE)**    | 帧头                     |
| 1-4      | float     | float    | None     | **MCU Time**         | 运行时间(s)              |
| 5        | uint8     | float    | 20~60V   | **Battery Voltage**  | 线性映射                 |
| 6        | int8      | int8     | None     | **MCU Temperature**  | 温度(°C)                |
| 7-18     | float[3]  | float    | None     | **Accelerometer**    | 顺序: x, y, z            |
| 19-30    | float[3]  | float    | None     | **Gyroscope**        | 顺序: x, y, z            |
| 31-46    | float[4]  | float    | None     | **Quaternion**       | 顺序: x, y, z, w         |
| 47-48    | uint8[2]  | Bits     | BitMap   | **Remote Keys**      | 按键位图 (Key1, Key2)    |
| 49-68    | float[5]  | float    | None     | **Remote Joysticks** | 顺序: lx, rx, ly, L2, ry |
| 69-92    | struct[6] | -        | -        | **Motor Feedback**   | 6个电机数据，详见下表    |
| 93       | uint8     | -        | -        | **CRC8**             | 校验位                   |

### 3.1 电机反馈数据块 (Motor Feedback Block)

包含6个电机，每个电机占用 4 Bytes。顺序：Motor 0 -> Motor 5。
*总计 24 Bytes (Bytes 69-92)*

| 内部偏移 | 数据类型 | 压缩范围 (Min, Max) | 内容描述                |
| :------- | :------- | :------------------ | :---------------------- |
| +0, +1   | uint16   | $[-\pi, \pi]$     | **q (当前角度)**  |
| +2, +3   | uint16   | $[-33, 33]$       | **dq (当前速度)** |

---

## 4. 下行数据：PC -> MCU (控制指令)  结构体里面为啥要再塞一个remote

**总长度**: 38 Bytes
**功能**: 下发电机控制指令。

| 字节偏移 | 数据类型  | 原始类型 | 压缩方式 | 内容描述                 | 备注                  |
| :------- | :-------- | :------- | :------- | :----------------------- | :-------------------- |
| 0        | uint8     | -        | -        | **Header (0xFE)**  | 帧头                  |
| 1-36     | struct[6] | -        | -        | **Motor Commands** | 6个电机指令，详见下表 |
| 37       | uint8     | -        | -        | **CRC8**           | 校验位                |

### 4.1 电机指令数据块 (Motor Command Block)

包含6个电机，每个电机占用 6 Bytes。顺序：Motor 0 -> Motor 5。
**注意**：Motor 0,1 (轮子) 与 Motor 2-5 (关节) 的第1个字段定义不同。

| 内部偏移         | 针对电机                | 数据类型         | 压缩范围 (Min, Max)       | 内容描述                |
| :--------------- | :---------------------- | :--------------- | :------------------------ | :---------------------- |
| **+0, +1** | **0, 1 (Wheels)** | **uint16** | **$[-200, 200]$** | **dq (目标速度)** |
| **+0, +1** | **2-5 (Joints)**  | **uint16** | **$[-3.2, 3.2]$** | **q (目标位置)**  |
| +2, +3           | All                     | uint16           | $[0, 1000]$             | **Kp (位置刚度)** |
| +4, +5           | All                     | uint16           | $[0, 1000]$             | **Kd (速度刚度)** |

---

## 5. 数据转换公式 (Data Conversion)

### 5.1 浮点数压缩 (Float -> Uint16)

用于发送端 (MCU反馈电机数据 或 PC发送控制指令)。

$$
\text{uint16\_val} = \frac{\text{float\_val} - \text{MIN}}{\text{MAX} - \text{MIN}} \times 65535
$$

*注意：需进行边界限制 (Clamp)，防止溢出。*

### 5.2 浮点数解压 (Uint16 -> Float)

用于接收端。

$$
\text{float\_val} = \frac{\text{uint16\_val}}{65535.0} \times (\text{MAX} - \text{MIN}) + \text{MIN}
$$

### 5.3 字节序处理 (Little Endian)

对于 `uint16` 类型的数据：

* **发送时**：先发低8位 (`val & 0xFF`)，再发高8位 (`val >> 8`)。
* **接收时**：`val = buf[0] | (buf[1] << 8)`。

---

## 6. 参数范围表 (Scale Ranges)

请确保上下位机严格使用以下宏定义，否则数据会产生整体偏移。

| 参数                    | 最小值 (MIN)   | 最大值 (MAX)  | 单位  | 备注             |
| :---------------------- | :------------- | :------------ | :---- | :--------------- |
| **Q (Angle)**     | -3.14159265... | 3.14159265... | rad   | SDK使用 `M_PI` |
| **DQ (Joint Rx)** | -33.0          | 33.0          | rad/s | 关节电机反馈速度 |
| **DQ (Wheel Tx)** | -200.0         | 200.0         | rad/s | 轮电机控制指令   |
| **Kp**            | 0.0            | 1000.0        | -     | 刚度系数         |
| **Kd**            | 0.0            | 1000.0        | -     | 阻尼系数         |
| **Voltage**       | 20.0           | 60.0          | V     | 电池电压         |

---

## 7. 遥控器键位映射 (Key BitMap)

*(参考自 SDK 解析逻辑)*

* **Byte 0 (Key1)**:
  * Bit 7: R1
  * Bit 6: L1
  * Bit 5: Start
  * Bit 4: Select
  * Bit 3: R2 (Digital)
  * Bit 2: L2 (Digital)
  * Bit 1: F1
  * Bit 0: F2
* **Byte 1 (Key2)**:
  * Bit 7: A
  * Bit 6: B
  * Bit 5: X
  * Bit 4: Y
  * Bit 3: Up
  * Bit 2: Right
  * Bit 1: Down
  * Bit 0: Left

---
