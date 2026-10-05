# Arm_Grabbing: SO-ARM101 机械臂混合控制与具身智能全栈系统 (Model-Based ➔ Learning-Based)

本项目基于开源桌面 6 轴机械臂 **SO-100M / SO-ARM101**，构建从**传统 Model-Based 动力学与运动学控制底座**到**具身智能 Learning-Based 模仿学习策略**的完整机械臂抓取系统。

> 📘 **阶段一保姆级精通讲义**：[docs/phase1_mastery_guide.md](docs/phase1_mastery_guide.md) （包含 ROS 2 核心通信、URDF/XACRO 语法逐行拆解、TF2 坐标树变换矩阵数学推导与自测题解答！）
> 
> 📘 **阶段二保姆级精通讲义**：[docs/phase2_mastery_guide.md](docs/phase2_mastery_guide.md) （**推荐进阶必读**：包含 Pinocchio 正逆运动学、DLS 阻尼最小二乘推导、五次多项式 3D 8字轨迹与 MoveIt 2/SRDF 架构逐行解读！）
> 
> 📖 **零基础保姆级教程与面试官必问硬核指南**：[docs/nanny_tutorial_and_interview_guide.md](docs/nanny_tutorial_and_interview_guide.md)
> 
> 📌 **5-6 个月全栈进阶路线与 JD 技能矩阵**：[docs/learning_roadmap.md](docs/learning_roadmap.md)

---

## 🛠️ 项目目录与 ROS 2 功能包架构

项目工作空间位置：`/home/freebird/AntigraProj/Arm_Grabbing`

```text
Arm_Grabbing/
├── docs/                       # 核心实战教程与路线文档
│   ├── phase1_mastery_guide.md    # 阶段一 ROS 2 与 URDF/TF2 坐标树保姆级精通讲义
│   ├── phase2_mastery_guide.md    # [重点进阶] 阶段二运动学算法、DLS IK、轨迹规划与 MoveIt 2 精通指南
│   ├── nanny_tutorial_and_interview_guide.md # 保姆级代码拆解与面试高频题库
│   ├── learning_roadmap.md        # 5-6个月机械臂开发工程师学习计划（先仿真后实物）
│   └── github_upload_tutorial.md  # GitHub 提交与推送教学指南
├── src/                        # ROS 2 功能包源码
│   ├── so_arm101_description/   # [阶段一已完成] 真实CAD URDF/XACRO 描述、3D Mesh 与 RViz 2 可视化
│   ├── so_arm101_moveit_config/ # [阶段二已完成] MoveIt 2 运动规划、SRDF 与多线程控制器配置
│   ├── so_arm101_model_based/   # [阶段二已完成] 空间数学、Pinocchio IK、五次多项式与 3D 8字形/圆弧轨迹生成
│   ├── so_arm101_teleop_data/   # [阶段三] 仿真与实物遥操作及 LeRobot 格式数据集采集
│   ├── so_arm101_hardware/      # [阶段四] STS3215 串口总线驱动、JointCalibration 适配器与硬件影子节点
│   └── so_arm101_learning_based/# [阶段五] ACT / Diffusion Policy ROS 2 实时推理与闭环 Demo
├── GLOSSARY.md                 # 机械臂领域核心术语表 (Domain Vocabulary)
├── .gitignore                  # Git 构建产物屏蔽规则
└── README.md                   # 本文档
```

---

## 🚀 快速开始与仿真实操验证 (Quick Start)

### 1. 编译工作空间
```bash
cd /home/freebird/AntigraProj/Arm_Grabbing

# 确保在系统 Python 环境下编译 ROS 2 (如安装了 Conda 需先 deactivate)
eval "$(conda shell.bash hook)" && conda deactivate
source /opt/ros/humble/setup.bash

colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 2. 运行自动化单元测试 (Unit Tests)
```bash
colcon test --event-handlers console_direct+
colcon test-result --all
```

### 3. 启动 URDF 机器人可视化与关节滑块控制 (仿真)
```bash
ros2 launch so_arm101_description display.launch.py
```

### 4. 启动传统控制 (正逆运动学 / DLS IK / 3D 8字形轨迹规划) 仿真
```bash
ros2 launch so_arm101_model_based kinematics_sim.launch.py
```

### 5. 启动 MoveIt 2 路径规划与 RViz 2 交互界面 (仿真)
```bash
ros2 launch so_arm101_moveit_config demo.launch.py
```

### 6. 启动真实硬件驱动 / 仿真影子同步 (Hardware Shadow)
```bash
ros2 launch so_arm101_hardware hardware_shadow.launch.py port:=/dev/ttyUSB0
```

---

## 📄 License
[Apache-2.0](LICENSE)
