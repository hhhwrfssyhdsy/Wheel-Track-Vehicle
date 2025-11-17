# 简介
本项目旨在开发基于轮履复合底盘以及六自由度舵机结构的小型救援机器人，上位机采用树莓派4B运行ROS2，下位机使用STM32F407控制底盘和舵机。

# 目录结构(开发中)
- `lowwer_computer` :下位机代码，包括底盘控制和硬件驱动代码
- `upper_computer` :上位机代码，主要为ros2的各个功能包、视觉识别等
    - `ros2_ws` :ros2工作区
        - `robot_bring_up` :机器人启动功能包
        - `robot_base_controller` :底盘控制功能包
        - `robot_arm_controller` :机械臂控制功能包
        - `robot_state_manager` :系统状态管理功能包
        - `robot_description` :机器人URDF模型和描述文件
        - `robot_teleop` :遥控操作功能包
        - `robot_sensor_fusion` :传感器数据融合功能包
        - `robot_sensor_drivers` :传感器驱动功能包
        - `robot_test` :单元测试包
    - `detection` :视觉识别相关代码
# 待办
- [ ] 电控
    - [ ] 硬件调试
        - [ ] 履带轮电机
        - [ ] 麦轮电机
        - [ ] 机械臂
    - [ ] 下位机控制
        - [ ] 履带轮控制
        - [ ] 麦轮控制
        - [ ] 机械臂控制
    
    - [ ] 上位机功能包开发
        - [ ] 整体机器人启动开发
        - [ ] 底盘控制开发
        - [ ] 机械臂控制开发
        - [ ] 系统状态管理
        - [ ] 机器人URDF模型和描述文件
        - [ ] 遥控操作
        - [ ] 传感器数据融合
        - [ ] 传感器驱动

- [ ] 视觉与导航
    - [ ] 路径规划
    - [ ] 视觉识别

- [ ] 综合调试
