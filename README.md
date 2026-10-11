<!--
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-10-11 16:24:37
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-10-11 16:27:45
 * @FilePath: \test_GM6020d:\rm\README_GM6020.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# GM6020 正弦角度跟踪

RoboMaster C 型开发板（STM32F407IGH6）通过 CAN1（1 Mbps）控制 DJI GM6020 电机按正弦规律往复转动。
采用位置环 + 速度环串级 PID，控制周期 1 kHz，含速度前馈与速度反馈低通滤波。

- `Tasks/Src/ControlTask.cpp` — 控制主流程：初始化 CAN 与 1 kHz 时基、接收电机反馈、生成正弦给定、串级 PID 运算、限幅与保护、下发电压指令，并输出调试变量

可调参数集中在 `ControlTask.cpp` 顶部：正弦幅值/频率、两个环的 PID 增益、限幅、保护阈值与滤波截止频率。