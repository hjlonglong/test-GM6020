#ifndef GM6020_H
#define GM6020_H

#include <cstdint>

// GM6020 直流无刷电机 CAN 协议封装类（电压控制模式）
//
// ============================== 控制 =================================
// 电压模式: 出厂默认, 无需升级固件。驱动器内部对转矩电压做闭环。
//     标识符 0x1FF (ID 1~4) / 0x2FF (ID 5~7), 标准帧, DLC = 8
//     每电机 2 字节 int16 电压给定, 范围 -25000 ~ +25000
//
//     ⚠ 前提: 电机的【电流环开关必须关闭】
//       (用 RoboMaster Assistant 在参数设置里关闭)。
//       若电机处于电流模式却收到本类的电压指令(0x1FF),
//       驱动器会丢弃该指令并让指示灯【橙灯常亮】
//       (手册指示灯表: "橙灯常亮 = 电流控制模式下接收到电压指令"),
//       表现为电机卡在固定角度不动, 而 CAN 反馈仍然正常接收。
//       正确配置下指示灯应为【绿灯每秒闪 N 次】(N = 电机 ID)。
//
// ============================== 反馈 =================================
// 标识符 0x204 + ID (ID=1 -> 0x205, ID=2 -> 0x206, ...), 发送频率 1 kHz
//     DATA[0..1] 机械角度 0~8191 (8192 = 一圈)
//     DATA[2..3] 转速 rpm (int16)
//     DATA[4..5] 实际转矩电流 (int16), ±16384 对应 ±3A
//     DATA[6]    电机温度 (°C)
//     DATA[7]    保留
//
// 注意 DATA[4..5] 是【实际转矩电流】, 是测量值而非指令值,
// 与命令模式无关, 始终可用(本工程用它做过流保护)。

class Gm6020 {
public:
    // 电压给定范围 ±25000 (手册)
    static constexpr int16_t kVoltageCmdMax = 25000;

    // 反馈转矩电流 ±16384 对应 ±3A (手册)
    static constexpr float kAmpPerRaw = 3.0f / 16384.0f;

    // id: 电机拨码开关设置的 ID (1~7), 对应 Bit[2:0]
    explicit Gm6020(uint8_t id);
    ~Gm6020() = default;

    // ---- 标识符 ----
    uint32_t rxId() const;   // 反馈标识符: 0x204 + id  (0x205 ~ 0x20B)
    uint32_t txId() const;   // 命令标识符: 0x1FF(id<=4) / 0x2FF(id>=5)

    // ---- 反馈 ----
    float   angle() const;     // 累计机械角度 (rad), 支持多圈连续
    float   vel() const;       // 转速 (rad/s)
    float   current() const;   // 实际转矩电流 (A), 由 DATA[4..5] 换算
    float   temp() const;      // 电机温度 (°C)
    int16_t velRpm() const;    // 原始转速 (rpm)
    int16_t rawCurrent() const;// 原始转矩电流计数

    // ---- 命令 ----
    // 参数为电压给定计数, 自动按 ±kVoltageCmdMax 钳位。
    void    setVoltage(int16_t v);
    int16_t voltage() const;

    // ---- 协议编解码 ----
    void decode(const uint8_t *data);   // 解析 8 字节反馈帧
    void encode(uint8_t *data) const;   // 把自己的 2 字节电压写进 8 字节命令帧

private:
    uint8_t id_;
    int16_t voltage_;

    uint16_t raw_angle_;      // 0~8191
    int16_t  raw_vel_rpm_;
    int16_t  raw_current_;
    uint8_t  raw_temp_;

    int32_t  full_turns_;     // 累计整圈数 (正转 +1, 反转 -1)
    uint16_t last_raw_angle_; // 上一拍原始角度, 用于跨零检测
};

#endif // GM6020_H
