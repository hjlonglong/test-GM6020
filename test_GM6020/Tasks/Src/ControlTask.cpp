/**
  ******************************************************************************
  * @file    ControlTask.cpp
  * @brief   GM6020 正弦角度跟踪 —— 串级(位置环 + 速度环) PID 实现
  ******************************************************************************
  */

#include "ControlTask.h"

#include <cmath>
#include <cstdint>

extern "C" {
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "main.h"
}

#include "Gm6020.h"
#include "Pid.h"

/* ==========================================================================
 * 一、可调参数
 * ========================================================================== */

namespace {

constexpr uint8_t kMotorId = 1;

constexpr float kOutMax = 25000.0f;
constexpr float kOutMin = -kOutMax;

// 斜坡限幅的满量程参考值(用于把"每秒爬升"换算成计数/拍)
constexpr float kOutFullScale = 25000.0f;

// ---- 正弦给定 ----
constexpr float kSinAmplitude = 3.0f;    // [rad]  1 圈 = 6.283 rad

constexpr float kSinFreqHz = 0.5f;       // [Hz]
constexpr float kSinOffset = 0.0f;       // [rad]

// ---- 采样周期 ----
constexpr float kSampleTime = 0.001f;    // [s]

/* ---- 外环: 位置环 ----*/
constexpr float kPosKp = 10.0f;
constexpr float kPosKi = 2.0f;
constexpr float kPosKd = 0.0f;

// 位置环输出限幅
constexpr float kVelRefMax = 30.0f;

/* ---- 内环: 速度环 ----*/
constexpr float kVelKp = 3700.0f;
constexpr float kVelKi = 1000.0f;
constexpr float kVelKd = 0.0f;

// ---- 内环输出限幅 ----
// 已在上方定义为 kOutMax / kOutMin (±25000 电压给定), 此处不再重复。

// ---- 速度前馈 ----
constexpr float kVoltPerRadS = 746.0f;

constexpr float kSlewRampTimeS     = 0.1f;    // 满量程爬升时间 [s]
constexpr float kOutputSlewPerStep = kOutFullScale * kSampleTime / kSlewRampTimeS;

// ---- 保护阈值 ----
constexpr float kMaxCurrentA = 3.0f;    
constexpr float kMaxTempC    = 80.0f;   

constexpr uint32_t kRxTimeoutTicks = 50;

// 调试变量刷新分频。
constexpr uint32_t kDbgDiv = 10;

// ---- 速度反馈低通滤波 ----

// 一阶低通(指数滑动平均), 每拍执行:
//     y += alpha * (x - y)
//   alpha = 1 - exp(-2*pi*fc*T)

constexpr float kVelFilterFcHz  = 50.0f;
constexpr float kVelFilterAlpha = 0.270f;   

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

}  // namespace

/* ==========================================================================
 * 二、模块级对象
 * ========================================================================== */

namespace {

Gm6020 g_motor(kMotorId);

// 外环
Pid g_posPid(kPosKp, kPosKi, kPosKd, kVelRefMax, -kVelRefMax);

// 内环
Pid g_velPid(kVelKp, kVelKi, kVelKd, kOutMax, kOutMin);

// 控制帧缓冲区
uint8_t g_txData[8] = {0};

// 正弦相位累加器
float g_phase = 0.0f;                    // [rad]

// 上一拍实际下发的命令值(计数), 用于斜坡限幅。
// 依模式不同含义不同: 电压模式=电压计数, 电流模式=电流计数。
float g_cmdPrev = 0.0f;

// 速度反馈低通滤波器的状态(上一拍的滤波输出)。
float g_velFiltered = 0.0f;

// 滤波器是否已用第一个有效测量值播种。
// 若从 0 开始, 上电瞬间反馈速度约 0 而滤波输出也是 0, 看似没问题;
// 但一旦电机在动, 滤波输出会从 0 慢慢爬到真实值, 期间速度环会
// 误以为"速度严重不足"而加大输出。播种可避免这个启动瞬态。
bool g_velFilterSeeded = false;

// 保护锁存标志: 一旦检测到异常就置位, 输出电压保持 0。
bool g_faultLatched = false;

// 通信存活检测。
uint32_t g_rxLastSeen = 0;
uint32_t g_rxStall    = 0;

// ---- 诊断变量(挂 Ozone / 调试器观察) ----
volatile float    g_dbgRef       = 0.0f;  // 目标角          [rad]
volatile float    g_dbgFdb       = 0.0f;  // 实际角          [rad]
volatile float    g_dbgErr       = 0.0f;  // 位置误差        [rad]
volatile float    g_dbgVelRef    = 0.0f;  // 外环输出的速度给定 [rad/s]
volatile float    g_dbgVelFdb    = 0.0f;  // 实际角速度      [rad/s]
volatile float    g_dbgVelFilt   = 0.0f;  // 滤波后的角速度  [rad/s] (送入速度环的那个)
volatile float    g_dbgVelErr    = 0.0f;  // 速度误差        [rad/s]
volatile float    g_dbgFF        = 0.0f;  // 前馈项          [计数]
volatile int16_t  g_dbgCmd       = 0;     // 实际下发的电压给定 [计数]
volatile float    g_dbgCurrent   = 0.0f;  // 反馈电流        [A]
volatile float    g_dbgTemp      = 0.0f;  // 反馈温度        [°C]
volatile uint32_t g_dbgRxCount   = 0;     // 累计收到的反馈帧数
volatile uint32_t g_dbgTickCount = 0;     // 累计进入 1kHz 中断次数
volatile uint32_t g_dbgTxFail    = 0;     // CAN 发送失败次数
volatile uint32_t g_dbgRxTimeout = 0;     // 通信超时触发次数
volatile uint32_t g_dbgFault     = 0;     // 保护触发次数

}  // namespace

/* ==========================================================================
 * 三、初始化
 * ========================================================================== */

extern "C" void ControlTaskInit(void)
{
    /* ---------------------------------------------------------------------
     * 3.1 CAN1 接收过滤器
     * --------------------------------------------------------------------- */
    const uint32_t rx_id   = g_motor.rxId();               // 0x205
    const uint32_t id_bits = (rx_id & 0x7FFu) << 21;       // STID 摆到 bit31..21

    // 掩码
    const uint32_t mask_bits = (0x7FFu << 21) | (1u << 2) | (1u << 1);

    CAN_FilterTypeDef filter = {0};
    filter.FilterIdHigh         = (id_bits   >> 16) & 0xFFFFu;   // 0x40A0
    filter.FilterIdLow          =  id_bits          & 0xFFFFu;   // 0x0000
    filter.FilterMaskIdHigh     = (mask_bits >> 16) & 0xFFFFu;   // 0xFFE0
    filter.FilterMaskIdLow      =  mask_bits        & 0xFFFFu;   // 0x0006
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = 0;
    
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation     = CAN_FILTER_ENABLE;

    /* 3.2 装过滤器。必须在 MX_CAN1_Init() 之后 ——
     * 该函数要求 hcan1.State 为 READY 或 LISTENING, 否则直接返回 HAL_ERROR。 */
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    /* 3.3 启动 CAN1: READY -> LISTENING, 硬件退出初始化模式。 */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    /* 3.4 打开接收中断。FIFO 编号必须与 FilterFIFOAssignment、
     * 以及取帧时的参数三处一致。 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    /* 3.5 启动 1 kHz 时基中断。
     * MX_TIM6_Init() 只配置时基, 不会自动开中断。 */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }

    /* ---------------------------------------------------------------------
     * 3.6 相位对齐 + 两个环复位
     *
     * 如果相位从 0 开始而电机停在别的角度, t=0 就是大阶跃误差,
     * 上电瞬间会猛冲一下。反解相位使 t=0 时 ref == 当前角度:
     *     ref = kSinOffset + kSinAmplitude * sin(phase) == angle
     *     => sin(phase) = (angle - kSinOffset) / kSinAmplitude
     * --------------------------------------------------------------------- */
    const float angle0 = g_motor.angle();
    if (kSinAmplitude > 1e-6f) {
        float s = (angle0 - kSinOffset) / kSinAmplitude;
        if (s >  1.0f) s =  1.0f;      // 超出正弦行程时钳位
        if (s < -1.0f) s = -1.0f;
        g_phase = asinf(s);
    } else {
        g_phase = 0.0f;
    }

    // 两个环都要复位! 只复位一个会在启停之间留下不一致的积分状态。
    g_posPid.reset();
    g_velPid.reset();

    g_cmdPrev      = 0.0f;
    g_rxLastSeen   = 0;
    g_rxStall      = 0;
    g_faultLatched = false;

    // 滤波器复位, 并取消播种标记 -> 第一个有效反馈会重新播种
    g_velFiltered    = 0.0f;
    g_velFilterSeeded = false;

    g_dbgRef = kSinOffset;
    g_dbgFdb = angle0;
}

/* ==========================================================================
 * 四、CAN 接收回调（在 CAN 中断里执行, 必须短）
 * ========================================================================== */

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    // 过滤器已只放行本电机 ID, 这里再判一次是防御性写法:
    // 万一以后改了过滤器却忘了同步这里的假设, 也能拦住错误解析。
    if (std_id != g_motor.rxId()) {
        return;
    }

    g_motor.decode(data);
    g_dbgRxCount += 1;      // volatile 上不能用 ++(C++20 起弃用)
}

/* ==========================================================================
 * 五、1 kHz 控制入口
 * ========================================================================== */

extern "C" void MainTask(void)
{
    g_dbgTickCount += 1;

    /* ---- 5.1 喂狗 -------------------------------------------------------
     * IWDG: LSI 32kHz / 64 分频, Reload 400 -> 超时 64*400/32000 = 0.8 s。
     * 放在 1 kHz 中断里, 让"控制回路还活着"成为喂狗条件。
     * ------------------------------------------------------------------- */
    HAL_IWDG_Refresh(&hiwdg);

    /* ---- 5.2 正弦给定 + 目标角速度 ------------------------------------- */
    g_phase += kTwoPi * kSinFreqHz * kSampleTime;
    if (g_phase >= kTwoPi) {
        g_phase -= kTwoPi;              // 取模, 防止长时间运行后精度下降
    }

    const float ref = kSinOffset + kSinAmplitude * sinf(g_phase);

    // 解析求导, 供前馈用:
    //   ref = offset + A*sin(2*pi*f*t)  ->  d(ref)/dt = A*2*pi*f*cos(2*pi*f*t)
    // 用解析导数而非数值微分: 没有噪声、没有相位滞后。
    const float ref_vel_ff = kSinAmplitude * kTwoPi * kSinFreqHz * cosf(g_phase);

    /* ---- 5.3 取反馈 -----------------------------------------------------
     * 位置用累计多圈角度(自己算的), 速度用电机的 rpm 换算值(电机直接给的)。
     * 两者来源不同, 但单位统一到 rad / rad/s, 可以同环使用。
     * ------------------------------------------------------------------- */
    const float fdb_angle = g_motor.angle();
    const float fdb_vel   = g_motor.vel();

    /* ---- 5.4 外环: 位置 -> 速度给定 ------------------------------------- */
    const float ref_vel = g_posPid.calc(ref, fdb_angle);

    /* ---- 5.5 内环: 速度 -> 命令值 ---------------------------------------
     * 速度环的给定 = 外环输出 + 前馈目标速度。
     *
     * 这是串级里最容易做错的一步: 前馈加在**内环给定**上, 而不是
     * 加在内环输出上。两者在稳态下等效, 但加在给定上时内环能"看到"
     * 前馈并据此调节, 抗扰性更好, 也不会让内环误以为存在速度误差。
     * ------------------------------------------------------------------- */
    const float vel_ref_total = ref_vel + ref_vel_ff;

    // 速度反馈低通滤波。注意:
    //   - 滤波只作用在【反馈】上, 不动给定。动给定会引入跟踪滞后。
    //   - 滤的是速度环的输入, 所以能同时抑制比例项和积分项对量化噪声的响应。
    //   - 第一拍用测量值播种, 避免滤波输出从 0 慢慢爬升造成启动瞬态。
    if (!g_velFilterSeeded) {
        g_velFiltered     = fdb_vel;      // 播种
        g_velFilterSeeded = true;
    } else {
        g_velFiltered += kVelFilterAlpha * (fdb_vel - g_velFiltered);
    }

    // 送进速度环的是【滤波后】的速度
    const float vel_loop_out  = g_velPid.calc(vel_ref_total, g_velFiltered);

    // 注意: 前馈是加在【内环给定】上的, 所以它不直接出现在 cmd 里,
    // 而是通过速度环的增益体现。kVoltPerRadS 只用于把这个速度前馈
    // 折算成"若直接前馈会是多少计数", 供调试对比, 不参与控制。
    const float ff_out = kVoltPerRadS * ref_vel_ff;
    (void)ff_out;

    // cmd 是电压给定计数, 会经 setVoltage() 按 ±25000 钳位
    float cmd = vel_loop_out;

    /* ---- 5.6 输出斜坡限幅 ----------------------------------------------- */
    const float dcmd = cmd - g_cmdPrev;
    if (dcmd >  kOutputSlewPerStep) cmd = g_cmdPrev + kOutputSlewPerStep;
    if (dcmd < -kOutputSlewPerStep) cmd = g_cmdPrev - kOutputSlewPerStep;

    /* ---- 5.7 输出硬限幅 ------------------------------------------------- */
    if (cmd > kOutMax) cmd = kOutMax;
    if (cmd < kOutMin) cmd = kOutMin;

    // 到这里才更新: 记录的是真正下发出去的受限值,
    // 下一拍的斜坡才是从实际值继续走。
    g_cmdPrev = cmd;

    /* ---- 5.8 保护判断 --------------------------------------------------- */
    const float current = g_motor.current();
    const float temp    = g_motor.temp();

    // 通信存活检测: g_dbgRxCount 不变 = 没有新反馈。
    // 此时继续输出电压就是开环乱转, 必须归零。
    if (g_dbgRxCount != g_rxLastSeen) {
        g_rxLastSeen = g_dbgRxCount;
        g_rxStall    = 0;
    } else {
        g_rxStall += 1;
        if (g_rxStall >= kRxTimeoutTicks) {
            g_dbgRxTimeout += 1;
            g_rxStall       = 0;
            g_faultLatched  = true;
        }
    }

    if (current > kMaxCurrentA || current < -kMaxCurrentA) {
        g_dbgFault += 1;
        g_faultLatched = true;
    }

    if (temp > kMaxTempC) {
        g_dbgFault += 1;
        g_faultLatched = true;
    }

    int16_t cmd_out = static_cast<int16_t>(cmd);
    if (g_faultLatched) {
        cmd_out    = 0;
        g_cmdPrev  = 0.0f;      // 保护后斜坡从 0 重新起步
        // 保护时把两个环都复位, 避免解除保护后残留积分导致冲击。
        g_posPid.reset();
        g_velPid.reset();
    }

    /* ---- 5.9 刷新调试变量(降频) ----------------------------------------- */
    if ((g_dbgTickCount % kDbgDiv) == 0) {
        g_dbgRef     = ref;
        g_dbgFdb     = fdb_angle;
        g_dbgErr     = ref - fdb_angle;
        g_dbgVelRef  = vel_ref_total;
        g_dbgVelFdb  = fdb_vel;                          // 原始反馈(未滤波)
        g_dbgVelFilt = g_velFiltered;                    // 滤波后(实际送入速度环)
        g_dbgVelErr  = vel_ref_total - g_velFiltered;    // 与速度环看到的误差一致
        g_dbgFF      = ff_out;
        g_dbgCmd     = cmd_out;
        g_dbgCurrent = current;
        g_dbgTemp    = temp;
    }

    /* ---- 5.10 下发电压并发送 CAN 帧 -------------------------------------
     * setVoltage 会按 ±25000 钳位, encode 写进 0x1FF 帧的对应字节。
     * ID 1~4 共用 0x1FF, 所以每周期只发一帧, 不是每台电机发一帧。
     * ------------------------------------------------------------------- */
    g_motor.setVoltage(cmd_out);
    g_motor.encode(g_txData);

    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = g_motor.txId();      // 0x1FF, 不是 rxId()
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx, g_txData, &mailbox) != HAL_OK) {
        // 三个发送邮箱都满 = 总线发不出去(断开/缺终端电阻/波特率不匹配)。
        // 正常 1kHz 发 8 字节远用不满 1Mbps, 所以只计数, 不中断控制 ——
        // 接收侧的通信超时保护会兜住真正的问题。
        g_dbgTxFail += 1;
    }
}

/* ==========================================================================
 * 六、把 HAL 的弱回调接到上面的函数
 * ========================================================================== */

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 这个弱回调是所有定时器共用的(不是 TIM6 专属), 必须判 Instance。
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) {
        return;
    }

    // 循环读空 FIFO0: 一次中断把积压的帧都取走, 否则高负载下会丢反馈。
    // 加上限防止异常情况下在中断里死循环。
    constexpr uint32_t kMaxFramesPerIrq = 8;

    CAN_RxHeaderTypeDef rx = {0};
    uint8_t data[8] = {0};

    for (uint32_t i = 0; i < kMaxFramesPerIrq; ++i) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &rx, data) != HAL_OK) {
            break;      // FIFO 空了, 正常退出
        }
        CanFeedbackCallback(rx.StdId, data);
    }
}
