/**
 * @file    main.c
 * @brief   STM32F103C8T6 + ULN2003 驱动板 + 28BYJ-48 云台 Demo — 主程序 (双轴)
 *
 * 功能概述:
 *   1. 双轴云台控制 (Pan 水平 + Tilt 垂直)
 *   2. 按键控制: 通过 MODE 切换活动轴
 *   3. 串口控制: 上位机发送指令控制指定电机
 *   4. 扫描模式: 自动来回摆动 (双轴)
 *   5. 步进模式: 支持全步进(大力矩)/半步进(平滑)/波驱动(省电)
 *
 * 硬件连线 (ULN2003 驱动板):
 *   ULN2003 #1 (Pan) : IN1→PA4, IN2→PA5, IN3→PA6, IN4→PA7
 *   ULN2003 #2 (Tilt): IN1→PB3, IN2→PB4, IN3→PB5, IN4→PB6
 *   28BYJ-48: 红(COM)→5V, 橙→A, 粉→B, 黄→C, 蓝→D
 *   ULN2003 板: 5-12V → 5V (供 28BYJ-48 COM), GND→GND
 *
 *   按键: PB12(UP加速) PB13(DOWN减速) PB14(DIR换向) PB15(MODE切换)
 *   MODE 循环: Pan手动 → Tilt手动 → 双轴扫描 → 串口模式
 *
 *   串口: PA9 TX / PA10 RX, 115200bps
 *   命令: p(选Pan) t(选Tilt) c(正转) a(反转)
 *         f(全步) h(半步) s(停) ?(状态)
 *         +N(前进步数) -N(后退步数) rN(旋转N°) wN(扫描±N°)
 *
 *   OLED (SSD1306, 辅助串口调试): I2C1 重映射到 PB8(SCL) / PB9(SDA), 地址 0x3C
 *        屏幕实时显示: 模式 / Pan·Tilt 状态 / 串口选中轴 / 最近命令(RX) / 回执(ACK)
 */

#include "main.h"
#include "stepper.h"
#include "ssd1306.h"
#include "oled_debug.h"
#include <stdlib.h>
#include <stdarg.h>      /* CmdReply() 需要 vsnprintf */
#include <string.h>      /* strcmp() 实时调参命令 g 需要 */
#include <stdio.h>       /* sscanf() 实时调参命令 g 需要 */

/* STM32 28BYJ-48 + ULN2003 驱动板已上电, 任何时候 4 个输入脚决定哪根线圈通电
 * (ULN2003 IN1=橙/IN2=粉/IN3=黄/IN4=蓝). 无 STBY 概念, 不用 gate 电流. */

/* ========================== 全局外设句柄 ========================== */
TIM_HandleTypeDef  htim2;       /* 步进定时器 (Pan) */
TIM_HandleTypeDef  htim3;       /* 步进定时器 (Tilt) */
UART_HandleTypeDef huart1;      /* 调试串口 */
UART_HandleTypeDef huart2;      /* 蓝牙串口 (JDY-31) */
UART_HandleTypeDef huart3;      /* K230 上位机串口 (PB10/PB11) */
I2C_HandleTypeDef  hi2c1;       /* OLED I2C 总线 (PB8/PB9) */

/* ========================== 电机实例 ========================== */
static StepperMotor g_motor_pan;    /* 水平电机 (Pan) */
static StepperMotor g_motor_tilt;   /* 垂直电机 (Tilt) */

/* ========================== 引脚配置表 ========================== */
static const StepperPins g_pan_pins = {
    .ain1_port = GPIOA, .ain1_pin = GPIO_PIN_4,
    .ain2_port = GPIOA, .ain2_pin = GPIO_PIN_5,
    .bin1_port = GPIOA, .bin1_pin = GPIO_PIN_6,
    .bin2_port = GPIOA, .bin2_pin = GPIO_PIN_7,
};

static const StepperPins g_tilt_pins = {
    .ain1_port = GPIOB, .ain1_pin = GPIO_PIN_3,
    .ain2_port = GPIOB, .ain2_pin = GPIO_PIN_4,
    .bin1_port = GPIOB, .bin1_pin = GPIO_PIN_5,
    .bin2_port = GPIOB, .bin2_pin = GPIO_PIN_6,
};

/* ========================== 云台控制状态 ========================== */

/** @brief 云台操作模式 */
typedef enum {
    GIMBAL_PAN     = 0,   /* 按键控制水平电机 (Pan) */
    GIMBAL_TILT    = 1,   /* 按键控制垂直电机 (Tilt) */
    GIMBAL_SWEEP   = 2,   /* 双轴自动扫描 */
    GIMBAL_SERIAL  = 3,   /* 串口控制 */
    GIMBAL_DANCE   = 4,   /* 舞蹈模式！云台蹦迪 */
    GIMBAL_AUTO    = 5,   /* K230 自动打靶模式 */
    GIMBAL_POSCTRL = 6,   /* 串口位置闭环控制模式 (默认) */
    GIMBAL_TRACE   = 7,   /* 矩形循迹测试模式 */
} GimbalMode;

static GimbalMode    g_gimbal_mode  = GIMBAL_POSCTRL;  /* 上电默认位置闭环, 串口调参 */
static StepperMotor *g_active_motor = &g_motor_pan;  /* 按键当前控制的电机 */
static StepperMotor *g_serial_motor = &g_motor_pan;  /* 串口当前选中的电机 */

/* ========================== 舞蹈模式状态 ========================== */
static uint32_t g_dance_phase_start = 0;   /* 当前舞蹈阶段的起始时间 */
static uint8_t  g_dance_phase       = 0;   /* 当前舞蹈阶段序号 */
static uint32_t g_dance_dir_toggle  = 0;   /* 甩头方向切换计时 */

/* ========================== 串口接收缓冲 (ISR → Main loop) ========================== */
#define SERIAL_BUF_SIZE  32
/* USART1 (CH340/调试口) 命令缓冲 */
static char            s_serial_buf[SERIAL_BUF_SIZE];
static volatile uint8_t s_serial_len  = 0;
static volatile uint8_t s_cmd_ready   = 0;

/* USART2 (蓝牙) 独立缓冲 — 防止两个接口的字节交叉污染同一缓冲区 */
static char            s_bt_buf[SERIAL_BUF_SIZE];
static volatile uint8_t s_bt_len      = 0;
static volatile uint8_t s_bt_cmd_ready = 0;

/* ========================== 命令反馈 (LED 视觉反馈) ========================== */
static uint32_t g_feedback_until = 0;   /* LED 保持亮到此时间戳 */

/* ========================== K230 上位机坐标帧接收 ========================== */
/** @brief 坐标帧解析状态 */
typedef enum {
    K230_IDLE      = 0,   /* 等待帧头 0x3C */
    K230_HEADER    = 1,   /* 等待 0x3B (坐标帧标识) */
    K230_DATA      = 2,   /* 收集 XH XL YH YL FLAG CRC (6字节) */
    K230_FOOTER1   = 3,   /* 等待 0x01 */
    K230_FOOTER2   = 4,   /* 等待帧尾 0x01 */
} K230FrameState;

static volatile K230FrameState g_k230_fstate = K230_IDLE;
static volatile uint8_t         g_k230_fbuf[6];    /* 坐标数据缓冲区(含CRC) */
static volatile uint8_t         g_k230_fidx  = 0;
static volatile int16_t         g_k230_err_x = 0;   /* Pan 方向误差 (像素) */
static volatile int16_t         g_k230_err_y = 0;   /* Tilt 方向误差 (像素) */
static volatile uint8_t         g_k230_flags = 0;   /* 标志位: bit0=valid, bit1=hit, bit2=hi_conf */
static volatile uint8_t         g_k230_frame_ready = 0;  /* 新帧到达标志 */
static volatile uint32_t        g_k230_last_frame = 0;   /* 最后一帧时间戳(ms) */

/* K230 超时 (2s 无有效帧 → 停止电机, 防乱跑) */
#define K230_FRAME_TIMEOUT_MS   2000
#define K230_ERROR_MAX_PX       400   /* 单帧最大合理误差 (画面半宽 800/2=400) */

/* K230 valid=0 时, 电机保持当前运动; 仅超时才停止 (v4.15: 去急停) */
static volatile uint32_t g_k230_last_valid = 0;  /* 最后有效帧时间戳(ms) */

/* CRC8 (poly 0x07, init 0x00) — 与 K230 端 MicroPython 实现逐位一致.
 * 用于坐标帧校验: 步进电机+TB6612 紧邻串口线, EMI 下静默误收会打偏,
 * 故坐标帧(而非状态帧回传)必须带校验. */
static uint8_t K230_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
            else            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static volatile uint32_t g_k230_crc_err = 0;  /* CRC 校验失败计数 (调试用) */

/* ========================== 1D 卡尔曼滤波器 ========================== */
/** @brief 一维卡尔曼滤波器 — 平滑 K230 像素误差 */
typedef struct {
    float x;            /* 状态估计 x(k|k) */
    float p;            /* 误差协方差 P(k|k) */
    float q;            /* 过程噪声 Q — 越大追踪越快, 越小越平滑 */
    float r;            /* 测量噪声 R — 越大越信任预测, 越小越信任测量 */
    uint8_t init;       /* 首次测量标志 */
} Kalman1D;

static float Kalman_Update(Kalman1D *kf, float z)
{
    if (!kf->init) {
        kf->x = z; kf->p = 1.0f; kf->init = 1;
        return kf->x;
    }
    /* 预测: x(k|k-1) = x(k-1|k-1),  P(k|k-1) = P(k-1|k-1) + Q */
    float p_pred = kf->p + kf->q;
    /* 更新: K(k) = P(k|k-1) / (P(k|k-1) + R) */
    float k_gain = p_pred / (p_pred + kf->r);
    kf->x += k_gain * (z - kf->x);
    kf->p = (1.0f - k_gain) * p_pred;
    return kf->x;
}

/* ---- 卡尔曼参数 (Q≈R, 平衡追踪与平滑) ---- */
/*    v4.3: Q=20, R=25 → 增益≈0.44, 不过度滤波 */
static Kalman1D g_kf_x = { .q = 20.0f, .r = 25.0f, .init = 0 };  /* Pan 误差 */
static Kalman1D g_kf_y = { .q = 20.0f, .r = 25.0f, .init = 0 };  /* Tilt 误差 */

/* ========================== PID 控制器 (v4.10 优化版) ========================== */
/**
 * @brief PID 控制器 — 像素误差 → 归一化控制量 (-1.0 ~ +1.0)
 *
 * === 优化要点 (v4.10) ===========================================
 *  1. 不完全微分 + 微分先行:
 *     - 微分对测量值取 (d_meas = (prev_meas - meas) / dt), 不放大设定值跳变
 *     - 一阶 IIR 低通: d_filt = α*d_filt_prev + (1-α)*d_raw
 *  2. 抗积分饱和 (Back-calculation):
 *     - 输出饱和时把 (out - out_lim) 按 Ktt 反向馈入积分器
 *     - 比单纯 clamping 退饱和快, 抑制过冲
 *  3. 死区内不积分:
 *     - |error| < deadzone 时不累加积分, 出死区不清零, 保持连续
 *  4. 段增益连续过渡 (3 段 + 线性插值):
 *     - 大误差: 高 Kp + 低 Ki (快逼近防过冲)
 *     - 中误差: 基础 Kp + 基础 Ki
 *     - 小误差: 低 Kp + 高 Ki (精细消静差)
 *  5. 积分分离:
 *     - 误差超过 integral_sep_th 时强制 Ki=0, 避免大误差下积分累积
 *  6. 输出限幅与积分限幅分离:
 *     - 积分限幅小, 抗饱和; 输出限幅大, 充分发挥 D 抑制超调
 */
typedef struct {
    /* ---- 基础增益 ---- */
    float kp;                 /* 比例基础增益 */
    float ki;                 /* 积分基础增益 */
    float kd;                 /* 微分基础增益 */

    /* ---- 段增益系数 (3 段: 远/中/近) ---- */
    float kp_far_scale;       /* 大误差段 Kp 系数 */
    float ki_far_scale;       /* 大误差段 Ki 系数 */
    float kp_mid_scale;       /* 中误差段 Kp 系数 */
    float ki_mid_scale;       /* 中误差段 Ki 系数 */
    float kp_near_scale;      /* 小误差段 Kp 系数 */
    float ki_near_scale;      /* 小误差段 Ki 系数 */
    float seg_far_th;         /* 大/中误差分界 (像素) */
    float seg_mid_th;         /* 中/小误差分界 (像素) */
    float seg_trans_width;    /* 段过渡平滑宽度 (像素) */

    /* ---- 抗积分饱和 ---- */
    float integral;           /* 积分累积 */
    float integral_limit;     /* 积分限幅 */
    float back_calc_gain;     /* 退饱和增益 Ktt (典型 0.5 ~ 2.0) */
    float integral_sep_th;    /* 积分分离阈值 (误差>此值则 Ki=0) */

    /* ---- 微分滤波 (不完全微分) ---- */
    float deriv_filtered;     /* 滤波后的微分 */
    float d_alpha;            /* 一阶 IIR 平滑系数 (0~1, 越大越平滑) */

    /* ---- 状态 ---- */
    float prev_error;         /* 上次误差 */
    float prev_measurement;   /* 上次测量值 (微分先行用) */
    uint32_t last_tick;       /* 上次更新 tick */
    uint8_t  initialized;

    /* ---- 死区 ---- */
    float deadzone;           /* 死区宽度 (像素), 此范围内输出为 0 且不积分 */

    /* ---- 输出限幅 ---- */
    float out_min, out_max;   /* 输出钳位范围 */
} K230_PID;

/**
 * @brief PID 更新 (内部自动计算 dt, 不依赖外部传入)
 * @param pid  PID 控制器
 * @param err  当前误差 (目标-测量 = 正数表示需要正转)
 * @return     控制输出 [-out_max, out_max]
 */
static float K230_PID_Update(K230_PID *pid, float err)
{
    /* ---- 速度控制器 (v5.0: 无积分, 根治过冲) ----
     *  输出 = Kp·误差  (符号含方向, 限幅 ±1) → 直接作为"目标速度指令"
     *  误差→0 时输出→0, 没有积分项, 故不会"刹不住 / 过冲靶心"。
     *  Kd 提供接近速度阻尼: 误差下降越快, 输出越小, 进一步抑制过冲。
     *  减速区(各轴 ki 字段, 单位 px)在 K230_ControlAxis 中把速度按 |误差|
     *  线性降到 0, 保证进入死区前已停稳。 */
    uint32_t now = HAL_GetTick();
    float dt = (now - pid->last_tick) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 1.0f)   dt = 1.0f;
    pid->last_tick = now;

    float out = pid->kp * err;                 /* 比例 → 速度指令 */

    float derr = (err - pid->prev_error) / dt; /* 误差变化率(接近速度) */
    pid->prev_error = err;
    out += pid->kd * derr;                     /* 阻尼: 接近越快, 输出越小 */

    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

/**
 * @brief 重置 PID 状态 (积分/微分/历史值), 切换目标/超时/限位时调用
 */
static void K230_PID_Reset(K230_PID *pid)
{
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->deriv_filtered = 0.0f;
    pid->last_tick      = HAL_GetTick();
    pid->initialized    = 1;
}

/**
 * @brief 初始化速度控制器 (v5.0)
 *  参数语义已重定义:
 *    kp  = 速度增益 (velocity gain), 越大越猛
 *    ki  = 减速区宽度(px) — 误差在此区间内速度线性降到 0, 越大越柔
 *    kd  = 接近阻尼 (0=关), 过冲严重时调到 0.002~0.005
 */
static void K230_PID_Init(K230_PID *pid,
                          float kp, float ki, float kd,
                          float deadzone)
{
    pid->kp = kp;  pid->ki = ki;  pid->kd = kd;

    pid->deadzone = deadzone;
    pid->out_min  = -1.0f;
    pid->out_max  =  1.0f;

    K230_PID_Reset(pid);
}

/* ---- 速度控制器参数 (v5.0) ----
 *   Kp = 速度增益;  Ki = 减速区宽度(px);  Kd = 接近阻尼(默认0)
 *   调参口诀: 追得慢→提 Kp; 仍过冲→调大 Ki(减速区)或加 Kd
 */
#define PAN_GAIN        1.0f    /* Pan 速度增益 */
#define PAN_DEC_ZONE    35.0f   /* Pan 减速区(px): 误差<35px 时速度线性降为0 */
#define TILT_GAIN       1.0f    /* Tilt 速度增益 */
#define TILT_DEC_ZONE   25.0f   /* Tilt 减速区(px) */
#define CTRL_KD         0.0f    /* 接近阻尼(默认关) */
#define PID_ILIMIT      20.0f
#define PID_DEADZONE    2       /* 死区 (像素) — Pan 轴 (收紧, 提升对中精度) */
#define PID_DEADZONE_TILT 3     /* Tilt 轴死区 (收紧, 与 Pan 接近) */

/* ---- Tilt 垂直轴专用限制 (防"激光打到天上") ---- */
/*   STEPPER_HALF_REV=4096 步 = 360°, 故 1 步 ≈ 0.088° */
#define TILT_DEADZONE      4       /* 垂直轴死区(像素) — 收紧, 避免停在离靶心 8px 处 */
#define TILT_MAX_STEPS     400     /* 垂直轴硬限位 = ±400步 ≈ ±35° (防打到天上) */
#define PAN_MAX_STEPS      0       /* 恢复水平轴自由旋转(与改前一致); 若需限位可设大值如 2048(≈±180°) */
#define TILT_SPEED_SCALE   0.6f    /* 垂直轴最高速度 = Pan 的 60% (小幅缓动) */

/* PID 实例: 字段全部用 K230_PID_Init 初始化 (见 main() 内调用) */
static K230_PID g_pid_pan;
static K230_PID g_pid_tilt;

/* ========================== 位置闭环 PID 控制器 (步进电机位置控制) ==========================
 * 输入: 目标位置(target_pos) - 当前位置(pos_steps)  单位: 步
 * 输出: 归一化速度指令 [-1.0, +1.0], 再映射到步间延时 (5~8ms/步合适)
 * 采用无积分 PD 控制, 避免积分过冲; 加梯形速度曲线(Stepper_UpdateTargetSpeed 实现).
 */
typedef struct {
    float kp;           /* 比例增益 (步→速度) */
    float ki;           /* 积分增益 — 用于消除长期静差 */
    float kd;           /* 微分阻尼 — 抑制接近目标时的过冲 */
    float prev_err;     /* 上次误差 (微分用) */
    float integral;     /* 积分累积 */
    float out_min, out_max; /* 输出限幅 */
    uint32_t last_tick;
    uint8_t  initialized;
} PosPID;

static void PosPID_Init(PosPID *pid, float kp, float ki, float kd)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
    pid->out_min = -1.0f;
    pid->out_max = 1.0f;
    pid->last_tick = HAL_GetTick();
    pid->initialized = 1;
}

static void PosPID_Reset(PosPID *pid)
{
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
    pid->last_tick = HAL_GetTick();
}

static float PosPID_Update(PosPID *pid, float err)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - pid->last_tick) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 1.0f) dt = 1.0f;
    pid->last_tick = now;

    /* 积分限幅, 防饱和 */
    pid->integral += err * dt;
    if (pid->integral >  5000.0f) pid->integral =  5000.0f;
    if (pid->integral < -5000.0f) pid->integral = -5000.0f;

    /* 微分: 误差变化率 */
    float derr = (err - pid->prev_err) / dt;
    pid->prev_err = err;

    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * derr;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

/* ---- 位置闭环默认参数 ----
 * 28BYJ-48: 4096步/圈, 1步≈0.088°.
 * Kp=0.0004: 误差200步(约17.6°) → 输出0.08 (较慢, 平稳)
 * Kp=0.001:  误差1000步(约88°) → 输出1.0 (全速)
 * 先用较保守值, 通过上位机 Kp 滑块加大. */
#define POS_KP_PAN      0.0006f
#define POS_KI_PAN      0.0000f
#define POS_KD_PAN      0.00005f
#define POS_KP_TILT     0.0006f
#define POS_KI_TILT     0.0000f
#define POS_KD_TILT     0.00005f

#define POS_DEADZONE_STEPS  2       /* 位置死区: 2步以内认为到位 */
#define POS_LIMIT_PAN_MIN   -2048   /* Pan ±180° */
#define POS_LIMIT_PAN_MAX   2048
#define POS_LIMIT_TILT_MIN  -400    /* Tilt ±35° */
#define POS_LIMIT_TILT_MAX  400

/* ---- 回程差补偿 (28BYJ-48 减速齿轮间隙) ----
 * 标准 28BYJ-48 64:1 减速箱回程差约 30~80 个半步步距.
 * 通过 serial 命令 'backlash <轴> <步数>' 可运行时调整并保存到 EEPROM.
 * 默认 50 步，对应约 4.4° 机械回程差。 */
#define BACKLASH_STEPS_DEFAULT  50
static int32_t g_backlash_pan  = BACKLASH_STEPS_DEFAULT;
static int32_t g_backlash_tilt = BACKLASH_STEPS_DEFAULT;
/* 上次运动方向 (用于检测换向) */
static Direction g_last_dir_pan  = DIR_CW;
static Direction g_last_dir_tilt = DIR_CW;

static PosPID g_pos_pid_pan;
static PosPID g_pos_pid_tilt;

/* 前向声明: 位置环辅助函数 */
static int32_t PosCtrl_AngleToSteps(float angle);
static float   PosCtrl_StepsToAngle(int32_t steps);
static void PosCtrl_BacklashComp(StepperMotor *motor, int32_t *target,
                                  Direction *last_dir, int32_t backlash_steps);
static void Trace_Init(int32_t half_size);
static void Trace_Update(void);

/* 目标位置 (步), 可串口/上位机设置 */
static int32_t g_pos_target_pan  = 0;
static int32_t g_pos_target_tilt = 0;

/* 输出 EMA 滤波, 让速度变化更平滑 */
static float g_pos_out_filt_pan  = 0.0f;
static float g_pos_out_filt_tilt = 0.0f;

/* ---- 矩形循迹测试 (GIMBAL_TRACE) ---- */
#define TRACE_SIZE_DEFAULT  300    /* 默认矩形半宽(步) ≈ ±26° */
#define TRACE_DWELL_MS      200    /* 每个角停留时间(ms) */
static uint8_t  g_trace_active = 0;      /* 循迹进行中 */
static uint8_t  g_trace_corner = 0;      /* 当前目标角 (0~3) */
static int32_t  g_trace_corners[4][2];   /* 4 个角 [Pan][Tilt] */
static uint32_t g_trace_last_ms = 0;     /* 到角时间戳 */

/* 丢步检测: 误差超过阈值且长时间不收敛则报警 */
#define POS_LOSTSTEP_THRESH 2000   /* 2000步≈176°, 连续超过认为严重丢步/限位异常 */
#define POS_LOSTSTEP_TIME_MS  3000   /* 持续3秒 */
static uint32_t g_pos_loststep_start = 0;
static uint8_t  g_pos_loststep_alarm = 0;

/** @brief 方向反转标志 (上电默认 Pan 反转向, 解决激光正反馈跑出视野) */
static uint8_t g_pan_dir_invert = 1;    /* 默认反转, 发 i 命令切换 */
static uint8_t g_tilt_dir_invert = 0;

/* ========================== OLED 调试显示缓冲 ========================== */
static char g_oled_cmd[20];             /* 最近收到的串口命令 (大写, 供 OLED 显示) */
static char g_oled_ack[20];             /* 最近回执: "OK" / "ERR" */

/* ========================== 系统时钟配置 ========================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================== GPIO 初始化 ========================== */

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* [修复] 解除 JTAG 复用，释放 PB3/PB4 作为普通 GPIO (Tilt 电机控制)
     * 用 NOJTAG 而不是 DISABLE: 仅关闭 JTAG, 保留 SWD (PA13/PA14) 调试接口,
     * 否则 DISABLE 会连 SWD 一起禁用, 导致后续无法再烧录/调试。 */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* ---- PA0, PA1: 未用 (保留为通用 GPIO, 上电拉低) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    /* ---- PA4~PA7: Pan 电机 ULN2003 驱动板 IN1~IN4 (橙/粉/黄/蓝) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);  /* 初始全关 (电机滑行) */

    /* ---- PB0: 未用 (原 STBY, 删) ---- */
    /* ---- PB1: 未用 ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    /* ---- PB3~PB6: Tilt 电机 ULN2003 驱动板 IN1~IN4 (橙/粉/黄/蓝) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /* ---- PB7: 未用 ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    /* ---- PB12~PB15: 按键 (上拉输入) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PC13: 板载 LED ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LED_OFF();
}

/* ========================== USART1 初始化 ========================== */

void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = DEBUG_USART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* [修复] 启用 RXNE 中断, 否则 ISR 不会因收到数据而触发 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/* ========================== USART2 初始化 (JDY-31 蓝牙) ========================== */

void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2: USART2 TX */
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3: USART2 RX */
    GPIO_InitStruct.Pin   = GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = BT_USART_BAUD;      /* 9600 (JDY-31 默认) */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* [修复] 启用 RXNE 中断, 否则蓝牙发来的命令不会被处理 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* ========================== 蓝牙(USART2) 非阻塞发送 + 链路状态 ==========================
 * [稳定性修复 1] 原 BT_SendReport / CmdReply 用 HAL_UART_Transmit 在 9600 波特下
 *   阻塞主循环约 250ms/次 → 控制帧处理被卡 → 云台抖动/命中率下降。
 *   改: 环形缓冲 + TXE 中断续发, 主循环只负责入队, 发送在后台完成。
 *   (对应架构评审 P1-2: 蓝牙/状态发送改非阻塞)
 */
#define BT_TX_RING_SIZE    256U
static volatile uint8_t  g_bt_tx_ring[BT_TX_RING_SIZE];
static volatile uint16_t g_bt_tx_head = 0;   /* 写指针(主循环入队) */
static volatile uint16_t g_bt_tx_tail = 0;   /* 读指针(TXE 中断发送) */
static volatile uint8_t  g_bt_tx_active = 0; /* 1=TXE 发送进行中 */

/* 链路状态: 用于连接可靠性指示与"重连"检测 */
static volatile uint8_t  g_bt_connected    = 0;   /* 收到过蓝牙数据=链路活跃 */
static volatile uint32_t g_bt_last_rx_tick = 0;   /* 最近一次收到蓝牙数据的 tick */
#define BT_LINK_LOST_MS   8000U   /* 超过该时间未收到蓝牙数据 → 判定链路断开 */

/** @brief 若当前未在发送且 ring 有数据, 启动发送(写首字节+使能 TXE 中断) */
static void BT_UART_Feed(void)
{
    if (g_bt_tx_active) return;
    if (g_bt_tx_head == g_bt_tx_tail) return;   /* 空 */
    g_bt_tx_active = 1;
    uint8_t b = g_bt_tx_ring[g_bt_tx_tail];
    g_bt_tx_tail = (g_bt_tx_tail + 1) % BT_TX_RING_SIZE;
    huart2.Instance->DR = b;                     /* 触发发送, TXE 清 0 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
}

/** @brief 非阻塞发送字符串(入队到环形缓冲) */
static void BT_UART_PutStr(const char *data, int len)
{
    if (!HAL_IS_BIT_SET(huart2.Instance->CR1, USART_CR1_UE)) return;
    __disable_irq();   /* 临界区: 防与 TXE ISR 竞争 ring 指针 */
    for (int i = 0; i < len; i++) {
        uint16_t next = (g_bt_tx_head + 1) % BT_TX_RING_SIZE;
        if (next == g_bt_tx_tail) {
            /* 满: 丢弃最旧一字节(覆盖), 保证最新状态能发出 */
            g_bt_tx_tail = (g_bt_tx_tail + 1) % BT_TX_RING_SIZE;
        }
        g_bt_tx_ring[g_bt_tx_head] = (uint8_t)data[i];
        g_bt_tx_head = next;
    }
    __enable_irq();
    BT_UART_Feed();
}

/** @brief 链路长时间丢失后的可选恢复: 复位蓝牙模块(需硬件接 RST 引脚)
 *        默认不定义 BT_RST_PORT → 空操作; 仅清除"已连接"标志, 等待对方重连。
 *        JDY-31 为从机, 真正的重连由对端(手机/PC)发起, 固件侧只做状态管理与复位辅助。 */
//#define BT_RST_PORT  GPIOA
//#define BT_RST_PIN   GPIO_PIN_0
static void BT_TryRecover(void)
{
#ifdef BT_RST_PORT
    static uint32_t last_rst = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_rst < 10000U) return;   /* 最多每 10s 复位一次, 避免抖动 */
    last_rst = now;
    HAL_GPIO_WritePin(BT_RST_PORT, BT_RST_PIN, GPIO_PIN_RESET);  /* 假设低有效复位 */
    HAL_Delay(50);
    HAL_GPIO_WritePin(BT_RST_PORT, BT_RST_PIN, GPIO_PIN_SET);
    printf("[BT] 已复位蓝牙模块, 等待重连...\r\n");
#endif
}

/* ========================== USART3 初始化 (K230 上位机) ========================== */

void USART3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB10: USART3 TX */
    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB11: USART3 RX */
    GPIO_InitStruct.Pin   = GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = K230_USART_BAUD;    /* 115200 */
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    /* 启用 RXNE 中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/* ========================== I2C1 初始化 (OLED PB8/PB9) ==========================
 * [重要] OLED 使用软件位敲 I2C (ssd1306.c 直接驱动 PB8/PB9 为 GPIO),
 *        因此这里【不能】初始化硬件 I2C1, 否则硬件 I2C 与软件 I2C 会在同一组引脚上
 *        互相抢驱动, 导致 SSD1306 收到错帧 → 整屏闪烁/花屏。
 *        故本函数留空, PB8/PB9 完全交给软件 I2C。
 */
void I2C1_Init(void)
{
    /* 故意留空: OLED 走软件 I2C, 不碰硬件 I2C1 外设与引脚复用 */
}

/* ========================== TIM2 初始化 (Pan 电机步进节拍) ========================== */

void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 72 - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = STEPPER_DEFAULT_DELAY - 1;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ========================== TIM3 初始化 (Tilt 电机步进节拍) ========================== */

void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = STEPPER_DEFAULT_DELAY - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ========================== NVIC 配置 ========================== */

static void NVIC_Config(void)
{
    /* USART1 中断优先级 (最高) */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART2 中断 (蓝牙) */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* USART3 中断 (K230) — 优先级最高 */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* SysTick 优先级 (最低) */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ========================== printf 重定向 ========================== */

#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    /* 调试输出走 USART1 (CH340), 非阻塞 10ms 超时.
     * 之前用 HAL_MAX_DELAY: UART TX 卡住 → 整个系统死锁 → CH340 无输出.
     * 超时则丢弃数据, 绝不阻塞主循环. */
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 10);
    return len;
}

/* 命令回显: 仅发蓝牙(USART2), 不污染 K230 接收缓冲.
 * 供 Serial_Execute() 的命令响应使用 (printf 已改为仅 USART1). */
static void CmdReply(const char *fmt, ...)
{
    static char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        BT_UART_PutStr(buf, n);   /* 非阻塞(环形缓冲+TXE中断), 不阻塞主循环 */
    }
}

/* stub: newlib-nano 不会实际调用，仅避免链接警告 */
int _read(int fd, char *ptr, int len)   { (void)fd; (void)ptr; (void)len; return 0; }
int _lseek(int fd, int ptr, int dir)    { (void)fd; (void)ptr; (void)dir; return 0; }
int _close(int fd)                      { (void)fd; return -1; }
int _fstat(int fd, void *buf)           { (void)fd; (void)buf; return -1; }
int _isatty(int fd)                     { (void)fd; return 1; }
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

/* ========================== 串口命令执行 (在主循环调用, 非 ISR) ========================== */

static void Serial_Execute(void)
{
    s_serial_buf[s_serial_len] = '\0';

    /* 跳过换行/空格 */
    char *p = s_serial_buf;
    while (*p == '\r' || *p == '\n' || *p == ' ') p++;
    if (*p == '\0') return;

    /* 拷贝到 OLED 显示缓冲 (转大写, 单色屏只支持 ASCII) */
    {
        char *d = g_oled_cmd; const char *s = p; int i = 0;
        while (*s && i < (int)sizeof(g_oled_cmd) - 1) {
            char c = *s;
            if (c >= 'a' && c <= 'z') c -= 32;
            *d++ = c; s++; i++;
        }
        *d = '\0';
    }
    snprintf(g_oled_ack, sizeof(g_oled_ack), "OK");

    /* ---- v5.1: 位置闭环命令 (优先处理, 不强制切 SERIAL 模式) ---- */
    if (*p == 'x' || *p == 'X') {
        float angle = 0.0f;
        if (sscanf(p + 1, "%f", &angle) == 1) {
            g_pos_target_pan = PosCtrl_AngleToSteps(angle);
            g_gimbal_mode = GIMBAL_POSCTRL;
            g_feedback_until = HAL_GetTick() + 150;
            CmdReply("# OK Pan 目标 %.1f° (%ld 步)\r\n", (double)angle, (long)g_pos_target_pan);
        } else {
            CmdReply("# ERR 格式: x<角度>, 如 x90 或 x-45\r\n");
        }
        return;
    }
    if (*p == 'y' || *p == 'Y') {
        float angle = 0.0f;
        if (sscanf(p + 1, "%f", &angle) == 1) {
            g_pos_target_tilt = PosCtrl_AngleToSteps(angle);
            g_gimbal_mode = GIMBAL_POSCTRL;
            g_feedback_until = HAL_GetTick() + 150;
            CmdReply("# OK Tilt 目标 %.1f° (%ld 步)\r\n", (double)angle, (long)g_pos_target_tilt);
        } else {
            CmdReply("# ERR 格式: y<角度>, 如 y30 或 y-10\r\n");
        }
        return;
    }
    if (*p == 'z' || *p == 'Z') {
        Stepper_ZeroPosition(&g_motor_pan);
        Stepper_ZeroPosition(&g_motor_tilt);
        g_pos_target_pan = 0;
        g_pos_target_tilt = 0;
        PosPID_Reset(&g_pos_pid_pan);
        PosPID_Reset(&g_pos_pid_tilt);
        g_gimbal_mode = GIMBAL_POSCTRL;
        g_feedback_until = HAL_GetTick() + 150;
        CmdReply("# OK 当前位置已归零, 目标清零\r\n");
        return;
    }
    if (strncmp(p, "pid", 3) == 0 || strncmp(p, "PID", 3) == 0) {
        char axis = 0;
        char param[4] = {0};
        float val = 0.0f;
        if (sscanf(p + 3, " %c %3s %f", &axis, param, &val) == 3) {
            PosPID *pid = ((axis | 0x20) == 't') ? &g_pos_pid_tilt : &g_pos_pid_pan;
            const char *name = ((axis | 0x20) == 't') ? "Tilt" : "Pan";
            if      (strcmp(param, "kp") == 0) pid->kp = val;
            else if (strcmp(param, "ki") == 0) pid->ki = val;
            else if (strcmp(param, "kd") == 0) pid->kd = val;
            else {
                CmdReply("# ERR 未知参数 '%s' (用 kp/ki/kd)\r\n", param);
                return;
            }
            g_gimbal_mode = GIMBAL_POSCTRL;
            g_feedback_until = HAL_GetTick() + 150;
            CmdReply("# OK %s %s = %.6f (实时生效)\r\n", name, param, (double)val);
        } else {
            CmdReply("# ERR 格式: pid <p|t> <kp|ki|kd> <值>\r\n");
            CmdReply("  例: pid p kp 0.001\r\n");
        }
        return;
    }
    if (*p == 'k' || *p == 'K') {
        g_gimbal_mode = GIMBAL_AUTO;
        g_feedback_until = HAL_GetTick() + 150;
        CmdReply("# OK 切换到 K230 自动打靶模式\r\n");
        return;
    }
    if (*p == 'm' || *p == 'M') {
        g_gimbal_mode = GIMBAL_POSCTRL;
        g_feedback_until = HAL_GetTick() + 150;
        CmdReply("# OK 切换回位置闭环模式\r\n");
        return;
    }
    if (strncmp(p, "backlash", 8) == 0 || strncmp(p, "BACKLASH", 8) == 0) {
        char axis = 0;
        int steps = 0;
        if (sscanf(p + 8, " %c %d", &axis, &steps) == 2) {
            if ((axis | 0x20) == 'p') { g_backlash_pan = steps; }
            else if ((axis | 0x20) == 't') { g_backlash_tilt = steps; }
            else { CmdReply("# ERR 轴: p=Pan, t=Tilt\r\n"); return; }
            g_feedback_until = HAL_GetTick() + 150;
            CmdReply("# OK %s 回程差 = %d 步\r\n", ((axis|0x20)=='p')?"Pan":"Tilt", steps);
        } else {
            CmdReply("# OK Pan=%ld步 Tilt=%ld步\r\n", (long)g_backlash_pan, (long)g_backlash_tilt);
        }
        return;
    }
    /* ---- 矩形循迹测试命令 ---- */
    if (strncmp(p, "trace", 5) == 0 || strncmp(p, "TRACE", 5) == 0) {
        int size = TRACE_SIZE_DEFAULT;
        sscanf(p + 5, "%d", &size);
        Trace_Init(size);
        CmdReply("# OK 矩形循迹 ±%d步 (4角)\r\n", size);
        return;
    }
    if (strncmp(p, "stop", 4) == 0 || strncmp(p, "STOP", 4) == 0) {
        g_trace_active = 0;
        Stepper_Stop(&g_motor_pan);
        Stepper_Stop(&g_motor_tilt);
        g_gimbal_mode = GIMBAL_POSCTRL;
        CmdReply("# OK 停止\r\n");
        return;
    }

    /* ---- K230 自动打靶模式: 锁定模式, 忽略串口误触 ----
     * 防止 CH340/蓝牙的 stray 字节把模式从 AUTO 踢出,
     * 导致 [K230] → 自动打靶 反复切进切出. */
    if (g_gimbal_mode == GIMBAL_AUTO) {
        CmdReply("# AUTO 忽略: %s\r\n", s_serial_buf);
        return;
    }

    /* ---- 串口接管 + 视觉反馈 ----
     * 其他串口命令先把云台切到 SERIAL 模式, 避免被 Pan/Sweep/Dance 的主循环逻辑覆盖；
     * 同时让板载 LED 亮 150ms，给出明确的"已收到"视觉反馈。 */
    g_gimbal_mode = GIMBAL_SERIAL;
    g_feedback_until = HAL_GetTick() + 150;
    LED_ON();
    CmdReply("\r\n> %s\r\n", s_serial_buf);   /* 回显用户发出的命令 (走蓝牙) */

    /* ---- 单字符命令 (不区分大小写) ---- */
    switch (*p | 0x20) {
        case 'p':   /* 选择 Pan 电机 */
            g_serial_motor = &g_motor_pan;
            CmdReply("# OK → 选中 Pan 电机 (水平)\r\n");
            return;
        case 't':   /* 选择 Tilt 电机 */
            g_serial_motor = &g_motor_tilt;
            CmdReply("# OK → 选中 Tilt 电机 (垂直)\r\n");
            return;
        case 'f':
            Stepper_SetMode(g_serial_motor, STEP_MODE_FULL);
            CmdReply("# OK 全步步进 (大力矩)\r\n");
            return;
        case 'h':
            Stepper_SetMode(g_serial_motor, STEP_MODE_HALF);
            CmdReply("# OK 半步步进 (平滑)\r\n");
            return;
        case 'c':   /* 连续正转 */
            Stepper_SetDirection(g_serial_motor, DIR_CW);
            Stepper_RunContinuously(g_serial_motor);
            CmdReply("# OK %s 连续正转 (CW)\r\n", g_serial_motor->label);
            return;
        case 'a':   /* 连续反转 */
            Stepper_SetDirection(g_serial_motor, DIR_CCW);
            Stepper_RunContinuously(g_serial_motor);
            CmdReply("# OK %s 连续反转 (CCW)\r\n", g_serial_motor->label);
            return;
        case 's':   /* 停止 */
            Stepper_Stop(g_serial_motor);
            CmdReply("# OK %s 停止\r\n", g_serial_motor->label);
            return;
        case 'd':   /* 进入舞蹈模式 */
            Stepper_RunContinuously(&g_motor_pan);
            Stepper_RunContinuously(&g_motor_tilt);
            g_gimbal_mode = GIMBAL_DANCE;
            g_dance_phase_start = 0;
            CmdReply("# OK 进入舞蹈模式 🎵\r\n");
            return;
        case 'i':   /* 切换 Pan 方向反转 */
            g_pan_dir_invert ^= 1;
            CmdReply("# OK Pan 方向反转: %s\n",
                   g_pan_dir_invert ? "ON (已反转)" : "OFF (正常)");
            return;
        case 'I':   /* 切换 Tilt 方向反转 */
            g_tilt_dir_invert ^= 1;
            CmdReply("# OK Tilt 方向反转: %s\n",
                   g_tilt_dir_invert ? "ON (已反转)" : "OFF (正常)");
            return;
        case 'P':   /* 打印速度控制器状态 (保留 Kp=/Ki=/Kd= 格式供上位机解析) */
            CmdReply("# Pan  PID: Kp=%.3f Ki=%.1f Kd=%.4f\n",
                     g_pid_pan.kp, g_pid_pan.ki, g_pid_pan.kd);
            CmdReply("  deadzone=%d invert=%d\n", PID_DEADZONE, g_pan_dir_invert);
            CmdReply("# Tilt PID: Kp=%.3f Ki=%.1f Kd=%.4f\n",
                     g_pid_tilt.kp, g_pid_tilt.ki, g_pid_tilt.kd);
            CmdReply("  deadzone=%d invert=%d\n", PID_DEADZONE_TILT, g_tilt_dir_invert);
            return;
        case '?':
            Stepper_PrintStatus(&g_motor_pan);    /* 详细步进状态→USART1(调试) */
            Stepper_PrintStatus(&g_motor_tilt);
            CmdReply("串口当前选中: %s\r\n", g_serial_motor->label);
            CmdReply("# OK 状态已上报 (详情见调试串口 USART1)\r\n");
            return;
        case 'g': {   /* 实时调参: g <p|t> <kp|ki|kd> <值> — 上位机 PID 调参面板用 */
            char axis = 0; char param[4] = {0}; float val = 0;
            if (sscanf(p + 1, "%c %3s %f", &axis, param, &val) == 3) {
                int isTilt = ((axis | 0x20) == 't');
                K230_PID *pid = isTilt ? &g_pid_tilt : &g_pid_pan;
                if      (strcmp(param, "kp") == 0) { if (val<0.1f)val=0.1f; if(val>3.0f)val=3.0f; pid->kp = val; }
                else if (strcmp(param, "ki") == 0) { if (val<10.0f)val=10.0f; if(val>120.0f)val=120.0f; pid->ki = val; }
                else if (strcmp(param, "kd") == 0) { if (val<0.0f)val=0.0f; if(val>0.02f)val=0.02f; pid->kd = val; }
                else { CmdReply("# ERR 未知参数 '%s' (用 kp/ki/kd)\r\n", param); return; }
                CmdReply("# OK %s %s = %.4f (实时生效, 断电不保存)\r\n",
                         isTilt ? "Tilt" : "Pan", param, (double)val);
            } else {
                CmdReply("# ERR 格式: g <p|t> <kp|ki|kd> <值>\r\n");
            }
            return;
        }
        default:
            break;
    }

    /* ---- 带数字参数的命令: +N, -N, rN, wN, vN ---- */
    char type = *p;
    char *num_str = p + 1;
    int32_t num = atoi(num_str);

    switch (type) {
        case '+':
            Stepper_SetDirection(g_serial_motor, DIR_CW);
            Stepper_StepBy(g_serial_motor, (uint32_t)num);
            CmdReply("# OK %s 前进 %ld 步\r\n", g_serial_motor->label, (long)num);
            break;
        case '-':
            Stepper_SetDirection(g_serial_motor, DIR_CCW);
            Stepper_StepBy(g_serial_motor, (uint32_t)num);
            CmdReply("# OK %s 后退 %ld 步\r\n", g_serial_motor->label, (long)num);
            break;
        case 'r':
        case 'R':
            Stepper_RotateDegrees(g_serial_motor, (float)num);
            CmdReply("# OK %s 旋转 %ld°\r\n", g_serial_motor->label, (long)num);
            break;
        case 'w':
            Stepper_SweepEnable(g_serial_motor, (uint32_t)num);
            CmdReply("# OK %s 扫描 ±%ld°\r\n", g_serial_motor->label, (long)(num / 2));
            break;
        case 'v':
        case 'V': {
            uint32_t d = (uint32_t)num;
            if (d < STEPPER_MIN_DELAY_US) d = STEPPER_MIN_DELAY_US;
            if (d > STEPPER_MAX_DELAY_US) d = STEPPER_MAX_DELAY_US;
            Stepper_SetSpeed(g_serial_motor, d);
            CmdReply("# OK 速度 %lu µs/步 (越小越快)\r\n", (unsigned long)d);
            break;
        }
        default:
            CmdReply("# ERR 未知命令 '%c' → 输入 ? 查看帮助\r\n", *p);
            snprintf(g_oled_ack, sizeof(g_oled_ack), "ERR");
            break;
    }
}

/* ========================== 位置闭环控制核心 ========================== */

static void PosCtrl_SetTarget(StepperMotor *motor, int32_t target_pos)
{
    Stepper_MoveToPosition(motor, target_pos);
}

static float PosCtrl_RunAxis(StepperMotor *motor, PosPID *pid,
                              int32_t target_pos, float *out_filter,
                              float max_speed_scale, const char *label)
{
    int32_t err = target_pos - motor->pos_steps;
    float err_f = (float)err;

    /* 丢步/异常检测: 误差极大且持续 */
    if (abs(err) > POS_LOSTSTEP_THRESH) {
        if (g_pos_loststep_start == 0) {
            g_pos_loststep_start = HAL_GetTick();
        } else if (HAL_GetTick() - g_pos_loststep_start > POS_LOSTSTEP_TIME_MS) {
            if (!g_pos_loststep_alarm) {
                g_pos_loststep_alarm = 1;
                printf("\r\n[ALARM] %s 严重丢步/限位异常: 误差 %ld 步持续 %d 秒\r\n",
                       label, (long)err, POS_LOSTSTEP_TIME_MS / 1000);
            }
        }
    } else {
        g_pos_loststep_start = 0;
        g_pos_loststep_alarm = 0;
    }

    /* 位置死区: 2步以内直接停 */
    if (abs(err) <= POS_DEADZONE_STEPS) {
        if (Stepper_GetState(motor) != STATE_IDLE) {
            Stepper_Stop(motor);
        }
        PosPID_Reset(pid);
        *out_filter = 0.0f;
        return 0.0f;
    }

    /* 确保电机在运行(方向由目标位置决定) */
    if (Stepper_GetState(motor) == STATE_IDLE) {
        PosCtrl_SetTarget(motor, target_pos);
    } else {
        /* 运行中同步目标位置 */
        motor->target_pos = target_pos;
    }

    /* PID 计算速度指令 [-1, 1] */
    float vel = PosPID_Update(pid, err_f);
    if (vel >  1.0f) vel =  1.0f;
    if (vel < -1.0f) vel = -1.0f;

    /* 速度幅值 → 步间延时
     * vel=0 → 最慢(8000us); vel=±1 → 最快(5000us)
     * 再乘以 max_speed_scale 限制该轴最高速 (Tilt 0.7)
     */
    float vel_mag = (vel > 0.0f) ? vel : -vel;
    vel_mag *= max_speed_scale;
    if (vel_mag < 0.0f) vel_mag = 0.0f;
    if (vel_mag > 1.0f) vel_mag = 1.0f;

    uint32_t range = STEPPER_MAX_DELAY_US - STEPPER_MIN_DELAY_US;
    uint32_t delay_us = STEPPER_MAX_DELAY_US - (uint32_t)(vel_mag * (float)range);
    if (delay_us < STEPPER_MIN_DELAY_US) delay_us = STEPPER_MIN_DELAY_US;
    if (delay_us > STEPPER_MAX_DELAY_US) delay_us = STEPPER_MAX_DELAY_US;

    /* 输出 EMA 平滑 (0.7/0.3, 让梯形加减速更柔) */
    if (*out_filter == 0.0f) *out_filter = (float)delay_us;
    else *out_filter = *out_filter * 0.7f + (float)delay_us * 0.3f;
    uint32_t final_delay = (uint32_t)(*out_filter + 0.5f);
    if (final_delay < STEPPER_MIN_DELAY_US) final_delay = STEPPER_MIN_DELAY_US;
    if (final_delay > STEPPER_MAX_DELAY_US) final_delay = STEPPER_MAX_DELAY_US;

    Stepper_UpdateTargetSpeed(motor, final_delay);
    return vel;
}

static void PosCtrl_Update(void)
{
    /* 回程差补偿 */
    int32_t pan_target  = g_pos_target_pan;
    int32_t tilt_target = g_pos_target_tilt;
    PosCtrl_BacklashComp(&g_motor_pan,  &pan_target,  &g_last_dir_pan,  g_backlash_pan);
    PosCtrl_BacklashComp(&g_motor_tilt, &tilt_target, &g_last_dir_tilt, g_backlash_tilt);

    /* 同步目标位置到电机结构体 */
    g_motor_pan.target_pos  = pan_target;
    g_motor_tilt.target_pos = tilt_target;

    PosCtrl_RunAxis(&g_motor_pan,  &g_pos_pid_pan,  pan_target,  &g_pos_out_filt_pan,  1.0f, "Pan");
    PosCtrl_RunAxis(&g_motor_tilt, &g_pos_pid_tilt, tilt_target, &g_pos_out_filt_tilt, 0.7f, "Tilt");
}

/* 角度 → 步数: 1步≈0.08789°, 4096步/圈 */
static int32_t PosCtrl_AngleToSteps(float angle)
{
    return (int32_t)(angle * (float)STEPPER_HALF_REV / 360.0f);
}

static float PosCtrl_StepsToAngle(int32_t steps)
{
    return (float)steps * 360.0f / (float)STEPPER_HALF_REV;
}

/* 位置闭环遥测: 100ms 发送一次 CSV 到 USART1 (CH340 调试口) */
static uint32_t pos_telemetry_tick = 0;

/**
 * @brief 回程差补偿: 检测到电机换向时, 在目标位置基础上多走 backlash 步
 * 
 * 28BYJ-48 的 64:1 减速齿轮存在 30~80 步的回程间隙.
 * 换向时前 N 步在"吃间隙", 输出轴不动.
 * 本函数在换向时临时把目标拉远 backlash 步, 确保间隙吃满后输出轴才开始动.
 * 副作用: 方向持续不变时误差无影响; 方向切换时位置计数会差 backlash 步,
 * 这是开环回程差的固有现象, PID 会弥补残余误差.
 */
static void PosCtrl_BacklashComp(StepperMotor *motor, int32_t *target,
                                  Direction *last_dir, int32_t backlash_steps)
{
    int32_t err = *target - motor->pos_steps;
    Direction want_dir;
    if (err > 4)      want_dir = DIR_CW;
    else if (err < -4) want_dir = DIR_CCW;
    else              return;  /* 死区内不触发换向补偿 */

    if (want_dir != *last_dir) {
        /* 换向: 在目标方向多吃 backlash 步 */
        if (want_dir == DIR_CW)
            *target += backlash_steps;
        else
            *target -= backlash_steps;
        /* printf("[%s] 回程差补偿 %+ld 步\n", motor->label, 
               (long)((want_dir==DIR_CW)?backlash_steps:-backlash_steps)); */
    }
    *last_dir = want_dir;
}

static void PosCtrl_SendTelemetry(void)
{
    if (HAL_GetTick() - pos_telemetry_tick < 100) return;
    pos_telemetry_tick = HAL_GetTick();

    float pa = PosCtrl_StepsToAngle(g_motor_pan.pos_steps);
    float ta = PosCtrl_StepsToAngle(g_motor_tilt.pos_steps);
    float p_err = (float)(g_pos_target_pan  - g_motor_pan.pos_steps);
    float t_err = (float)(g_pos_target_tilt - g_motor_tilt.pos_steps);

    char buf[120];
    int n = snprintf(buf, sizeof(buf),
        "POS,%.1f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f,%.3f\r\n",
        pa, PosCtrl_StepsToAngle(g_pos_target_pan), p_err, g_pos_out_filt_pan,
        ta, PosCtrl_StepsToAngle(g_pos_target_tilt), t_err, g_pos_out_filt_tilt);
    if (n > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 10);
    }
}

/* ========================== 矩形循迹测试模式 ========================== */

static void Trace_Init(int32_t half_size)
{
    if (half_size < 50)  half_size = 50;
    if (half_size > 2048) half_size = 2048;

    int32_t cx = g_motor_pan.pos_steps;
    int32_t cy = g_motor_tilt.pos_steps;

    /* 4 个角: 以当前位置为中心, 逆时针绕矩形 */
    g_trace_corners[0][0] = cx - half_size;  g_trace_corners[0][1] = cy - half_size;
    g_trace_corners[1][0] = cx + half_size;  g_trace_corners[1][1] = cy - half_size;
    g_trace_corners[2][0] = cx + half_size;  g_trace_corners[2][1] = cy + half_size;
    g_trace_corners[3][0] = cx - half_size;  g_trace_corners[3][1] = cy + half_size;

    g_trace_corner   = 0;
    g_trace_active   = 1;
    g_trace_last_ms  = HAL_GetTick();
    g_gimbal_mode    = GIMBAL_TRACE;

    /* 设置第一个角为位置闭环目标 */
    g_pos_target_pan  = g_trace_corners[0][0];
    g_pos_target_tilt = g_trace_corners[0][1];
    PosPID_Reset(&g_pos_pid_pan);
    PosPID_Reset(&g_pos_pid_tilt);

    printf("[TRACE] 启动矩形循迹 ±%ld步 (当前角%d)\r\n",
           (long)half_size, g_trace_corner);
}

static void Trace_Update(void)
{
    if (!g_trace_active || g_gimbal_mode != GIMBAL_TRACE) {
        g_trace_active = 0;
        return;
    }

    /* 两个轴都到达当前角附近(死区5步) 且 停留足够久 → 切下一角 */
    int32_t dx = g_trace_corners[g_trace_corner][0] - g_motor_pan.pos_steps;
    int32_t dy = g_trace_corners[g_trace_corner][1] - g_motor_tilt.pos_steps;
    uint32_t now = HAL_GetTick();

    if (abs(dx) <= 5 && abs(dy) <= 5) {
        if (now - g_trace_last_ms >= TRACE_DWELL_MS) {
            /* 到达! 切下一角 */
            g_trace_corner = (g_trace_corner + 1) % 4;
            g_pos_target_pan  = g_trace_corners[g_trace_corner][0];
            g_pos_target_tilt = g_trace_corners[g_trace_corner][1];
            g_trace_last_ms = now;
            printf("[TRACE] 角%d 目标(%ld,%ld)\r\n",
                   g_trace_corner,
                   (long)g_pos_target_pan, (long)g_pos_target_tilt);
        }
    } else {
        g_trace_last_ms = now;  /* 没到就刷新计时 */
    }

    /* 更新位置闭环控制（双轴同时逼近当前角） */
    PosCtrl_Update();
}

/**
 * @brief 向 K230 回传云台状态帧
 *        格式: 0x3C 0x3D [YAW] [PITCH] [STATE] [FLAGS] [PAD] [CKSUM]
 *        K230 端 recv_status() 解析: yaw=b2*270/255, pitch=b3*180/255, state=b4
 */
static void K230_SendStatus(void)
{
    uint8_t buf[8];
    buf[0] = 0x3C;
    buf[1] = 0x3D;
    /* Yaw: Pan 位置估算 (0-255 → 0-270°) — 使用步序索引作为简易位置指示 */
    buf[2] = (uint8_t)(g_motor_pan.step_index * 32);
    /* Pitch: Tilt 位置估算 */
    buf[3] = (uint8_t)(g_motor_tilt.step_index * 32);
    /* State: 双轴平均状态 */
    {
        uint8_t ps = (uint8_t)g_motor_pan.state;
        uint8_t ts = (uint8_t)g_motor_tilt.state;
        buf[4] = (ps + ts) / 2;
    }
    /* Flags: bit0=running, bit1=K230 mode active */
    buf[5] = (Stepper_IsRunning(&g_motor_pan) ? 0x01 : 0x00) |
             ((g_gimbal_mode == GIMBAL_AUTO) ? 0x02 : 0x00);
    buf[6] = 0;  /* 填充字节 */
    buf[7] = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]) & 0xFF;
    HAL_UART_Transmit(&huart3, buf, 8, 10);
}

/* ---- v4.3: PID 输出低通滤波 (每轴) ---- */
static float g_pan_out_filter = 0.0f;
static float g_tilt_out_filter = 0.0f;  /* Tilt 输出 EMA 滤波 */

/**
 * @brief 控制单个轴 — PID + 方向反转 + 速度映射 (v4.3)
 *
 * v4.3 关键修复:
 *   1. 移除 abs_out³ 立方映射 — 小误差完全不动
 *   2. 改用线性 + 最低速度地板 — 小误差也能慢慢追
 *   3. 输出低通滤波 (EMA) — 消除 PID 抖动
 *   4. 积分抗饱和 — 输出饱和时停止积分
 *   5. Kd 降 33 倍 — D 项不再主导控制
 */
static void K230_ControlAxis(StepperMotor *motor, K230_PID *pid,
                              int16_t err, Kalman1D *kf,
                              uint8_t invert, const char *axis_label,
                              float *out_filter,
                              int16_t deadzone_px,
                              int32_t pos_limit_steps,
                              float max_speed_scale)
{
    /* 异常值保护 */
    if (abs(err) > K230_ERROR_MAX_PX) err = 0;

    if (abs(err) > deadzone_px) {
        /* 卡尔曼平滑噪声 */
        float kf_err = Kalman_Update(kf, (float)err);

        /* v5.0 速度控制器: 输出为带符号速度指令 [-1,1] (无积分, 不过冲) */
        float vel_out = K230_PID_Update(pid, kf_err);
        if (vel_out >  1.0f) vel_out =  1.0f;
        if (vel_out < -1.0f) vel_out = -1.0f;

        float dir_out = invert ? -vel_out : vel_out;
        Direction dir = (dir_out >= 0.0f) ? DIR_CW : DIR_CCW;
        float vel_mag = (dir_out >= 0.0f) ? dir_out : -dir_out;   /* 速度幅值 [0,1] */

        /* ---- 减速区: 误差越接近靶心, 速度越低, 进死区前线性归零 ----
         *  shape = (|err| - deadzone) / (dec_zone - deadzone), 限幅 [0,1]
         *  这是根治"过冲靶心"的关键: 速度在到达死区前已被迫降到 0 */
        float dec_zone = pid->ki;                       /* ki 字段 = 减速区宽度(px) */
        if (dec_zone <= (float)deadzone_px) dec_zone = (float)deadzone_px + 1.0f;
        float a = (kf_err > 0.0f) ? kf_err : -kf_err;
        float shape = (a - (float)deadzone_px) / (dec_zone - (float)deadzone_px);
        if (shape < 0.0f) shape = 0.0f;
        if (shape > 1.0f) shape = 1.0f;

        if (vel_mag > shape) vel_mag = shape;          /* 减速区限幅 */
        vel_mag *= max_speed_scale;                    /* 该轴最高速度(垂直轴 0.6) */
        if (vel_mag < 0.0f) vel_mag = 0.0f;

        /* 速度幅值 → 步间延时 (vel=0→最慢, vel=1→最快) */
        uint32_t range = STEPPER_MAX_DELAY_US - STEPPER_MIN_DELAY_US;
        uint32_t delay_us = STEPPER_MAX_DELAY_US - (uint32_t)(vel_mag * (float)range);
        if (delay_us < STEPPER_MIN_DELAY_US) delay_us = STEPPER_MIN_DELAY_US;
        if (delay_us > STEPPER_MAX_DELAY_US) delay_us = STEPPER_MAX_DELAY_US;

        /* 垂直轴硬限位: 超出安全行程则停转并清状态 */
        if (pos_limit_steps > 0) {
            int32_t pos = Stepper_GetPosition(motor);
            int32_t next = (dir == DIR_CW) ? pos + 1 : pos - 1;
            if (next > pos_limit_steps || next < -pos_limit_steps) {
                if (Stepper_GetState(motor) != STATE_IDLE) Stepper_Stop(motor);
                K230_PID_Reset(pid);
                kf->init = 0;
                *out_filter = 0.0f;
                static uint8_t lim_print = 0;
                if (!lim_print) {
                    lim_print = 1;
                    printf("\r\n[%s] 到达限位 ±%ld 步, 停止转动\r\n",
                           axis_label, (long)pos_limit_steps);
                }
                return;
            }
        }

        /* 输出 EMA 轻平滑 (0.5/0.5, 兼顾跟手与抑抖) */
        if (*out_filter == 0.0f) *out_filter = (float)delay_us;
        else *out_filter = *out_filter * 0.5f + (float)delay_us * 0.5f;
        uint32_t final_delay = (uint32_t)(*out_filter + 0.5f);
        if (final_delay < STEPPER_MIN_DELAY_US) final_delay = STEPPER_MIN_DELAY_US;
        if (final_delay > STEPPER_MAX_DELAY_US) final_delay = STEPPER_MAX_DELAY_US;

        Stepper_SetDirection(motor, dir);
        if (Stepper_GetState(motor) != STATE_RUNNING) {
            /* 仅 IDLE→RUNNING 启动一次(含 soft_start), 之后每帧只平滑调速 */
            Stepper_RunContinuously(motor);
        }
        Stepper_UpdateTargetSpeed(motor, final_delay);  /* 每帧按加速度限制平滑调速 */

        /* 调试打印 (每 5 帧) */
        static uint8_t dbg_cnt[2] = {0, 0};
        int idx = (motor == &g_motor_pan) ? 0 : 1;
        if (++dbg_cnt[idx] >= 5) {
            dbg_cnt[idx] = 0;
            printf("[%s] err=%+3d KF=%.0f → vel=%.3f shape=%.2f → %luus pos=%ld\n",
                   axis_label, err, (double)kf_err, (double)vel_mag, (double)shape,
                   (unsigned long)final_delay, (long)Stepper_GetPosition(motor));
        }
    } else {
        /* 死区内: 停止, 重置速度控制器 + 卡尔曼 + 滤波器 */
        if (Stepper_GetState(motor) != STATE_IDLE) {
            Stepper_Stop(motor);
        }
        K230_PID_Reset(pid);
        kf->init = 0;
        *out_filter = 0.0f;
    }
}

/**
 * @brief 处理 K230 坐标帧，驱动 Pan/Tilt 电机
 *        主循环中调用，保证不会在 ISR 上下文中操作电机
 */
static void K230_ProcessFrame(void)
{
    if (!g_k230_frame_ready) return;
    g_k230_frame_ready = 0;

    int16_t err_x = g_k230_err_x;
    int16_t err_y = g_k230_err_y;
    uint8_t flags = g_k230_flags;
    uint8_t valid = flags & 0x01;
    uint8_t hit   = flags & 0x02;

    /* 更新时间戳 */
    g_k230_last_frame = HAL_GetTick();

    if (!valid) {
        /* v4.15: valid=0 时不要急停, 让电机保持当前运动.
         * 超时停止由主循环中的 K230_TIMEOUT 检查负责. */
        return;
    }

    /* 循迹测试模式: 忽略 K230 坐标, 不切 AUTO */
    if (g_gimbal_mode == GIMBAL_TRACE) return;

    /* 记录最后有效帧时间 */
    g_k230_last_valid = HAL_GetTick();

    /* 切换到自动打靶模式:
     *   - 开机前 3 秒不进 AUTO (防"上电就乱转")
     *   - 不在 AUTO 模式时不驱动电机 (避免积积分/空转) */
    if (HAL_GetTick() <= 3000) {
        /* 开机保护期: 不切模式, 不驱动电机 */
        return;
    }

    if (g_gimbal_mode != GIMBAL_AUTO) {
        g_gimbal_mode = GIMBAL_AUTO;
        K230_PID_Reset(&g_pid_pan);
        K230_PID_Reset(&g_pid_tilt);
        g_kf_x.init = 0; g_kf_y.init = 0;
        printf("\r\n[K230] → 自动打靶 双轴控制\n");
    }

    /* ---- Pan 轴 (X 误差) ——— 自由旋转, 标准死区 ---- */
    K230_ControlAxis(&g_motor_pan, &g_pid_pan, err_x, &g_kf_x,
                     g_pan_dir_invert, "Pan", &g_pan_out_filter,
                     PID_DEADZONE, PAN_MAX_STEPS, 1.0f);

    /* ---- Tilt 轴 (Y 误差) ——— 双轴闭环打靶, 限幅防"打到天上" ---- */
    K230_ControlAxis(&g_motor_tilt, &g_pid_tilt, err_y, &g_kf_y,
                     g_tilt_dir_invert, "Tilt", &g_tilt_out_filter,
                     TILT_DEADZONE, TILT_MAX_STEPS, TILT_SPEED_SCALE);

    /* 命中反馈 */
    if (hit) {
        g_feedback_until = HAL_GetTick() + 200;
        LED_ON();
    }
}

/**
 * @brief K230 超时检测 — 长时间未收到帧时安全停止电机
 */
static void K230_TimeoutCheck(void)
{
    if (g_gimbal_mode != GIMBAL_AUTO) return;
    if (g_k230_last_frame == 0) return;

    uint32_t now = HAL_GetTick();

    /* 通信完全超时 (无任何帧超过 2s) → 急停 */
    if (now - g_k230_last_frame > K230_FRAME_TIMEOUT_MS) {
        printf("[K230] 通信超时! 停止电机\n");
        goto k230_stop;
    }

    /* 持续无效帧超时 (一直在收帧但全是 valid=0 超过 2s) → 缓停 */
    if (g_k230_last_valid > 0 &&
        now - g_k230_last_valid > K230_FRAME_TIMEOUT_MS) {
        printf("[K230] 持续无效帧! 停止电机 (laser lost)\n");
        goto k230_stop;
    }

    return;

k230_stop:
    Stepper_Stop(&g_motor_pan);
    Stepper_Stop(&g_motor_tilt);
    K230_PID_Reset(&g_pid_pan);
    K230_PID_Reset(&g_pid_tilt);
    g_gimbal_mode = GIMBAL_SERIAL;
    g_k230_last_frame = 0;
    g_k230_last_valid = 0;
}

/* ========================== 云台按键扫描 ========================== */

static void Gimbal_KeyScan(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < 50) return;  /* 50ms 消抖 */
    last_tick = now;

    /* ---- KEY_MODE: 循环切换云台模式 (Pan → Tilt → Sweep → Serial) ---- */
    if (KEY_MODE_READ() == GPIO_PIN_RESET) {
        Stepper_Stop(&g_motor_pan);
        Stepper_Stop(&g_motor_tilt);

        g_gimbal_mode = (GimbalMode)((g_gimbal_mode + 1) % 8);

        switch (g_gimbal_mode) {
            case GIMBAL_PAN:
                g_active_motor = &g_motor_pan;
                printf("\r\n[云台] → Pan 手动模式 (控制水平电机)\r\n");
                printf("  UP=加速  DOWN=减速  DIR=换向\r\n");
                Stepper_RunContinuously(&g_motor_pan);
                break;
            case GIMBAL_TILT:
                g_active_motor = &g_motor_tilt;
                printf("\r\n[云台] → Tilt 手动模式 (控制垂直电机)\r\n");
                printf("  UP=加速  DOWN=减速  DIR=换向\r\n");
                Stepper_RunContinuously(&g_motor_tilt);
                break;
            case GIMBAL_SWEEP:
                printf("\r\n[云台] → 双轴扫描模式\r\n");
                Stepper_SweepEnable(&g_motor_pan, 180);   /* ±90° */
                Stepper_SweepEnable(&g_motor_tilt, 180);
                break;
            case GIMBAL_SERIAL:
                printf("\r\n[云台] → 串口控制模式\r\n");
                printf("  p(选Pan) t(选Tilt) f(全步) h(半步)\r\n");
                printf("  c(正转) a(反转) s(停) ?(状态)\r\n");
                printf("  +N(步进) -N(后退) rN(旋转°) wN(扫描±°)\r\n");
                break;
            case GIMBAL_DANCE:
                printf("\r\n🎵 [云台] → 舞蹈模式！云台开始蹦迪！\r\n");
                Stepper_RunContinuously(&g_motor_pan);
                Stepper_RunContinuously(&g_motor_tilt);
                g_dance_phase_start = 0;  /* 重头开始跳 */
                break;
            case GIMBAL_AUTO:
                printf("\r\n[云台] → K230 自动打靶模式\r\n");
                printf("  等待 K230 坐标指令...\r\n");
                break;
            case GIMBAL_POSCTRL:
                printf("\r\n[云台] → 位置闭环模式\r\n");
                printf("  x<角度> y<角度> 设置目标\r\n");
                printf("  pid <p|t> <kp|ki|kd> <值> 调PID\r\n");
                printf("  z 归零  k 切K230  m 切回位置环\r\n");
                printf("  trace <大小> 矩形循迹测试 (默认300步≈±26°)\r\n");
                printf("  stop 停止  backlash <p|t> <步数> 调回程差\r\n");
                break;
            case GIMBAL_TRACE:
                printf("\r\n[云台] → 矩形循迹测试\r\n");
                printf("  trace <大小> 启动 (当前: %s)\r\n",
                       g_trace_active ? "运行中" : "已停止");
                printf("  stop 停止, 回到位置闭环\r\n");
                break;
        }
        return;
    }

    /* 扫描模式: UP/DOWN 同时调双轴速度 */
    if (g_gimbal_mode == GIMBAL_SWEEP) {
        if (KEY_UP_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s > STEPPER_MIN_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s - 200);
                Stepper_SetSpeed(&g_motor_tilt, s - 200);
            }
            return;
        }
        if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s < STEPPER_MAX_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s + 200);
                Stepper_SetSpeed(&g_motor_tilt, s + 200);
            }
            return;
        }
        return;
    }

    /* 舞蹈模式: UP/DOWN 调整体速度 */
    if (g_gimbal_mode == GIMBAL_DANCE) {
        if (KEY_UP_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s > STEPPER_MIN_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s - 200);
                Stepper_SetSpeed(&g_motor_tilt, s - 200);
            }
            return;
        }
        if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s < STEPPER_MAX_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s + 200);
                Stepper_SetSpeed(&g_motor_tilt, s + 200);
            }
            return;
        }
        return;
    }

    /* 串口模式: 按键无效 */
    if (g_gimbal_mode == GIMBAL_SERIAL) {
        return;
    }

    /* ====== Pan / Tilt 手动模式 ====== */

    /* KEY_UP: 加速 */
    if (KEY_UP_READ() == GPIO_PIN_RESET) {
        uint32_t new_delay = g_active_motor->step_delay_us;
        if (new_delay > STEPPER_MIN_DELAY_US) {
            new_delay -= 200;
            Stepper_SetSpeed(g_active_motor, new_delay);
        }
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }

    /* KEY_DOWN: 减速 */
    if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
        uint32_t new_delay = g_active_motor->step_delay_us;
        if (new_delay < STEPPER_MAX_DELAY_US) {
            new_delay += 200;
            Stepper_SetSpeed(g_active_motor, new_delay);
        }
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }

    /* KEY_DIR: 换向 */
    if (KEY_DIR_READ() == GPIO_PIN_RESET) {
        Stepper_SetDirection(g_active_motor,
            (g_active_motor->dir == DIR_CW) ? DIR_CCW : DIR_CW);
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }
}

/* ========================== 舞蹈模式 ========================== */

/**
 * @brief 云台跳舞！按时间编排的舞蹈动作序列
 * @note  在 GIMBAL_DANCE 模式下每轮主循环调用
 */
static void Gimbal_Dance(void)
{
    uint32_t now = HAL_GetTick();

    /* 首次进入或重新进入: 重置舞蹈阶段 */
    if (g_dance_phase_start == 0) {
        g_dance_phase_start = now;
        g_dance_phase = 0;
        g_dance_dir_toggle = 0;
        printf("\r\n💃 云台开始跳舞！\r\n");
        return;
    }

    uint32_t elapsed = now - g_dance_phase_start;

    switch (g_dance_phase) {

    /* ── Phase 0: 热身 — 慢速同向 2秒 ── */
    case 0:
        Stepper_SetSpeed(&g_motor_pan, 4000);
        Stepper_SetSpeed(&g_motor_tilt, 4000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        Stepper_SetDirection(&g_motor_tilt, DIR_CW);
        if (elapsed > 2000) {
            g_dance_phase = 1;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 1: 甩头 — 快速方向切换 3秒 ── */
    case 1:
        Stepper_SetSpeed(&g_motor_pan, 1200);
        Stepper_SetSpeed(&g_motor_tilt, 1500);
        if ((now - g_dance_dir_toggle) > 350) {
            g_dance_dir_toggle = now;
            Stepper_SetDirection(&g_motor_pan,
                (g_motor_pan.dir == DIR_CW) ? DIR_CCW : DIR_CW);
            Stepper_SetDirection(&g_motor_tilt,
                (g_motor_tilt.dir == DIR_CW) ? DIR_CCW : DIR_CW);
        }
        if (elapsed > 3000) {
            g_dance_phase = 2;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 2: 旋转 — Pan 狂转, Tilt 扫描头上扬 3秒 ── */
    case 2:
        Stepper_SetSpeed(&g_motor_pan, 1000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        if (Stepper_GetState(&g_motor_tilt) != STATE_RUNNING ||
            g_motor_tilt.work_mode != WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_SweepEnable(&g_motor_tilt, 160);
            Stepper_SetSpeed(&g_motor_tilt, 2500);
        }
        if (elapsed > 3000) {
            g_dance_phase = 3;
            g_dance_phase_start = now;
        }
        break;

    /* ── Phase 3: 反旋转 — Pan 反向, Tilt 低头 3秒 ── */
    case 3:
        Stepper_SetSpeed(&g_motor_pan, 1000);
        Stepper_SetDirection(&g_motor_pan, DIR_CCW);
        if (Stepper_GetState(&g_motor_tilt) != STATE_RUNNING ||
            g_motor_tilt.work_mode != WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_SweepEnable(&g_motor_tilt, 160);
            Stepper_SetSpeed(&g_motor_tilt, 2500);
        }
        if (elapsed > 3000) {
            g_dance_phase = 4;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 4: 加速冲刺 — 逐渐加速后变向 4秒 ── */
    case 4: {
        float progress = (float)elapsed / 4000.0f;
        uint32_t speed = 2500 - (uint32_t)(progress * 1500);
        if (speed < 1000) speed = 1000;
        Stepper_SetSpeed(&g_motor_pan, speed);
        /* 恢复连续模式 (如果之前是扫描) */
        if (g_motor_tilt.work_mode == WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_RunContinuously(&g_motor_tilt);
        }
        Stepper_SetSpeed(&g_motor_tilt, speed);
        /* 每 500ms 换向 */
        if ((now - g_dance_dir_toggle) > 500) {
            g_dance_dir_toggle = now;
            Stepper_SetDirection(&g_motor_pan,
                (g_motor_pan.dir == DIR_CW) ? DIR_CCW : DIR_CW);
        }
        /* Tilt 每 1000ms 换向 (慢于 Pan, 有交错感) */
        if ((now - g_dance_dir_toggle) > 1000 || g_dance_dir_toggle == 0) {
            if ((now - g_dance_dir_toggle) > 1000) {
                Stepper_SetDirection(&g_motor_tilt,
                    (g_motor_tilt.dir == DIR_CW) ? DIR_CCW : DIR_CW);
            }
        }
        if (elapsed > 4000) {
            g_dance_phase = 5;
            g_dance_phase_start = now;
        }
        break;
    }

    /* ── Phase 5: 缓步收尾 — 放慢速 2秒 ── */
    case 5:
        Stepper_SetSpeed(&g_motor_pan, 5000);
        Stepper_SetSpeed(&g_motor_tilt, 5000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        Stepper_SetDirection(&g_motor_tilt, DIR_CCW);
        if (elapsed > 2000) {
            /* 舞蹈结束, 回到 Pan 手动模式 */
            printf("\r\n🎉 舞蹈表演结束！\r\n");
            g_gimbal_mode = GIMBAL_PAN;
            g_active_motor = &g_motor_pan;
            g_dance_phase_start = 0;
            g_dance_phase = 0;
            printf("[云台] → Pan 手动模式\r\n");
            printf("  UP=加速  DOWN=减速  DIR=换向  MODE=切换\r\n");
        }
        break;

    default:
        g_dance_phase = 0;
        g_dance_phase_start = 0;
        break;
    }
}

/* ========================== OLED 调试刷新 ========================== */

/* ========================== 蓝牙状态上报 ========================== */

/** @brief 向蓝牙(USART2)发送一份紧凑状态报告 (仅蓝牙, 不污染 K230 串口)
 *        格式与手机/PC 上位机解析器兼容; 单独发送避免 printf 经 _write 同时灌给 K230 */
static void BT_SendReport(void)
{
    if (!HAL_IS_BIT_SET(huart2.Instance->CR1, USART_CR1_UE)) return;

    /* v5.0 速度指令代理 = Kp·误差 (限幅 ±1), 供上位机 PID 曲线绘制 */
    float p_pid = g_pid_pan.kp * g_k230_err_x;
    float t_pid = g_pid_tilt.kp * g_k230_err_y;
    if (p_pid >  1.0f) p_pid =  1.0f;
    if (p_pid < -1.0f) p_pid = -1.0f;
    if (t_pid >  1.0f) t_pid =  1.0f;
    if (t_pid < -1.0f) t_pid = -1.0f;

    /* [上位机优化] 紧凑单行高频遥测(150ms), 替代原 3s 块状报告, 便于实时 PID 曲线 */
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
        "K230,P:%+5d,T:%+5d,PP:%.3f,PT:%.3f,I:%.0f,J:%.0f,H:%d,S:%c\r\n",
        g_k230_err_x, g_k230_err_y,
        (double)p_pid, (double)t_pid,
        (double)g_pid_pan.ki, (double)g_pid_tilt.ki,
        (g_k230_flags & 0x02) ? 1 : 0,
        (g_gimbal_mode == GIMBAL_AUTO) ? 'A' : 'M');

    if (n > 0) {
        BT_UART_PutStr(buf, n);   /* 非阻塞发送, 不再阻塞主循环(原 250ms 卡顿根因) */
    }
}

/** @brief 收集当前云台状态并刷新到 OLED (无 OLED 时直接返回) */
static void OLED_Refresh(void)
{
    if (!OLED_Debug_IsReady()) return;

    static const char *mode_names[6] = {
        "PAN", "TILT", "SWEEP", "SERIAL", "DANCE", "AUTO"
    };

    OledDebugInfo info;
    memset(&info, 0, sizeof(info));

    info.mode = mode_names[g_gimbal_mode];
    info.k230_mode = (g_gimbal_mode == GIMBAL_AUTO) ? 1 : 0;
    snprintf(info.bt_line, sizeof(info.bt_line), "BT:%s",
             g_bt_connected ? "ON " : "LOST");

    if (info.k230_mode) {
        /* === K230 自动打靶模式: 误差 + 误差条 + PID === */
        int16_t ex = g_k230_err_x;
        int16_t ey = g_k230_err_y;
        float aex = (ex < 0) ? -ex : ex;
        float aey = (ey < 0) ? -ey : ey;
        info.pan_frac  = (aex > 160) ? 1.0f : aex / 160.0f;
        info.tilt_frac = (aey > 160) ? 1.0f : aey / 160.0f;

        snprintf(info.pan_line,  sizeof(info.pan_line),  "PAN%+4d", ex);
        snprintf(info.tilt_line, sizeof(info.tilt_line), "TIL%+4d", ey);

        float pp = g_pid_pan.kp * g_k230_err_x;
        float tp = g_pid_tilt.kp * g_k230_err_y;
        if (g_motor_pan.state == STATE_IDLE && g_motor_tilt.state == STATE_IDLE)
            snprintf(info.pid_line, sizeof(info.pid_line), "P:0.00 T:0.00");
        else
            snprintf(info.pid_line, sizeof(info.pid_line), "P:%.2f T:%.2f",
                     (double)pp, (double)tp);

        uint8_t f = g_k230_flags;
        const char *src = (!f) ? "LOST" : (f & 0x04) ? "YOLO" : "BORD";
        uint8_t hit = f & 0x02;
        snprintf(info.cfg_line, sizeof(info.cfg_line), "SRC:%s %c",
                 src, hit ? 'H' : '-');

        snprintf(info.time_line, sizeof(info.time_line), "UP:%.0fs %c%c",
                 (double)HAL_GetTick() / 1000.0,
                 g_pan_dir_invert  ? 'P' : 'p',
                 g_tilt_dir_invert ? 'T' : 't');
    } else {
        /* === 非自动模式: 简洁显示 === */
        const char *ps = (g_motor_pan.state  == STATE_IDLE) ? "IDLE" : "RUN";
        const char *ts = (g_motor_tilt.state == STATE_IDLE) ? "IDLE" : "RUN";
        snprintf(info.pan_line,  sizeof(info.pan_line),  "PAN %s", ps);
        snprintf(info.tilt_line, sizeof(info.tilt_line), "TIL %s", ts);
        info.pan_frac  = (g_motor_pan.state  == STATE_IDLE) ? 0.0f : 0.5f;
        info.tilt_frac = (g_motor_tilt.state == STATE_IDLE) ? 0.0f : 0.5f;

        snprintf(info.pid_line,  sizeof(info.pid_line),  "SEL:%s", g_serial_motor->label);
        snprintf(info.cfg_line,  sizeof(info.cfg_line),  "RX:%s",  g_oled_cmd);
        snprintf(info.time_line, sizeof(info.time_line), "UP:%.0fs %s",
                 (double)HAL_GetTick() / 1000.0, info.mode);
    }

    OLED_Debug_Update(&info);
}

/* ========================== 主函数 ========================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    USART1_Init();
    USART2_Init();
    USART3_Init();      /* K230 上位机串口 (PB10/PB11, 115200) */
    I2C1_Init();
    NVIC_Config();

    /* 诊断: 打印上次复位原因 (RCC_CSR) — 判断是否为掉电复位 */
    {
        uint32_t csr = RCC->CSR;
        printf("[诊断] RCC_CSR=0x%08lX 原因:", (unsigned long)csr);
        if (csr & (1UL<<31)) printf(" BOR(掉电)");
        if (csr & (1UL<<30)) printf(" POR(上电)");
        if (csr & (1UL<<29)) printf(" SW(软件复位)");
        if (csr & (1UL<<28)) printf(" IWDG(看门狗)");
        if (csr & (1UL<<27)) printf(" WWDG");
        if (csr & (1UL<<26)) printf(" PIN(NRST)");
        if (csr & (1UL<<25)) printf(" LPWR(低功耗)");
        printf("\r\n");
        /* 清除标志位, 确保下次读数准确 */
        RCC->CSR |= (1UL << 24);
    }

    /* OLED 初始化 (设备不存在时自动跳过, 不影响其余功能) */
    {
        uint8_t oled_ok = OLED_Debug_Init();
        if (oled_ok) {
            printf("[系统] OLED 已就绪 (I2C1 PB8/PB9)\r\n");
        } else {
            printf("[系统] ⚠ OLED 未检测到 (已尝试 0x3C 和 0x3D, I2C 总线已复位)\r\n");
            printf("      请确认:\r\n");
            printf("      ① VCC→3.3V (少数模块要 5V, 可尝试改接 5V)\r\n");
            printf("      ② GND→GND\r\n");
            printf("      ③ SCL→PB8  SDA→PB9\r\n");
            printf("      ④ 模块 SCK/SDA 引脚接了 4.7kΩ 上拉到 3.3V? (部分模块不自带上拉)\r\n");
        }
    }

    /* 初始化两个步进电机 (Pan + Tilt) */
    Stepper_Init(&g_motor_pan,  &g_pan_pins,  &htim2, "Pan");
    Stepper_Init(&g_motor_tilt, &g_tilt_pins, &htim3, "Tilt");

    /* 启动两个步进定时器 */
    TIM2_Init();
    TIM3_Init();

    /* v4.10: 初始化两个 PID 控制器 (设置基础增益/段增益/微分滤波/退饱和等所有参数) */
    K230_PID_Init(&g_pid_pan,  PAN_GAIN, PAN_DEC_ZONE,  CTRL_KD, (float)PID_DEADZONE);
    K230_PID_Init(&g_pid_tilt, TILT_GAIN, TILT_DEC_ZONE, CTRL_KD, (float)PID_DEADZONE_TILT);

    /* v5.1: 初始化位置闭环 PID 并设置限位 */
    PosPID_Init(&g_pos_pid_pan,  POS_KP_PAN,  POS_KI_PAN,  POS_KD_PAN);
    PosPID_Init(&g_pos_pid_tilt, POS_KP_TILT, POS_KI_TILT, POS_KD_TILT);
    Stepper_SetPositionLimits(&g_motor_pan,  POS_LIMIT_PAN_MIN,  POS_LIMIT_PAN_MAX);
    Stepper_SetPositionLimits(&g_motor_tilt, POS_LIMIT_TILT_MIN, POS_LIMIT_TILT_MAX);
    g_pos_target_pan  = 0;
    g_pos_target_tilt = 0;
    /* 用户要求: 上电电机不动, 收到串口/蓝牙命令才动 */
    /* (电机初始化后处于 idle 状态, step 0 保持力矩) */

    printf("\r\n========================================\r\n");
    printf("  28BYJ-48 双轴云台 Demo 已就绪\r\n");
    printf("  STM32F103C8T6 @ 72MHz\r\n");
    printf("  电机: Pan(水平) + Tilt(垂直)\r\n");
    printf("  位置闭环: 串口目标角度/调PID, 100ms 遥测\r\n");
    printf("  K230: 发 'k' 切换到自动打靶模式\r\n");
    printf("========================================\r\n\r\n");
    printf("[云台] 默认位置闭环模式. 发送 ? 查看命令\r\n");
    printf("  x<角度>  y<角度>  设置目标角度(°)\r\n");
    printf("  pid p kp 0.001     设置 Pan 位置环 Kp\r\n");
    printf("  pid p ki 0.00001   设置 Pan 积分 Ki\r\n");
    printf("  pid p kd 0.00005   设置 Pan 微分 Kd\r\n");
    printf("  z                  当前位置归零\r\n");
    printf("  s                  停止\r\n");
    printf("  k                  切换 K230 自动打靶模式\r\n\r\n");

    while (1) {
        /* [修复] 串口命令在主循环处理，避免 printf 在 ISR 中阻塞 */
        if (s_cmd_ready) {
            s_cmd_ready = 0;
            Serial_Execute();
            s_serial_len = 0;
        }

        /* 蓝牙命令处理 (独立缓冲, 不与 USART1 交叉) */
        if (s_bt_cmd_ready) {
            s_bt_cmd_ready = 0;
            /* 拷贝蓝牙命令到主缓冲并执行 */
            memcpy(s_serial_buf, s_bt_buf, s_bt_len);
            s_serial_len = s_bt_len;
            /* 直接调用 Serial_Execute 处理蓝牙命令 */
            Serial_Execute();
            s_serial_len = 0;
            s_bt_len = 0;
        }

        /* 按键扫描 */
        Gimbal_KeyScan();

        /* 舞蹈模式: 编排动作 */
        if (g_gimbal_mode == GIMBAL_DANCE) {
            Gimbal_Dance();
        }

        /* v5.1: 位置闭环模式 (默认) */
        if (g_gimbal_mode == GIMBAL_POSCTRL) {
            PosCtrl_Update();
            PosCtrl_SendTelemetry();
        }

        /* 矩形循迹测试模式 */
        if (g_gimbal_mode == GIMBAL_TRACE) {
            Trace_Update();
            PosCtrl_SendTelemetry();
        }

        /* OLED 调试显示: 每 200ms 刷新一次 (I2C 不宜过快) */
        static uint32_t oled_tick = 0;
        if (HAL_GetTick() - oled_tick >= 200) {
            oled_tick = HAL_GetTick();
            OLED_Refresh();
        }

        /* K230 坐标帧处理 */
        K230_ProcessFrame();
        K230_TimeoutCheck();

        /* 向 K230 回传状态: 每 500ms */
        static uint32_t k230_status_tick = 0;
        if (HAL_GetTick() - k230_status_tick >= 500) {
            k230_status_tick = HAL_GetTick();
            if (HAL_IS_BIT_SET(huart3.Instance->CR1, USART_CR1_UE)) {
                K230_SendStatus();
            }
        }

        /* 蓝牙串口状态推送: 每 3s (手机/上位机调试用) */
        static uint32_t bt_report_tick = 0;
        if (HAL_GetTick() - bt_report_tick >= 150) {   /* 上位机 PID 曲线: 150ms 高频遥测 */
            bt_report_tick = HAL_GetTick();
            BT_SendReport();
        }

        /* 蓝牙链路健康检测 + 重连指示 (每 500ms) */
        static uint32_t bt_link_tick = 0;
        if (HAL_GetTick() - bt_link_tick >= 500) {
            bt_link_tick = HAL_GetTick();
            if (g_bt_connected &&
                (HAL_GetTick() - g_bt_last_rx_tick > BT_LINK_LOST_MS)) {
                g_bt_connected = 0;
                printf("[BT] 链路疑似断开 (超过 %dms 未收到数据), 等待重连...\r\n",
                       (int)BT_LINK_LOST_MS);
                BT_TryRecover();
            }
        }

        /* LED 反馈: 串口命令到达时亮 150ms；平时双轴空闲则 500ms 慢闪 */
        static uint32_t led_tick = 0;
        if (HAL_GetTick() < g_feedback_until) {
            LED_ON();
        } else if (HAL_GetTick() - led_tick > 500) {
            led_tick = HAL_GetTick();
            if (Stepper_GetState(&g_motor_pan) == STATE_IDLE &&
                Stepper_GetState(&g_motor_tilt) == STATE_IDLE) {
                LED_TOG();
            }
        }
    }
}

/* ========================== 中断服务函数 ========================== */

/**
 * @brief TIM2 中断 — Pan 电机步进节拍
 */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            Stepper_TIM_IRQHandler(&g_motor_pan);
        }
    }
}

/**
 * @brief TIM3 中断 — Tilt 电机步进节拍
 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            Stepper_TIM_IRQHandler(&g_motor_tilt);
        }
    }
}

/**
 * @brief USART1 中断 — [修复] 只缓冲字节，不调用 printf
 *        命令解析在主循环 Serial_Execute() 中执行
 */
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFF);

        /* 缓冲接收数据 (不在 ISR 内做回显: 阻塞发送会与 _write/printf 共用
         * huart1 发送状态机, 存在重入/卡死风险; 终端通常自带本地回显) */
        if (s_serial_len < SERIAL_BUF_SIZE - 1) {
            s_serial_buf[s_serial_len++] = (char)byte;
        }
        if (byte == '\n' || byte == '\r') {
            s_cmd_ready = 1;
        } else if (s_serial_len >= SERIAL_BUF_SIZE - 1) {
            /* 缓冲区满，丢弃后续字符，强制触发命令处理 */
            s_cmd_ready = 1;
        }
    }

    /* 清除溢出错误 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

/**
 * @brief USART2 中断 (JDY-31 蓝牙)
 *        - TXE : 从环形缓冲续发剩余字节(非阻塞发送)
 *        - RXNE: 缓冲接收到的命令, 并更新链路活跃时间戳
 *        - 错误标志(ORE/NE/FE/PE)清除: 防止 UART 因噪声/溢出卡死
 */
void USART2_IRQHandler(void)
{
    /* ---- TXE: 续发 ring 中剩余字节 ---- */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != RESET &&
        __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE) != RESET) {
        if (g_bt_tx_head != g_bt_tx_tail) {
            uint8_t b = g_bt_tx_ring[g_bt_tx_tail];
            g_bt_tx_tail = (g_bt_tx_tail + 1) % BT_TX_RING_SIZE;
            huart2.Instance->DR = b;
        } else {
            g_bt_tx_active = 0;
            __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
        }
    }

    /* ---- RXNE: 缓冲接收到的命令 ---- */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart2.Instance->DR & 0xFF);

        /* 链路活跃: 收到任意蓝牙数据 → 标记已连接 + 记录时间戳(重连检测用) */
        g_bt_connected = 1;
        g_bt_last_rx_tick = HAL_GetTick();

        /* 缓冲接收数据 (使用独立 bt 缓冲, 不与 USART1 交叉) */
        if (s_bt_len < SERIAL_BUF_SIZE - 1) {
            s_bt_buf[s_bt_len++] = (char)byte;
        }
        if (byte == '\n' || byte == '\r') {
            s_bt_cmd_ready = 1;
        } else if (s_bt_len >= SERIAL_BUF_SIZE - 1) {
            s_bt_cmd_ready = 1;
        }
    }

    /* ---- 错误恢复: 清 ORE/NE/FE/PE, 防止 UART 卡死导致后续再也收不到数据 ---- */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(&huart2);
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(&huart2);
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(&huart2);
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE)  != RESET) __HAL_UART_CLEAR_PEFLAG(&huart2);
}

/**
 * @brief USART3 中断 (K230 上位机) — 坐标帧状态机解析
 *        帧格式: 0x3C 0x3B [XH][XL][YH][YL][FLAG] 0x01 0x01
 *        X/Y = signed int16 (Pan/Tilt 像素误差)
 *        FLAG: bit0=valid, bit1=hit, bit2=hi_conf
 */
void USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart3.Instance->DR & 0xFF);

        switch (g_k230_fstate) {
            case K230_IDLE:
                if (byte == 0x3C) {
                    g_k230_fstate = K230_HEADER;
                }
                break;

            case K230_HEADER:
                if (byte == 0x3B) {
                    /* 坐标帧开始 */
                    g_k230_fstate = K230_DATA;
                    g_k230_fidx = 0;
                } else if (byte == 0x3C) {
                    /* 连续 0x3C, 保持等待 */
                } else {
                    g_k230_fstate = K230_IDLE;  /* 非坐标帧, 丢弃 */
                }
                break;

        case K230_DATA:
            if (g_k230_fidx < 6) {
                g_k230_fbuf[g_k230_fidx++] = byte;
            }
            if (g_k230_fidx >= 6) {
                g_k230_fstate = K230_FOOTER1;
            }
            break;

            case K230_FOOTER1:
                if (byte == 0x01) {
                    g_k230_fstate = K230_FOOTER2;
                } else {
                    g_k230_fstate = K230_IDLE;  /* 帧损坏, 丢弃 */
                }
                break;

        case K230_FOOTER2:
            if (byte == 0x01) {
                /* 校验 CRC8 (覆盖 XH XL YH YL FLAG 共 5 字节) */
                uint8_t crc_calc = K230_CRC8((const uint8_t *)g_k230_fbuf, 5);
                if (crc_calc == g_k230_fbuf[5]) {
                    /* 帧完整且校验通过: 解析坐标 */
                    g_k230_err_x = (int16_t)((g_k230_fbuf[0] << 8) | g_k230_fbuf[1]);
                    g_k230_err_y = (int16_t)((g_k230_fbuf[2] << 8) | g_k230_fbuf[3]);
                    g_k230_flags = g_k230_fbuf[4];
                    g_k230_frame_ready = 1;
                } else {
                    g_k230_crc_err++;   /* CRC 失败: 静默丢弃本帧 */
                }
            }
            g_k230_fstate = K230_IDLE;
            break;

            default:
                g_k230_fstate = K230_IDLE;
                break;
        }
    }

    /* 清除溢出错误 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
}

/**
 * @brief SysTick 中断 — HAL 库时基
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief HardFault 异常
 */
void HardFault_Handler(void)
{
    /* 注意: 异常处理器内禁止调用 printf (会经 _write → HAL_UART_Transmit 再次触碰
     * 可能已损坏的 huart1, 二次故障会升级为 Lockup 需外部复位)。仅用 LED 闪烁指示。 */
    while (1) {
        LED_TOG();
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}

/**
 * @brief 错误处理
 */
void Error_Handler(void)
{
    /* 同 HardFault: 不在异常内调用 printf, 仅 LED 闪烁 + 关中断防止进一步恶化 */
    __disable_irq();
    while (1) {
        LED_TOG();
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}

