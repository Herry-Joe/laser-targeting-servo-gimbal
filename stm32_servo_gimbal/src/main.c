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

/* ========================== 看门狗 ========================== */
/* 如果选项字节开启了 IWDG 硬件模式, 上电后 IWDG 即开始计数, 须定时喂狗否则复位.
 * 使用寄存器直写 (不依赖 HAL), 无论 IWDG 是否使能均可安全调用. */
#define IWDG_FEED()  do { IWDG->KR = 0xAAAA; } while(0)

/* STM32 28BYJ-48 + ULN2003 驱动板已上电, 任何时候 4 个输入脚决定哪根线圈通电
 * (ULN2003 IN1=橙/IN2=粉/IN3=黄/IN4=蓝). 无 STBY 概念, 不用 gate 电流. */

/* ========================== 全局外设句柄 ========================== */
TIM_HandleTypeDef  htim2;       /* Pan 步进节拍定时器 */
TIM_HandleTypeDef  htim1;       /* Tilt 舵机 PWM 定时器 (TIM1, PA11) */
TIM_HandleTypeDef  htim3;       /* Pan 舵机 PWM 定时器 (TIM3, PB1) */
TIM_HandleTypeDef  htim4;       /* Tilt 步进节拍定时器 */
UART_HandleTypeDef huart1;      /* 调试串口 */
UART_HandleTypeDef huart2;      /* 蓝牙串口 (JDY-31) */
UART_HandleTypeDef huart3;      /* K230 上位机串口 (PB10/PB11) */
I2C_HandleTypeDef  hi2c1;       /* OLED I2C 总线 (PB8/PB9) */

/* ========================== 电机实例 ========================== */
static StepperMotor g_motor_pan;    /* 水平电机 (Pan) */
static StepperMotor g_motor_tilt;   /* 垂直电机 (Tilt) */

/* ========================== 引脚配置表 (舵机方案) ========================== */
/* 舵机 PWM 分两路独立定时器:
 *   Pan 舵机 : PB1 = TIM3_CH4 (50Hz)
 *   Tilt舵机 : PA11 = TIM1_CH4 (50Hz)
 * 引脚分别在 TIM3_Init() / TIM1_Init() 中配置为复用推挽。 */

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
static uint8_t  g_locked_feedback = 0;  /* 命中锁定提示只打印一次 */
static uint8_t  g_hit_locked      = 0;  /* 命中后双轴锁存(停稳); 误差明显偏离或丢帧才解锁 */

/* ========================== K230 上位机坐标帧接收 ========================== */
/** @brief 坐标帧解析状态 */
typedef enum {
    K230_IDLE      = 0,   /* 等待帧头 0x3C */
    K230_HEADER    = 1,   /* 等待第二字节 (0x3B=坐标帧 / 0x3A=矩形帧) */
    K230_DATA      = 2,   /* 收集数据(误差帧6字节 / 矩形帧11字节) */
    K230_RECT_CRC  = 3,   /* 矩形帧: 收集 CRC 字节 */
    K230_FOOTER1   = 4,   /* 等待 0x01 */
    K230_FOOTER2   = 5,   /* 等待帧尾 0x01 */
    K230_CMD       = 6,   /* 命令帧: 收集 CMD,PARAM,0,0,0 */
} K230FrameState;

static volatile K230FrameState g_k230_fstate = K230_IDLE;
static volatile uint8_t         g_k230_fbuf[12];   /* 坐标/矩形数据缓冲区(矩形帧需11字节) */
static volatile uint8_t         g_k230_fidx  = 0;
static volatile uint8_t         g_k230_ftype = 0;  /* 0=误差帧 1=矩形帧 */
static volatile uint8_t         g_k230_rcrc  = 0;  /* 矩形帧 CRC 字节暂存 */
static volatile int16_t         g_k230_err_x = 0;   /* Pan 方向误差 (像素) */
static volatile int16_t         g_k230_err_y = 0;   /* Tilt 方向误差 (像素) */
static volatile uint8_t         g_k230_flags = 0;   /* 标志位: bit0=valid, bit1=hit, bit2=hi_conf */
static volatile uint8_t         g_k230_corner = 0;  /* bit5: 接近矩形拐角 → 减速防过冲 [v5.7] */
static volatile uint8_t         g_k230_frame_ready = 0;  /* 新帧到达标志 */
static volatile uint32_t        g_k230_last_frame = 0;   /* 最后一帧时间戳(ms) */

/* K230 超时 (2s 无有效帧 → 停止电机, 防乱跑) */
#define K230_FRAME_TIMEOUT_MS   2000
#define K230_ERROR_MAX_PX       400   /* 单帧最大合理误差 (画面半宽 800/2=400) */

/* 任务控制相关常量 (v5.2: K230 命令帧 + 完成上报) */
#define K230_HOLD_TIMEOUT_MS   500   /* HOLD 标志持续超此时间(激光仍丢失) → 停车 */
#define K230_STEADY_DONE_MS    800   /* 双轴进入死区并稳定持续超此时间 → 标记任务完成 */

/* K230 坐标帧 FLAG 位定义 (与 K230 端 v5.2 send_binary 逐位一致) */
#define K230_FLAG_VALID   0x01   /* bit0 = 目标/激光有效 */
#define K230_FLAG_HIT     0x02   /* bit1 = 命中(靶心锁定/循迹一圈完成) */
#define K230_FLAG_CONF    0x04   /* bit2 = 高置信度 */
#define K230_FLAG_BOOT    0x08   /* bit3 = 开机回中 (v5.4 起不再发送) */
#define K230_FLAG_HOLD    0x10   /* bit4 = 激光丢失保持速度 */
#define K230_FLAG_CORNER  0x20   /* bit5 = 接近矩形拐角 (v5.7: 请求减速防过冲) */

/* K230 命令帧 CMD 值 (STM32→K230 / K230→STM32 共用) */
#define K230_CMD_AIM        0x01
#define K230_CMD_RECENTER   0x02
#define K230_CMD_TRACE_BORDER 0x03
#define K230_CMD_TRACE_INNER  0x04
#define K230_CMD_STOP       0x05

/* K230 任务状态 (v5.2: 命令帧切换 + 完成上报上位机) */
static volatile uint8_t  g_k230_task_id     = 0;   /* 任务: 0=未知/IDLE/AIM 1=RECENTER 2=BORDER 3=INNER */
static volatile uint8_t  g_k230_task_done   = 0;   /* 任务完成(命中锁存/稳态)标志 */
static volatile uint8_t  g_k230_cmd_pending = 0;   /* ISR 收到的待处理命令帧 CMD (主循环消费) */
static volatile uint8_t  g_auto_enabled     = 1;   /* 自动打靶使能(PA0 停止/K230 STOP 清0, 新命令恢复) */
static volatile uint32_t g_k230_hold_first  = 0;   /* HOLD 连续起始时刻(0=非保持态) */
static volatile uint32_t g_k230_steady_tick = 0;   /* 进入死区稳态起始时刻(0=未稳态) */

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
static volatile uint32_t g_k230_frames = 0;   /* 成功解析的坐标帧计数 (调试心跳用) */

/* K230 矩形识别结果 (相对靶心原点) */
static volatile int16_t  g_rect_dx = 0, g_rect_dy = 0;   /* 矩形中心相对靶心偏移(px) */
static volatile uint16_t g_rect_w = 0, g_rect_h = 0;     /* 矩形宽高(px) */
static volatile int16_t  g_rect_ang = 0;                 /* 倾斜角(0.1°) */
static volatile uint8_t  g_rect_valid = 0;               /* bit0: 矩形检测到 */
static volatile uint8_t  g_rect_origin_valid = 0;        /* bit1: 靶心有效→偏移可信 */
static volatile uint8_t  g_rect_frame_ready = 0;         /* 新矩形帧到达 */

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
    uint8_t  locked;          /* 滞回锁存: 进死区后置1, 误差超 死区+迟滞 才清除(防抖) */

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
    pid->locked         = 0;
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
 *   Kp = 速度增益: 越小越柔和, 全速冲刺区= (1/Kp) px
 *   Ki = 减速区宽度(px): 误差在此区间内速度线性降到 0, 越大越柔
 *   Kd = 接近阻尼(默认0)
 *   调参口诀: 追得慢→提 Kp; 仍过冲→调大 Ki(减速区)或加 Kd
 *
 *   v5.2 调整: Kp 从 1.0 降至 0.005, 让速度在 0~200px 范围内比例变化,
 *   而非"非全速即停", 配合更宽的减速区实现平滑逼近, 根治过冲振荡. */
#define PAN_GAIN        0.005f  /* [v5.10] Pan 速度增益: 原0.006→0.005 (200px→全速), 减少大误差过冲 */
#define PAN_DEC_ZONE    140.0f  /* [v5.10] Pan 减速区: 原120→140 (更早减速, 根治RECENTER过冲) */
#define TILT_GAIN       0.003f  /* [v5.10] Tilt 速度增益: 原0.004→0.003 (333px→全速), 抑制Y轴±50px振荡 */
#define TILT_DEC_ZONE   110.0f  /* [v5.10] Tilt 减速区: 原90→110 (更柔) */
#define CTRL_KD         0.0015f /* 接近阻尼: 加大抑制过冲(原 0.0005) */
#define PID_ILIMIT      20.0f
#define PID_DEADZONE    3       /* 死区 (像素) — Pan 轴 */
#define PID_DEADZONE_TILT 4     /* Tilt 轴死区 */
#define K230_HYST_PX    12      /* 滞回迟滞(px): 锁靶后误差需超 死区+此值 才重新驱动.
                                    * [v5.8] 4→12: K230 误差噪声约 ±15px, 增大迟滞避免噪声触发反复反向(消 AIM 模式摇摆) */
#define HIT_RELEASE_PX  40      /* 命中锁存后, 误差超此值(px)才重新捕获(远大于命中容差15px, 根治靶心附近抖停不停) */

/* ---- [v5.7/v5.10] 矩形循迹速度整形 ---- */
#define TRACE_SPEED      0.45f   /* [v5.10] 循迹直线段目标速度(地板): 原0.60→0.45, 减少过冲和LOST */
#define CORNER_FACTOR    0.35f   /* 接近矩形拐角降速系数: 强减速防过冲冲出角外 */
#define VEL_STOP_BAND    0.015f  /* 定点锁靶停止带: 速度指令低于此值直接停机, 消除靶心附近微抖/摇摆 */
/* [v5.8/v5.10] 误差相关地板: 远线(|err|>TRACE_FLOOR_ERR_PX)用 TRACE_SPEED 跟线速度地板,
 *   近线降到 TRACE_NEAR_FLOOR, 允许误差归零, 消除恒定地板导致的极限环(抖动/偏移) */
#define TRACE_FLOOR_ERR_PX  25
#define TRACE_NEAR_FLOOR    0.03f   /* [v5.10] 原0.06→0.03, 近线时几乎停转让误差归零 */

/* ---- Tilt 垂直轴专用限制 (防"激光打到天上") ---- */
/*   STEPPER_HALF_REV=4096 步 = 360°, 故 1 步 ≈ 0.088° */
#define TILT_DEADZONE      4       /* 垂直轴死区(像素) — 收紧, 避免停在离靶心 8px 处 */
#define TILT_MAX_STEPS     300     /* 垂直轴软限位 = ±300步 ≈ ±30° (1M 靶距足够, 垂直向不需大幅摆动) */
#define PAN_MAX_STEPS      450     /* 水平轴软限位 = ±450步 ≈ ±45° (与位置环一致; 实测可到 ±45° 安全) */
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
#define POS_LIMIT_PAN_MIN   -1350   /* Pan ±135° (S20F 270° 舵机物理行程) */
#define POS_LIMIT_PAN_MAX   1350
#define POS_LIMIT_TILT_MIN  -800    /* Tilt ±80° (S20F 180° 舵机, 留 10° 余量防撞) */
#define POS_LIMIT_TILT_MAX  800

/* ---- 回程差补偿 (28BYJ-48 减速齿轮间隙) ----
 * 标准 28BYJ-48 64:1 减速箱回程差约 30~80 个半步步距.
 * 通过 serial 命令 'backlash <轴> <步数>' 可运行时调整并保存到 EEPROM.
 * 默认 50 步，对应约 4.4° 机械回程差。 */
#define BACKLASH_STEPS_DEFAULT  0       /* 舵机基本无齿轮回程差, 默认 0 */
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
#define POS_LOSTSTEP_THRESH 3000   /* 3000步=300° (>舵机全行程2700步), 防止误报 */
#define POS_LOSTSTEP_TIME_MS  3000   /* 持续3秒 */
static uint32_t g_pos_loststep_start = 0;
static uint8_t  g_pos_loststep_alarm = 0;

/** @brief 方向反转标志 (上电默认 Pan/Tilt 都需要反转, 解决激光正反馈跑出视野) */
static uint8_t g_pan_dir_invert = 1;    /* 水平机械方向需反转(靶心/激光坐标系统一后实测为负反馈); 若进 AUTO 反向则发 i 翻转 */
static uint8_t g_tilt_dir_invert = 0;   /* 默认不反转(负反馈); 若进 AUTO 冲到限位则发 I 翻转 */

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

    /* ---- PA0, PA1: 用户加装独立按键 (上拉输入, 按下=低电平) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- PA4, PA5: 未用 (舵机方案释放, 悬空输入) ---- */
    /* ---- PA11: TIM1_CH4 (Tilt) 舵机 PWM, 在 TIM1_Init 中配置为复用推挽 ---- */
    /* ---- PB1 : TIM3_CH4 (Pan)  舵机 PWM, 在 TIM3_Init 中配置为复用推挽 ---- */
    /* ---- PB0~PB7: 舵机方案释放, 不再作步进驱动 (PB3/PB4 已 NOJTAG 释放) ---- */

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
 *
 * [v5.2] 增大缓冲区至 512, 防 150ms 高频遥测+CmdReply 并发时溢出丢包.
 *        BT_SendReport 入队前检查缓冲水位, ≥60% 满时跳帧.
 */
#define BT_TX_RING_SIZE    512U
static volatile uint8_t  g_bt_tx_ring[BT_TX_RING_SIZE];
static volatile uint16_t g_bt_tx_head = 0;   /* 写指针(主循环入队) */
static volatile uint16_t g_bt_tx_tail = 0;   /* 读指针(TXE 中断发送) */
static volatile uint8_t  g_bt_tx_active = 0; /* 1=TXE 发送进行中 */

/* 链路状态: 用于连接可靠性指示与"重连"检测 */
static volatile uint8_t  g_bt_connected    = 0;   /* 收到过蓝牙数据=链路活跃 */
static volatile uint32_t g_bt_last_rx_tick = 0;   /* 最近一次收到蓝牙数据的 tick */
#define BT_LINK_LOST_MS   8000U   /* 超过该时间未收到蓝牙数据 → 判定链路断开 */

/** @brief 返回环形缓冲当前已用字节数 (线程不安全, 仅用于水位估计) */
static uint16_t BT_RingUsed(void)
{
    uint16_t h = g_bt_tx_head, t = g_bt_tx_tail;
    return (h >= t) ? (h - t) : (BT_TX_RING_SIZE - t + h);
}

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

/* ========================== TIM1 初始化 (Tilt 舵机 PWM, 50Hz) ========================== */
/* PA11 = TIM1_CH4 (Tilt 舵机) */

void TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* PA11 配置为 TIM1_CH4 复用推挽 (Tilt) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = SERVO_TILT_PIN;  /* PA11 */
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SERVO_TILT_PORT, &GPIO_InitStruct);

    /* 50Hz PWM: PSC=72-1 → 1us tick, ARR=20000-1 → 20ms 周期 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 72 - 1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 20000 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = TILT_CENTER_US;       /* 上电中位 1500us */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, SERVO_TILT_CHANNEL);

    HAL_TIM_PWM_Start(&htim1, SERVO_TILT_CHANNEL);

    /* TIM1 仅作 Tilt PWM 输出, 不再用作步进节拍 (无 Update 中断) */
}

/* ========================== TIM3 初始化 (Pan 舵机 PWM, 50Hz) ========================== */
/* PB1 = TIM3_CH4 (Pan 舵机) */

void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PB1 配置为 TIM3_CH4 复用推挽 (Pan) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = SERVO_PAN_PIN;  /* PB1 */
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SERVO_PAN_PORT, &GPIO_InitStruct);

    /* 50Hz PWM: PSC=72-1 → 1us tick, ARR=20000-1 → 20ms 周期 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 20000 - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = PAN_CENTER_US;       /* 上电中位 1500us */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, SERVO_PAN_CHANNEL);

    HAL_TIM_PWM_Start(&htim3, SERVO_PAN_CHANNEL);

    /* TIM3 仅作 Pan PWM 输出 */
}

/* ========================== TIM4 初始化 (Tilt 电机步进节拍) ========================== */

void TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 72 - 1;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = STEPPER_DEFAULT_DELAY - 1;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim4);

    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
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

/* ========================== 帮助菜单 ========================== */
static void PrintHelp(void)
{
    printf("\r\n");
    printf("  ╔══════════════════════════════════════════╗\r\n");
    printf("  ║           舵机云台 控制面板              ║\r\n");
    printf("  ╠══════════════════════════════════════════╣\r\n");
    printf("  ║ [位置]                                   ║\r\n");
    printf("  ║   x<°>         Pan → 目标角度            ║\r\n");
    printf("  ║   y<°>         Tilt → 目标角度            ║\r\n");
    printf("  ║   home         双轴回零 (0°, 0°)          ║\r\n");
    printf("  ║   z            当前位置归零                ║\r\n");
    printf("  ╠══════════════════════════════════════════╣\r\n");
    printf("  ║ [模式]                                   ║\r\n");
    printf("  ║   k            K230 自动打靶模式           ║\r\n");
    printf("  ║   m            手动/位置闭环模式            ║\r\n");
    printf("  ║   stop         紧急停止(同步停 K230 任务)   ║\r\n");
    printf("  ╠══════════════════════════════════════════╣\r\n");
    printf("  ║ [K230 任务]                              ║\r\n");
    printf("  ║   aim          自动打靶(跟随误差)          ║\r\n");
    printf("  ║   recenter     激光对中到 A4 中心          ║\r\n");
    printf("  ║   border       外框循迹(固定矩形/单轴)      ║\r\n");
    printf("  ║   inner        内框循迹(电工胶带四点定位)    ║\r\n");
    printf("  ╠══════════════════════════════════════════╣\r\n");
    printf("  ║ [PID 调参]                                ║\r\n");
    printf("  ║   pid p/t kp/ki/kd <值>  位置环调参       ║\r\n");
    printf("  ║   g p/t kp/ki/kd <值>    K230 追踪调参    ║\r\n");
    printf("  ║                                         ║\r\n");
    printf("  ╠══════════════════════════════════════════╣\r\n");
    printf("  ║ [信息]                                   ║\r\n");
    printf("  ║   help 或 ?   显示本菜单                  ║\r\n");
    printf("  ║   status      显示当前状态                 ║\r\n");
    printf("  ║   pid         显示位置环参数               ║\r\n");
    printf("  ║   P           显示 K230 追踪参数           ║\r\n");
    printf("  ║   tele on/off 遥测开关 (默认 off)          ║\r\n");
    printf("  ╚══════════════════════════════════════════╝\r\n");
    printf("\r\n");
}

/* ========================== 状态显示 ========================== */
static void PrintStatus(void)
{
    float pa = PosCtrl_StepsToAngle(g_motor_pan.pos_steps);
    float ta = PosCtrl_StepsToAngle(g_motor_tilt.pos_steps);
    float pt = PosCtrl_StepsToAngle(g_pos_target_pan);
    float tt = PosCtrl_StepsToAngle(g_pos_target_tilt);
    const char *mode_str = "???";
    switch (g_gimbal_mode) {
        case GIMBAL_SERIAL:  mode_str = "串口"; break;
        case GIMBAL_POSCTRL: mode_str = "位置闭环"; break;
        case GIMBAL_AUTO:    mode_str = "K230 打靶"; break;
        case GIMBAL_DANCE:   mode_str = "舞蹈"; break;
        case GIMBAL_TRACE:   mode_str = "循迹"; break;
    }
    printf("\r\n");
    printf("  ┌──────────┬────────┬────────┐\r\n");
    printf("  │  轴      │  当前位置 │  目标  │\r\n");
    printf("  ├──────────┼────────┼────────┤\r\n");
    printf("  │  Pan     │ %+6.1f° │ %+6.1f° │\r\n", (double)pa, (double)pt);
    printf("  │  Tilt    │ %+6.1f° │ %+6.1f° │\r\n", (double)ta, (double)tt);
    printf("  ├──────────┴────────┴────────┤\r\n");
    printf("  │  模式: %-16s    │\r\n", mode_str);
    printf("  │  Pan PID: Kp=%.4f Ki=%.1f Kd=%.5f │\r\n",
           (double)g_pos_pid_pan.kp, (double)g_pos_pid_pan.ki, (double)g_pos_pid_pan.kd);
    printf("  │ Tilt PID: Kp=%.4f Ki=%.1f Kd=%.5f │\r\n",
           (double)g_pos_pid_tilt.kp, (double)g_pos_pid_tilt.ki, (double)g_pos_pid_tilt.kd);
    printf("  └─────────────────────────────┘\r\n");
    printf("\r\n");
}

/* ========================== 遥测开关 ========================== */
static uint8_t g_telemetry_enabled = 0;  /* 默认关闭遥测 */

/* ========================== 串口命令执行 (在主循环调用, 非 ISR) ========================== */

/* [v5.10] 前向声明: 供串口任务命令(aim/recenter/border/inner)在 Serial_Execute 中调用 */
static void K230_SendCmd(uint8_t cmd);
static void K230_HandleCommand(uint8_t cmd);
static void TriggerK230Task(uint8_t cmd);

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
        K230_SendCmd(K230_CMD_STOP);   /* [v5.10] 同步通知 K230 停止任务 */
        CmdReply("# OK 停止\r\n");
        return;
    }
    if (strncmp(p, "help", 4) == 0 || strncmp(p, "HELP", 4) == 0) {
        PrintHelp();
        return;
    }
    if (strncmp(p, "status", 6) == 0 || strncmp(p, "STATUS", 6) == 0) {
        PrintStatus();
        return;
    }
    if (strncmp(p, "home", 4) == 0 || strncmp(p, "HOME", 4) == 0) {
        g_pos_target_pan = 0;
        g_pos_target_tilt = 0;
        g_gimbal_mode = GIMBAL_POSCTRL;
        g_feedback_until = HAL_GetTick() + 150;
        CmdReply("# OK 双轴归零 (0°, 0°)\r\n");
        return;
    }
    if (strncmp(p, "tele", 4) == 0 || strncmp(p, "TELE", 4) == 0) {
        char *arg = p + 4;
        while (*arg == ' ') arg++;
        if (strncmp(arg, "on", 2) == 0 || strncmp(arg, "ON", 2) == 0) {
            g_telemetry_enabled = 1;
            CmdReply("# OK 遥测已开启 (每 100ms)\r\n");
        } else if (strncmp(arg, "off", 3) == 0 || strncmp(arg, "OFF", 3) == 0) {
            g_telemetry_enabled = 0;
            CmdReply("# OK 遥测已关闭\r\n");
        } else {
            CmdReply("# tele on/off  — 遥测开关\r\n");
        }
        return;
    }

    /* ---- [v5.10] K230 任务触发命令 (上位机菜单用): 与板载 PA0/PA1 等价 ---- */
    if (strncmp(p, "aim", 3) == 0) {
        TriggerK230Task(K230_CMD_AIM);
        CmdReply("# OK → AIM 自动打靶(跟随误差)\r\n");
        return;
    }
    if (strncmp(p, "recenter", 8) == 0) {
        TriggerK230Task(K230_CMD_RECENTER);
        CmdReply("# OK → RECENTER 激光对中到 A4 中心\r\n");
        return;
    }
    if (strncmp(p, "border", 6) == 0) {
        TriggerK230Task(K230_CMD_TRACE_BORDER);
        CmdReply("# OK → BORDER 外框循迹(固定矩形/单轴)\r\n");
        return;
    }
    if (strncmp(p, "inner", 5) == 0) {
        TriggerK230Task(K230_CMD_TRACE_INNER);
        CmdReply("# OK → INNER 内框循迹(电工胶带四点定位)\r\n");
        return;
    }

    /* ---- K230 自动打靶模式下: 仅放行"安全控制命令"(退出/调参/查询),
     *      拦截会抢占电机的手动步进命令(p/t/c/a/s/f/h/d 等),
     *      防止 AUTO 被误踢或自相冲突。这样串口/上位机就能在 AUTO 下
     *      正常切换功能 (stop / m / u / home / trace / k / pid / g / x / y ...)。 ----
     */
    if (g_gimbal_mode == GIMBAL_AUTO) {
        char c = *p;
        int takeover = 0;
        if (c=='p'||c=='P'||c=='t'||c=='T'||c=='f'||c=='F'||c=='h'||c=='H'||
            c=='c'||c=='C'||c=='a'||c=='A'||c=='s'||c=='S'||c=='d'||c=='D'||
            c=='+'||c=='-'||c=='r'||c=='w') takeover = 1;
        if (takeover) {
            CmdReply("# AUTO 忽略手动步进命令: %s\r\n", s_serial_buf);
            return;
        }
    }

    /* ---- 串口接管 + 视觉反馈 ----
     * 其他串口命令先把云台切到 SERIAL 模式, 避免被 Pan/Sweep/Dance 的主循环逻辑覆盖；
     * 同时让板载 LED 亮 150ms，给出明确的"已收到"视觉反馈。 */
    g_gimbal_mode = GIMBAL_SERIAL;
    g_feedback_until = HAL_GetTick() + 150;
    LED_ON();
    CmdReply("\r\n> %s\r\n", s_serial_buf);   /* 回显用户发出的命令 (走蓝牙) */

    /* ---- 单字符命令 (区分 i/I 和 p/P, 其余不区分大小写) ---- */
    switch (*p) {
        case 'p':   /* 选择 Pan 电机 (小写) */
            g_serial_motor = &g_motor_pan;
            CmdReply("# OK → 选中 Pan 电机 (水平)\r\n");
            return;
        case 'P':   /* 大写 P = 打印 PID 状态 */
            CmdReply("# Pan  PID: Kp=%.3f Ki=%.1f Kd=%.4f\n",
                     g_pid_pan.kp, g_pid_pan.ki, g_pid_pan.kd);
            CmdReply("  deadzone=%d invert=%d\n", PID_DEADZONE, g_pan_dir_invert);
            CmdReply("# Tilt PID: Kp=%.3f Ki=%.1f Kd=%.4f\n",
                     g_pid_tilt.kp, g_pid_tilt.ki, g_pid_tilt.kd);
            CmdReply("  deadzone=%d invert=%d\n", PID_DEADZONE_TILT, g_tilt_dir_invert);
            return;
        case 't': case 'T':   /* 选择 Tilt 电机 */
            g_serial_motor = &g_motor_tilt;
            CmdReply("# OK → 选中 Tilt 电机 (垂直)\r\n");
            return;
        case 'f': case 'F':
            Stepper_SetMode(g_serial_motor, STEP_MODE_FULL);
            CmdReply("# OK 全步步进 (大力矩)\r\n");
            return;
        case 'h': case 'H':
            Stepper_SetMode(g_serial_motor, STEP_MODE_HALF);
            CmdReply("# OK 半步步进 (平滑)\r\n");
            return;
        case 'c': case 'C':   /* 连续正转 */
            Stepper_SetDirection(g_serial_motor, DIR_CW);
            Stepper_RunContinuously(g_serial_motor);
            CmdReply("# OK %s 连续正转 (CW)\r\n", g_serial_motor->label);
            return;
        case 'a': case 'A':   /* 连续反转 */
            Stepper_SetDirection(g_serial_motor, DIR_CCW);
            Stepper_RunContinuously(g_serial_motor);
            CmdReply("# OK %s 连续反转 (CCW)\r\n", g_serial_motor->label);
            return;
        case 's': case 'S':   /* 停止 */
            Stepper_Stop(g_serial_motor);
            CmdReply("# OK %s 停止\r\n", g_serial_motor->label);
            return;
        case 'd': case 'D':   /* 进入舞蹈模式 */
            Stepper_RunContinuously(&g_motor_pan);
            Stepper_RunContinuously(&g_motor_tilt);
            g_gimbal_mode = GIMBAL_DANCE;
            g_dance_phase_start = 0;
            CmdReply("# OK 进入舞蹈模式 🎵\r\n");
            return;
        case 'i':   /* 小写 i = Pan 方向反转 */
            g_pan_dir_invert ^= 1;
            CmdReply("# OK Pan 方向反转: %s\n",
                   g_pan_dir_invert ? "ON (已反转)" : "OFF (正常)");
            return;
        case 'I':   /* 大写 I = Tilt 方向反转 */
            g_tilt_dir_invert ^= 1;
            CmdReply("# OK Tilt 方向反转: %s\n",
                   g_tilt_dir_invert ? "ON (已反转)" : "OFF (正常)");
            return;
        case 'u': case 'U':   /* 解除命中锁存, 重新捕获下一靶 */
            if (g_hit_locked) {
                g_hit_locked = 0;
                g_locked_feedback = 0;
                K230_PID_Reset(&g_pid_pan);
                K230_PID_Reset(&g_pid_tilt);
                g_pid_pan.locked  = 0;
                g_pid_tilt.locked = 0;
                CmdReply("# OK 解除命中锁存, 重新捕获\r\n");
            } else {
                CmdReply("# 当前未命中锁存\r\n");
            }
            return;
        case '?':
            PrintHelp();
            PrintStatus();
            return;
        case 'g': case 'G': {   /* 实时调参: g <p|t> <kp|ki|kd> <值> — 上位机 PID 调参面板用 */
            char axis = 0; char param[4] = {0}; float val = 0;
            if (sscanf(p + 1, "%c %3s %f", &axis, param, &val) == 3) {
                int isTilt = ((axis | 0x20) == 't');
                K230_PID *pid = isTilt ? &g_pid_tilt : &g_pid_pan;
                if      (strcmp(param, "kp") == 0) { if (val<0.001f)val=0.001f; if(val>0.05f)val=0.05f; pid->kp = val; }
                else if (strcmp(param, "ki") == 0) { if (val<10.0f)val=10.0f; if(val>150.0f)val=150.0f; pid->ki = val; }
                else if (strcmp(param, "kd") == 0) { if (val<0.0f)val=0.0f; if(val>0.01f)val=0.01f; pid->kd = val; }
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

/* 角度 → 步数: 舵机 1步=0.1°, 3600步/圈 */
static int32_t PosCtrl_AngleToSteps(float angle)
{
    return (int32_t)(angle * (float)SERVO_STEPS_PER_REV / 360.0f);
}

static float PosCtrl_StepsToAngle(int32_t steps)
{
    return (float)steps * 360.0f / (float)SERVO_STEPS_PER_REV;
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
    if (!g_telemetry_enabled) return;
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

/* 镜像一行遥测到 USART1(CH340/上位机): 单口即可收全部数据, 不依赖蓝牙 */
static void Telemetry_MirrorToU1(const char *buf, int n)
{
    if (n <= 0) return;
    if (!HAL_IS_BIT_SET(huart1.Instance->CR1, USART_CR1_UE)) return;
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 10);
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
    /* Yaw/Pitch: 真实舵机位置 → 0~255 (各轴硬限位线性映射) */
    int32_t p = g_motor_pan.pos_steps;
    if (p < g_motor_pan.pos_limit_min_hard) p = g_motor_pan.pos_limit_min_hard;
    if (p > g_motor_pan.pos_limit_max_hard) p = g_motor_pan.pos_limit_max_hard;
    buf[2] = (uint8_t)((p - g_motor_pan.pos_limit_min_hard) * 255 /
                        (g_motor_pan.pos_limit_max_hard - g_motor_pan.pos_limit_min_hard));
    int32_t t = g_motor_tilt.pos_steps;
    if (t < g_motor_tilt.pos_limit_min_hard) t = g_motor_tilt.pos_limit_min_hard;
    if (t > g_motor_tilt.pos_limit_max_hard) t = g_motor_tilt.pos_limit_max_hard;
    buf[3] = (uint8_t)((t - g_motor_tilt.pos_limit_min_hard) * 255 /
                        (g_motor_tilt.pos_limit_max_hard - g_motor_tilt.pos_limit_min_hard));
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

/**
 * @brief 向蓝牙(USART2)转发矩形识别结果
 *        格式: RECT,<dx>,<dy>,<w>,<h>,<ang>,<valid>\n
 *        dx,dy = 矩形中心相对靶心偏移(px); w,h = 矩形宽高(px); ang = 倾斜角(0.1°)
 *        valid = bit0 矩形有效 | bit1 靶心(原点)有效
 */
static void K230_SendRectTelemetry(void)
{
    if (!HAL_IS_BIT_SET(huart2.Instance->CR1, USART_CR1_UE)) return;
    if (BT_RingUsed() > BT_TX_RING_SIZE * 3 / 5) return;   /* 防溢出 */
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        "RECT,%+6d,%+6d,%5u,%5u,%+6d,%d\r\n",
        (int)g_rect_dx, (int)g_rect_dy,
        (unsigned)g_rect_w, (unsigned)g_rect_h,
        (int)g_rect_ang,
        (g_rect_valid ? 1 : 0) | (g_rect_origin_valid ? 2 : 0));
    if (n > 0) {
        BT_UART_PutStr(buf, n);
        Telemetry_MirrorToU1(buf, n);   /* [v5.10] 镜像到 USART1 */
    }
}

/**
 * @brief 向上位机/蓝牙转发 K230 任务状态
 *        格式: TASK,<id>,<done>\n
 *        id:  0=未知/IDLE/AIM 1=RECENTER 2=BORDER 3=INNER
 *        done:0=进行中 1=命中锁定/稳态达成(完成)
 */
static void K230_SendTaskTelemetry(void)
{
    if (!HAL_IS_BIT_SET(huart2.Instance->CR1, USART_CR1_UE)) return;
    if (BT_RingUsed() > BT_TX_RING_SIZE * 3 / 5) return;   /* 防溢出 */
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "TASK,%d,%d\r\n",
                     (int)g_k230_task_id, (int)g_k230_task_done);
    if (n > 0) {
        BT_UART_PutStr(buf, n);
        Telemetry_MirrorToU1(buf, n);   /* [v5.10] 镜像到 USART1 */
    }
}

/**
 * @brief 向 K230 发送命令帧 (PA0/PA1 按键经 STM32 控制 K230 任务)
 *        帧: 0x3C 0x3C [CMD][PARAM] 0x00 0x00 0x00 0x01 0x01 (9字节)
 *        CMD: 0x02=RECENTER 0x03=TRACE_BORDER 0x04=TRACE_INNER 0x05=STOP
 */
static void K230_SendCmd(uint8_t cmd)
{
    if (!HAL_IS_BIT_SET(huart3.Instance->CR1, USART_CR1_UE)) return;
    uint8_t buf[9];
    buf[0] = 0x3C; buf[1] = 0x3C;
    buf[2] = cmd;  buf[3] = 0;
    buf[4] = 0;    buf[5] = 0; buf[6] = 0;
    buf[7] = 1;    buf[8] = 1;
    HAL_UART_Transmit(&huart3, buf, 9, 20);
}

/**
 * @brief [v5.10] 串口/上位机触发 K230 任务: 既通知 K230 切换任务,
 *        又同步 STM32 本地镜像(任务ID/模式/解锁), 与板载 PA0/PA1 等价.
 *        PC 上位机发送 aim/recenter/border/inner 即走此路径.
 */
static void TriggerK230Task(uint8_t cmd)
{
    K230_SendCmd(cmd);        /* 命令帧发往 K230 (USART3) */
    K230_HandleCommand(cmd);  /* 更新 STM32 本地状态: 任务ID/AUTO/解锁/复位 */
}

/* ---- v4.3: PID 输出低通滤波 (每轴) ---- */
static float g_pan_out_filter = 0.0f;
static float g_tilt_out_filter = 0.0f;  /* Tilt 输出 EMA 滤波 */

/* ---- v5.5: 速度指令 EMA (加速度限幅, 抑制矩形拐角处方向突变导致的过冲) ---- */
static float g_pan_vel_ema  = 0.0f;
static float g_tilt_vel_ema = 0.0f;

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

    /* [v5.7] 当前是否处于矩形循迹(外框 task_id=2 / 内框 task_id=3) */
    int tracing = (g_k230_task_id == 2 || g_k230_task_id == 3);
    if (tracing) pid->locked = 0;   /* 循迹需连续跟随, 不进入定点锁靶态 */

    /* 滞回防抖: 已锁定时, 误差需超过 死区+迟滞 才解锁重新驱动, 否则保持停稳.
     * [v5.7] 循迹时跳过锁靶逻辑(须持续跟随移动 setpoint, 否则会脉冲式启停) */
    if (pid->locked && !tracing) {
        if (abs(err) <= deadzone_px + K230_HYST_PX) {
            if (Stepper_GetState(motor) != STATE_IDLE) Stepper_Stop(motor);
            return;
        } else {
            pid->locked = 0;
            K230_PID_Reset(pid);   /* 清除微分历史, 避免解锁瞬间 Kd 尖刺 */
        }
    }

    /* 速度指令 EMA 指针 (每轴独立, 用于加速度限幅) */
    float *vel_ema = (motor == &g_motor_pan) ? &g_pan_vel_ema : &g_tilt_vel_ema;

    if (abs(err) > deadzone_px || tracing) {
        /* 卡尔曼平滑噪声 */
        float kf_err = Kalman_Update(kf, (float)err);

        /* v5.0 速度控制器: 输出为带符号速度指令 [-1,1] (无积分, 不过冲) */
        float vel_out = K230_PID_Update(pid, kf_err);
        if (vel_out >  1.0f) vel_out =  1.0f;
        if (vel_out < -1.0f) vel_out = -1.0f;

        float dir_out = invert ? -vel_out : vel_out;   /* 带符号方向正确速度指令 [-1,1] */

        /* ---- 减速区: 误差越接近靶心, 速度越低, 进死区前线性归零 ----
         *  shape = (|err| - deadzone) / (dec_zone - deadzone), 限幅 [0,1]
         *  这是根治"过冲靶心"的关键: 速度在到达死区前已被迫降到 0.
         *  [v5.7] 循迹时解除 dec 限幅(dec_zone 拉大), 改由速度地板控制, 实现直线提速 */
        float dec_zone = pid->ki;                       /* ki 字段 = 减速区宽度(px) */
        if (dec_zone <= (float)deadzone_px) dec_zone = (float)deadzone_px + 1.0f;
        if (tracing) dec_zone = 1.0e6f;                 /* 循迹: 取消 dec 限幅 */
        float a = (kf_err > 0.0f) ? kf_err : -kf_err;
        float shape = (a - (float)deadzone_px) / (dec_zone - (float)deadzone_px);
        if (shape < 0.0f) shape = 0.0f;
        if (shape > 1.0f) shape = 1.0f;

        /* 减速区限幅(定点锁靶用): 幅值上限 = shape, 方向保持 dir_out 符号 */
        float mag = (dir_out >= 0.0f) ? dir_out : -dir_out;
        if (mag > shape) mag = shape;
        mag *= max_speed_scale;                          /* 该轴最高速度(垂直轴 0.6) */
        if (mag < 0.0f) mag = 0.0f;
        float signed_cmd = (dir_out >= 0.0f) ? mag : -mag;  /* 重新带符号的速度指令 */

        /* [v5.7] 矩形循迹速度整形:
         *  - 直线段: 速度地板 = TRACE_SPEED, 提升跟随速度(画直线提速, 保证流畅)
         *  - 接近拐角(K230 置 bit5): 地板降为 TRACE_SPEED*CORNER_FACTOR, 强减速防过冲
         *  [v5.8] 误差相关地板: |err|<TRACE_FLOOR_ERR_PX 时地板降为 TRACE_NEAR_FLOOR,
         *   允许误差归零, 消除恒定 0.60 地板在误差→0 时仍强推导致的极限环(抖动/偏移).
         *  地板保留符号; err≈0 时沿用 EMA 历史方向, 避免强制单向漂移 */
        if (tracing) {
            float tspeed = g_k230_corner ? (TRACE_SPEED * CORNER_FACTOR) : TRACE_SPEED;
            if (abs(err) < TRACE_FLOOR_ERR_PX) {
                tspeed = TRACE_NEAR_FLOOR;   /* 近线极小地板: 误差可归零, 激光贴线不抖 */
            }
            float sc_mag = (signed_cmd >= 0.0f) ? signed_cmd : -signed_cmd;
            if (sc_mag < tspeed) {
                float sgn = (signed_cmd > 0.0f) ? 1.0f
                          : (signed_cmd < 0.0f) ? -1.0f
                          : ((*vel_ema >= 0.0f) ? 1.0f : -1.0f);  /* err≈0 → 沿用上次方向 */
                signed_cmd = sgn * tspeed;
            }
        }

        /* [v5.7] 定点锁靶停止带: 非循迹且速度指令低于阈值, 直接停机并锁靶,
         * 彻底消除靶心附近"爬行微抖/摇摆不定". 循迹时不用(需连续跟随). */
        if (!tracing) {
            float sc2 = (signed_cmd >= 0.0f) ? signed_cmd : -signed_cmd;
            if (sc2 < VEL_STOP_BAND) {
                if (Stepper_GetState(motor) != STATE_IDLE) Stepper_Stop(motor);
                K230_PID_Reset(pid);
                kf->init = 0;
                *out_filter = 0.0f;
                /* 注意: 不清 vel_ema, 保留上次方向记忆, 避免噪声使误差小幅反向时反复启停(消摇摆) */
                pid->locked = 1;
                return;
            }
        }

        /* 速度指令 EMA (加速度限幅, 带符号): 平滑速度变化, 抑制矩形拐角处方向突变导致的过冲.
         * [v5.7] 0.6/0.4 略增响应, 配合停止带消除微抖 */
        if (*vel_ema == 0.0f) *vel_ema = signed_cmd;
        else *vel_ema = *vel_ema * 0.6f + signed_cmd * 0.4f;

        /* 带符号速度 → 方向 + 幅值, 供后续步进驱动 */
        float vel_mag = (*vel_ema >= 0.0f) ? *vel_ema : -(*vel_ema);
        Direction dir = (*vel_ema >= 0.0f) ? DIR_CW : DIR_CCW;

        /* 速度幅值 → 步间延时 (vel=0→最慢, vel=1→最快) */
        uint32_t range = STEPPER_MAX_DELAY_US - STEPPER_MIN_DELAY_US;
        uint32_t delay_us = STEPPER_MAX_DELAY_US - (uint32_t)(vel_mag * (float)range);
        if (delay_us < STEPPER_MIN_DELAY_US) delay_us = STEPPER_MIN_DELAY_US;
        if (delay_us > STEPPER_MAX_DELAY_US) delay_us = STEPPER_MAX_DELAY_US;

        /* 软限位: 超出安全行程则停转并清状态.
         * [v5.8 修复] 旧逻辑 next>+LIM||next<-LIM 在电机已停在限位外(如位置环先把它
         *   带到了 ±LIM 之外)时会双向锁死, 该轴永远无法回到限位内 → 长期不动(Pan 卡死即此因).
         *   新逻辑: 仅禁止"进一步向外"的步进, 允许从限位外向中心回拉, 保证可恢复. */
        if (pos_limit_steps > 0) {
            int32_t pos = Stepper_GetPosition(motor);
            int32_t next = (dir == DIR_CW) ? pos + 1 : pos - 1;
            int blocked = 0;
            if (pos >= pos_limit_steps) {
                if (dir == DIR_CW) blocked = 1;          /* 已在 +限位外, 禁止继续 + */
            } else if (pos <= -pos_limit_steps) {
                if (dir == DIR_CCW) blocked = 1;         /* 已在 -限位外, 禁止继续 - */
            } else if (next > pos_limit_steps || next < -pos_limit_steps) {
                blocked = 1;                             /* 正常区间: 禁止越界 */
            }
            if (blocked) {
                if (Stepper_GetState(motor) != STATE_IDLE) Stepper_Stop(motor);
                K230_PID_Reset(pid);
                kf->init = 0;
                *out_filter = 0.0f;
                *vel_ema = 0.0f;
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
        /* 死区内: 停止并锁存, 重置速度控制器 + 卡尔曼 + 滤波器 */
        if (Stepper_GetState(motor) != STATE_IDLE) {
            Stepper_Stop(motor);
        }
        K230_PID_Reset(pid);
        kf->init = 0;
        *out_filter = 0.0f;
        *vel_ema = 0.0f;
        pid->locked = 1;     /* 进入锁靶态, 后续靠滞回迟滞防抖 */
    }
}

/**
 * @brief 处理 K230 坐标帧，驱动 Pan/Tilt 电机
 *        主循环中调用，保证不会在 ISR 上下文中操作电机
 */
static uint8_t g_conf_streak = 0;   /* 置信度连续有效帧计数(去抖) */

/**
 * @brief 冻结云台 - 检测不可靠/丢失时停机并重置控制器, 防止用脏数据做闭环
 */
static void K230_Freeze(const char *reason)
{
    if (Stepper_GetState(&g_motor_pan)  != STATE_IDLE) Stepper_Stop(&g_motor_pan);
    if (Stepper_GetState(&g_motor_tilt) != STATE_IDLE) Stepper_Stop(&g_motor_tilt);
    K230_PID_Reset(&g_pid_pan);
    K230_PID_Reset(&g_pid_tilt);
    g_pid_pan.locked  = 0;
    g_pid_tilt.locked = 0;
    g_pan_out_filter  = 0.0f;
    g_tilt_out_filter = 0.0f;
    g_kf_x.init = 0; g_kf_y.init = 0;
    g_conf_streak = 0;
    g_hit_locked  = 0;   /* 检测丢失时解除命中锁存, 允许重新捕获 */
    g_k230_hold_first  = 0;   /* 冻结时清除 HOLD/稳态计时与完成标志 */
    g_k230_steady_tick = 0;
    g_k230_task_done   = 0;
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_print > 1000) {
        last_print = now;
        printf("\r\n[K230] 冻结云台: %s | frames=%lu crc_err=%lu last_err=(%d,%d) flags=0x%02X\r\n",
               reason, (unsigned long)g_k230_frames, (unsigned long)g_k230_crc_err,
               (int)g_k230_err_x, (int)g_k230_err_y, (unsigned)g_k230_flags);
    }
}

/**
 * @brief 处理 K230 命令帧 (按键触发, 主循环上下文调用)
 *        命令: 0x01=AIM 0x02=RECENTER 0x03=TRACE_BORDER 0x04=TRACE_INNER 0x05=STOP
 *        - 切换显示用任务 ID
 *        - 解除命中锁存, 允许重新捕获/再运行
 *        - STOP 直接冻结云台
 */
static void K230_HandleCommand(uint8_t cmd)
{
    const char *name = "?";
    uint8_t id = 0, stop = 0;
    switch (cmd) {
        case 0x01: name = "AIM";      id = 0; break;  /* 通用自动打靶(跟随误差) */
        case 0x02: name = "RECENTER"; id = 1; break;  /* 激光对中到 A4 中心 */
        case 0x03: name = "BORDER";   id = 2; break;  /* 激光绕外框一周 */
        case 0x04: name = "INNER";    id = 3; break;  /* 激光绕内框一周 */
        case 0x05: name = "STOP";     id = 0; stop = 1; break;
        default:   name = "UNK";      break;
    }
    g_k230_task_id   = id;
    g_k230_task_done = 0;          /* 新任务: 清除完成标志 */
    g_k230_steady_tick = 0;
    g_hit_locked     = 0;          /* 解除命中锁存, 重新捕获 */
    g_locked_feedback = 0;
    K230_PID_Reset(&g_pid_pan);
    K230_PID_Reset(&g_pid_tilt);
    g_pid_pan.locked  = 0;
    g_pid_tilt.locked = 0;
    g_pan_vel_ema = 0.0f; g_tilt_vel_ema = 0.0f;   /* 清除速度滤波历史 */
    if (stop) {
        K230_Freeze("收到 STOP 命令");
        g_auto_enabled = 0;
        g_gimbal_mode = GIMBAL_POSCTRL;
        printf("\r\n[K230] 收到命令[STOP] → 停车/位置闭环 (自动打靶已禁用)\r\n");
    } else {
        g_auto_enabled = 1;            /* 命令即进入自动打靶, 等待 K230 坐标帧驱动 */
        g_gimbal_mode = GIMBAL_AUTO;
        g_k230_last_valid = 0;
        g_k230_last_frame = HAL_GetTick();
        printf("\r\n[K230] 收到命令[%s] 任务=%d → 自动打靶(AUTO)\r\n", name, id);
    }
}

/**
 * @brief 处理 K230 坐标帧，驱动 Pan/Tilt 电机
 *        主循环中调用，保证不会在 ISR 上下文中操作电机
 *
 * 坐标系统一约定 (图像坐标系 -> 云台物理运动):
 *   err_x = 靶心_x - 激光_x  (图像 x 向右为正)
 *   err_y = 靶心_y - 激光_y  (图像 y 向下为正)
 *   闭环目标: 让激光移动到靶心, 即消除 err。
 *   控制量符号: dir_out = invert ? -vel : vel;  dir_out>=0 -> DIR_CW -> 位置计数+。
 *   机械安装方向差异由 per-axis invert 吸收:
 *     Pan : g_pan_dir_invert  = 1  (水平机械方向需反转)
 *     Tilt: g_tilt_dir_invert = 0
 *   -> 两轴共用同一控制函数与符号约定, 仅 invert 不同, 坐标系统一。
 */
static void K230_ProcessFrame(void)
{
    /* 处理来自 K230 的命令帧(按键触发): 切换任务/解锁 — 主循环上下文, 可安全 printf */
    if (g_k230_cmd_pending) {
        K230_HandleCommand(g_k230_cmd_pending);
        g_k230_cmd_pending = 0;
    }

    if (!g_k230_frame_ready) return;
    g_k230_frame_ready = 0;

    int16_t err_x = g_k230_err_x;
    int16_t err_y = g_k230_err_y;
    uint8_t flags = g_k230_flags;
    uint8_t valid = flags & 0x01;
    uint8_t hit   = flags & 0x02;
    uint8_t conf  = flags & 0x04;   /* 高置信度(置信度检测通过) */
    uint8_t hold  = flags & K230_FLAG_HOLD;  /* 激光丢失: 保持速度 500ms 再停车 */

    /* 更新时间戳 */
    g_k230_last_frame = HAL_GetTick();

    /* ---- 置信度门控: 必须靶心+激光都识别(valid)才控制; 否则冻结(停机+重置) ---- */
    if (!valid) {
        K230_Freeze("目标/激光丢失(valid=0)");
        return;
    }
    if (++g_conf_streak < 2) {
        return;   /* 置信度刚建立, 等稳 1 帧再驱动 */
    }

    /* 循迹测试模式: 忽略 K230 坐标, 不切 AUTO */
    if (g_gimbal_mode == GIMBAL_TRACE) return;

    /* 记录最后有效帧时间 */
    g_k230_last_valid = HAL_GetTick();

    /* HOLD 处理: 激光丢失时 K230 保持设定点并外推激光, 请求维持当前速度 500ms 再停车.
     * 激光短暂丢失(<500ms)时云台平滑跟随外推设定点, 不抖动; 持续丢失则安全停车. */
    if (hold) {
        uint32_t now = HAL_GetTick();
        if (g_k230_hold_first == 0) g_k230_hold_first = now;
        else if (now - g_k230_hold_first > K230_HOLD_TIMEOUT_MS) {
            K230_Freeze("HOLD 超时(激光持续丢失>500ms)");
            return;
        }
    } else {
        g_k230_hold_first = 0;
    }

    /* 置信度决定精度: conf=1 精确模式(紧死区, 可命中); conf=0 粗跟模式(宽死区, 不追噪声尖端) */
    int16_t pan_dz  = conf ? PID_DEADZONE     : (PID_DEADZONE + 6);
    int16_t tilt_dz = conf ? TILT_DEADZONE     : (TILT_DEADZONE + 6);

    /* 切换到自动打靶模式:
     *   - 开机前 3 秒不进 AUTO (防"上电就乱转")
     *   - 不在 AUTO 模式时不驱动电机 (避免积积分/空转) */
    if (HAL_GetTick() <= 3000) {
        /* 开机保护期: 不切模式, 不驱动电机 */
        return;
    }

    if (g_auto_enabled && g_gimbal_mode != GIMBAL_AUTO) {
        g_gimbal_mode = GIMBAL_AUTO;
        K230_PID_Reset(&g_pid_pan);
        K230_PID_Reset(&g_pid_tilt);
        g_kf_x.init = 0; g_kf_y.init = 0;
        printf("\r\n[K230] → 自动打靶 双轴控制\n");
    } else if (!g_auto_enabled) {
        /* 用户已通过 PA0/STOP 显式停用自动打靶: 即便收到坐标帧也保持冻结, 不抢占 */
        return;
    }

    /* 任务完成(稳态)检测: 有活动任务 + 双轴进入死区并稳定 ≥ STEADY_DONE_MS → 标记完成.
     * 对 RECENTER 等"到达即完成"的任务尤为关键(K230 不会发 hit); 循迹任务主要靠 hit 锁存. */
    if (g_k230_task_id != 0 && !hold) {
        if (abs(err_x) <= pan_dz && abs(err_y) <= tilt_dz) {
            if (g_k230_steady_tick == 0) g_k230_steady_tick = HAL_GetTick();
            else if (HAL_GetTick() - g_k230_steady_tick >= K230_STEADY_DONE_MS) {
                if (!g_k230_task_done) {
                    g_k230_task_done = 1;
                    printf("\r\n[K230] 任务%d 稳态达成 → 完成\r\n", g_k230_task_id);
                }
            }
        } else {
            g_k230_steady_tick = 0;
        }
    } else {
        g_k230_steady_tick = 0;
    }

    /* ---- 命中锁存: 一旦 hit&&conf 即双轴锁死停稳, 不再受小误差扰动 ----
     * 根治"打到靶上停不下来": 原锁存解锁阈值仅 死区+4px=7px, 而命中容差15px,
     * 导致 7~15px 区间内反复解锁→重追→再锁。现改为粘滞锁存:
     *   - 命中即锁, 电机停、PID 重置
     *   - 仅当误差明显偏离(>HIT_RELEASE_PX=40px, 目标真的移动)或丢帧才释放
     *   - 可发 'u' 命令手动解锁, 便于连续打多个靶 */
    if (g_hit_locked) {
        /* 已锁存: 默认保持停稳; 检查释放条件 */
        int reacq = (abs(err_x) > HIT_RELEASE_PX) || (abs(err_y) > HIT_RELEASE_PX);
        if (reacq) {
            g_hit_locked = 0;
            g_k230_task_done   = 0;   /* 命中解除: 任务视为未完成, 重新捕获 */
            g_k230_steady_tick = 0;
            g_locked_feedback = 0;
            K230_PID_Reset(&g_pid_pan);
            K230_PID_Reset(&g_pid_tilt);
            g_pid_pan.locked  = 0;
            g_pid_tilt.locked = 0;
            printf("\r\n[K230] 误差偏离>%dpx, 解除命中锁存重新捕获\r\n", HIT_RELEASE_PX);
        } else {
            /* 保持锁定: 确保电机停稳 */
            if (Stepper_GetState(&g_motor_pan)  != STATE_IDLE) Stepper_Stop(&g_motor_pan);
            if (Stepper_GetState(&g_motor_tilt) != STATE_IDLE) Stepper_Stop(&g_motor_tilt);
            return;
        }
    }

    /* ---- Pan 轴 (X 误差) 软限位 ±40°, 置信度决定死区 ---- */
    K230_ControlAxis(&g_motor_pan, &g_pid_pan, err_x, &g_kf_x,
                     g_pan_dir_invert, "Pan", &g_pan_out_filter,
                     pan_dz, PAN_MAX_STEPS, 1.0f);

    /* ---- Tilt 轴 (Y 误差) 软限位 ±30°, 置信度决定死区 ---- */
    K230_ControlAxis(&g_motor_tilt, &g_pid_tilt, err_y, &g_kf_y,
                     g_tilt_dir_invert, "Tilt", &g_tilt_out_filter,
                     tilt_dz, TILT_MAX_STEPS, TILT_SPEED_SCALE);

    /* 命中锁存触发: K230 确认靶心锁定(hit=1) 且高置信度 → 进入锁存 */
    if (hit && conf) {
        g_feedback_until = HAL_GetTick() + 200;
        LED_ON();
        g_k230_task_done = 1;   /* K230 报告命中(循迹一圈/靶心锁定) → 任务完成 */
        if (!g_locked_feedback) {
            g_locked_feedback = 1;
            printf("\r\n[K230] 命中锁定 → 双轴保持 (发 u 解锁/再捕获)\r\n");
        }
        Stepper_Stop(&g_motor_pan);
        Stepper_Stop(&g_motor_tilt);
        K230_PID_Reset(&g_pid_pan);
        K230_PID_Reset(&g_pid_tilt);
        g_pid_pan.locked  = 1;   /* 锁靶, 后续靠滞回迟滞防抖 */
        g_pid_tilt.locked = 1;
        g_hit_locked = 1;        /* 粘滞锁存 */
    } else {
        g_locked_feedback = 0;   /* 未命中, 正常追踪 */
    }

    /* [调试心跳] 每 1s 打印 K230 帧统计, 便于串口定位通信问题 */
    {
        static uint32_t k230_hb_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - k230_hb_tick > 1000) {
            k230_hb_tick = now;
            printf("\r\n[K230] hb: frames=%lu crc_err=%lu valid=%d err=(%d,%d) flags=0x%02X task=%d done=%d",
                   (unsigned long)g_k230_frames, (unsigned long)g_k230_crc_err,
                   (int)valid, (int)g_k230_err_x, (int)g_k230_err_y,
                   (unsigned)g_k230_flags, (int)g_k230_task_id, (int)g_k230_task_done);
        }
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

/* ========================== PA0/PA1 用户独立按键 ==========================
 * PA0 (功能键):
 *   短按 → 自动打靶 开/关  (GIMBAL_AUTO ↔ GIMBAL_POSCTRL)
 *   长按(≥1.5s) → 回中 (目标清零 + 释放命中锁存 + PID 复位)
 * PA1 (循迹键):
 *   短按 → 矩形循迹 开/关  (GIMBAL_TRACE ↔ 停止/POSCTRL)
 *   长按(≥1.5s) → 解锁命中锁存 (同 'u' 命令)
 * 设计说明: 串口功能切换偶发不可靠时, 这两个板载按键提供确定的硬件控制.
 */
static void PA0_ShortPress(void)
{
    if (g_gimbal_mode == GIMBAL_AUTO) {
        K230_Freeze("PA0 退出自动打靶");
        g_auto_enabled = 0;                 /* 停用自动打靶, 不被坐标帧抢占 */
        g_gimbal_mode  = GIMBAL_POSCTRL;
        K230_SendCmd(K230_CMD_STOP);        /* 同步告知 K230 停止任务 */
        printf("\r\n[PA0] 退出自动打靶 → 位置闭环\r\n");
    } else {
        g_auto_enabled = 1;
        g_gimbal_mode  = GIMBAL_AUTO;
        g_k230_task_id   = 0;
        g_k230_task_done = 0;
        g_k230_last_valid = 0;
        g_k230_last_frame = HAL_GetTick();
        g_hit_locked = 0;
        g_pan_vel_ema = 0.0f; g_tilt_vel_ema = 0.0f;
        K230_SendCmd(K230_CMD_RECENTER);    /* 同步告知 K230: 激光对中到矩形中心 */
        printf("\r\n[PA0] → 自动打靶(AUTO), K230 对中\r\n");
    }
}

static void PA0_LongPress(void)
{
    /* 回中: 目标清零 + 释放命中锁存 + PID 复位 */
    g_pos_target_pan = 0; g_pos_target_tilt = 0;
    g_hit_locked = 0; g_locked_feedback = 0;
    K230_PID_Reset(&g_pid_pan); K230_PID_Reset(&g_pid_tilt);
    g_pid_pan.locked = 0; g_pid_tilt.locked = 0;
    g_pan_vel_ema = 0.0f; g_tilt_vel_ema = 0.0f;
    g_auto_enabled = 0;
    g_gimbal_mode  = GIMBAL_POSCTRL;
    K230_SendCmd(K230_CMD_STOP);            /* 同步停止 K230 任务 */
    g_feedback_until = HAL_GetTick() + 150;
    printf("\r\n[PA0长按] 回中: 目标清零 / 释放命中锁存 / PID复位\r\n");
}

static void PA1_ShortPress(void)
{
    /* [v5.7] PA1 真正驱动 K230 视觉循迹: 发命令给 K230, STM32 保持 AUTO 跟随坐标帧.
     *   不再使用开环 GIMBAL_TRACE(它会在 K230_ProcessFrame 早退, 完全忽略 K230). */
    if (g_k230_task_id == 2 || g_k230_task_id == 3) {
        /* 正在循迹 → 停止 */
        g_k230_task_id   = 0;
        g_k230_task_done = 0;
        g_auto_enabled = 0;
        g_gimbal_mode  = GIMBAL_POSCTRL;
        K230_SendCmd(K230_CMD_STOP);        /* 通知 K230 停下循迹 */
        g_pan_vel_ema = 0.0f; g_tilt_vel_ema = 0.0f;
        printf("\r\n[PA1] 停止矩形循迹 → 位置闭环\r\n");
    } else {
        /* 开始外框循迹: 命令 K230 进入 TRACE_BORDER, STM32 保持 AUTO 跟随 */
        g_k230_task_id   = 2;
        g_k230_task_done = 0;
        g_auto_enabled = 1;
        g_gimbal_mode  = GIMBAL_AUTO;        /* 关键: 保持 AUTO, 处理 K230 坐标帧 */
        g_hit_locked = 0;
        g_pan_vel_ema = 0.0f; g_tilt_vel_ema = 0.0f;
        K230_SendCmd(K230_CMD_TRACE_BORDER);
        printf("\r\n[PA1] → 矩形循迹(K230 外框周长), STM32 跟随\r\n");
    }
}

static void PA1_LongPress(void)
{
    if (g_hit_locked) {
        g_hit_locked = 0; g_locked_feedback = 0;
        K230_PID_Reset(&g_pid_pan); K230_PID_Reset(&g_pid_tilt);
        g_pid_pan.locked = 0; g_pid_tilt.locked = 0;
        printf("\r\n[PA1长按] 解除命中锁存, 重新捕获\r\n");
    } else {
        printf("\r\n[PA1长按] 当前未命中锁存\r\n");
    }
}

/* PA0/PA1 扫描: 20ms 去抖 + 短按/长按(≥1.5s)区分, 长按在按住期间即触发 */
static void PA_KeyScan(void)
{
    static uint32_t scan_tick = 0;
    static uint8_t  pa0_state = 0, pa1_state = 0;
    static uint32_t pa0_down  = 0, pa1_down  = 0;
    static uint8_t  pa0_long  = 0, pa1_long  = 0;
    const uint32_t LONG_MS = 1500;
    uint32_t now = HAL_GetTick();

    if ((now - scan_tick) < 20) return;    /* 20ms 扫描周期 */
    scan_tick = now;

    uint8_t pa0 = (KEY_PA0_READ() == GPIO_PIN_RESET) ? 1 : 0;  /* 按下=低 */
    uint8_t pa1 = (KEY_PA1_READ() == GPIO_PIN_RESET) ? 1 : 0;

    /* ---- PA0 ---- */
    if (pa0 && !pa0_state) {            /* 下降沿: 刚按下 */
        pa0_state = 1; pa0_down = now; pa0_long = 0;
    } else if (pa0 && pa0_state) {      /* 按住中 */
        if (!pa0_long && (now - pa0_down) >= LONG_MS) { pa0_long = 1; PA0_LongPress(); }
    } else if (!pa0 && pa0_state) {     /* 上升沿: 释放 */
        if (!pa0_long) PA0_ShortPress();
        pa0_state = 0; pa0_long = 0;
    }

    /* ---- PA1 ---- */
    if (pa1 && !pa1_state) {
        pa1_state = 1; pa1_down = now; pa1_long = 0;
    } else if (pa1 && pa1_state) {
        if (!pa1_long && (now - pa1_down) >= LONG_MS) { pa1_long = 1; PA1_LongPress(); }
    } else if (!pa1 && pa1_state) {
        if (!pa1_long) PA1_ShortPress();
        pa1_state = 0; pa1_long = 0;
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

    /* [v5.2] 遥测防溢出: 缓冲≥60% 满时跳帧, 避免 CmdReply 发不出 */
    if (BT_RingUsed() > BT_TX_RING_SIZE * 3 / 5) return;

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
        Telemetry_MirrorToU1(buf, n);   /* [v5.10] 镜像到 USART1(CH340/上位机) */
    }

    /* 同步转发矩形识别结果(相对靶心原点) */
    K230_SendRectTelemetry();
    /* 转发 K230 任务状态(命令帧切换 + 完成标志) */
    K230_SendTaskTelemetry();
}

/** @brief 将 GimbalMode 枚举映射为可显示字符串 (安全, 无越界) */
static const char* ModeName(GimbalMode mode)
{
    switch (mode) {
        case GIMBAL_PAN:     return "PAN";
        case GIMBAL_TILT:    return "TILT";
        case GIMBAL_SWEEP:   return "SWEEP";
        case GIMBAL_SERIAL:  return "SERIAL";
        case GIMBAL_DANCE:   return "DANCE";
        case GIMBAL_AUTO:    return "AUTO";
        case GIMBAL_POSCTRL: return "POS";
        case GIMBAL_TRACE:   return "TRACE";
        default:             return "???";
    }
}

/** @brief 收集当前云台状态并刷新到 OLED (无 OLED 时直接返回) */
static void OLED_Refresh(void)
{
    if (!OLED_Debug_IsReady()) return;

    OledDebugInfo info;
    memset(&info, 0, sizeof(info));

    info.mode = ModeName(g_gimbal_mode);
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

    /* 【关键】立即喂狗 — 若选项字节开启了 IWDG, 上电即开始计数, 不喂狗约 1s 后复位 */
    IWDG_FEED();

    /* 心跳: PC13 LED 闪烁 3 次确认 MCU 运行 */
    GPIO_InitTypeDef gpio_led = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio_led.Pin = GPIO_PIN_13;
    gpio_led.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio_led);
    for (int i=0; i<3; i++) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); HAL_Delay(100); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_Delay(100); }
    /* 注意: GPIO_Init() 中会重新初始化 PC13, 上面的闪烁仅用于诊断 */

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
    IWDG_FEED();    /* OLED 初始化有多个 HAL_Delay, 喂狗防复位 */

    /* 先初始化定时器硬件, 再初始化舵机 (Stepper_Init 内调 Servo_UpdatePWM 需要 htim1/htim3.Instance != NULL) */
    TIM1_Init();        /* Tilt 舵机 PWM 定时器 (PA11=TIM1_CH4) */
    IWDG_FEED();
    TIM3_Init();        /* Pan 舵机 PWM 定时器 (PB1=TIM3_CH4) */
    IWDG_FEED();
    TIM2_Init();        /* Pan 步进节拍定时器 */
    IWDG_FEED();
    TIM4_Init();        /* Tilt 步进节拍定时器 */
    IWDG_FEED();

    /* 初始化两个舵机 (PWM 定时器已就绪, 可以安全写入 CCR)
     * Pan : S20F 270° 舵机, TIM2节拍 + TIM3_CH4(PB1) PWM
     * Tilt: S20F 180° 舵机, TIM4节拍 + TIM1_CH4(PA11) PWM */
    Stepper_Init(&g_motor_pan,  &htim2, &htim3, SERVO_PAN_CHANNEL,
                  PAN_PULSE_MIN_US, PAN_PULSE_MAX_US, PAN_CENTER_US, PAN_US_PER_DEG,
                  POS_LIMIT_PAN_MIN, POS_LIMIT_PAN_MAX,
                  "Pan");
    IWDG_FEED();
    Stepper_Init(&g_motor_tilt, &htim4, &htim1, SERVO_TILT_CHANNEL,
                  TILT_PULSE_MIN_US, TILT_PULSE_MAX_US, TILT_CENTER_US, TILT_US_PER_DEG,
                  POS_LIMIT_TILT_MIN, POS_LIMIT_TILT_MAX,
                  "Tilt");
    IWDG_FEED();

    /* 舵机自检已取消 (上电直接回中位, 省去 3 次扫角) */
    __HAL_TIM_SET_COMPARE(&htim3, SERVO_PAN_CHANNEL,  PAN_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim1, SERVO_TILT_CHANNEL, TILT_CENTER_US);

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

    printf("\r\n");
    printf("  ╔═══════════════════════════╗\r\n");
    printf("  ║   舵机云台 控制系统 V1.0   ║\r\n");
    printf("  ║   Firmware v5.11 (K230任务串口触发) ║\r\n");
    printf("  ║   STM32F103C8T6 @ 72MHz  ║\r\n");
    printf("  ║   Pan: PB1  Tilt: PA11   ║\r\n");
    printf("  ╚═══════════════════════════╝\r\n");
    printf("  输入 help 查看命令菜单\r\n");
    printf("  例: x30 y10 → Pan=30°, Tilt=10°\r\n\r\n");

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
        PA_KeyScan();     /* PA0/PA1 用户独立按键: 自动打靶开关/回中, 矩形循迹/解锁 */

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
        IWDG_FEED();    /* 主循环喂狗 — IWDG 超时约 1s, 循环内至少每秒喂 1 次 */
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
 * @brief TIM4 中断 — Tilt 电机步进节拍 (TIM3 现为 Pan 舵机 PWM 输出)
 */
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
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
    /* 先清除溢出错误 (ORE 置位时 RXNE 读到的是脏数据) */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

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
}

/**
 * @brief USART2 中断 (JDY-31 蓝牙)
 *        - TXE : 从环形缓冲续发剩余字节(非阻塞发送)
 *        - RXNE: 缓冲接收到的命令, 并更新链路活跃时间戳
 *        - 错误标志(ORE/NE/FE/PE)清除: 防止 UART 因噪声/溢出卡死
 */
void USART2_IRQHandler(void)
{
    /* ---- 错误先清除: ORE/NE/FE/PE 必须优先处理, 否则 RXNE 可能读到脏数据 ---- */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(&huart2);
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)  != RESET) { volatile uint8_t tmp __attribute__((unused)) = (uint8_t)(huart2.Instance->DR & 0xFF); }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)  != RESET) { volatile uint8_t tmp __attribute__((unused)) = (uint8_t)(huart2.Instance->DR & 0xFF); }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE)  != RESET) { volatile uint8_t tmp __attribute__((unused)) = (uint8_t)(huart2.Instance->DR & 0xFF); }

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
                    /* 坐标帧开始 (误差帧) */
                    g_k230_ftype = 0;
                    g_k230_fstate = K230_DATA;
                    g_k230_fidx = 0;
                } else if (byte == 0x3A) {
                    /* 矩形帧开始 */
                    g_k230_ftype = 1;
                    g_k230_fstate = K230_DATA;
                    g_k230_fidx = 0;
                } else if (byte == 0x3C) {
                    /* 连续 0x3C → 命令帧开始 (0x3C 0x3C [CMD][PARAM] 0 0 0 1 1) */
                    g_k230_ftype = 2;
                    g_k230_fstate = K230_CMD;
                    g_k230_fidx = 0;
                } else {
                    g_k230_fstate = K230_IDLE;  /* 非预期, 丢弃 */
                }
                break;

            case K230_DATA:
                if (g_k230_ftype == 0) {
                    /* 误差帧: 收集 XH XL YH YL FLAG CRC (6字节) */
                    if (g_k230_fidx < 6) {
                        g_k230_fbuf[g_k230_fidx++] = byte;
                    }
                    if (g_k230_fidx >= 6) {
                        g_k230_fstate = K230_FOOTER1;
                    }
                } else {
                    /* 矩形帧: 收集 CXH..ANGL + FLAGS (11字节) */
                    if (g_k230_fidx < 11) {
                        g_k230_fbuf[g_k230_fidx++] = byte;
                    }
                    if (g_k230_fidx >= 11) {
                        g_k230_fstate = K230_RECT_CRC;
                    }
                }
                break;

            case K230_CMD:
                /* 命令帧: 收集 CMD, PARAM, 0, 0, 0 (5字节); 之后是 0x01 0x01 帧尾 */
                if (g_k230_fidx < 5) {
                    g_k230_fbuf[g_k230_fidx++] = byte;
                }
                if (g_k230_fidx >= 5) {
                    g_k230_fstate = K230_FOOTER1;
                }
                break;

            case K230_RECT_CRC:
                g_k230_rcrc = byte;          /* 矩形帧 CRC 字节 */
                g_k230_fstate = K230_FOOTER1;
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
                if (g_k230_ftype == 0) {
                    /* 误差帧: CRC 覆盖 XH XL YH YL FLAG (5字节) */
                    uint8_t crc_calc = K230_CRC8((const uint8_t *)g_k230_fbuf, 5);
                    if (crc_calc == g_k230_fbuf[5]) {
                        g_k230_err_x = (int16_t)((g_k230_fbuf[0] << 8) | g_k230_fbuf[1]);
                        g_k230_err_y = (int16_t)((g_k230_fbuf[2] << 8) | g_k230_fbuf[3]);
                        g_k230_flags = g_k230_fbuf[4];
                        g_k230_corner = (g_k230_flags & K230_FLAG_CORNER) ? 1 : 0;  /* [v5.7] 接近拐角→减速 */
                        g_k230_frame_ready = 1;
                        g_k230_frames++;
                    } else {
                        g_k230_crc_err++;   /* CRC 失败: 静默丢弃本帧 */
                    }
                } else if (g_k230_ftype == 1) {
                    /* 矩形帧: CRC 覆盖 CXH..ANGH ANGL FLAGS (11字节) */
                    uint8_t crc_calc = K230_CRC8((const uint8_t *)g_k230_fbuf, 11);
                    if (crc_calc == g_k230_rcrc) {
                        g_rect_dx = (int16_t)((g_k230_fbuf[0] << 8) | g_k230_fbuf[1]);
                        g_rect_dy = (int16_t)((g_k230_fbuf[2] << 8) | g_k230_fbuf[3]);
                        g_rect_w  = (uint16_t)((g_k230_fbuf[4] << 8) | g_k230_fbuf[5]);
                        g_rect_h  = (uint16_t)((g_k230_fbuf[6] << 8) | g_k230_fbuf[7]);
                        g_rect_ang = (int16_t)((g_k230_fbuf[8] << 8) | g_k230_fbuf[9]);
                        g_rect_valid = (g_k230_fbuf[10] & 0x01) ? 1 : 0;
                        g_rect_origin_valid = (g_k230_fbuf[10] & 0x02) ? 1 : 0;
                        g_rect_frame_ready = 1;
                    } else {
                        g_k230_crc_err++;
                    }
                } else if (g_k230_ftype == 2) {
                    /* 命令帧: 解析 CMD (buf[0]), 切换任务/解锁 (主循环消费) */
                    g_k230_cmd_pending = g_k230_fbuf[0];
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

