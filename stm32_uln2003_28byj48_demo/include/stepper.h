/**
 * @file    stepper.h
 * @brief   28BYJ-48 步进电机驱动模块 (TB6612 版本) — 支持多实例
 * @note    提供全步进、半步进两种模式的速度/方向/角度控制接口
 *          支持多电机实例，用于云台双轴控制
 */

#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"

/* ========================== 速度/加速度参数 ========================== */
/* 每帧允许的速度变化上限(us) — 加速度限制, 保证加减速斜率一致、不突跳。
 * 参考 Simple_Tracking_Device 的加减速/速度配置思路: 控制器侧给出目标速度,
 * 由驱动层按固定斜率爬升, 避免 PID 输出抖动直接耦合到电机转速。 */
#ifndef STEPPER_ACCEL_US_PER_FRAME
#define STEPPER_ACCEL_US_PER_FRAME  3000U
#endif

/* ========================== 枚举定义 ========================== */

/** @brief 步进模式 */
typedef enum {
    STEP_MODE_FULL = 0,     /* 全步进：4 拍/周期，力矩大 (2相励磁 ~400mA/轴) */
    STEP_MODE_HALF = 1,     /* 半步进：8 拍/周期，运行平滑 (1.5相平均 ~300mA/轴) */
    STEP_MODE_WAVE = 2,     /* 波驱动：4 拍/周期，最省电 (1相励磁 ~200mA/轴, 默认) */
} StepMode;

/** @brief 旋转方向 */
typedef enum {
    DIR_CW  = 0,            /* 顺时针 (俯视) */
    DIR_CCW = 1,            /* 逆时针 (俯视) */
} Direction;

/** @brief 电机运行状态 */
typedef enum {
    STATE_IDLE      = 0,    /* 空闲 */
    STATE_RUNNING   = 1,    /* 连续运行 */
    STATE_STEPPING  = 2,    /* 定步数运行中 */
    STATE_TARGET    = 3,    /* 定角度运行中 */
} MotorState;

/** @brief 工作模式 */
typedef enum {
    WORK_MODE_MANUAL   = 0, /* 手动模式：按键控制方向 */
    WORK_MODE_SWEEP    = 1, /* 扫描模式：自动来回摆动 */
    WORK_MODE_SERIAL   = 2, /* 串口模式：上位机控制 */
    WORK_MODE_POSCTRL  = 3, /* 位置闭环模式：串口给定目标位置，PID 调速 */
} WorkMode;

/* ========================== 结构体定义 ========================== */

/** @brief 步进电机引脚配置 */
typedef struct {
    GPIO_TypeDef *ain1_port; uint16_t ain1_pin;
    GPIO_TypeDef *ain2_port; uint16_t ain2_pin;
    GPIO_TypeDef *bin1_port; uint16_t bin1_pin;
    GPIO_TypeDef *bin2_port; uint16_t bin2_pin;
} StepperPins;

/** @brief 步进电机控制结构体 */
typedef struct {
    StepMode    mode;               /* 当前步进模式 */
    Direction   dir;                /* 当前方向 */
    Direction   saved_dir;          /* 方向备份 (旋转后恢复用) */
    uint8_t     restore_dir_flag;   /* 旋转完成后恢复方向 */
    MotorState  state;              /* 运行状态 */
    WorkMode    work_mode;          /* 工作模式 */
    uint8_t     step_index;         /* 当前步序索引 (0~7) */
    uint32_t    step_delay_us;      /* 步间延时 (微秒)，越小越快 */
    int32_t     target_steps;       /* 目标步数（剩余） */
    uint32_t    angle_limit_steps;  /* 扫描角度限制步数 */
    StepperPins pins;               /* 引脚映射 */
    TIM_HandleTypeDef *htim;        /* 关联的定时器句柄 */
    char        label[8];           /* 标签: "Pan"/"Tilt" */
    /* ---- 软件加速 ---- */
    uint8_t     soft_start;         /* >0 = 软启动剩余步数 */
    uint32_t    soft_start_delay;   /* 软启动起始延迟 (比目标慢) */
    int32_t     pos_steps;          /* 绝对位置 (相对上电零点, CW 为正) — 用于限位 */
    int32_t     target_pos;         /* 位置闭环目标位置 (步) */
    int32_t     pos_limit_min;      /* 位置下限 (步, 0 表示不限制) */
    int32_t     pos_limit_max;      /* 位置上限 (步, 0 表示不限制) */
} StepperMotor;

/* ========================== 公共接口 ========================== */

/* 初始化 */
void Stepper_Init(StepperMotor *motor, const StepperPins *pins,
                  TIM_HandleTypeDef *htim, const char *label);

/* 模式设置 */
void Stepper_SetMode(StepperMotor *motor, StepMode mode);
void Stepper_SetDirection(StepperMotor *motor, Direction dir);
void Stepper_SetSpeed(StepperMotor *motor, uint32_t delay_us);
void Stepper_UpdateTargetSpeed(StepperMotor *motor, uint32_t target_delay_us); /* 带加速度限制的非阻塞调速(每帧调用) */

/* 运动控制 */
void Stepper_RunContinuously(StepperMotor *motor);                  /* 连续运转 */
void Stepper_StepBy(StepperMotor *motor, uint32_t steps);           /* 运行指定步数 */
void Stepper_RotateDegrees(StepperMotor *motor, float degrees);     /* 旋转指定角度 */
void Stepper_SweepEnable(StepperMotor *motor, uint32_t limit_degrees); /* 启用扫描模式 */
void Stepper_MoveToPosition(StepperMotor *motor, int32_t target_pos);/* 位置闭环：设定目标位置 (步) */
void Stepper_SetPositionLimits(StepperMotor *motor, int32_t min_steps, int32_t max_steps); /* 设置位置限位 */
void Stepper_ZeroPosition(StepperMotor *motor);                     /* 当前位置设为零点 */
void Stepper_Stop(StepperMotor *motor);                             /* 停止 */

/* 查询位置控制目标 */
int32_t Stepper_GetTargetPosition(const StepperMotor *motor);

/* 定时器中断回调 — 在 TIMx_IRQHandler 中调用 */
void Stepper_TIM_IRQHandler(StepperMotor *motor);

/* 获取状态 */
void Stepper_PrintStatus(const StepperMotor *motor);
uint8_t Stepper_IsRunning(const StepperMotor *motor);
MotorState Stepper_GetState(const StepperMotor *motor);
int32_t Stepper_GetPosition(const StepperMotor *motor);   /* 绝对位置(步) CW为正 */

#endif /* __STEPPER_H */
