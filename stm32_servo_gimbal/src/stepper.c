/**
 * @file    stepper.c
 * @brief   舵机 (Servo) 二维云台驱动实现 — STM32F103C8T6 TIM3 PWM 输出 (多实例)
 *
 * === 硬件连接说明 ==============================================
 *
 *  STM32F103C8T6                舵机 (270° 标准舵机)
 *  ┌──────────────┐            ┌──────────────┐
 *  │ PA6 (TIM3_CH1) ────────>  │ Pan  舵机 PWM │
 *  │ PA7 (TIM3_CH2) ────────>  │ Tilt 舵机 PWM │
 *  │ 5V ───────────────────>   │ VCC          │
 *  │ GND ───────────────────>  │ GND          │
 *  └──────────────┘            └──────────────┘
 *
 * === 设计要点 ==================================================
 *  本模块把"虚拟步进"模型 (pos_steps, 单位 0.1°) 复用到舵机上, 以最大限度
 *  复用原 K230 视觉闭环 / 位置闭环 PID 算法, 仅替换最底层驱动方式:
 *   - 原方案: GPIO 4 相步进序列 → 28BYJ-48 旋转
 *   - 本方案: 每个节拍定时器 (TIM2=Pan, TIM4=Tilt) 的 Update 中断里,
 *             pos_steps 按方向 ±1 (0.1°), 再映射成舵机脉宽写入 TIM3 CCR。
 *   - 舵机自身以机械响应速度逼近"指令位置", 因此云台实际运动速度由节拍
 *     周期 step_delay_us 决定 (1 步 = 0.1°, 周期越短转得越快)。
 */

#include "stepper.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>   /* fabsf */

/* 无刷云台速度比例(定义在 bldc_gimbal.c, 可被串口 rpm 命令改写) */
extern float g_motor_rpm_scale;

/* ========================== 底层执行机构输出(无刷电机) ========================== */

/**
 * @brief 把当前模型速度换算成无刷电机指令转速(RPM), 存入 motor->cmd_rpm
 *
 * 替代原舵机方案的"写 PWM 脉宽"。无刷电机(WHEELTEC F32C)走速度闭环模式:
 *   - 模型角速度 = 100000 / step_delay_us  (°/s, step_delay_us 即每 0.1° 的节拍周期)
 *   - 指令 RPM  = 模型角速度 * g_motor_rpm_scale, 方向由 motor->dir 决定
 *   - 空闲(state=IDLE)时转速为 0 → 电机自锁保持当前角度
 * 主循环 BLDCGimbal_Task() 读取 cmd_rpm 并组帧发给电机。
 */
static void Actuator_Update(StepperMotor *motor)
{
    int16_t rpm = 0;

    if (motor->state != STATE_IDLE) {
        /* 模型角速度 (°/s) */
        float degps = 100000.0f / (float)motor->step_delay_us;
        float rpmf  = degps * g_motor_rpm_scale;
        if (motor->dir == DIR_CCW) rpmf = -rpmf;     /* 方向含机械反转(已由上层 bake 进 dir) */
        if (rpmf >  32767.0f) rpmf =  32767.0f;
        if (rpmf < -32768.0f) rpmf = -32768.0f;
        rpm = (int16_t)rpmf;
    }
    motor->cmd_rpm = rpm;
}

/* ========================== 步进电机核心逻辑 ========================== */

static uint8_t Stepper_AdvanceOneStep(StepperMotor *motor)
{
    /* 绝对位置累计 (CW 正向, CCW 负向) */
    if (motor->dir == DIR_CW) {
        motor->pos_steps++;
    } else {
        motor->pos_steps--;
    }

    motor->step_index = (motor->step_index + 1) & 0x07;  /* 仅用于遥测/心跳 */

    /* 舵机物理行程保护: 触底即停 (使用该轴硬限位) */
    if (motor->pos_steps <= motor->pos_limit_min_hard) {
        motor->pos_steps = motor->pos_limit_min_hard;
        if (motor->work_mode == WORK_MODE_SWEEP) {
            motor->dir = DIR_CW;
        } else {
            motor->state = STATE_IDLE;
        }
    } else if (motor->pos_steps >= motor->pos_limit_max_hard) {
        motor->pos_steps = motor->pos_limit_max_hard;
        if (motor->work_mode == WORK_MODE_SWEEP) {
            motor->dir = DIR_CCW;
        } else {
            motor->state = STATE_IDLE;
        }
    }

    /* 写入舵机 PWM (指令位置更新) */
    Actuator_Update(motor);

    /* 目标步数递减 */
    if (motor->target_steps > 0) {
        motor->target_steps--;
        if (motor->target_steps == 0) {
            /* 扫描模式：到达限位后由 IRQ 处理自动换向，不进入空闲 */
            if (motor->work_mode == WORK_MODE_SWEEP) {
                /* 保持运行状态，IRQ 中 target_steps <= 0 触发换向 */
            } else {
                motor->state = STATE_IDLE;
                /* 旋转完成后恢复原始方向 */
                if (motor->restore_dir_flag) {
                    motor->dir = motor->saved_dir;
                    motor->restore_dir_flag = 0;
                }
            }
        }
    }

    return motor->step_index;
}

/* ========================== 公共接口实现 ========================== */

void Stepper_Init(StepperMotor *motor, TIM_HandleTypeDef *htim_step,
                  TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel,
                  int32_t pmin_us, int32_t pmax_us, int32_t center_us,
                  float us_per_deg,
                  int32_t pos_min_hard, int32_t pos_max_hard,
                  const char *label)
{
    memset(motor, 0, sizeof(StepperMotor));

    motor->mode           = STEP_MODE_HALF;   /* 占位 (舵机无步进模式概念) */
    motor->dir            = DIR_CW;
    motor->state          = STATE_IDLE;
    motor->work_mode      = WORK_MODE_MANUAL;
    motor->step_delay_us  = STEPPER_DEFAULT_DELAY;
    motor->angle_limit_steps = SERVO_STEPS_PER_REV / 4;  /* 默认扫描 ±90° */
    motor->htim           = htim_step;        /* 步进节拍定时器 (TIM2/TIM4) */
    motor->htim_pwm       = htim_pwm;         /* 舵机 PWM 定时器 (TIM1) */
    motor->pwm_channel    = pwm_channel;      /* TIM_CHANNEL_x */

    /* 每轴独立脉宽映射 */
    motor->pulse_min_us       = pmin_us;
    motor->pulse_max_us       = pmax_us;
    motor->center_us          = center_us;
    motor->us_per_deg         = us_per_deg;
    motor->pos_limit_min_hard = pos_min_hard;
    motor->pos_limit_max_hard = pos_max_hard;

    motor->soft_start     = 0;
    motor->soft_start_delay = 0;
    motor->pos_steps      = 0;   /* 上电位置记为绝对零点 (舵机中位) */

    if (label) {
        strncpy(motor->label, label, sizeof(motor->label) - 1);
        motor->label[sizeof(motor->label) - 1] = '\0';
    }

    /* 上电将舵机置于中位 (pos_steps=0 → center_us), 避免舵机上电乱摆 */
    Actuator_Update(motor);

    printf("\r\n%s 电机初始化完成 (虚拟步进模型, 无刷速度闭环执行)\r\n",
           motor->label);
}

void Stepper_SetMode(StepperMotor *motor, StepMode mode)
{
    motor->mode = mode;
    /* [无刷] 步进模式已无意义, 仅记录并打印 */
    printf("[%s] 无刷电机 (速度闭环, 无步进模式概念)\r\n", motor->label);
}

void Stepper_SetDirection(StepperMotor *motor, Direction dir)
{
    motor->dir = dir;
    printf("[%s] %s\r\n", motor->label,
           (dir == DIR_CW) ? "顺时针 (CW)" : "逆时针 (CCW)");
}

void Stepper_SetSpeed(StepperMotor *motor, uint32_t delay_us)
{
    if (delay_us < STEPPER_MIN_DELAY_US)  delay_us = STEPPER_MIN_DELAY_US;
    if (delay_us > STEPPER_MAX_DELAY_US)  delay_us = STEPPER_MAX_DELAY_US;

    motor->step_delay_us = delay_us;

    /* [修复] 更新定时器周期前先停止中断，防止 ISR 中读取不一致 */
    __HAL_TIM_DISABLE_IT(motor->htim, TIM_IT_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(motor->htim, delay_us - 1);
    __HAL_TIM_ENABLE_IT(motor->htim, TIM_IT_UPDATE);

    printf("[%s] %lu us/步\r\n", motor->label, (unsigned long)delay_us);
}

/* ========================== 带加速度限制的非阻塞调速 ========================== */

/**
 * @brief 设定目标速度, 但以"每帧最大变化量"为上限平滑爬升/下降
 *        (替代每帧直接 Stepper_SetSpeed 硬改周期, 后者会造成转速突跳/抖动)
 *
 * 实现要点:
 *   - TIM 的 AutoReload 已开启 ARPE (TIM_AUTORELOAD_PRELOAD_ENABLE),
 *     直接写 ARR 会在下一个 Update 事件才生效, 因此【无需关中断】,
 *     彻底消除 Stepper_SetSpeed 中 __HAL_TIM_DISABLE_IT 带来的控制抖动。
 *   - 每帧 speed 只允许变化 STEPPER_ACCEL_US_PER_FRAME us,
 *     使电机按固定斜率加减速, 响应更一致、更平稳。
 *   - 该函数由云台控制循环每帧调用; 不修改电机运行状态/方向。
 */
void Stepper_UpdateTargetSpeed(StepperMotor *motor, uint32_t target_delay_us)
{
    if (target_delay_us < STEPPER_MIN_DELAY_US) target_delay_us = STEPPER_MIN_DELAY_US;
    if (target_delay_us > STEPPER_MAX_DELAY_US) target_delay_us = STEPPER_MAX_DELAY_US;

    int32_t cur = (int32_t)motor->step_delay_us;
    int32_t tgt = (int32_t)target_delay_us;
    int32_t diff = tgt - cur;
    if (diff >  (int32_t)STEPPER_ACCEL_US_PER_FRAME) diff =  (int32_t)STEPPER_ACCEL_US_PER_FRAME;
    if (diff < -(int32_t)STEPPER_ACCEL_US_PER_FRAME) diff = -(int32_t)STEPPER_ACCEL_US_PER_FRAME;

    uint32_t new_delay = (uint32_t)(cur + diff);
    if (new_delay < STEPPER_MIN_DELAY_US) new_delay = STEPPER_MIN_DELAY_US;
    if (new_delay > STEPPER_MAX_DELAY_US) new_delay = STEPPER_MAX_DELAY_US;

    motor->step_delay_us = new_delay;
    __HAL_TIM_SET_AUTORELOAD(motor->htim, new_delay - 1);  /* ARPE: 下个更新周期生效 */
}

void Stepper_RunContinuously(StepperMotor *motor)
{
    /* [修复] 临界区保护，防止 main loop 和 ISR 竞态 */
    __disable_irq();

    /* 从空闲进入连续运行 → 启用软加速, 起步更平稳 */
    if (motor->state == STATE_IDLE) {
        motor->soft_start = 8;                      /* 8 步内完成加速 */
        motor->soft_start_delay = motor->step_delay_us * 3; /* 起步延迟 = 3x 目标 */
        if (motor->soft_start_delay > STEPPER_MAX_DELAY_US) {
            motor->soft_start_delay = STEPPER_MAX_DELAY_US;
        }
        /* 设置定时器为软启动起始速度 */
        __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->soft_start_delay - 1);
    }

    motor->state = STATE_RUNNING;
    motor->target_steps = 0;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    printf("[%s] 连续运转\r\n", motor->label);
}

void Stepper_MoveToPosition(StepperMotor *motor, int32_t target_pos)
{
    __disable_irq();

    /* 进入位置闭环模式 */
    motor->work_mode = WORK_MODE_POSCTRL;
    motor->target_pos = target_pos;
    motor->target_steps = 0;
    motor->state = STATE_RUNNING;

    /* 方向由目标位置决定 */
    int32_t err = target_pos - motor->pos_steps;
    if (err > 0) {
        motor->dir = DIR_CW;
    } else if (err < 0) {
        motor->dir = DIR_CCW;
    }

    /* 起步软启动 */
    if (motor->soft_start == 0) {
        motor->soft_start = 8;
        motor->soft_start_delay = motor->step_delay_us * 3;
        if (motor->soft_start_delay > STEPPER_MAX_DELAY_US) {
            motor->soft_start_delay = STEPPER_MAX_DELAY_US;
        }
        __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->soft_start_delay - 1);
    }

    __enable_irq();
    printf("[%s] 目标位置 %ld 步 (当前 %ld)\r\n",
           motor->label, (long)target_pos, (long)motor->pos_steps);
}

void Stepper_SetPositionLimits(StepperMotor *motor, int32_t min_steps, int32_t max_steps)
{
    __disable_irq();
    motor->pos_limit_min = min_steps;
    motor->pos_limit_max = max_steps;
    __enable_irq();
    printf("[%s] 位置限位 [%ld, %ld] 步\r\n",
           motor->label, (long)min_steps, (long)max_steps);
}

void Stepper_ZeroPosition(StepperMotor *motor)
{
    __disable_irq();
    motor->pos_steps = 0;
    motor->target_pos = 0;
    __enable_irq();
    /* 立即把舵机指令到中位 */
    Actuator_Update(motor);
    printf("[%s] 位置归零 (舵机回中位)\r\n", motor->label);
}

int32_t Stepper_GetTargetPosition(const StepperMotor *motor)
{
    return motor->target_pos;
}

void Stepper_StepBy(StepperMotor *motor, uint32_t steps)
{
    __disable_irq();
    motor->target_steps = (int32_t)steps;
    motor->state = STATE_STEPPING;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    printf("[%s] %lu 步\r\n", motor->label, (unsigned long)steps);
}

void Stepper_RotateDegrees(StepperMotor *motor, float degrees)
{
    uint32_t steps;

    /* [舵机] 固定 3600 步/圈 (0.1°/步) */
    steps = (uint32_t)((fabsf(degrees) / 360.0f) * (float)SERVO_STEPS_PER_REV);

    /* [修复] 保存当前方向，旋转完成后恢复 */
    __disable_irq();
    motor->saved_dir = motor->dir;
    motor->restore_dir_flag = 1;

    if (degrees < 0) {
        motor->dir = DIR_CCW;
    } else {
        motor->dir = DIR_CW;
    }

    motor->target_steps = (int32_t)steps;
    motor->state = STATE_TARGET;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();

    printf("[%s] 旋转 %.1f° → %lu 步 (%s)\r\n",
           motor->label, (double)degrees, (unsigned long)steps,
           (motor->dir == DIR_CW) ? "CW" : "CCW");
}

void Stepper_SweepEnable(StepperMotor *motor, uint32_t limit_degrees)
{
    if (limit_degrees > 360) limit_degrees = 360;
    if (limit_degrees == 0)  limit_degrees = 90;

    motor->angle_limit_steps = (uint32_t)(
        ((float)limit_degrees / 360.0f) * (float)SERVO_STEPS_PER_REV
    );

    motor->work_mode = WORK_MODE_SWEEP;
    motor->target_steps = (int32_t)motor->angle_limit_steps;
    motor->state = STATE_RUNNING;

    printf("[%s] 扫描 ±%lu° → %lu 步/方向\r\n",
           motor->label,
           (unsigned long)(limit_degrees / 2),
           (unsigned long)motor->angle_limit_steps);
}

void Stepper_Stop(StepperMotor *motor)
{
    __disable_irq();
    motor->state = STATE_IDLE;
    motor->target_steps = 0;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    /* 无刷: 发速度 0 指令 → 电机自锁保持当前角度 (云台不下垂) */
    printf("[%s] 停止 (电机保持位置)\r\n", motor->label);
}

/* ========================== 定时器中断处理 ========================== */

void Stepper_TIM_IRQHandler(StepperMotor *motor)
{
    if (motor->state == STATE_IDLE) {
        return;
    }

    /* 扫描模式：到达限位自动换向 */
    if (motor->work_mode == WORK_MODE_SWEEP) {
        if (motor->target_steps <= 0) {
            motor->dir = (motor->dir == DIR_CW) ? DIR_CCW : DIR_CW;
            motor->target_steps = (int32_t)motor->angle_limit_steps;
        }
    }

    /* 位置闭环模式：由目标位置决定方向和是否继续走 */
    if (motor->work_mode == WORK_MODE_POSCTRL) {
        int32_t err = motor->target_pos - motor->pos_steps;
        if (err == 0) {
            /* 到达目标：停止并保持力矩 */
            motor->state = STATE_IDLE;
            return;
        }

        /* 根据目标方向自动设定方向 */
        Direction want_dir = (err > 0) ? DIR_CW : DIR_CCW;

        /* 限位保护：下一步会超出机械行程则停转 */
        if (motor->pos_limit_min != 0 || motor->pos_limit_max != 0) {
            int32_t next_pos = motor->pos_steps + (want_dir == DIR_CW ? 1 : -1);
            if (next_pos < motor->pos_limit_min || next_pos > motor->pos_limit_max) {
                motor->state = STATE_IDLE;
                printf("\r\n[%s] 到达位置限位 %ld, 停止\r\n",
                       motor->label, (long)next_pos);
                return;
            }
        }

        motor->dir = want_dir;
    }

    Stepper_AdvanceOneStep(motor);

    /* 软件加速: 每一步逐渐提高速度 (降低延迟), 8 步后到达目标 */
    if (motor->soft_start > 0) {
        motor->soft_start--;
        uint32_t ramp_steps = 8;  /* 总加速步数 */
        uint32_t done = ramp_steps - motor->soft_start;  /* 已完成步数 */
        /* 线性插值: 从 soft_start_delay 逐步降至 step_delay_us */
        uint32_t target = motor->step_delay_us;
        uint32_t start = motor->soft_start_delay;
        if (done > 0) {
            uint32_t new_delay;
            if (start > target) {
                new_delay = start - (start - target) * done / ramp_steps;
            } else {
                new_delay = target;  /* 目标已经比起点快 */
            }
            if (new_delay < target) new_delay = target;
            __HAL_TIM_SET_AUTORELOAD(motor->htim, new_delay - 1);
        }
    }
}

/* ========================== 状态查询 ========================== */

uint8_t Stepper_IsRunning(const StepperMotor *motor)
{
    return (motor->state != STATE_IDLE);
}

MotorState Stepper_GetState(const StepperMotor *motor)
{
    return motor->state;
}

int32_t Stepper_GetPosition(const StepperMotor *motor)
{
    return motor->pos_steps;
}

void Stepper_PrintStatus(const StepperMotor *motor)
{
    const char *dir_str[]   = {"顺时针(CW)", "逆时针(CCW)"};
    const char *state_str[] = {"空闲", "连续运行", "定步运行", "定角运行"};
    const char *work_str[]  = {"手动", "扫描", "串口", "位置闭环"};

    printf("\r\n========== %s 电机状态 ==========\r\n", motor->label);
    printf(" 驱动方式 : 无刷电机速度闭环 (USART3 PB10 硬件串口 TX)\r\n");
    printf(" 旋转方向 : %s\r\n",   dir_str[motor->dir]);
    printf(" 运行状态 : %s\r\n",   state_str[motor->state]);
    printf(" 工作模式 : %s\r\n",   work_str[motor->work_mode]);
    printf(" 指令周期 : %lu us/步\r\n", (unsigned long)motor->step_delay_us);
    printf(" 当前位置 : %ld 步 (%.1f°)\r\n",
           (long)motor->pos_steps, (double)((float)motor->pos_steps / (float)SERVO_STEPS_PER_DEG));
    printf(" 剩余步数 : %ld\r\n",  (long)motor->target_steps);
    printf("===============================\r\n\r\n");
}
