/**
 * @file    stepper.c
 * @brief   28BYJ-48 步进电机驱动实现 — ULN2003 四相达林顿管驱动板版本 (多实例)
 *
 * === 硬件连接说明 ==============================================
 *
 *  ULN2003 (4路)              28BYJ-48 (5线)         STM32F103C8T6
 *  ┌──────────┐             ┌──────────┐           ┌──────────────┐
 *  │ 5-12V ──────────────>   │ 红 (COM) │─── 5V     │              │
 *  │ GND  ──────────────────────────────  ───>    │ GND          │
 *  │ IN1   ──────────────────────────  橙 (Coil1)│              │ ──> ain1_pin
 *  │ IN2   ──────────────────────────  粉 (Coil2)│              │ ──> ain2_pin
 *  │ IN3   ──────────────────────────  黄 (Coil3)│              │ ──> bin1_pin
 *  │ IN4   ──────────────────────────  蓝 (Coil4)│              │ ──> bin2_pin
 *  └──────────┘
 *
 *  ULN2003: INx=HIGH → 输出拉低到 GND → 线圈通电 (COM=5V)
 *
 * === 步进序列 (ULN2003 直接驱动 4 相) ==============================
 *
 *  半步步进序列 (8拍)，bit[3:0] = (IN4, IN3, IN2, IN1):
 *   拍号 | 橙 A | 粉 B | 黄 C | 蓝 D | 编码
 *   ─────┼──────┼──────┼──────┼──────┼──────
 *    0   |  ON  |      |      |      |  0x01
 *    1   |  ON  |  ON  |      |      |  0x03
 *    2   |      |  ON  |      |      |  0x02
 *    3   |      |  ON  |  ON  |      |  0x06
 *    4   |      |      |  ON  |      |  0x04
 *    5   |      |      |  ON  |  ON  |  0x0C
 *    6   |      |      |      |  ON  |  0x08
 *    7   |  ON  |      |      |  ON  |  0x09
 *
 *  全步进序列 (4拍，2相)：取奇数拍 0x03, 0x06, 0x0C, 0x09
 *  波驱动序列 (4拍，1相)：取偶数拍 0x01, 0x02, 0x04, 0x08
 */

#include "stepper.h"
#include <stdlib.h>

/* ========================== 步进序列查找表 ========================== */

/**
 * @brief 半步步进序列 (8 拍) — ULN2003 4-bit 直接编码
 *
 * 采用 Arduino 社区通用 28BYJ-48+ULN2003 标准步进序列:
 *   IN1→A, IN2→B, IN3→C, IN4→D
 *   板上丝印: A=蓝(Coil4), B=粉(Coil2), C=黄(Coil3), D=橙(Coil1)
 *
 * bit[3:0] = (IN4, IN3, IN2, IN1)
 */
static const uint8_t half_step_seq[8] = {
    0x01,   /* IN1=A=蓝 */
    0x03,   /* A+B=蓝+粉 */
    0x02,   /* B=粉 */
    0x06,   /* B+C=粉+黄 */
    0x04,   /* C=黄 */
    0x0C,   /* C+D=黄+橙 */
    0x08,   /* D=橙 */
    0x09,   /* D+A=橙+蓝 */
};

/** @brief 全步进序列 (4拍, 2相) */
static const uint8_t full_step_seq[4] = {
    0x03,   /* A+B=蓝+粉 */
    0x06,   /* B+C=粉+黄 */
    0x0C,   /* C+D=黄+橙 */
    0x09,   /* D+A=橙+蓝 */
};

/** @brief 波驱动序列 (4拍, 1相) */
static const uint8_t wave_step_seq[4] = {
    0x01,   /* A=蓝 */
    0x02,   /* B=粉 */
    0x04,   /* C=黄 */
    0x08,   /* D=橙 */
};

/* ========================== 底层硬件操作 ========================== */

/**
 * @brief 直接输出 4-bit 步进码到 ULN2003 IN1~IN4
 * @param step_code bit[3:0]=(IN4,IN3,IN2,IN1)
 */
static inline void Stepper_OutputStep(StepperMotor *motor, uint8_t step_code)
{
    HAL_GPIO_WritePin(motor->pins.ain1_port, motor->pins.ain1_pin,
                      (step_code & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* IN1 → A → 蓝(CoilD) */
    HAL_GPIO_WritePin(motor->pins.ain2_port, motor->pins.ain2_pin,
                      (step_code & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* IN2 → B → 粉(CoilB) */
    HAL_GPIO_WritePin(motor->pins.bin1_port, motor->pins.bin1_pin,
                      (step_code & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* IN3 → C → 黄(CoilC) */
    HAL_GPIO_WritePin(motor->pins.bin2_port, motor->pins.bin2_pin,
                      (step_code & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* IN4 → D → 橙(CoilA) */
}

/** @brief 全部线圈断电 (ULN2003 输入全 0 → 全部浮空, 电机滑行) */
static inline void Stepper_Coast(StepperMotor *motor)
{
    Stepper_OutputStep(motor, 0x00);
}

/* ========================== 步进电机核心逻辑 ========================== */

static uint8_t Stepper_AdvanceOneStep(StepperMotor *motor)
{
    const uint8_t *seq;
    uint8_t max_steps;

    if (motor->mode == STEP_MODE_HALF) {
        seq = half_step_seq;
        max_steps = 8;
    } else if (motor->mode == STEP_MODE_WAVE) {
        seq = wave_step_seq;
        max_steps = 4;
    } else {
        seq = full_step_seq;
        max_steps = 4;
    }

    /* 更新步序索引 (CW 递增, CCW 递减) */
    if (motor->dir == DIR_CW) {
        motor->step_index = (motor->step_index + 1) % max_steps;
    } else {
        motor->step_index = (motor->step_index == 0)
                            ? (max_steps - 1)
                            : (motor->step_index - 1);
    }

    Stepper_OutputStep(motor, seq[motor->step_index]);

    /* 绝对位置累计 (CW 正向, CCW 负向) — 用于 Tilt 限位防"打到天上" */
    if (motor->dir == DIR_CW) {
        motor->pos_steps++;
    } else {
        motor->pos_steps--;
    }

    /* 目标步数递减 */
    if (motor->target_steps > 0) {
        motor->target_steps--;
        if (motor->target_steps == 0) {
            /* 扫描模式：到达限位后由 IRQ 处理自动换向，不进入空闲 */
            if (motor->work_mode == WORK_MODE_SWEEP) {
                /* 保持运行状态，IRQ 中 target_steps <= 0 触发换向 */
            } else {
                motor->state = STATE_IDLE;
                /* [修复] 旋转完成后恢复原始方向 */
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

void Stepper_Init(StepperMotor *motor, const StepperPins *pins,
                  TIM_HandleTypeDef *htim, const char *label)
{
    memset(motor, 0, sizeof(StepperMotor));

    motor->mode           = STEP_MODE_HALF;   /* 平滑: 半步进 (1.5相平均 ~300mA/轴) */
    motor->dir            = DIR_CW;
    motor->state          = STATE_IDLE;
    motor->work_mode      = WORK_MODE_MANUAL;
    motor->step_delay_us  = STEPPER_DEFAULT_DELAY;
    motor->angle_limit_steps = STEPPER_HALF_REV / 4;
    motor->htim           = htim;
    motor->soft_start     = 0;
    motor->soft_start_delay = 0;
    motor->pos_steps      = 0;   /* 上电位置记为绝对零点 */

    if (pins) {
        motor->pins = *pins;
    }
    if (label) {
        strncpy(motor->label, label, sizeof(motor->label) - 1);
        motor->label[sizeof(motor->label) - 1] = '\0';
    }

    /* 上电不输出励磁, 避免电机瞬间大电流把蓝牙模块电压拉垮 */
    /* 等收到串口命令时 Stepper_RunContinuously / StepBy 才会通电 */
    Stepper_Coast(motor);

    printf("\r\n%s 电机初始化完成\r\n", motor->label);
}

void Stepper_SetMode(StepperMotor *motor, StepMode mode)
{
    motor->mode = mode;
    /* [修复] 切换模式时复位步序索引，防止数组越界 */
    motor->step_index = 0;
    /* 输出新模式第一拍，保持线圈励磁 */
    Stepper_OutputStep(motor,
        (mode == STEP_MODE_HALF) ? half_step_seq[0] :
        (mode == STEP_MODE_WAVE) ? wave_step_seq[0] : full_step_seq[0]);
    printf("[%s] %s\r\n", motor->label,
           (mode == STEP_MODE_HALF) ? "半步步进" :
           (mode == STEP_MODE_WAVE) ? "波驱动(省电)" : "全步步进");
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
    printf("[%s] 位置归零\r\n", motor->label);
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
    uint32_t steps_per_rev;
    uint32_t steps;

    steps_per_rev = (motor->mode == STEP_MODE_HALF)
                    ? STEPPER_HALF_REV : STEPPER_FULL_REV;

    float abs_degrees = (degrees < 0.0f) ? -degrees : degrees;
    steps = (uint32_t)((abs_degrees / 360.0f) * (float)steps_per_rev);

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

    uint32_t steps_per_rev = (motor->mode == STEP_MODE_HALF)
                             ? STEPPER_HALF_REV : STEPPER_FULL_REV;

    motor->angle_limit_steps = (uint32_t)(
        ((float)limit_degrees / 360.0f) * (float)steps_per_rev
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
    /* 不调用 Stepper_Coast: 保持当前步序的线圈励磁, 输出保持力矩,
     * 云台不会因重力而下垂。若需自由滑行, 可调用 Stepper_Coast */
    printf("[%s] 停止 (保持力矩)\r\n", motor->label);
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
    const char *mode_str[]  = {"全步步进", "半步步进", "波驱动(省电)"};
    const char *dir_str[]   = {"顺时针(CW)", "逆时针(CCW)"};
    const char *state_str[] = {"空闲", "连续运行", "定步运行", "定角运行"};
    const char *work_str[]  = {"手动", "扫描", "串口", "位置闭环"};

    printf("\r\n========== %s 电机状态 ==========\r\n", motor->label);
    printf(" 步进模式 : %s\r\n",   mode_str[motor->mode]);
    printf(" 旋转方向 : %s\r\n",   dir_str[motor->dir]);
    printf(" 运行状态 : %s\r\n",   state_str[motor->state]);
    printf(" 工作模式 : %s\r\n",   work_str[motor->work_mode]);
    printf(" 当前速度 : %lu us/步\r\n", (unsigned long)motor->step_delay_us);
    printf(" 当前步序 : %d\r\n",   motor->step_index);
    printf(" 剩余步数 : %ld\r\n",  (long)motor->target_steps);
    printf("===============================\r\n\r\n");
}