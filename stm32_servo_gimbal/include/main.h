/**
 * @file    main.h
 * @brief   STM32F103C8T6 舵机二维云台 Demo (TIM3 PWM + 滑环)
 * @note    主头文件 — 包含所有外设引脚定义和公共宏
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 头文件 ========================== */
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ========================== 系统时钟 ========================== */
/* HSE_VALUE / HSI_VALUE 由框架 stm32f1xx_hal_conf.h 定义，此处不重复 */
#define SYSCLK_FREQ       72000000UL  /* 72MHz 系统时钟 */
#define APB1_TIMER_CLK    72000000UL  /* APB1 定时器时钟 (x2) */
#define APB2_TIMER_CLK    72000000UL  /* APB2 定时器时钟 */

/* ========================== TB6612 控制引脚 ========================== */
/* ---- H 桥 A：控制 28BYJ-48 线圈 1 (Orange) 和 线圈 2 (Pink) ---- */
#define TB6612_AIN1_PORT       GPIOA
#define TB6612_AIN1_PIN        GPIO_PIN_4
#define TB6612_AIN2_PORT       GPIOA
#define TB6612_AIN2_PIN        GPIO_PIN_5

/* ---- H 桥 B：控制 28BYJ-48 线圈 3 (Yellow) 和 线圈 4 (Blue) ---- */
#define TB6612_BIN1_PORT       GPIOA
#define TB6612_BIN1_PIN        GPIO_PIN_6
#define TB6612_BIN2_PORT       GPIOA
#define TB6612_BIN2_PIN        GPIO_PIN_7

/* ---- STBY：高电平使能，低电平待机 ---- */
#define TB6612_STBY_PORT       GPIOB
#define TB6612_STBY_PIN        GPIO_PIN_0

/* ---- PWM 引脚 (可选，本例用 GPIO 输出高电平 = 全速) ---- */
#define TB6612_PWMA_PORT       GPIOA
#define TB6612_PWMA_PIN        GPIO_PIN_0
#define TB6612_PWMB_PORT       GPIOA
#define TB6612_PWMB_PIN        GPIO_PIN_1

/* ========================== 垂直电机 (Tilt) TB6612 引脚 ========================== */
#define M2_AIN1_PORT           GPIOB
#define M2_AIN1_PIN            GPIO_PIN_3
#define M2_AIN2_PORT           GPIOB
#define M2_AIN2_PIN            GPIO_PIN_4
#define M2_BIN1_PORT           GPIOB
#define M2_BIN1_PIN            GPIO_PIN_5
#define M2_BIN2_PORT           GPIOB
#define M2_BIN2_PIN            GPIO_PIN_6
#define M2_PWMA_PORT           GPIOB
#define M2_PWMA_PIN            GPIO_PIN_1
#define M2_PWMB_PORT           GPIOB
#define M2_PWMB_PIN            GPIO_PIN_7

/* ========================== 按键引脚 ========================== */
#define KEY_UP_PORT            GPIOB
#define KEY_UP_PIN             GPIO_PIN_12      /* 加速 */
#define KEY_DOWN_PORT          GPIOB
#define KEY_DOWN_PIN           GPIO_PIN_13      /* 减速 */
#define KEY_DIR_PORT           GPIOB
#define KEY_DIR_PIN            GPIO_PIN_14      /* 换向 */
#define KEY_MODE_PORT          GPIOB
#define KEY_MODE_PIN           GPIO_PIN_15      /* 模式切换 */

/* PA0/PA1: 用户加装独立按键 (上拉输入, 按下=低电平) */
#define KEY_PA0_PORT           GPIOA
#define KEY_PA0_PIN            GPIO_PIN_0       /* 功能键: 自动打靶开关 / 长按回中 */
#define KEY_PA1_PORT           GPIOA
#define KEY_PA1_PIN            GPIO_PIN_1       /* 循迹键: 矩形循迹开关 / 长按解锁命中 */

/* ========================== LED ========================== */
#define LED_PORT               GPIOC
#define LED_PIN                GPIO_PIN_13      /* 板载 LED (Blue Pill) */

/* ========================== UART 调试串口 ========================== */
#define DEBUG_USART            USART1
#define DEBUG_USART_BAUD       115200
#define DEBUG_TX_PORT          GPIOA
#define DEBUG_TX_PIN           GPIO_PIN_9
#define DEBUG_RX_PORT          GPIOA
#define DEBUG_RX_PIN           GPIO_PIN_10

/* ========================== 蓝牙串口 (JDY-31) ========================== */
#define BT_USART               USART2
#define BT_USART_BAUD          9600            /* JDY-31 默认波特率 */
#define BT_TX_PORT             GPIOA
#define BT_TX_PIN              GPIO_PIN_2      /* USART2 TX */
#define BT_RX_PORT             GPIOA
#define BT_RX_PIN              GPIO_PIN_3      /* USART2 RX */

/* ========================== K230 上位机串口 (USART3, PB10/PB11) ========================== */
#define K230_USART             USART3
#define K230_USART_BAUD        115200
#define K230_TX_PORT           GPIOB
#define K230_TX_PIN            GPIO_PIN_10     /* USART3 TX */
#define K230_RX_PORT           GPIOB
#define K230_RX_PIN            GPIO_PIN_11     /* USART3 RX */

/* ========================== K230 矩形识别帧 ========================== */
/* 矩形帧 K230 → STM32 (USART3): 0x3C 0x3A [10数据][FLAGS][CRC8] 0x01 0x01
 * 数据(大端): CXH CXL CYH CYL WH WL HH HL ANGH ANGL
 *   CX/CY = int16 矩形中心相对靶心偏移(px)
 *   W/H   = uint16 矩形宽高(px)
 *   ANG   = int16 倾斜角(0.1°)
 * FLAGS  = bit0 rect_valid, bit1 origin_valid(靶心有效→偏移可信)
 * CRC8 poly0x07 init0 覆盖 [10数据+FLAGS] 共 11 字节, 与 K230 端一致 */
#define K230_RECT_HEADER0   0x3C
#define K230_RECT_HEADER1   0x3A

/* ========================== OLED (SSD1306 I2C, 辅助串口调试) ========================== */
/* 注意: I2C1 默认引脚是 PB6/PB7, 但本工程中 PB6/PB7 已被 Tilt 电机占用,
 *       因此必须把 I2C1 重映射到 PB8/PB9 才能接 OLED。 */
#define OLED_I2C              I2C1
#define OLED_I2C_REMAP        1               /* 1=使用 PB8/PB9 重映射 I2C1 */
#define OLED_SCL_PORT         GPIOB
#define OLED_SCL_PIN          GPIO_PIN_8      /* I2C1 SCL (重映射) */
#define OLED_SDA_PORT         GPIOB
#define OLED_SDA_PIN          GPIO_PIN_9      /* I2C1 SDA (重映射) */
#define OLED_I2C_ADDR         0x3C            /* 7位地址 (SA0=GND) */
#define OLED_I2C_SPEED        100000          /* I2C 速率 100kHz (比 400kHz 更可靠,多数 OLED 模块无外部上拉也能工作) */

/* ========================== 步进电机参数 ========================== */
/* 28BYJ-48 减速步进电机 */
#define STEPPER_GEAR_RATIO     64U             /* 减速比：1:64 */
#define STEPPER_STRIDE_ANGLE   5.625f          /* 步距角：5.625°/64 = 0.08789° */
#define STEPPER_FULL_STEPS      4U              /* 全步进每圈步数 (电气) */
#define STEPPER_HALF_STEPS      8U              /* 半步进每圈步数 (电气) */
#define STEPPER_FULL_REV        2048U           /* 全步进输出轴一圈 = 64*32 = 2048 */
#define STEPPER_HALF_REV       4096U           /* 半步进输出轴一圈 = 64*64 = 4096 */
/* 舵机方案: step_delay_us 表示"指令刷新节拍"(1步=0.1°), 越小舵机转得越快.
 * 最小 800us (≈125°/s), 默认巡航 8000us, 最大 20000us (≈5°/s, 精调慢速). */
#define STEPPER_MIN_DELAY_US   800U            /* 最小步进节拍 0.8ms/步 ≈125°/s, 提速 */
#define STEPPER_MAX_DELAY_US   20000U          /* 最大步进节拍 20ms/步 ≈5°/s */
#define STEPPER_DEFAULT_DELAY  8000U           /* 默认步进节拍 (8ms/步) */

/* ========================== 舵机 (Servo) PWM 配置 ========================== */
/* 舵机 PWM 分两路独立定时器:
 *   Pan 舵机 : PB1 = TIM3_CH4 (50Hz)
 *   Tilt舵机 : PA11 = TIM1_CH4 (50Hz) */
#define SERVO_PAN_TIM           TIM3
#define SERVO_PAN_CHANNEL       TIM_CHANNEL_4   /* PB1 (TIM3_CH4) */
#define SERVO_PAN_PORT          GPIOB
#define SERVO_PAN_PIN           GPIO_PIN_1
#define SERVO_TILT_TIM          TIM1
#define SERVO_TILT_CHANNEL      TIM_CHANNEL_4   /* PA11 (TIM1_CH4) */
#define SERVO_TILT_PORT         GPIOA
#define SERVO_TILT_PIN          GPIO_PIN_11

/* 舵机角度映射 — S20F 二自由度云台
 * Pan (下层水平): S20F 20kg, 270° 行程 (0.5ms=0°, 2.5ms=270°)
 * Tilt(上层俯仰): S20F 20kg, 180° 行程 (0.5ms=0°, 2.5ms=180°)
 */
#define PAN_PULSE_MIN_US       500             /* Pan 0° */
#define PAN_PULSE_MAX_US      2500            /* Pan 270° */
#define PAN_CENTER_US         1500            /* Pan 中位 = 135° */
#define PAN_RANGE_DEG         270.0f
#define PAN_US_PER_DEG        ((PAN_PULSE_MAX_US - PAN_PULSE_MIN_US) / PAN_RANGE_DEG)   /* ≈7.407 */

#define TILT_PULSE_MIN_US      500             /* Tilt 0° */
#define TILT_PULSE_MAX_US     2500            /* Tilt 180° */
#define TILT_CENTER_US        1500            /* Tilt 中位 = 90° */
#define TILT_RANGE_DEG        180.0f
#define TILT_US_PER_DEG       ((TILT_PULSE_MAX_US - TILT_PULSE_MIN_US) / TILT_RANGE_DEG)  /* ≈11.111 */

/* 步进当量: 1 步 = 0.1° (虚拟步进, 兼容原位置闭环算法) */
#define SERVO_STEPS_PER_DEG    10
#define SERVO_STEPS_PER_REV    3600            /* 360° / 0.1° */

/* ========================== 宏函数 ========================== */
#define LED_ON()    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TOG()   HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

#define KEY_UP_READ()     HAL_GPIO_ReadPin(KEY_UP_PORT, KEY_UP_PIN)
#define KEY_DOWN_READ()   HAL_GPIO_ReadPin(KEY_DOWN_PORT, KEY_DOWN_PIN)
#define KEY_DIR_READ()    HAL_GPIO_ReadPin(KEY_DIR_PORT, KEY_DIR_PIN)
#define KEY_MODE_READ()   HAL_GPIO_ReadPin(KEY_MODE_PORT, KEY_MODE_PIN)
#define KEY_PA0_READ()    HAL_GPIO_ReadPin(KEY_PA0_PORT, KEY_PA0_PIN)
#define KEY_PA1_READ()    HAL_GPIO_ReadPin(KEY_PA1_PORT, KEY_PA1_PIN)

/* ========================== 公共函数声明 ========================== */
void SystemClock_Config(void);
void GPIO_Init(void);
void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);
void I2C1_Init(void);
void TIM2_Init(void);
void TIM1_Init(void);   /* Tilt 舵机 PWM 50Hz: PA11=TIM1_CH4 */
void TIM3_Init(void);   /* Pan 舵机 PWM 50Hz: PB1=TIM3_CH4 */
void TIM4_Init(void);   /* Tilt 步进节拍定时器 */
void Error_Handler(void);

/* ========================== 外设句柄 (extern) ========================== */
extern TIM_HandleTypeDef  htim2;     /* Pan 步进节拍定时器 */
extern TIM_HandleTypeDef  htim1;     /* Tilt 舵机 PWM 定时器 (TIM1, PA11) */
extern TIM_HandleTypeDef  htim3;     /* Pan 舵机 PWM 定时器 (TIM3, PB1) */
extern TIM_HandleTypeDef  htim4;     /* Tilt 步进节拍定时器 */
extern UART_HandleTypeDef huart1;    /* 调试串口 */
extern UART_HandleTypeDef huart2;    /* 蓝牙串口 */
extern UART_HandleTypeDef huart3;    /* K230 上位机串口 */
extern I2C_HandleTypeDef  hi2c1;     /* OLED I2C 总线 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
