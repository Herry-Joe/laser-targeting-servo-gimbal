/**
 * @file    stm32f1xx_hal_conf.h
 * @brief   STM32F1xx HAL 库配置 — 精简版
 *
 * @note   本 Demo 只用到了 GPIO、TIM、UART、RCC，其他模块全部关闭。
 *         如果使用 CubeMX 生成的项目，此文件会被自动覆盖，无需手动修改。
 */

#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 模块开关 ========================== */
#define HAL_MODULE_ENABLED          /* HAL 核心 */

/* 本项目需要的模块 */
#define HAL_GPIO_MODULE_ENABLED     /* GPIO */
#define HAL_TIM_MODULE_ENABLED      /* 定时器 */
#define HAL_UART_MODULE_ENABLED     /* 串口 */
#define HAL_RCC_MODULE_ENABLED      /* 时钟 */
#define HAL_CORTEX_MODULE_ENABLED   /* NVIC/SysTick */
#define HAL_DMA_MODULE_ENABLED      /* DMA (UART 可能需要) */
#define HAL_FLASH_MODULE_ENABLED    /* Flash 延迟配置 */

/* 不需要的模块 — 全部注释掉以减小体积 */
/* #define HAL_ADC_MODULE_ENABLED */
/* #define HAL_CAN_MODULE_ENABLED */
/* #define HAL_I2C_MODULE_ENABLED */
/* #define HAL_SPI_MODULE_ENABLED */
/* #define HAL_PWR_MODULE_ENABLED */
/* #define HAL_RTC_MODULE_ENABLED */

/* ========================== 时钟配置 ========================== */
#define HSE_VALUE          8000000U
#define HSE_STARTUP_TIMEOUT 100U
#define HSI_VALUE          8000000U
#define LSI_VALUE           40000U
#define LSE_VALUE           32768U
#define LSE_STARTUP_TIMEOUT 5000U

/* ========================== HAL 全局配置 ========================== */
#define TICK_INT_PRIORITY    0x0FU   /* SysTick 优先级 (最低) */
#define USE_RTOS             0U
#define PREFETCH_ENABLE      1U
#define ASSERT_ENABLE        0U      /* 关闭 assert 以减小体积 */

/* ========================== 包含 HAL 头文件 ========================== */
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
