/**
 * @file    ssd1306.h
 * @brief   SSD1306 (128x64) OLED 驱动 — I2C 接口 (STM32 HAL)
 * @note    适用于 0.96" / 1.3" 单色 OLED, I2C 地址 0x3C (SA0=0)
 *          自带 1024 字节显存缓冲, 调用 SSD1306_UpdateScreen() 刷新到屏幕
 */
#ifndef __SSD1306_H
#define __SSD1306_H

#include "main.h"
#include "stm32f1xx_hal.h"

/* ========================== 屏幕参数 ========================== */
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64
#define SSD1306_PAGES   (SSD1306_HEIGHT / 8)   /* 64/8 = 8 页 */

/* ========================== 显存缓冲 (全局) ========================== */
extern uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_PAGES];

/* ========================== 公共接口 ========================== */
/**
 * @brief  初始化 SSD1306 (软件 I2C, PB8=SCL, PB9=SDA)
 * @retval 1=初始化成功(设备存在)  0=未检测到 OLED (后续操作自动跳过)
 */
uint8_t SSD1306_Init(void);

/** @brief OLED 是否就绪 (设备存在) */
uint8_t SSD1306_IsReady(void);

/** @brief 清空显存 */
void SSD1306_Clear(void);

/** @brief 将显存内容刷新到屏幕 */
void SSD1306_UpdateScreen(void);

/** @brief 画点 (color: 1=亮 0=灭) */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/** @brief 画一个 5x7 ASCII 字符 */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color);

/** @brief 画字符串 (字符宽 6px, 仅支持 ASCII 0x20~0x7E) */
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t color);

/** @brief 画水平线 (含端点) */
void SSD1306_DrawHLine(uint8_t x0, uint8_t x1, uint8_t y, uint8_t color);

/** @brief 填充矩形 (实心/镂空由 color 决定) */
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

#endif /* __SSD1306_H */
