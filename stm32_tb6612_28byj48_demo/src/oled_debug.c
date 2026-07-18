/**
 * @file    oled_debug.c
 * @brief   云台调试信息 OLED 显示界面实现 — K230 自动打靶精简版
 *
 * 屏幕布局 (128x64, 5 行 + 误差条, 页对齐不拥挤):
 *   L0:  " K230 TARGETING "      (标题 + 分隔线)
 *   L1:  PAN +12   [====    ]    (误差条右对齐)
 *   L2:  TIL -5    [==      ]
 *   L3:  P:1.23 T:0.45           (PID 输出)
 *   L4:  SRC:YOLO H              (靶源/命中)
 *   L5:  UP:34s pT               (运行时间/反转)
 */
#include "oled_debug.h"
#include "ssd1306.h"

uint8_t OLED_Debug_Init(void)
{
    return SSD1306_Init();
}

uint8_t OLED_Debug_IsReady(void)
{
    return SSD1306_IsReady();
}

/* 局部: 误差条 (外框 + 进度填充) */
static void OLED_DrawBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, float frac)
{
    if (frac < 0.0f) frac = 0.0f;
    else if (frac > 1.0f) frac = 1.0f;
    uint8_t fw = (uint8_t)(frac * (w - 2) + 0.5f);
    SSD1306_FillRect(x, y, w, h, 1);                  /* 外框(实心) */
    SSD1306_FillRect(x + 1, y + 1, w - 2, h - 2, 0);  /* 镂空内部 */
    if (fw > 0)
        SSD1306_FillRect(x + 1, y + 1, fw, h - 2, 1); /* 进度填充 */
}

void OLED_Debug_Update(const OledDebugInfo *info)
{
    if (!SSD1306_IsReady() || !info) return;

    SSD1306_Clear();

    /* 标题 + 分隔线, 留出呼吸空间 */
    SSD1306_DrawString(0, 0, " K230 TARGETING ", 1);
    SSD1306_DrawHLine(0, SSD1306_WIDTH - 1, 7, 1);

    /* PAN 行 (y=8, 页1) + 右侧误差条 */
    SSD1306_DrawString(0, 8, info->pan_line, 1);
    OLED_DrawBar(64, 8, 62, 7, info->pan_frac);

    /* TILT 行 (y=16, 页2) + 右侧误差条 */
    SSD1306_DrawString(0, 16, info->tilt_line, 1);
    OLED_DrawBar(64, 16, 62, 7, info->tilt_frac);

    /* PID / 状态 / 时间 / 蓝牙 (页3~6, 下方留白) */
    SSD1306_DrawString(0, 24, info->pid_line, 1);
    SSD1306_DrawString(0, 32, info->cfg_line, 1);
    SSD1306_DrawString(0, 40, info->time_line, 1);
    SSD1306_DrawString(0, 48, info->bt_line, 1);

    SSD1306_UpdateScreen();
}
