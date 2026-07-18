/**
 * @file    oled_debug.h
 * @brief   云台调试信息 OLED 显示界面 — K230 自动打靶版
 * @note    8行布局:
 *   L0: == LASER TRACKING ==
 *   L1: MODE:AUTO SRC:YOLO
 *   L2: PAN:+12px CW RUN 1.4ms
 *   L3: TIL: -5px -- IDLE  --
 *   L4: PID:+720/s I:3.2
 *   L5: off:(+3,-2) INV:N
 *   L6: ERR(12,-5) HIT:N
 *   L7: UPT:34s FPS:8.5
 */
#ifndef __OLED_DEBUG_H
#define __OLED_DEBUG_H

#include "main.h"

/* ========================== 调试信息结构体 ========================== */
typedef struct {
    const char *mode;            /* 当前云台模式: AUTO/PAN/TILT/SWEEP... */
    char pan_line[16];           /* Pan: "PAN +12" */
    char tilt_line[16];          /* Tilt: "TIL -5" */
    char pid_line[22];           /* PID 输出: "P:1.23 T:0.45" */
    char cfg_line[24];           /* 状态: "SRC:YOLO H" */
    char time_line[22];          /* 运行时间/反转: "UP:34s pT" */

    float pan_frac;              /* Pan 误差条填充比例 0..1 */
    float tilt_frac;             /* Tilt 误差条填充比例 0..1 */

    char bt_line[16];            /* 蓝牙链路状态: "BT:ON "/"BT:LOST" */

    /* v3 新增: K230 调试信息自动填充标志 */
    uint8_t k230_mode;           /* 1 = 自动模式下显示 K230 信息 */
} OledDebugInfo;

/* ========================== 公共接口 ========================== */
uint8_t OLED_Debug_Init(void);      /* 返回 1=OLED 就绪, 0=未检测到 */
void OLED_Debug_Update(const OledDebugInfo *info);
uint8_t OLED_Debug_IsReady(void);

#endif /* __OLED_DEBUG_H */
