/**
 * @file    bldc_gimbal.h
 * @brief   WHEELTEC F32C 无刷电机云台驱动 (USART3 硬件串口, 速度闭环)
 *
 * 协议 (官方 STM32F1 例程): 帧格式 0x7A [ID] [CMD] [DATA...] [BCC] 0x7B
 *   - BCC = 前面所有字节的异或 (XOR)
 *   - 波特率 115200, 8N1
 *   - 指令: 0x06=使能, 0x00=设置模式(0=速度闭环), 0x01=设置速度(2字节 RPM 有符号),
 *           0x0E=读取反馈
 *
 * 硬件连接 [v6.0]: MCU PB10(USART3 TX) → 电机驱动板 RXD ; PB11(USART3 RX) ← 电机 TXD
 *   (GND 共地)。原 PA4 软件串口(TIM3 位时钟)已废弃 — K230 迁到 USART2 后,
 *   USART3 空出来, 直接用硬件串口驱动无刷云台, 不再需要位时钟软串口。
 *
 *   电机ID: 1 = Pan(水平/Yaw), 2 = Tilt(俯仰/Pitch)
 */
#ifndef __BLDC_GIMBAL_H
#define __BLDC_GIMBAL_H

#include <stdint.h>
#include "stepper.h"   /* StepperMotor 完整定义 (取 ->cmd_rpm) */

/* 速度比例: 模型角速度(°/s = 100000/step_delay_us) 乘以本系数得到指令 RPM。
 * 默认 0.5, 若云台转动过慢/过快, 用串口命令 `rpm <值>` 实时调整, 或改这里重编。 */
extern float g_motor_rpm_scale;

void BLDCGimbal_Init(void);            /* 初始化 USART3(已由 USART3_Init 配置), 并使能/设速度模式(含 1.5s 唤醒) */
void BLDCGimbal_Task(StepperMotor *pan, StepperMotor *tilt);  /* 主循环调用(建议 ~50Hz), 发送两轴速度指令 */
void BLDCGimbal_SetSpeed(uint8_t id, int16_t rpm);  /* 设置某电机目标转速(RPM, 有符号) */
void BLDCGimbal_SetAcc(uint8_t id, uint16_t acc);   /* [v6.11] 设置加速度(转/s²), 对照官方 init 的 75 */
void BLDCGimbal_RunSelfTest(void);     /* [v6.8] 电机自检: 直接驱动 F32C 扫两轴, 绕开 K230, 用于定位"云台不动"属硬件域(A)还是链路域(B) */

/* [v6.10] 角度反馈(闭环控制用): F32C 支持 0x0E 读反馈, 回 9 字节帧
 *   0x7A + addr + type + [4字节 大端 int32, 0.1°] + BCC + 0x7B
 *   需在 USART3_Init 中把 PB11 配成 RX 并开 RXNE 中断, 接收字节喂 BLDCGimbal_RxByte()。 */
void BLDCGimbal_SetZero(void);         /* 设零位: 当前机械角设为零点(0x0A) + 多圈角清零(0x09), 使初始位置=0 */
void BLDCGimbal_RequestAngle(uint8_t id);  /* 请求指定轴总角度反馈(type=1) */
void BLDCGimbal_RxByte(uint8_t b);     /* USART3 RX 中断逐字节喂入, 解析 9 字节反馈帧 */
int32_t BLDCGimbal_GetAngle(uint8_t id);   /* 取某轴当前角度(0.1°, 有符号) */
uint8_t BLDCGimbal_AngleValid(void);   /* 两轴角度近期是否均更新过(用于闭环/回退判断) */
uint8_t BLDCGimbal_AngleValidId(uint8_t id);  /* 单轴角度是否有效(id=1/2) */

/* [v6.8] 手动点动覆盖: 串口 pan/tilt 命令设置, 在 BLDCGimbal_Task 中优先于 cmd_rpm 发送,
 * 用于不依赖 K230 直接验证 F32C 是否响应。g_jog_until 为过期时间戳(ms)。 */
extern int16_t  g_jog_pan_rpm;
extern int16_t  g_jog_tilt_rpm;
extern uint32_t g_jog_until;

#endif /* __BLDC_GIMBAL_H */
