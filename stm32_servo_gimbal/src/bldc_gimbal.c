/**
 * @file    bldc_gimbal.c
 * @brief   WHEELTEC F32C 无刷电机云台驱动 — USART3 硬件串口 + 速度闭环协议
 *
 * 详见 bldc_gimbal.h。本文件实现:
 *   1) 协议组帧: 使能 / 模式 / 速度 (0x7A [ID][CMD][DATA...][BCC] 0x7B)
 *   2) 帧发送: 统一用 HAL_UART_Transmit(阻塞, 自带超时与错误恢复)。
 *              [v6.5] 弃用旧版 HAL_UART_Transmit_IT: 其依赖 TC 中断把 gState 恢复 READY,
 *              一旦 TX 中断偶发未触发, gState 永久卡 BUSY → 后续帧全被跳过 → 云台"断联"。
 *              阻塞发送必然结束并复位 gState, 失败则清错误标志强制解锁, 永不死锁。
 *   3) 初始化序列: 等待 1.5s 电机启动 → 发 0x00 唤醒(激活 F32C 串口接收) → 使能两轴(×3) → 设速度模式
 *   4) 心跳保活: 周期性重发 使能+速度模式, 防 F32C 偶发掉状态后整机失联。
 *   5) [v6.6] 指令间隔: F32C 手册要求多条指令间 >=1ms(驱动内部 DMA 空闲中断拼帧),
 *      连发会被丢帧 → 使能/速度失效。所有发送统一保证 >=2ms 间隔; 唤醒字节 0x00 必须在
 *      电机就绪(1.5s 后)才发, 否则被忽略 → 整机"无响应"。以上均为对照 WHEELTEC 官方驱动核对。
 *
 * [v6.0] 改走 USART3 硬件串口(PB10 TX → 电机 RXD), 取代原 PA4 软件串口 + TIM3 位时钟。
 *   原因为: K230 从 USART3 迁到 USART2 后, USART3 空出, 直接用硬件 UART 更稳更省。
 *
 * 速度指令来源: 主循环调用 BLDCGimbal_Task(), 把 StepperMotor.cmd_rpm(由 stepper.c
 *   依据当前模型速度算出) 发给对应电机。电机在速度模式(0)下积分转速即得到位置,
 *   停发/发 0 即自锁保持当前角度。
 */
#include "bldc_gimbal.h"
#include "main.h"      /* huart3 (extern), 无刷云台串口句柄 */
#include "stepper.h"   /* StepperMotor 完整定义, 取 ->cmd_rpm */
#include <string.h>

/* [v6.8] 手动点动覆盖(由 main.c 串口 pan/tilt 命令写入) */
extern int16_t  g_jog_pan_rpm;
extern int16_t  g_jog_tilt_rpm;
extern uint32_t g_jog_until;

/* 前向声明(定义见下方"底层发送"): 反馈组帧函数需复用阻塞发送, 故提前声明 */
static int Gimbal_UART_Send(const uint8_t *f, uint8_t len);

/* ========================== [v6.10] 角度反馈(闭环控制) ==========================
 * F32C 支持 0x0E 读反馈, 回 9 字节帧:
 *   0x7A + addr + type + [4字节 大端 int32, 单位 0.1°] + BCC + 0x7B
 *   BCC = 前 7 字节(0x7A..数据末)异或.
 * 需在 USART3_Init 把 PB11 配成 RX 并开 RXNE 中断, 逐字节喂 BLDCGimbal_RxByte()。
 * 角度单位统一为 0.1° (与 F32C total_angle 一致, 与步进模型 pos_steps 同单位),
 * 坐标系: Pan=轴1, Tilt=轴2; 正角度 = 正向(与 +cmd_rpm 同向). */
#define F32C_FB_HEAD         0x7A
#define F32C_FB_TAIL         0x7B
#define F32C_CMD_FEEDBACK    0x0E
#define F32C_FB_TOTAL_ANGLE  0x01    /* type=1: 多圈总角度 */
#define F32C_CMD_SET_ZERO    0x0A    /* 设当前机械角为 0 点 */
#define F32C_CMD_CLEAR_ANGLE 0x09    /* 多圈总角度清零 */

static int32_t  g_fb_angle[2] = {0, 0};   /* [0]=Pan(id1) [1]=Tilt(id2), 单位 0.1° 有符号 */
static uint8_t  g_fb_valid[2] = {0, 0};
static uint32_t g_fb_tick[2]  = {0, 0};

/* 9 字节反馈帧解析状态机 */
typedef enum {
    FB_SOF = 0, FB_ADDR, FB_TYPE, FB_V0, FB_V1, FB_V2, FB_V3, FB_BCC, FB_EOF
} fb_state_t;
static fb_state_t fb_state = FB_SOF;
static uint8_t fb_addr = 0, fb_type = 0, fb_v[4] = {0,0,0,0}, fb_bcc = 0;

void BLDCGimbal_RxByte(uint8_t b)
{
    switch (fb_state) {
    case FB_SOF:
        if (b == F32C_FB_HEAD) { fb_addr = 0; fb_type = 0; fb_v[0]=fb_v[1]=fb_v[2]=fb_v[3]=0; fb_state = FB_ADDR; }
        break;
    case FB_ADDR:  fb_addr = b; fb_state = FB_TYPE; break;
    case FB_TYPE:  fb_type = b; fb_state = FB_V0;   break;
    case FB_V0:    fb_v[0] = b;  fb_state = FB_V1;   break;
    case FB_V1:    fb_v[1] = b;  fb_state = FB_V2;   break;
    case FB_V2:    fb_v[2] = b;  fb_state = FB_V3;   break;
    case FB_V3:    fb_v[3] = b;  fb_state = FB_BCC;  break;
    case FB_BCC:
        fb_bcc = b;
        {
            uint8_t x = (uint8_t)(F32C_FB_HEAD ^ fb_addr ^ fb_type ^ fb_v[0] ^ fb_v[1] ^ fb_v[2] ^ fb_v[3]);
            fb_state = (x == fb_bcc) ? FB_EOF : FB_SOF;   /* BCC 错 → 重新同步 */
        }
        break;
    case FB_EOF:
        if (b == F32C_FB_TAIL) {
            if (fb_addr >= 1 && fb_addr <= 2) {
                int32_t v = ((int32_t)fb_v[0] << 24) | ((int32_t)fb_v[1] << 16) |
                            ((int32_t)fb_v[2] << 8)  |  (int32_t)fb_v[3];
                g_fb_angle[fb_addr - 1] = v;   /* 0.1° 有符号 */
                g_fb_valid[fb_addr - 1] = 1;
                g_fb_tick[fb_addr - 1]  = HAL_GetTick();
            }
        }
        /* 无论是否接受, 用本字节尝试重新同步(可能紧跟新帧头) */
        fb_state = (b == F32C_FB_HEAD) ? FB_ADDR : FB_SOF;
        break;
    default:
        fb_state = FB_SOF;
        break;
    }
}

void BLDCGimbal_RequestAngle(uint8_t id)
{
    if (id < 1 || id > 2) return;
    /* [v6.11] 对照 WHEELTEC 官方固件(motor.c BLDC_ReqFeedback): 0x0E 读反馈只带 1 个数据字节(type),
     *   帧长 = 6: 0x7A + addr + 0x0E + type + BCC + 0x7B。v6.10 误多发了一个 0x00 → BCC 错配、
     *   F32C 拒收 → 永远收不到反馈(闭环形同虚设)。此处修正为 6 字节。 */
    uint8_t f[6];
    f[0] = F32C_FB_HEAD;
    f[1] = id;
    f[2] = F32C_CMD_FEEDBACK;
    f[3] = F32C_FB_TOTAL_ANGLE;          /* type = 1 (多圈总角度, 0.1°) */
    f[4] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3]);   /* BCC: 仅 4 载荷字节 */
    f[5] = F32C_FB_TAIL;
    Gimbal_UART_Send(f, 6);
}

int32_t BLDCGimbal_GetAngle(uint8_t id)
{
    if (id < 1 || id > 2) return 0;
    return g_fb_angle[id - 1];          /* 0.1° 有符号 */
}

uint8_t BLDCGimbal_AngleValid(void)
{
    return (g_fb_valid[0] && g_fb_valid[1]) ? 1 : 0;
}

uint8_t BLDCGimbal_AngleValidId(uint8_t id)
{
    if (id < 1 || id > 2) return 0;
    return g_fb_valid[id - 1];
}

/* 设零位: 当前机械角设为零点(0x0A) + 多圈总角清零(0x09), 使初始位置=0 */
void BLDCGimbal_SetZero(void)
{
    uint8_t f[5];
    /* 帧: 0x7A + addr + cmd + BCC + 0x7B (无数据字节, 与 ENABLE 同构) */
    for (uint8_t id = 1; id <= 2; id++) {
        f[0] = F32C_FB_HEAD; f[1] = id; f[2] = F32C_CMD_SET_ZERO;
        f[3] = (uint8_t)(f[0] ^ f[1] ^ f[2]); f[4] = F32C_FB_TAIL;
        Gimbal_UART_Send(f, 5);
        f[1] = id; f[2] = F32C_CMD_CLEAR_ANGLE;
        f[3] = (uint8_t)(f[0] ^ f[1] ^ f[2]); f[4] = F32C_FB_TAIL;
        Gimbal_UART_Send(f, 5);
    }
    printf("[BLDC] 两轴设零完成 (SET_ZERO 0x0A + CLEAR_ANGLE 0x09), 初始位置=0\r\n");
}

/* ========================== 协议组帧参数 ========================== */
/* 帧长: 速度/模式指令 7 字节 (0x7A ID CMD D0 D1 BCC 0x7B); 使能指令 5 字节 */
#define FRAME_MAX_LEN   7

/* 心跳间隔: 周期性重发 使能+速度模式, 防 F32C 偶发掉状态后失联 */
#define BLDC_HEARTBEAT_MS  5000

/* 指令最小间隔: F32C 手册要求多条指令间 >=1ms, 留余量取 2ms, 否则丢帧(使能/速度失效) */
#define BLDC_CMD_INTERVAL_MS  2

float g_motor_rpm_scale = MOTOR_RPM_PER_DEGPS;   /* 速度比例(可被串口 rpm 命令改写) */

/* 上一次发帧的 tick, 用于保证指令间隔(见 Gimbal_UART_Send) */
static uint32_t g_last_send_tick = 0;

/* ========================== 底层发送 ========================== */

/* 阻塞发送一帧(初始化与主循环通用): 用 HAL_UART_Transmit 阻塞发送, 自带超时,
 * 成功必然复位 gState; 失败(超时/错误)则清 ORE/NE/FE/PE 错误标志并强制解锁 gState,
 * 下次重试自愈。彻底消除旧版 IT 发送在 TC 中断偶发未触发时 gState 永久卡 BUSY、
 * 导致云台"断联"的致命问题。返回 0=成功, -1=失败(已尝试恢复)。 */
static int Gimbal_UART_Send(const uint8_t *f, uint8_t len)
{
    if (len > FRAME_MAX_LEN) len = FRAME_MAX_LEN;
    /* [v6.6] F32C 手册要求指令间隔 >=1ms(驱动内部 DMA 空闲中断逐字节拼帧),
     * 连续发送会被丢帧 → 使能/速度指令失效。统一在此保证 >=2ms 间隔。 */
    uint32_t now = HAL_GetTick();
    int32_t delta = (int32_t)(now - g_last_send_tick);
    if (delta < (int32_t)BLDC_CMD_INTERVAL_MS) {
        HAL_Delay((uint32_t)(BLDC_CMD_INTERVAL_MS - delta));
    }
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart3, (uint8_t *)f, len, 10);
    g_last_send_tick = HAL_GetTick();
    if (st != HAL_OK) {
        /* F1 HAL(UART V1) 用 __HAL_UART_CLEAR_FLAG + UART_FLAG_* 清错误标志;
         * 同时强制解除 gState 锁死, 保证下次发送自愈 */
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_NE |
                                          UART_FLAG_FE  | UART_FLAG_PE);
        if (huart3.gState != HAL_UART_STATE_READY) {
            huart3.gState = HAL_UART_STATE_READY;   /* 强制解除 BUSY 锁死 */
        }
        return -1;
    }
    return 0;
}

/* 阻塞发送一字节(仅初始化激活字节用, 保证时序) */
static void Gimbal_UART_SendByte(uint8_t b)
{
    HAL_UART_Transmit(&huart3, &b, 1, 100);
}

/* ========================== 协议组帧 ========================== */

/* 速度指令: 0x7A ID 0x01 [RPM_H] [RPM_L] BCC 0x7B (7 字节) */
void BLDCGimbal_SetSpeed(uint8_t id, int16_t rpm)
{
    uint8_t f[7];
    int16_t r = (int16_t)RPM_CLAMP(rpm);
    f[0] = 0x7A;
    f[1] = id;
    f[2] = 0x01;                          /* CMD: 设置速度 */
    f[3] = (uint8_t)((int16_t)r >> 8);
    f[4] = (uint8_t)((int16_t)r & 0xFF);
    f[5] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3] ^ f[4]);   /* BCC */
    f[6] = 0x7B;
    Gimbal_UART_Send(f, 7);
}

/* 通用两数据指令(用于模式设置): 0x7A ID CMD d0 d1 BCC 0x7B (7 字节) */
static void BLDCGimbal_SendCmd(uint8_t id, uint8_t cmd, uint8_t d0, uint8_t d1)
{
    uint8_t f[7];
    f[0] = 0x7A; f[1] = id; f[2] = cmd; f[3] = d0; f[4] = d1;
    f[5] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3] ^ f[4]);
    f[6] = 0x7B;
    Gimbal_UART_Send(f, 7);
}

/* 使能指令(5 字节): 0x7A ID 0x06 BCC 0x7B */
static void BLDCGimbal_SendEnable(uint8_t id)
{
    uint8_t f[5];
    f[0] = 0x7A; f[1] = id; f[2] = 0x06;
    f[3] = (uint8_t)(f[0] ^ f[1] ^ f[2]);
    f[4] = 0x7B;
    Gimbal_UART_Send(f, 5);
}

/* 失能指令(5 字节): 0x7A ID 0x05 BCC 0x7B (与 ENABLE 同构) */
static void BLDCGimbal_SendDisable(uint8_t id)
{
    uint8_t f[5];
    f[0] = 0x7A; f[1] = id; f[2] = 0x05;
    f[3] = (uint8_t)(f[0] ^ f[1] ^ f[2]);
    f[4] = 0x7B;
    Gimbal_UART_Send(f, 5);
}

/* [v6.11] 设置加速度(单位: 转/s²)。对照 WHEELTEC 官方固件(motor.c), 其在 init 中调用
 *   BLDC_SetAcc(addr, 75) 把两轴加速度设为 75 转/s²。不显式设加速度时电机用出厂默认值,
 *   往往过小 → 速度指令突变时电机跟不上 → 闭环阻尼滞后、过冲。帧: 0x7A ID 0x07 [HI][LO] BCC 0x7B。 */
void BLDCGimbal_SetAcc(uint8_t id, uint16_t acc)
{
    uint8_t f[7];
    f[0] = 0x7A; f[1] = id; f[2] = 0x07;            /* CMD_SET_ACCEL */
    f[3] = (uint8_t)((uint16_t)acc >> 8);
    f[4] = (uint8_t)((uint16_t)acc & 0xFF);
    f[5] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3] ^ f[4]);
    f[6] = 0x7B;
    Gimbal_UART_Send(f, 7);
}

/* [v6.12] 0x00 唤醒字节开关。
 * 官方 WHEELTEC 参考固件(MiniBalance.c) 的 init 中从不发 0x00。
 * 本硬件先前(用户实测)曾需此字节"先于 1.5s 延时发出"才能使电机响应;
 * 但 v6.12 已按官方序列全面重写, 若仍不动 → 尝试把此值改为 0(去掉 0x00)。 */
#define BLDC_WAKEUP_ENABLE  1

/* ========================== 初始化 ========================== */
void BLDCGimbal_Init(void)
{
    /* [v6.12] 全面重写 init, 严格对照 WHEELTEC 官方 MiniBalance.c 序列:
     *   USART3_Init(115200) → 200ms → 逐轴操作:
     *     Disable → 200ms → SetAcc(75) → 200ms → Enable → 200ms
     *     → SetMode(SPEED) → 200ms → SetSpeed(0) → 200ms
     *   (官方每步间 200ms, 与我旧版 2ms 天壤之别; F32C 上电初期软硬件未稳,
     *    连续快速发帧被丢 → 使能/模式/加速全失效 → 云台不动)。再补充:
     *   - 保留 0x00 唤醒字节(用户此前确认必要, 但可通过 BLDC_WAKEUP_ENABLE 关掉)
     *   - 保留 1.5s 上电等待
     *   - 末尾 SetZero(设当前机械角为 0 点, 用户要求"上电设基准零点")。
     *   坐标系: Pan=轴1 / Tilt=轴2。*/
    printf("[BLDC] 初始化云台 (USART3 PB10 TX → 电机RX, 115200)...\r\n");

#if BLDC_WAKEUP_ENABLE
    /* 0x00 唤醒字节(激活 F32C 串口接收, 必须在电机就绪前发出) */
    Gimbal_UART_SendByte(0x00);
    HAL_Delay(50);
    IWDG->KR = 0xAAAA;
#endif

    /* 等待 1.5s 让 F32C 上电初始化完毕 */
    for (int i = 0; i < 150; i++) {
        HAL_Delay(10);
        IWDG->KR = 0xAAAA;
    }

    /* ==== 轴1 (Pan) ==== */
    printf("[BLDC] 轴1(Pan): Disable → SetAcc → Enable → Mode → Speed(0)\r\n");

    BLDCGimbal_SendDisable(1);                   /* CMD_DISABLE 5 字节 (0 数据) */
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SetAcc(1, 75);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SendEnable(1);                     /* CMD_ENABLE 5 字节 */
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SendCmd(1, 0x00, 0x00, 0x00);     /* CMD_MODE, MODE_SPEED */
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SetSpeed(1, 0);                    /* 初速=0 → 电磁锁定 */
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    /* ==== 轴2 (Tilt) ==== */
    printf("[BLDC] 轴2(Tilt): Disable → SetAcc → Enable → Mode → Speed(0)\r\n");

    BLDCGimbal_SendDisable(2);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SetAcc(2, 75);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SendEnable(2);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SendCmd(2, 0x00, 0x00, 0x00);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    BLDCGimbal_SetSpeed(2, 0);
    HAL_Delay(200); IWDG->KR = 0xAAAA;

    /* 两轴设零: 当前机械位置 → 坐标原点 0 */
    printf("[BLDC] 设零位...\r\n");
    BLDCGimbal_SetZero();
    HAL_Delay(100); IWDG->KR = 0xAAAA;

    printf("[BLDC] 无刷云台初始化完成 (速度模式, 加速度=75 转/s², 已零位)\r\n");
    printf("       速度比例 MOTOR_RPM_PER_DEGPS = %.3f (串口 `rpm <值>` 可调)\r\n",
           (double)g_motor_rpm_scale);
}

/* ========================== 电机自检 (v6.8, v6.12 更新 init 序列) ========================== */
/* 直接驱动 F32C 扫两轴, 完全绕开 K230 与控制链路。用途: 定位"云台不动"根因。
 *   - 若自检时云台动 → 域 A(F32C/硬件)正常, 问题在域 B(K230→STM32 链路/控制);
 *   - 若自检时云台仍不动 → 域 A 故障(供电/共地/接线/电机地址/波特率), 与固件无关。
 * 注意: 自检为阻塞函数(约 7s), 在串口命令处理上下文中调用, 期间持续喂独立看门狗。 */
void BLDCGimbal_RunSelfTest(void)
{
    const int16_t seq[] = {70, 170, 0, -70, -170, 0};
    const int n = sizeof(seq) / sizeof(seq[0]);
    printf("\r\n[TEST] === 无刷云台自检开始 (直接驱动 F32C, 绕开 K230) ===\r\n");

#if BLDC_WAKEUP_ENABLE
    printf("[TEST] 0) 发 0x00 唤醒字节...\r\n");
    Gimbal_UART_SendByte(0x00);
    HAL_Delay(50); IWDG->KR = 0xAAAA;
#endif

    printf("[TEST] 1) 初始化双轴(Disable→SetAcc→Enable→Mode→Speed0)...\r\n");

    BLDCGimbal_SendDisable(1);  HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SetAcc(1, 75);   HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SendEnable(1);   HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SendCmd(1, 0x00, 0x00, 0x00);  HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SetSpeed(1, 0);  HAL_Delay(200);  IWDG->KR = 0xAAAA;

    BLDCGimbal_SendDisable(2);  HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SetAcc(2, 75);   HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SendEnable(2);   HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SendCmd(2, 0x00, 0x00, 0x00);  HAL_Delay(200);  IWDG->KR = 0xAAAA;
    BLDCGimbal_SetSpeed(2, 0);  HAL_Delay(200);  IWDG->KR = 0xAAAA;

    printf("[TEST] 3) 扫 PAN 轴 (addr=1), 每档 ~0.6s:\r\n");
    for (int i = 0; i < n; i++) {
        BLDCGimbal_SetSpeed(1, seq[i]);
        printf("   PAN  rpm=%d\r\n", (int)seq[i]);
        for (int j = 0; j < 30; j++) { HAL_Delay(20); IWDG->KR = 0xAAAA; }
    }
    printf("[TEST] 4) 扫 TILT 轴 (addr=2), 每档 ~0.6s:\r\n");
    for (int i = 0; i < n; i++) {
        BLDCGimbal_SetSpeed(2, seq[i]);
        printf("   TILT rpm=%d\r\n", (int)seq[i]);
        for (int j = 0; j < 30; j++) { HAL_Delay(20); IWDG->KR = 0xAAAA; }
    }
    BLDCGimbal_SetSpeed(1, 0);
    BLDCGimbal_SetSpeed(2, 0);
    printf("[TEST] === 自检结束 ===\r\n");
    printf("[TEST] 判断: 任一轴有转动=F32C正常(问题在 K230链路/控制); 完全不动=查供电/共地/接线/电机地址(1和2)/波特率\r\n");

    /* [v6.10] 读回当前真实角度, 验证 0x0E 反馈链路是否通(RX 中断 + 解析器) */
    HAL_Delay(20); IWDG->KR = 0xAAAA;
    BLDCGimbal_RequestAngle(1); HAL_Delay(25); IWDG->KR = 0xAAAA;
    BLDCGimbal_RequestAngle(2); HAL_Delay(45); IWDG->KR = 0xAAAA;
    printf("[TEST] 反馈链路: Pan 角度=%ld(0.1°) valid=%d | Tilt 角度=%ld(0.1°) valid=%d\r\n",
           (long)BLDCGimbal_GetAngle(1), (int)BLDCGimbal_AngleValidId(1),
           (long)BLDCGimbal_GetAngle(2), (int)BLDCGimbal_AngleValidId(2));
    if (!BLDCGimbal_AngleValid()) {
        printf("[TEST][WARN] 反馈未收到! 检查 PB11(RX)接线/共地/波特率, 或 F32C 是否应答 0x0E\r\n");
    }
}

/* ========================== 周期任务(主循环调用) ========================== */
void BLDCGimbal_Task(StepperMotor *pan, StepperMotor *tilt)
{
    /* [v6.5] 心跳保活: 周期性重发 使能+速度模式, 防 F32C 偶发掉状态后整机失联。
     * 电机若因干扰丢失使能/模式配置, 会停止响应速度指令 → 表现为"断联"但串口其实没断。
     * 每 BLDC_HEARTBEAT_MS 重发一次, 让其在超时窗口内自愈(阻塞发送, 不占额外中断)。 */
    static uint32_t last_heartbeat_tick = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_heartbeat_tick) >= BLDC_HEARTBEAT_MS) {
        last_heartbeat_tick = now;
        BLDCGimbal_SendEnable(1);
        BLDCGimbal_SendEnable(2);
        BLDCGimbal_SendCmd(1, 0x00, 0x00, 0x00);  /* 速度闭环模式 */
        BLDCGimbal_SendCmd(2, 0x00, 0x00, 0x00);
    }

    /* [v6.8] 手动点动覆盖: 若 jog 未过期, 优先发送 jog 速度(不依赖 K230, 直接验证 F32C) */
    int16_t pan_rpm  = pan->cmd_rpm;
    int16_t tilt_rpm = tilt->cmd_rpm;
    if (now < g_jog_until) {
        pan_rpm  = g_jog_pan_rpm;
        tilt_rpm = g_jog_tilt_rpm;
    }

    /* 把模型当前速度转成 RPM 发给电机; 空闲(state=IDLE)时 cmd_rpm=0 → 自锁保持 */
    BLDCGimbal_SetSpeed(1, pan_rpm);   /* Pan  = 电机 ID1 */
    BLDCGimbal_SetSpeed(2, tilt_rpm);  /* Tilt = 电机 ID2 */

    /* [v6.10] 周期性请求角度反馈(每 ~40ms 轮询一轴), 供闭环 PID 用真实位置/速度.
     * 反馈响应由 USART3 RX 中断逐字节喂入 BLDCGimbal_RxByte() 解析(见 main.c). */
    static uint8_t fb_toggle = 0;
    if (fb_toggle == 0) BLDCGimbal_RequestAngle(1);
    else                 BLDCGimbal_RequestAngle(2);
    fb_toggle ^= 1;
}
