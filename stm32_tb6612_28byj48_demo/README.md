# STM32F103C8T6 + ULN2003 驱动 28BYJ-48 双轴云台 Demo

## 项目概述

使用 **STM32F103C8T6 (Blue Pill)** + **两片 ULN2003 达林顿驱动板**（俗称 28BYJ-48 专用驱动板），控制 **两台** 28BYJ-48 微型减速步进电机，实现 **双轴云台（Pan 水平 + Tilt 俯仰）** 的 **K230 视觉闭环激光打靶**。

> 2025 电赛 E 题打靶视觉任务：K230（庐山派/CanMV）识别圆形靶心 + 矩形边框，通过 USART3 (PB10/PB11) 发送靶心坐标给 STM32，STM32 用 PID+卡尔曼滤波闭环驱动云台，将靶心对准画面中央。激光与摄像头同轴安装，靶心居中即命中。

### 功能特性

| 功能               | 说明                                                |
| ---------------- | ------------------------------------------------- |
| 双轴控制             | Pan（水平）+ Tilt（俯仰），每帧 25Hz 双轴同时 PID 更新             |
| 步进模式             | 全步进（大力矩）/ 半步进（平滑，默认）/ 波驱动（省电）                     |
| 速度控制             | 步间延时 1300~20000 µs 可调                             |
| 角度控制             | 精确旋转指定角度（半步步进 ≈ 0.088°/步）                         |
| 扫描模式             | 自动来回摆动（±N°，到限位自动换向）                               |
| **🎯 K230 自动打靶** | YOLO 检测 + 矩形边框边缘检测 + 红激光检测                        |
| **PID 闭环控制**     | 分段增益 PID (Kp=0.005/Ki=0.002/Kd=0.0003) + 1D 卡尔曼滤波 |
| **绝对靶心控制**       | 误差 = 靶心 - 画面中心（不依赖激光检测，激光仅用于命中判定）                 |
| **CRC8 校验**      | K230→STM32 坐标帧带 CRC8 (poly 0x07)，防 EMI 篡改         |
| **OLED 自动恢复**    | 电机噪声打死 OLED I2C 时自动重新初始化                          |
| 串口控制             | UART 指令集，可接 USB-TTL / 蓝牙 / 上位机                    |
| 蓝牙推送             | 每 3s 推送 K230 追踪状态到手机                              |

### 硬件参数

- **28BYJ-48 步进电机（每台）**
  - 额定电压：5V DC
  - 相数：4 相（5 线单极性）
  - 减速比：1:64
  - 步距角：5.625°/64 ≈ 0.088°（半步步进）
  - 全步进一圈：2048 步（输出轴）｜半步进一圈：4096 步（输出轴）
- **ULN2003 驱动板（每片）**
  - 4 路达林顿管，开漏输出
  - INx=HIGH → 输出拉低 → 线圈通电
  - 每路持续电流 500mA，完全可以驱动 28BYJ-48
  - 板上自带 LED 指示 A/B/C/D 四相 和电源
  - 4 针白色插座直插 28BYJ-48

---

## 硬件连线

### 注意：28BYJ-48 有两种常见排线顺序，请确认板上标注

大部分 ULN2003 模块的白色插座接线顺序为：

```
28BYJ-48 排线（从左到右）→ 蓝 / 黄 / 粉 / 橙 / 红
                                            │
                                            │ 直接接 5V（不经过驱动板）
                                            │
ULN2003 板上对应（从左到右）→ D / C / B / A  / COM
```

即 **橙(A)→IN1**，**粉(B)→IN2**，**黄(C)→IN3**，**蓝(D)→IN4**。若你的模块排线顺序不同，核对一下。

### 总体接线图（双 ULN2003）

```
                    ┌────────────────────┐
                    │   ULN2003 #1 (Pan) │
  PA4 ────────────▶ │ IN1    A_OUT(蓝) ──┼───▶ 蓝线 (28BYJ-48 蓝/Coil D)
  PA5 ────────────▶ │ IN2    B_OUT(粉) ──┼───▶ 粉线 (28BYJ-48 粉/Coil B)
  PA6 ────────────▶ │ IN3    C_OUT(黄) ──┼───▶ 黄线 (28BYJ-48 黄/Coil C)
  PA7 ────────────▶ │ IN4    D_OUT(橙) ──┼───▶ 橙线 (28BYJ-48 橙/Coil A)
  5V  ────────────▶ │ 5-12V              │
  GND ────────────▶ │ GND       COM ─────┼───▶ 红线 (28BYJ-48 公共端 → 5V)
                    └────────────────────┘

                    ┌────────────────────┐
                    │   ULN2003 #2 (Tilt)│
  PB3 ────────────▶ │ IN1    A_OUT(蓝) ──┼───▶ 蓝线 (28BYJ-48 蓝/Coil D)
  PB4 ────────────▶ │ IN2    B_OUT(粉) ──┼───▶ 粉线 (28BYJ-48 粉/Coil B)
  PB5 ────────────▶ │ IN3    C_OUT(黄) ──┼───▶ 黄线 (28BYJ-48 黄/Coil C)
  PB6 ────────────▶ │ IN4    D_OUT(橙) ──┼───▶ 橙线 (28BYJ-48 橙/Coil A)
  5V  ────────────▶ │ 5-12V              │
  GND ────────────▶ │ GND       COM ─────┼───▶ 红线 (28BYJ-48 公共端 → 5V)
                    └────────────────────┘

  按键: PB12(UP) PB13(DOWN) PB14(DIR) PB15(MODE)  ── 另一端接 GND (内部上拉)
  串口: PA9(TX) ── USB-TTL RX, PA10(RX) ── USB-TTL TX
  SWD : PA13(SWDIO) PA14(SWCLK) ── ST-Link (保留用于调试/烧录)
```

### 模块接线明细

| STM32 引脚 |        接       | 功能                     |
| :------: | :------------: | :--------------------- |
|    PA4   | ULN2003 #1 IN1 | Pan 线圈 D（蓝/A_OUT） |
|    PA5   | ULN2003 #1 IN2 | Pan 线圈 B（粉/B_OUT） |
|    PA6   | ULN2003 #1 IN3 | Pan 线圈 C（黄/C_OUT） |
|    PA7   | ULN2003 #1 IN4 | Pan 线圈 A（橙/D_OUT） |
|    PB3   | ULN2003 #2 IN1 | Tilt 线圈 D（蓝/A_OUT） |
|    PB4   | ULN2003 #2 IN2 | Tilt 线圈 B（粉/B_OUT） |
|    PB5   | ULN2003 #2 IN3 | Tilt 线圈 C（黄/C_OUT） |
|    PB6   | ULN2003 #2 IN4 | Tilt 线圈 A（橙/D_OUT） |
|    PA9   |   USB-TTL RX   | 调试串口 TX (115200)       |
|   PA10   |   USB-TTL TX   | 调试串口 RX                |
|    PB8   |    OLED SCL    | SSD1306 时钟线（软件 I2C 位敲） |
|    PB9   |    OLED SDA    | SSD1306 数据线            |
|   PB10   |  K230 UART2 RX | STM32 USART3 TX → K230 |
|   PB11   |  K230 UART2 TX | STM32 USART3 RX ← K230 |

### 其他模块

**&#x20;JDY-31 蓝牙（USART2）：**

| 模块  | 引脚              |
| :-- | :-------------- |
| VCC | 3.3V            |
| GND | GND             |
| TXD | PA3 (USART2 RX) |
| RXD | PA2 (USART2 TX) |
| 波特率 | 9600            |

**OLED 调试屏（SSD1306，软件 I2C 位敲）：**

| 模块     | 引脚                             |
| :----- | :----------------------------- |
| VCC    | 3.3V（少数模块需 5V）                 |
| GND    | GND                            |
| SCL    | PB8（需 4.7kΩ 上拉到 3.3V，模块自带上拉则免） |
| SDA    | PB9（同上）                        |
| I2C 地址 | 0x3C（自动探测，0x3D 也兼容）            |

**K230 庐山派：**

| K230 引脚           | STM32 引脚         |
| :---------------- | :--------------- |
| GPIO05 (UART2 TX) | PB11 (USART3 RX) |
| GPIO06 (UART2 RX) | PB10 (USART3 TX) |
| 波特率               | 115200           |

### 供电说明

ULN2003 驱动板 **5-12V 输入端接 5V**（与 28BYJ-48 COM 同一电源）。两片驱动板可共用一路 5V 电源。

**重要**：建议 **电机 + STM32 统一用 ≥1A 的 5V 电源供电**，不要仅靠 ST-Link 的 5V（通常不足 200mA）。如果使用 USB 供电，确认 USB 口能提供 ≥500mA。28BYJ-48 在半步进模式下每轴约 300mA，双轴同时转约 600mA。

> ULN2003 没有 STBY 引脚，线圈通电与否完全由 IN1~IN4 的 GPIO 状态决定（IN=HIGH → 线圈 ON）。空闲时代码会输出全 0 → 线圈全断 → 电机靠齿轮自锁保持位置。

---

## 步进序列（ULN2003 4 位直接编码）

28BYJ-48 是单极性 4 相步进电机，ULN2003 的 IN1~IN4 直接驱动各线圈（IN=1 → 线圈通电）。

编码规则：`bit[3:0] = (IN4, IN3, IN2, IN1)`

板上丝印: **A=蓝、B=粉、C=黄、D=橙**（IN1→A=蓝, IN2→B=粉, IN3→C=黄, IN4→D=橙）

### 半步步进（8 拍，默认，平滑）

|  拍号 | 橙 A(IN4) | 粉 B(IN2) | 黄 C(IN3) | 蓝 D(IN1) |  编码  |
| :-: | :------: | :------: | :------: | :------: | :--: |
|  0  |    ON    |          |          |          | 0x08 |
|  1  |    ON    |    ON    |          |          | 0x0A |
|  2  |          |    ON    |          |          | 0x02 |
|  3  |          |    ON    |    ON    |          | 0x06 |
|  4  |          |          |    ON    |          | 0x04 |
|  5  |          |          |    ON    |    ON    | 0x05 |
|  6  |          |          |          |    ON    | 0x01 |
|  7  |    ON    |          |          |    ON    | 0x09 |

### 全步进（4 拍，大力矩）

|  拍号 |     编码     |
| :-: | :--------: |
|  0  | 0x0A (A+B) |
|  1  | 0x06 (B+C) |
|  2  | 0x05 (C+D) |
|  3  | 0x09 (D+A) |

### 波驱动（4 拍，最省电）

|  拍号 |    编码    |
| :-: | :------: |
|  0  | 0x08 (A) |
|  1  | 0x02 (B) |
|  2  | 0x04 (C) |
|  3  | 0x01 (D) |

---

## 软件架构

```
main.c
├── SystemClock_Config()    → 72MHz (HSE 8MHz ×9 PLL)
├── GPIO_Init()             → ULN2003 IN1-4 + PB3/PB4 JTAG 释放
├── USART1_Init()           → 调试串口 115200
├── USART2_Init()           → 蓝牙 9600
├── USART3_Init()           → K230 坐标帧 115200
├── I2C1_Init()             → (留空, OLED 走软件 I2C)
├── TIM2_Init()             → Pan 步进节拍定时器
├── TIM3_Init()             → Tilt 步进节拍定时器
└── while(1)
    ├── Serial_Execute()    → 串口命令解析
    ├── Gimbal_KeyScan()    → 按键扫描
    ├── K230_ProcessFrame() → 坐标帧 → PID → 双轴控制
    ├── K230_TimeoutCheck() → 2s 超时安全停车
    ├── OLED_Refresh()      → 每 200ms 刷新
    ├── BT_SendReport()     → 每 3s 蓝牙推送
    ├── K230_SendStatus()   → 每 500ms 回传云台状态给 K230
    └── LED 心跳            → 双轴空闲时 500ms 慢闪

中断:
├── TIM2_IRQHandler         → Pan 推进一拍 (Stepper_TIM_IRQHandler)
├── TIM3_IRQHandler         → Tilt 推进一拍
├── USART1/2/3_IRQHandler  → 缓冲字节, 主循环处理
└── SysTick_Handler         → HAL 时基
```

---

## 编译与烧录

### 前提

VS Code + PlatformIO 插件（自动下载 arm-none-eabi-gcc 和 STM32CubeF1 HAL）。

### 编译

```bash
cd stm32_tb6612_28byj48_demo
pio run
```

产物：`.pio/build/bluepill_f103c8/firmware.hex`

### 烧录

**ST-Link（推荐）：**

```bash
pio run -t upload
```

**串口 ISP（USB-TTL，免 ST-Link）：**

1. `platformio.ini` 改 `upload_protocol = serial`、`upload_port = COM3`
2. BOOT0=1、BOOT1=0，按 RESET
3. Upload → BOOT0=0 → 按 RESET

### 串口监视

```bash
pio device monitor
```

波特率 115200。

---

## 操作说明

### 按键

| 按键   |  引脚  | 功能                                    |
| ---- | :--: | ------------------------------------- |
| UP   | PB12 | 加速（当前活动轴）                             |
| DOWN | PB13 | 减速                                    |
| DIR  | PB14 | 换向 CW ↔ CCW                           |
| MODE | PB15 | 模式循环：Pan→Tilt→Sweep→Serial→Dance→Auto |

### 串口命令（115200 / 9600 蓝牙，回车结束）

| 命令   | 功能         | 示例      |
| ---- | ---------- | ------- |
| `p`  | 选中 Pan     | `p`     |
| `t`  | 选中 Tilt    | `t`     |
| `c`  | 顺时针连续转     | `c`     |
| `a`  | 逆时针连续转     | `a`     |
| `s`  | 停止         | `s`     |
| `f`  | 全步步进       | `f`     |
| `h`  | 半步步进（默认）   | `h`     |
| `+N` | 前进 N 步     | `+2048` |
| `-N` | 后退 N 步     | `-512`  |
| `rN` | 转 N 度      | `r90`   |
| `wN` | ±N° 扫描     | `w180`  |
| `vN` | 设速度 µs/步   | `v3000` |
| `i`  | 反转 Pan 方向  | `i`     |
| `I`  | 反转 Tilt 方向 | `I`     |
| `P`  | 打印 PID 状态  | `P`     |
| `?`  | 查询电机状态     | `?`     |

### OLED 显示（K230 AUTO 模式）

```
=LASER TRACKING=
AUTO YOLO
PAN:+12 CW 1.4ms
TIL: -5 --
PID:+0.23 I:1
INV:N
E:(+12,-5) -
UP:34s
```

---

## 更新记录

- **v4.8 (切 ULN2003)**：
  - 驱动板从 TB6612 双 H 桥改为 **ULN2003 达林顿驱动板**（28BYJ-48 专用）
  - 步进编码从 2-bit 桥对码改为 **4-bit 直接编码**
  - 删除 PWM 引脚（PA0/PA1/PB1/PB7）和 STBY 引脚（PB0）相关代码
  - 删除 STBY 门控（`Gimbal_UpdateSTBY`）等 TB6612 专用逻辑
  - 接线简化：每电机 4 根信号线，无 PWM 无 STBY
  - 文档全面更新
- **v4.7.x (综合修复)**：
  - STBY 动态门控（已随 TB6612 删除）
  - OLED 健康检查 + 自动恢复
  - 控制帧 CRC8 (poly 0x07)
  - 异常处理器加固（删除 printf）
  - USART1 ISR 删除阻塞回显
  - K230 超时 500ms → 2000ms
  - 绝对靶心控制（K230 侧误差公式改为 `靶心 - 画面中心`）
  - K230_ERROR_MAX_PX 200→400

---

## 文件结构

```
stm32_tb6612_28byj48_demo/
├── platformio.ini
├── include/
│   ├── main.h / stepper.h / ssd1306.h / oled_debug.h / stm32f1xx_hal_conf.h
├── src/
│   ├── main.c / stepper.c / ssd1306.c / oled_debug.c
├── bluetooth_app/
│   ├── webserial.html / index.html / README.md
├── firmware.hex / .gitignore / README.md

k230/
├── main.py               # K230 打靶追踪程序
└── deploy_config.json
```
