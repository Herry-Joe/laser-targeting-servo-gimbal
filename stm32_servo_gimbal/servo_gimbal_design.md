# 无刷电机二维云台改造方案（WHEELTEC F32C）

> 原方案：双轴 ULN2003 + 28BYJ-48 步进电机（Pan 水平 + Tilt 俯仰），K230 视觉闭环激光打靶。
> 中间方案：双轴 270° 标准舵机 + 空心滑环，STM32F103C8T6 用 50Hz PWM 驱动（PB1=TIM3_CH4=Pan，PA11=TIM1_CH4=Tilt）。
> **本次方案（当前）**：WHEELTEC F32C 二自由度无刷电机云台，自带驱动板、走 TTL 串口速度闭环协议，
>   MCU 用 **USART3 硬件串口（PB10 TX）→ 驱动板 RXD** 发速度指令（ID1=Pan，ID2=Tilt），115200/8N1。
> 上层 K230 视觉闭环、速度控制器、位置 PID、虚拟步进模型**全部不变**，仅替换最底层执行机构。
> **[v6.0 串口重分配]** K230 视觉从原 USART3 迁到 **USART2（PA2/PA3）**；空出的 USART3 改接无刷云台；
>   蓝牙（JDY-31）整体移除，命令/遥测统一走调试串口 **USART1（PA9/PA10）**。
> 固件改动后需重新编译，产物 `servo_gimbal.hex`。

---

## 一、硬件改动清单

| 项目 | 原方案 | 新方案 | 说明 |
|---|---|---|---|
| 驱动 | 2× ULN2003 驱动板 | 直接 PWM | 舵机自带驱动电路，省掉 ULN2003 |
| 电机 | 2× 28BYJ-48 步进 | 2× 270° 标准舵机 | Pan(水平) / Tilt(俯仰) 各一 |
| 旋转机构 | 齿轮减速，有限角度 | Pan 轴经**空心滑环**连续旋转 | 滑环解决线束缠绕 |
| 电源 | 5V 小电流 | 5V / ≥2A 独立供电 | 舵机启动电流大，勿与 MCU 共用 LDO |
| 走线 | 定子侧直接接电机 | 定子侧→滑环→转子侧 | 见第三节 |

**舵机选型建议**
- 扭矩：金属齿轮，≥ 10 kg·cm（云台要带 K230 模组 + 激光器，惯量不小）。
- 角度：270° 标准舵机（0.5ms=0°、2.5ms=270°）。不要用 360° 连续旋转舵机（无法定位）。
- 典型型号：MG996R、DS3218、LD-27MG 等；赛道可用更高扭矩型号。

**滑环选型**
- 空心滑环，过中心轴，≥ 6 路（5V / GND / Pan_PWM / Tilt_PWM / K230_TX / K230_RX / Laser，实际 6~7 线）。
- 额定电流 ≥ 2A（给上平台 5V 供电留余量）。
- 若改用**串行总线舵机**（如 STS3215），只需 3 线（5V/GND/UART），滑环降到 3~4 路即可，且天然支持连续旋转+位置反馈。

---

## 二、引脚重分配

| 信号 | 原（步进） | 现（舵机） | 备注 |
|---|---|---|---|
| PA4 | Pan IN1（早期步进） | 释放（无刷云台改走 USART3 硬件串口，不再用 PA4 软串口） | 悬空 |
| PA5 | Pan IN2（早期步进） | 释放 | 悬空 |
| PB1 | TIM3_CH4 → Pan 舵机 PWM | 释放 | 无刷方案不再作 PWM |
| PA11 | TIM1_CH4 → Tilt 舵机 PWM | 释放 | 无刷方案不再作 PWM |
| PB3~PB6 | Tilt IN1~IN4 | 释放 | PB3/PB4 已 NOJTAG 释放 |
| TIM2 | Pan 步进节拍 | Pan 步进节拍（不变） | Update 中断推进 pos_steps |
| **TIM3** | 50Hz PWM（Pan） | **弃用（空函数）** | 无刷云台改用 USART3 硬件串口，不再需要位时钟 |
| **TIM1** | 50Hz PWM（Tilt） | **弃用（空函数）** | 无刷方案不写 PWM |
| **TIM4** | Tilt 步进节拍 | Tilt 步进节拍（不变） | Update 中断推进 pos_steps |

> 关键设计：保留原"虚拟步进"模型（pos_steps，单位 0.1°）和全部已调好的 PID / 位置闭环算法，
> 仅把最底层"舵机 PWM 角度映射"换成"无刷速度串口指令"。因此追踪/定位逻辑零改动、风险最小。
> STM32F103 共 3 个硬件串口：USART1(调试/PC)、USART2(K230 视觉)、USART3(无刷云台)。
> 蓝牙（JDY-31）于 v6.0 移除；K230 由 USART3 迁到 USART2 后，USART3 专用于无刷云台硬件串口。

---

## 三、滑环走线（Pan 轴连续旋转）

```
        ┌──────────── 上平台（转子，随 Pan 转） ────────────┐
        │  K230 视觉模组 + 激光器                          │
        │      │                                           │
        │   K230_TX / K230_RX / 5V / GND  ←── 滑环 ──→      │
        │                                        │          │
        └────────────────────────────────────────┼─────────┘
                                           （空心滑环，过中心轴）
                                                  │
        ┌──────────── 底座（定子，固定） ─────────┼────────┐
        │  STM32F103C8T6                            │        │
        │   PA8 / PA11 ── 释放（无刷方案不用舵机 PWM）          │        │
        │   5V / GND ──────── 滑环定子侧            │        │
        │   USART3 (PB10) ── 无刷云台 TX → F32C RXD  │       │
        │   USART2 (PA2/PA3) ── K230_TX/RX           │       │
        └───────────────────────────────────────────┘        │
                                                              │
                                            Tilt 舵机固定在
                                            上平台，信号由滑环
                                            转子侧引出
```

走线合计（无刷方案）：`5V、GND、GIMBAL_TX(PB10)、K230_TX、K230_RX` = 5 线（云台单向 TX，不开 RX）。
（总线舵机方案仅 `5V、GND、UART` = 3 线。）

---

## 四、角度 / 速度映射（虚拟步进模型）

> 无刷方案下不再有"舵机脉宽"概念：模型仍以 `pos_steps`（0.1°/步）描述位置，由 `Actuator_Update()`
> 把模型角速度换算成指令 RPM 发给 F32C（速度闭环模式积分为位置）。下列步进节拍关系仍适用。

- 270° 标准舵机脉宽：0°→0.5ms，270°→2.5ms，中位 135°→1.5ms。（历史舵机参数，无刷方案仅参考）
- `pos_steps` 单位 **0.1°**，上电零点 = 舵机中位（135°）。
- 脉宽计算：
  ```
  pulse_us = 1500 + (pos_steps / 10.0) * 7.4074      // 7.4074 = 2000us / 270°
  钳位到 [500, 2500] us
  ```
- **物理限位保护**：Pan ±135°（±1350 步）/ Tilt ±80°（±800 步），ISR 内钳位，防止舵机超程打坏齿轮。
- **位置闭环软限位**（可调）：Pan ±400 步（±40°），Tilt ±300 步（±30°）。
- 节拍周期 `step_delay_us` 决定舵机指令刷新率：1ms/步≈100°/s，20ms/步≈5°/s。

---

## 五、固件改动小结（无刷方案）

- `src/bldc_gimbal.c` / `include/bldc_gimbal.h`（**新增**）
  - WHEELTEC F32C 协议：帧 `0x7A [ID] [CMD] [DATA...] [BCC] 0x7B`，BCC=前面字节异或，115200/8N1。
  - 指令：使能 `0x06`、设模式 `0x00`(0=速度闭环)、设速度 `0x01`(2 字节 RPM 有符号)、读反馈 `0x0E`。
  - USART3 硬件串口（PB10 TX），`HAL_UART_Transmit`（初始化阻塞）/ `HAL_UART_Transmit_IT`（主循环非阻塞）发帧，115200/8N1，单向 TX。
  - `BLDCGimbal_Init()`：发 0x00 激活 → 等 1.5s 电机启动（喂 IWDG）→ 使能两轴 → 设速度模式。
  - `BLDCGimbal_Task(pan,tilt)`：主循环调用，把 `motor->cmd_rpm` 发给电机 ID1/ID2；空闲发 0 自锁。
  - USART3 发送由 `HAL_UART_IRQHandler` 推进 IT 传输（USART3_IRQHandler 内调用），无需 TIM3 位输出。
- `include/stepper.h` / `src/stepper.c`
  - `Actuator_Update()` 替换原 `Servo_UpdatePWM()`：模型角速度(°/s = 100000/step_delay_us) × `g_motor_rpm_scale` 得指令 RPM，写入 `motor->cmd_rpm`（`DIR_CCW` 取负）。
  - `Stepper_Init()` 签名保留 `htim_pwm / pwm_channel` 参数但**不再写 PWM CCR**（仅占位）。
  - 保留全部 `Stepper_*` 接口、状态机、WORK_MODE、软启动、限位保护；扫描到物理限位自动反向。
  - 上电 `cmd_rpm=0` → 电机自锁保持当前角度（无刷无"中位脉宽"概念）。
- `include/main.h`
  - 新增 `MOTOR_RPM_PER_DEGPS=0.5f`、`BLDC_RPM_MAX=300`；`SERVO_*` 宏保留（参数仅占位）。`SOFT_UART_BAUD` 随 PA4 软串口弃用已删除。
  - `htim3` 注释改为"保留(无刷云台改走 USART3 后未使用)"；`htim1` 注释改为"已弃用(空函数)"。
- `src/main.c`
  - `TIM1_Init()` / `TIM3_Init()` 均改为空函数（无刷云台走 USART3 硬件串口，不再需要 PWM / 位时钟）。
  - 初始化序列插入 `BLDCGimbal_Init()`（在 `Stepper_Init` 之后、主循环之前）；删除两条舵机置中 `__HAL_TIM_SET_COMPARE`。
  - 主循环调用 `BLDCGimbal_Task(&g_motor_pan, &g_motor_tilt)`；新增串口命令 `rpm <值>` 实时调速度比例。
  - 角度换算 `PosCtrl_AngleToSteps/StepsToAngle` 仍用 3600 步/圈；限位 `POS_LIMIT_PAN ±1350 / TILT ±800`。

---

## 六、编译与烧录

```bash
# 编译（按本机 workbuddy 环境约定加 PYTHONPATH=""）
cd stm32_servo_gimbal
PYTHONPATH="" pio run

# 生成 HEX（objcopy 不能写中文路径，先输出到英文临时路径再拷回）
arm-none-eabi-objcopy -O ihex .pio/build/bluepill_f103c8/firmware.elf servo_gimbal.hex
```

烧录方式（platformio.ini 已配 `upload_protocol = stlink`）：
- ST-Link：`pio run -t upload`
- 串口 ISP：`upload_protocol = serial`，BOOT0=1 后烧录

---

## 七、上电后验证建议

1. 上电看无刷云台是否停在当前角度（电机自锁，激光大致朝前），无剧烈摆动；调试串口应打印 `[BLDC] 无刷云台初始化完成`。
2. 串口发 `x30` / `y20` 等角度指令，观察 Pan/Tilt 是否平滑转到目标并自锁（速度模式：指令 RPM 决定转速，停发即保持）。
3. 若云台转动过慢/过快，发 `rpm 0.6`（或 0.3~1.0 间调整）实时改变速度比例，直到手感合适。
4. 发 `k` 切 K230 自动打靶，观察追踪是否跟手（若偏慢，调小 `STEPPER_MIN_DELAY_US` 或增大位置环 Kp）。
5. 扫描模式 `SWEEP` 应在限位间平稳往返，不卡死在端点。
