# 舵机二维云台改造方案（带滑环）

> 原方案：双轴 ULN2003 + 28BYJ-48 步进电机（Pan 水平 + Tilt 俯仰），K230 视觉闭环激光打靶。
> 新方案：双轴 270° 标准舵机 + 空心滑环，STM32F103C8T6 用 TIM1 输出 50Hz PWM 驱动（PA8=Pan/CH1，PA11=Tilt/CH4）。
> 固件已编译通过，产物 `servo_gimbal.hex`（Flash 77.2% / RAM 22.9%）。

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
| PA4 / PA5 | Pan IN1 / IN2 | 释放 | 悬空 |
| **PA8** | （原未用） | **TIM1_CH1 → Pan 舵机 PWM** | 复用推挽 |
| **PA11** | （原未用） | **TIM1_CH4 → Tilt 舵机 PWM** | 复用推挽 |
| PB3~PB6 | Tilt IN1~IN4 | 释放 | PB3/PB4 已 NOJTAG 释放 |
| TIM2 | Pan 步进节拍 | Pan 步进节拍（不变） | Update 中断推进 pos_steps |
| **TIM1** | （原未用） | **50Hz PWM 输出（舵机，PA8/PA11）** | 仅 PWM，无节拍中断 |
| **TIM4** | 未用 | **Tilt 步进节拍（新增）** | Update 中断推进 pos_steps |

> 关键设计：保留原"虚拟步进"模型（pos_steps，单位 0.1°）和全部已调好的 PID / 位置闭环算法，
> 仅把最底层"GPIO 步进序列"换成"TIM3 PWM 角度映射"。因此追踪/定位逻辑零改动、风险最小。

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
        │   PA8 (TIM1_CH1) ── Pan 舵机 PWM          │        │
        │   PA11 (TIM1_CH4) ── Tilt 舵机 PWM        │        │
        │   5V / GND ──────── 滑环定子侧            │        │
        │   USART3 (PB10/PB11) ── K230_TX/RX         │       │
        └───────────────────────────────────────────┘        │
                                                              │
                                            Tilt 舵机固定在
                                            上平台，信号由滑环
                                            转子侧引出
```

走线合计（模拟 PWM 舵机方案）：`5V、GND、Pan_PWM、Tilt_PWM、K230_TX、K230_RX` = 6 线。
（总线舵机方案仅 `5V、GND、UART` = 3 线。）

---

## 四、舵机角度映射

- 270° 标准舵机脉宽：0°→0.5ms，270°→2.5ms，中位 135°→1.5ms。
- `pos_steps` 单位 **0.1°**，上电零点 = 舵机中位（135°）。
- 脉宽计算：
  ```
  pulse_us = 1500 + (pos_steps / 10.0) * 7.4074      // 7.4074 = 2000us / 270°
  钳位到 [500, 2500] us
  ```
- **物理限位保护**：±135°（±1350 步），ISR 内钳位，防止舵机超程打坏齿轮。
- **位置闭环限位**（可调）：Pan ±1350 步（±135°），Tilt ±400 步（±40°）。
- 节拍周期 `step_delay_us` 决定舵机指令刷新率：1ms/步≈100°/s，20ms/步≈5°/s。

---

## 五、固件改动小结（已编译通过）

- `include/stepper.h` / `src/stepper.c`
  - 删除 4 相 GPIO 步进序列；新增 `Servo_UpdatePWM()` 把 pos_steps 映射成 TIM1 CCR 脉宽。
  - `Stepper_Init()` 新签名：`(motor, htim_step, htim_pwm, pwm_channel, label)`。
  - 保留全部 `Stepper_*` 接口、状态机、WORK_MODE、软启动、限位保护；扫描到物理限位自动反向。
  - 上电即把舵机置于中位（1500us），避免乱摆。
- `include/main.h`
  - 新增 `SERVO_*` 宏（PWM 引脚/通道、脉宽、角度映射、`SERVO_STEPS_PER_DEG=10`、`SERVO_POS_MIN/MAX`）。
  - `TIM3` 弃用，`TIM1` 作舵机 PWM；新增 `htim4` / `TIM4_Init()` 声明；`STEPPER_MIN_DELAY_US` 调为 1000。
- `src/main.c`
  - GPIO 释放 PA4~PA7、PB0~PB7 线圈配置；`TIM1_Init()` 改为 50Hz PWM（PA8/PA11 复用推挽）。
  - 新增 `TIM4_Init()`（Tilt 节拍）；TIM1 仅作 PWM（无 Update 中断），TIM4 作 Tilt 节拍。
  - `Stepper_Init` 调用改为 `(&htim2,&htim1,CH1,"Pan")` / `(&htim4,&htim1,CH4,"Tilt")`。
  - 角度换算 `PosCtrl_AngleToSteps/StepsToAngle` 改用 3600 步/圈；K230 状态帧回传真实位置。
  - 限位 `POS_LIMIT_PAN ±1350`、回程差默认 0、丢步阈值 3000 步。

---

## 六、编译与烧录

```bash
# 编译（按本机 workbuddy 环境约定加 PYTHONPATH=""）
cd stm32_uln2003_28byj48_demo
PYTHONPATH="" pio run

# 生成 HEX（objcopy 不能写中文路径，先输出到英文临时路径再拷回）
arm-none-eabi-objcopy -O ihex .pio/build/bluepill_f103c8/firmware.elf servo_gimbal.hex
```

烧录方式（platformio.ini 已配 `upload_protocol = stlink`）：
- ST-Link：`pio run -t upload`
- 串口 ISP：`upload_protocol = serial`，BOOT0=1 后烧录

---

## 七、上电后验证建议

1. 上电看两舵机是否停在中位（激光大致朝前），无剧烈摆动。
2. 串口发 `x30` / `y20` 等角度指令，观察 Pan/Tilt 是否平滑转到目标并自锁。
3. 发 `k` 切 K230 自动打靶，观察追踪是否跟手（若偏慢，调小 `STEPPER_MIN_DELAY_US` 或增大位置环 Kp）。
4. 扫描模式 `SWEEP` 应在限位间平稳往返，不卡死在端点。
