# STM32F103C8T6 舵机二维云台固件（激光打靶）

> 双轴 **S20F 舵机 + 空心滑环** 方案：Pan 水平 / Tilt 俯仰由 **50Hz PWM** 驱动。
> - **Pan 舵机** → **PB1 = TIM3_CH4**
> - **Tilt 舵机** → **PA11 = TIM1_CH4**
>
> K230（庐山派 / CanMV）识别圆形靶心 + 红色激光光斑，通过 USART3 发送“靶心-画面中心误差”帧，STM32 用速度控制器闭环驱动云台把激光对准靶心。激光与摄像头同轴，靶心居中即命中。
>
> 完整的项目总览、硬件清单、引脚表、部署与协议见根目录 **[`../README.md`](../README.md)**；舵机改造与滑环走线见 **[`servo_gimbal_design.md`](./servo_gimbal_design.md)**。

---

## 一、功能特性

| 功能 | 说明 |
|------|------|
| 双轴 PWM 控制 | Pan（水平，PB1/TIM3）、Tilt（俯仰，PA11/TIM1），50Hz，S20F 270°/180° 舵机 |
| 位置闭环（默认）| 给定目标角度 → 位置 PID（Kp/Kd）→ 梯形加减速走到位置；软限位保护 |
| **AUTO 自动打靶** | K230 误差帧 → 无积分速度控制器 `out = Kp·err + Kd·derr` + 减速区线性收敛；根治过冲 |
| **粘性命中锁** | 命中（误差 < 容差）后双轴锁存停稳；仅当误差明显偏离（>40px）或丢帧才重捕，根治“打到靶上停不下来” |
| **置信度门控** | 靶心+激光均有效且连续 2 帧才驱动；丢失即冻结（停机+重置 PID），数据不可靠时不误动 |
| **软限位** | Pan ±40°（±400 步）/ Tilt ±30°（±300 步），控转动范围防过冲 |
| **物理限位** | Pan ±135°（±1350 步）/ Tilt ±80°（±800 步，S20F 行程），ISR 内钳位防打坏齿轮 |
| **CRC8 校验** | K230→STM32 坐标帧带 CRC8（poly 0x07），防 EMI 篡改 |
| **OLED 自动恢复** | 电机噪声打死 I2C 时自动重新初始化 |
| 串口 / 实时调参 | UART 指令集 + 上位机 Web Serial；`g`/`pid` 命令实时改参（断电不保存）|
| 蓝牙推送 | 每 150ms 紧凑单行遥测（Pan/Tilt 状态 + PID 输出 + 减速区）|

---

## 二、硬件参数（舵机）

- **S20F 270° 舵机（Pan，水平，经滑环连续旋转）**
  - 脉宽：0.5ms = 0°，2.5ms = 270°；中位 1.5ms = 135°
  - 扭矩 20kg·cm，金属齿轮
- **S20F 180° 舵机（Tilt，俯仰）**
  - 脉宽：0.5ms = 0°，2.5ms = 180°；中位 1.5ms = 90°
- **空心滑环**：≥6 路，过 Pan 中心轴，解决线束缠绕

### 引脚 / 接线

| STM32 引脚 | 信号 | 说明 |
| :---: | :---: | :--- |
| **PB1** | Pan 舵机 PWM | TIM3_CH4，50Hz |
| **PA11** | Tilt 舵机 PWM | TIM1_CH4，50Hz |
| PB10 | K230 USART3 TX | → K230 RX (GPIO06) |
| PB11 | K230 USART3 RX | ← K230 TX (GPIO05) |
| PA9 / PA10 | 调试串口 TX / RX | USART1 @115200 |
| PA2 / PA3 | 蓝牙 TX / RX | USART2 @9600（JDY-31）|
| PB8 / PB9 | OLED SCL / SDA | I2C1 重映射，0x3C |
| PB12~PB15 | 按键 UP/DOWN/DIR/MODE | 内部上拉，另一端接 GND |

> 供电注意：舵机启动电流大，建议 **5V / ≥2A 独立供电**，勿与 MCU 共用 LDO。

---

## 三、控制模式

系统有 8 种工作模式，上电默认 **位置闭环（POSCTRL）**：

| 模式 | 名称 | 说明 |
| :-- | :-- | :-- |
| 0 | PAN | 按键控制水平电机 |
| 1 | TILT | 按键控制垂直电机 |
| 2 | SWEEP | 双轴自动扫描 |
| 3 | SERIAL | 串口连续转/步进控制 |
| 4 | DANCE | 云台演示 |
| 5 | AUTO | **K230 自动打靶**（速度控制器闭环）|
| 6 | POSCTRL | **位置闭环**（默认）|
| 7 | TRACE | 矩形循迹测试 |

### AUTO 模式（K230 自动打靶）

- K230 通过 USART3 每帧发送误差帧（带 CRC8），约 30fps 级。
- STM32 用**无积分速度控制器** `out = Kp·err + Kd·derr` 把误差映射成双轴速度。
- **减速区**：误差进入减速区后速度线性归零，物理上不会过冲靶心。
- **置信度门控 + 冻结**：靶心/激光任一带 `valid=0` 或置信度不足 2 帧 → 立即冻结（停机+重置 PID），不急停、不误发大误差。
- **粘性命中锁**：误差 < 命中容差（15px）→ 锁存停机；目标明显移动（>40px）或丢帧才解锁重捕。
- 该模式被**锁定**：进入后串口其他命令一律忽略，避免杂散字节把模式踢出。退出用 `m` 命令或 MODE 按键；`u` 可手动解除命中锁。

### POSCTRL 模式（位置闭环，默认）

- 发送 `x<角度>` / `y<角度>` 设定 Pan / Tilt 目标角度（°）。
- 位置 PID 输出速度指令 → 梯形加减速走到目标位置，到达后自动停转。
- **限位**：软限位 Pan ±40° / Tilt ±30°；物理限位 Pan ±135° / Tilt ±80°（ISR 钳位）。

---

## 四、软件架构（要点）

```
main.c
├── SystemClock_Config()    → 72MHz (HSE 8MHz ×9 PLL)
├── GPIO_Init()             → 释放 PA4~PA7/PB0~PB7 旧线圈引脚；按键/LED/OLED 配置
├── USART1_Init()           → 调试串口 115200
├── USART2_Init()           → 蓝牙 9600
├── USART3_Init()           → K230 坐标帧 115200 (PB10/PB11)
├── TIM1_Init()             → Tilt 舵机 PWM 50Hz (PA11=TIM1_CH4)
├── TIM3_Init()             → Pan 舵机 PWM 50Hz (PB1=TIM3_CH4)
├── TIM2_Init() / TIM4_Init()→ Pan / Tilt 步进节拍定时器（虚拟步进 pos_steps）
└── while(1)
    ├── Serial_Execute()    → 串口命令解析（位置闭环/实时调参/模式切换）
    ├── Gimbal_KeyScan()    → 按键扫描
    ├── PosCtrl_Update()    → 100Hz 位置闭环 PID（默认模式）
    ├── K230_ProcessFrame() → 误差帧 → 速度控制器 → 双轴控制（AUTO 模式）
    ├── K230_TimeoutCheck() → 2s 超时 / 持续无效帧安全停车
    ├── OLED_Refresh()      → 每 200ms 刷新
    └── BT_SendReport()     → 每 150ms 紧凑单行遥测（蓝牙口）
```

> 设计要点：保留原“虚拟步进”模型（`pos_steps`，单位 0.1°）和全部已调好的 PID / 位置闭环算法，仅把最底层“GPIO 步进序列”换成“TIM PWM 角度映射”，追踪/定位逻辑零改动、风险最小。

---

## 五、编译与烧录

### 前提

VS Code + PlatformIO 插件（自动下载 arm-none-eabi-gcc 和 STM32CubeF1 HAL）。

### 编译

```bash
cd stm32_servo_gimbal
PYTHONPATH="" pio run
```

产物：`.pio/build/bluepill_f103c8/firmware.elf`

生成 HEX（objcopy 不能写中文路径，先输出到英文临时路径再拷回）：

```bash
arm-none-eabi-objcopy -O ihex .pio/build/bluepill_f103c8/firmware.elf servo_gimbal.hex
```

### 烧录

- **ST-Link（推荐）**：`pio run -t upload`
- **串口 ISP（USB-TTL，免 ST-Link）**：改 `platformio.ini` 的 `upload_protocol = serial`、`upload_port = COMx`；BOOT0=1、BOOT1=0，按 RESET，Upload 后 BOOT0=0 再按 RESET

### 串口监视

```bash
pio device monitor
```

波特率 115200（调试串口 PA9/PA10，接 CH340）。实时调参遥测走 **USART2@9600（蓝牙口）**。

---

## 六、操作说明

### 按键

| 按键 | 引脚 | 功能 |
| ---- | :--: | ---- |
| UP | PB12 | 加速（当前活动轴）|
| DOWN | PB13 | 减速 |
| DIR | PB14 | 换向 CW ↔ CCW |
| MODE | PB15 | 模式循环（上电默认 POSCTRL）|

### 串口命令（115200 / 9600 蓝牙，回车结束）

| 命令 | 功能 | 示例 |
| ---- | ---- | ---- |
| `p` / `t` | 选中 Pan / Tilt 电机 | `p` |
| `c` / `a` | 选中轴连续正转 / 反转 | `c` |
| `s` | 停止选中轴 | `s` |
| `f` / `h` | 全步 / 半步（虚拟步进节拍）| `h` |
| `+N` / `-N` | 前进 / 后退 N 步 | `+2048` |
| `rN` | 旋转 N 度 | `r90` |
| `vN` | 设节拍 µs/步（越小越快）| `v3000` |
| `i` / `I` | 反转 Pan / Tilt 方向 | `i` |
| `k` | 切到 K230 自动打靶（锁定）| `k` |
| `m` | 退回位置闭环（默认）| `m` |
| `u` | 解除命中锁存 | `u` |
| `x<角度>` / `y<角度>` | Pan / Tilt 目标角度°（位置闭环）| `x90` |
| `z` | 当前位置归零 | `z` |
| `g p kp 0.005` | 设 AUTO 速度控制器增益（实时生效、断电不保存）| `g p kp 0.005` |
| `g p ki 60` | 设 AUTO 减速区宽度（px）| `g p ki 60` |
| `g p kd 0.0015` | 设 AUTO 阻尼 | `g p kd 0.0015` |
| `pid p kp 0.001` | 设位置环 PID（轴 p/t；参数 kp/ki/kd）| `pid p kp 0.001` |
| `backlash p 50` | 设回程差补偿步数（舵机版默认 0）| `backlash p 0` |
| `trace 300` | 矩形循迹测试 | `trace 300` |
| `stop` | 停止循迹，回位置闭环 | `stop` |

> 调参口诀（AUTO 速度控制器）：追得慢 → 提 `g p kp`；仍过冲 → 调大 `g p ki`（减速区）或加 `g p kd` 阻尼。
> 位置闭环：若换向后停止点漂移 → 调大 `backlash p/t`；响应用 `pid p kp` 微调。

### OLED 显示（示意）

位置闭环模式（默认）：
```
=POS CTRL=
PAN : +12°  → 1.4ms
TIL :  -5°  → 0.9ms
TGT : (90,30)
```
AUTO 打靶模式：
```
=LASER TRACKING=
AUTO
PAN:+12 CW 1.4ms
TIL: -5 --
PID:+0.23 I:60
INV:N
E:(+12,-5)
```

---

## 七、上位机（Web Serial 调试面板）

`bluetooth_app/webserial.html` 用 Chrome/Edge 的 Web Serial API 直连 CH340（默认 9600，蓝牙口）：

- 串口连接 / 断开
- **PID 实时调参面板**：Pan/Tilt 的 Kp/Ki/Kd 滑块，拖动即发 `g` 命令
- **PID 双轴曲线**：误差(px) / PID 输出(±1)，含减速区与模式状态
- 串口监视终端 + 自定义命令、手动云台控制、追踪状态卡

> `bluetooth_app/posctrl.html` 是位置闭环专用调试页；`bluetooth_app/index.html` 是 Web Bluetooth（JDY-31）遥控路径，与 Web Serial 不是一套。

---

## 八、K230 视觉端

详见 **[`../k230/README.md`](../k230/README.md)**。要点：

- **靶心检测**：cv2 灰度+自适应阈值 → findContours 取最大轮廓（优先 4 边方框靶，否则同心圆矩心）
- **激光检测**：CanMV 原生 `find_blobs`（LAB 阈值，已验证可靠）；cv2 HSV 为可选加速路径
- **决策**：置信度门控（2 帧去抖）+ 粘性命中锁；误差 < 15px 判命中
- **通信**：UART2 @115200，坐标帧 `0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC] 0x01 0x01`
- 部署：覆盖 K230 SD 卡 `main.py` 并重启；需带 OpenCV 的 CanMV 固件

---

## 九、文件结构

```
stm32_servo_gimbal/
├── platformio.ini
├── include/
│   ├── main.h / stepper.h / ssd1306.h / oled_debug.h / stm32f1xx_hal_conf.h
├── src/
│   ├── main.c / stepper.c / ssd1306.c / oled_debug.c
├── bluetooth_app/
│   ├── webserial.html / posctrl.html / index.html
├── servo_gimbal.hex / servo_gimbal_design.md / README.md
```

---

## 十、更新记录（节选）

- **舵机改造（当前主版本）**
  - 双轴 28BYJ-48 步进 → 双轴 S20F 舵机 + 空心滑环；最底层 GPIO 步进序列换成 TIM PWM 角度映射
  - Pan = **PB1 / TIM3_CH4**，Tilt = **PA11 / TIM1_CH4**，50Hz
  - 虚拟步进 `pos_steps`（0.1°/步）保留，PID / 位置闭环零改动
  - 软限位 Pan ±40° / Tilt ±30°；物理限位 Pan ±135° / Tilt ±80°
- **闭环可靠性改进**
  - 坐标系统一、Pan 方向反转（`g_pan_dir_invert=1`）、置信度门控 + 2 帧去抖 + 失冻结
  - 粘性命中锁（`HIT_RELEASE_PX=40`），根治“打到靶上停不下来”
  - 增益下调（PAN_GAIN 0.006 / TILT_GAIN 0.004）+ 减速区加宽，根治过冲
- **K230 端**（见 `../k230/README.md`）
  - 去 YOLO，改 cv2 轮廓 + find_blobs，帧率 1~2fps → ~35fps
  - 通信延迟降低，命中即锁存停机

---

## 附录：历史版本（28BYJ-48 步进）

本工程最早为 **双 ULN2003 + 28BYJ-48 步进电机** 方案（PA4~PA7 驱动 Pan、PB3~PB6 驱动 Tilt）。
改舵机方案后，步进相关引脚与序列已弃用，相关死宏保留在 `include/main.h` 中（不影响编译）。
如需了解改造前后的差异与滑环走线，见 **[`servo_gimbal_design.md`](./servo_gimbal_design.md)**。
