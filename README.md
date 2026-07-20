# 激光打靶舵机云台（K230 视觉 + STM32 舵机云台）

> 基于 **K230（庐山派 / CanMV）视觉识别** + **STM32F103C8T6 舵机二维云台** 的激光自动打靶系统。
> K230 识别靶心与红色激光光斑，通过串口把“靶心相对画面中心的误差”发给 STM32；STM32 用速度控制器闭环驱动双轴舵机，把激光对准靶心即为命中。

---

## 一、系统架构

```
        ┌──────────────── K230（庐山派 / CanMV）────────────────┐
        │  摄像头 + 同轴红色激光器                                │
        │  靶心检测 : cv2 轮廓（优先方框 4 边，否则同心圆矩心）     │
        │  激光检测 : CanMV find_blobs（LAB 阈值，已验证可用）      │
        │  置信度门控 + 2 帧去抖 + 粘性命中锁                      │
        └───────────────┬ USART2 @115200 ───────────┬────────────┘
                        │ GPIO05(TX) → PB11          │ GPIO06(RX) ← PB10
                        ▼                            ▼
        ┌──────────────── STM32F103C8T6 ────────────────────────┐
        │  USART3 中断收误差帧（0x3C 0x3B ...）                   │
        │  速度控制器 : out = Kp·err + Kd·derr + 减速区线性收敛     │
        │  TIM3_CH4 (PB1)  → Pan  舵机 50Hz PWM                  │
        │  TIM1_CH4 (PA11) → Tilt 舵机 50Hz PWM                  │
        │  Pan 轴经空心滑环连续旋转                               │
        └───────────────────────────────────────────────────────┘
```

---

## 二、硬件清单

| 部件 | 型号 / 说明 |
|------|------------|
| 主控 MCU | STM32F103C8T6（Blue Pill）|
| 视觉模组 | K230 / 庐山派（CanMV 固件，**需带 OpenCV 的 daily build**）|
| 水平舵机 Pan | S20F 20kg·cm，270° 行程（0.5ms = 0°，2.5ms = 270°）|
| 俯仰舵机 Tilt | S20F 20kg·cm，180° 行程（0.5ms = 0°，2.5ms = 180°）|
| 滑环 | 空心滑环（≥6 路，过 Pan 中心轴，解决线束缠绕）|
| 激光器 | 红色激光模组，与摄像头同轴安装 |
| OLED | SSD1306（I2C，0x3C，重映射到 PB8/PB9）|
| 蓝牙（可选）| JDY-31（USART2 @9600）|

---

## 三、引脚定义（STM32）

| 信号 | 引脚 | 复用 / 说明 |
|------|------|------------|
| **Pan 舵机 PWM** | **PB1** | TIM3_CH4，50Hz |
| **Tilt 舵机 PWM** | **PA11** | TIM1_CH4，50Hz |
| K230 串口 TX | PB10 | USART3_TX → K230 RX (GPIO06) |
| K230 串口 RX | PB11 | USART3_RX ← K230 TX (GPIO05) |
| 调试串口 TX / RX | PA9 / PA10 | USART1 @115200 |
| 蓝牙 TX / RX | PA2 / PA3 | USART2 @9600（JDY-31）|
| OLED SCL / SDA | PB8 / PB9 | I2C1 重映射，地址 0x3C |
| 按键 UP/DOWN/DIR/MODE | PB12/PB13/PB14/PB15 | 内部上拉，另一端接 GND |

---

## 四、目录结构

```
.
├── stm32_servo_gimbal/        # STM32 舵机云台固件（PlatformIO）
│   ├── src/main.c             # 主逻辑：串口解析、速度控制器、位置闭环、命中锁
│   ├── include/main.h         # 引脚与参数宏
│   ├── servo_gimbal.hex       # 编译产物（可直接烧录，亦可用 objcopy 重新生成）
│   ├── servo_gimbal_design.md # 舵机改造方案与滑环走线
│   ├── bluetooth_app/         # Web Serial / Web Bluetooth 调试面板
│   └── README.md              # STM32 端详细说明
│
├── k230/                      # K230 视觉端程序（CanMV / MicroPython）
│   ├── main.py                # 打靶追踪主程序（cv2 靶心 + find_blobs 激光）
│   ├── backup/                # 历史版本备份（YOLO 版 / 非 YOLO 版）
│   └── README.md              # K230 端说明
│
└── README.md                  # 本文档（项目总览）
```

---

## 五、快速开始

### 1. 编译并烧录 STM32 固件

环境：VS Code + PlatformIO 插件（自动拉取 arm-none-eabi-gcc 与 STM32CubeF1 HAL）。

```bash
cd stm32_servo_gimbal
PYTHONPATH="" pio run                       # 编译（按本机约定加 PYTHONPATH=""）
# 生成 HEX（objcopy 不支持中文路径，先输出到英文临时路径再拷回）
arm-none-eabi-objcopy -O ihex .pio/build/bluepill_f103c8/firmware.elf servo_gimbal.hex
```

烧录：
- **ST-Link**：`pio run -t upload`
- **串口 ISP**：改 `platformio.ini` 的 `upload_protocol = serial`，BOOT0=1 后烧录

> 也可直接用仓库里的 `servo_gimbal.hex` 烧录（STM32CubeProgrammer / 立创 Web Flasher 等）。
> 注意部分烧录工具只认 TI-TXT 等特定格式，必要时用 `objcopy -O ihex` 或对应格式转换。

### 2. 部署 K230 程序

1. 确认 K230 烧录了**带 OpenCV 的 CanMV daily build 固件**（否则 cv2 不可用，会回退到慢速 find_rects 路径，帧率仅 1~2fps）。
2. 把 `k230/main.py` 复制到 K230 的 SD 卡根目录（覆盖原有 `main.py`），重启。
3. 串口监视 K230 REPL，应看到：
   ```
   [CV] cv2 可用, 启用 OpenCV+RVV 加速路径
   === K230 激光打靶追踪 ===
   ```
4. 把激光对准靶面、靶心进入画面后，在 STM32 调试串口发 `k` 进入自动打靶模式即可。

### 3. 通信协议

**坐标帧（K230 → STM32，10 字节）：**
```
0x3C 0x3B [XH][XL] [YH][YL] [FLAG] [CRC8] 0x01 0x01
```
- **X / Y**：靶心相对画面中心的误差（像素，显示空间 800×480 放大后；正 = 靶心在中心右侧 / 下方）
- **FLAG**：`bit0=valid`（靶心+激光有效）、`bit1=hit`（命中，误差 < 容差）、`bit2=hi_conf`（高置信）
- **CRC8**：poly 0x07，覆盖 `[XH .. FLAG]`

**命令帧（STM32 → K230，9 字节）：** `0x3C 0x3C [CMD][PARAM] ...`（预留，主要用于解锁等）。

---

## 六、使用说明

上电默认进入**位置闭环（POSCTRL）**模式。常用串口命令（调试串口 115200 或蓝牙 9600，回车结束）：

| 命令 | 功能 |
|------|------|
| `k` | 进入 K230 自动打靶模式（锁定，忽略误触）|
| `m` | 退回位置闭环模式 |
| `x<角度>` / `y<角度>` | 设定 Pan / Tilt 目标角度（°）|
| `i` / `I` | 反转 Pan / Tilt 机械方向 |
| `u` | 解除命中锁存（命中后停稳，目标移动 > 40px 才自动重捕）|
| `g p kp 0.005` | 实时调 AUTO 速度控制器增益（断电不保存）|
| `pid p kp 0.001` | 调位置环 PID |

**调参口诀（AUTO 速度控制器）：** 追得慢 → 提 `g p kp`；仍过冲 → 调大减速区 `g p ki` 或加阻尼 `g p kd`。

---

## 七、关键参数（`stm32_servo_gimbal/src/main.c`）

| 参数 | 值 | 说明 |
|------|----|------|
| PAN_GAIN / TILT_GAIN | 0.006 / 0.004 | 速度增益（降增益抑过冲）|
| CTRL_KD | 0.0015 | 阻尼 |
| PID_DEADZONE | Pan 3 / Tilt 4 px | 死区 |
| PAN_MAX_STEPS / TILT_MAX_STEPS | 400 / 300 | 软限位 ±40° / ±30° |
| POS_LIMIT Pan / Tilt | ±1350 / ±800 步 | 物理限位 ±135° / ±80°（S20F 行程）|
| HIT_RELEASE_PX | 40 | 命中锁存后误差超此值才重新捕获 |

---

## 八、已知限制 / 备注

- 仓库根目录若残留 `stm32_uln2003_28byj48_demo/` 目录，为早期 **28BYJ-48 步进版**遗留，已被 `.gitignore` 忽略，不影响本仓库，可手动删除。
- `k230/deploy_config.json` 是早期 YOLO 方案的历史配置；当前 `main.py` **不使用模型**，改由 cv2 轮廓 + find_blobs 实现，无需 kmodel。
- `servo_gimbal.hex` 为编译产物，修改源码后请用上面的 `objcopy` 命令重新生成。

---

## 九、License

本仓库代码按 **MIT** 许可证开源（如需其他协议请自行调整 `LICENSE` 文件）。
