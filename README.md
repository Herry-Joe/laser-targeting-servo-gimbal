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
        │  矩形检测 : cv2.minAreaRect（旋转框，任意位置/倾斜）      │
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
| **用户按键 PA0**（功能键）| **PA0** | 内部上拉，按下=低；短按=自动打靶开关，长按(≥1.5s)=回中 |
| **用户按键 PA1**（循迹键）| **PA1** | 内部上拉，按下=低；短按=矩形循迹开关，长按(≥1.5s)=解锁命中锁存 |

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
│   ├── main.py                # 打靶追踪主程序（cv2 靶心 + find_blobs 激光 + 矩形识别）
│   ├── rect_detector.py       # 矩形检测模块（cv2.minAreaRect，任意位置/倾斜）
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

**命令帧（K230 → STM32，9 字节）：** K230 端按键触发，`0x3C 0x3C [CMD][PARAM] 0 0 0 0x01 0x01`。
- `CMD`：`0x01=AIM` `0x02=RECENTER` `0x03=BORDER`(外框循迹) `0x04=INNER`(内框循迹) `0x05=STOP`
- STM32 收到后**真正切换工作模式**：AIM/RECENTER/BORDER/INNER → 进入自动打靶（AUTO）；STOP → 停车并退回位置闭环。

### 4. 矩形帧（K230 → STM32，16 字节，新增 / E 题基础要求）

```
0x3C 0x3A [DXH][DXL] [DYH][DYL] [WH][WL] [HH][HL] [ANGH][ANGL] [FLAGS] [CRC8] 0x01 0x01
```

- **DX / DY**：矩形中心相对**靶心（画面坐标原点）**的像素偏移（显示空间 800×480；正 = 矩形在靶心右 / 下方）。
- **W / H**：矩形宽、高（像素，显示空间）。
- **ANG**：矩形倾斜角，以 0.1° 为单位的有符号整数（int16，范围约 ±450 = ±45°）。
- **FLAGS**：`bit0 = rect_valid`（已检测到矩形）、`bit1 = origin_valid`（靶心有效，此时偏移才有意义）。
- **CRC8**：poly 0x07，init 0x00，覆盖前面 11 字节数据 `[DXH..ANGL]`。

> STM32 在 USART3 中断里按 `g_k230_ftype` 分支解析（0=坐标帧 / 1=矩形帧），与原有坐标帧共存、互不干扰。解析成功后通过 `K230_SendRectTelemetry()` 以 `RECT,<dx>,<dy>,<w>,<h>,<ang>,<origin>` 行经蓝牙（USART2@9600）转发至上位机。

### 5. 矩形识别与 E 题基础要求（2023 全国大学生电子设计竞赛 E 题）

**目标**：用 K230 的 OpenCV 支持库识别靶板上**黑色外框矩形**（A4 边框），矩形摆放位置不固定、可倾斜，需具备任意位置检测能力；激光仍瞄准靶心，矩形只作为相对靶心的位置信息上报。

**实现要点**（`k230/rect_detector.py` + `k230/main.py`）：

1. **检测算法**：在 320×240 检测空间做灰度 → 高斯模糊(5×5) → 自适应阈值二值化（THRESH_BINARY_INV，block=11，C=5）→ 找轮廓 → 对每个轮廓取 `cv2.minAreaRect` 旋转外接矩形（天然支持倾斜/任意位置）。
2. **筛选**：面积 ≥ 200 像素，宽高比 0.25~4.0；按「面积 × 1/(1+|宽高比−1|)」打分取最优，角度归一化到 [−45°,45°)。
3. **坐标系**：以已识别的**靶心**为坐标原点（非画面中心），矩形中心偏移 = 矩形中心 − 靶心（右/下为正），并做 EMA 平滑（`RECT_SMOOTH_ALPHA = 0.30`）提升稳定性。
4. **回退路径**：若 cv2 不可用（非 OpenCV daily build 固件），自动回退到 CanMV `img.find_rects()` 慢速路径（1~2fps），保证功能可用。
5. **OSD**：画面叠加旋转矩形框（紫色）、靶心十字（绿）、矩形中心圆点、偏移向量线及 `RECT:(dx,dy) WxH ang` 文本，便于现场调试。

**上位机显示**（`stm32_servo_gimbal/bluetooth_app/webserial.html`，新增「矩形识别」卡片）：
- 解析 `RECT,` 遥测行，以画布中心为靶心原点，按缩放比例绘制紫色旋转矩形与青色偏移向量；
- 实时显示 `dx / dy / W / H / 角度 / 靶心有效` 读数与 `rect_valid` 状态徽标。

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

### 硬件按键控制（PA0 / PA1，板载独立按键）

当串口/上位机控制偶发不可靠时，PA0/PA1 提供确定的硬件控制（上拉输入，按下=低电平）：

| 按键 | 短按 | 长按（按住 ≥1.5s）|
|------|------|------|
| **PA0（功能键）** | 自动打靶 **开 / 关**（AUTO ↔ POSCTRL）| **回中**：目标清零 + 释放命中锁存 + PID 复位 |
| **PA1（循迹键）** | 矩形循迹 **开 / 关**（TRACE ↔ 停止/位置闭环）| **解锁命中锁存**（同 `u` 命令）|

- 进入 AUTO 后，STM32 自动等待 K230 的有效坐标帧驱动云台；K230 命令帧（AIM/RECENTER/BORDER/INNER）也会直接切到 AUTO。
- 手动「停止」会置 `g_auto_enabled=0`，此后即便 K230 持续发坐标帧也不会被抢占，直到再次 PA0 短按或收到新的 K230 命令帧才恢复自动打靶。

### 串口功能切换说明（已修复）

- **AUTO 模式下不再一刀切忽略串口命令**：仅拦截会抢占电机的手动步进命令（`p/t/c/a/s/f/h/d` 等），放行 `stop` / `m` / `u` / `home` / `trace` / `k` / `pid` / `g` / `x` / `y` 等安全控制命令，因此上位机/串口可正常在 AUTO 下切换功能。
- K230 命令帧（AIM/RECENTER/BORDER/INNER/STOP）现在**真正切换 `g_gimbal_mode`**，不再只是更新任务 ID。

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
| 速度指令 EMA（加速度限幅）| 0.55/0.45 | 平滑速度变化，抑制矩形拐角处方向突变导致的过冲 |
| 循迹降速系数 | 0.7× | BORDER/INNER 任务时降低最高速度，让激光更贴线、减少冲出线外 |

---

## 八、已知限制 / 备注

- 仓库根目录若残留 `stm32_uln2003_28byj48_demo/` 目录，为早期 **28BYJ-48 步进版**遗留，已被 `.gitignore` 忽略，不影响本仓库，可手动删除。
- `k230/deploy_config.json` 是早期 YOLO 方案的历史配置；当前 `main.py` **不使用模型**，改由 cv2 轮廓 + find_blobs 实现，无需 kmodel。
- `servo_gimbal.hex` 为编译产物，修改源码后请用上面的 `objcopy` 命令重新生成。

---

## 九、License

本仓库代码按 **MIT** 许可证开源（如需其他协议请自行调整 `LICENSE` 文件）。
