# 项目全面检查报告

> 日期: 2026-07-18  
> 项目: 28BYJ-48 微型减速步进电机云台 - 打靶

---

## 一、目录结构与文件组织问题

### ⚠️ P0 - 目录名与硬件不符

```
stm32_uln2003_28byj48_demo/   ← 实际已切到 ULN2003，v4.8 就换了
```

README 明确写 "v4.8 (切 ULN2003)"，但目录名还挂着 TB6612。新接手的人会以为这项目还是用 TB6612 H 桥，导致接线/排查浪费大量时间。

### ⚠️ P1 - K230 代码两份副本且版本不同

| 位置 | 版本 | 说明 |
|------|------|------|
| `k230/main.py` | v4.15 | 最新版，有多阈值激光检测 |
| `stm32_uln2003_28byj48_demo/k230_main.py` | v4.11 | 旧版，放在 STM32 目录下 |

两个文件内容不同，不知道哪个是"真"的。多人协作极大概率改错文件。

### ⚠️ P1 - 双份 firmware.hex

- **根目录**: `firmware.hex` (139KB)
- **子目录**: `stm32_uln2003_28byj48_demo/firmware.hex`

hex 文件是构建产物，不应该同时存在两份也不应该被 git 追踪。

### ⚠️ P2 - 架构评审报告已过时

`comms_architecture_review.md` 是针对旧代码（v4.5）写的，提到的问题（如"控制帧无校验和"）在实际代码中已经修复了（有 CRC8）。这份文档现在只会误导人。

### 🟢 正常 - .workbuddy/ 内存文件

`.workbuddy/memory/` 下有 4 天的工作日志（07-15 ~ 07-18），有延续性，不错。

---

## 二、构建配置问题

### ⚠️ P2 - .gitignore 不完整

根目录 `.gitignore` 缺少：
```gitignore
firmware.hex        # 构建产物不应追踪
*.pyc               # 已在，但还需忽略
```

目前 `firmware.hex` 和 `.pio/build/` 下的大量 `.o` 文件虽然被子目录 `.gitignore` 挡住了，但根级别的 ignore 也应该覆盖。

### ✅ 正常

`platformio.ini` 配置完整合理，注释详细，烧录方式注释清晰。

---

## 三、代码 BUG（运行时）

### 🔴 P0 - `I` 命令永远切不了 Tilt 方向反转

**位置**: `main.c:979`

```c
switch (*p | 0x20) {  // 转小写
    // ...
    case 'i':   // Pan 反转 (1017行)
    case 'I':   // Tilt 反转 (1022行)  ← 死代码！永远进不来
```

`*p | 0x20` 把输入字符一律转成**小写**，所以无论发 `I` 还是 `i`，switch 值都是 `'i'`，`case 'I'` 永远不会命中。**I 命令功能不可用。**

### 🔴 P0 - OLED mode_names 越界

**位置**: `main.c:1896`

```c
static const char *mode_names[6] = {
    "PAN", "TILT", "SWEEP", "SERIAL", "DANCE", "AUTO"
};
// ...
info.mode = mode_names[g_gimbal_mode];  // 1903行
```

`GimbalMode` 有 8 个值（`GIMBAL_POSCTRL=6`, `GIMBAL_TRACE=7`），但数组只有 6 个元素。当 `g_gimbal_mode=6` 或 `7` 时 → **数组越界，OLED 显示乱码甚至 HardFault**。

### 🟡 P1 - 位置闭环与 K230 自动模式的 PID 控制器重叠

`main.c` 中同时存在两个 PID 控制系统：
1. **K230_PID** (速度控制器, v5.0) — 用于 `GIMBAL_AUTO` 模式
2. **PosPID** (位置闭环控制器) — 用于 `GIMBAL_POSCTRL` / `GIMBAL_TRACE` 模式

两套参数不同、结构不同、调优方式不同，且共用同一组电机（`g_motor_pan`/`g_motor_tilt`），切换模式时可能出现状态残留。

### 🟡 P1 - 舞蹈模式硬编码速度超出限幅

舞蹈模式多个 phase 直接将 `step_delay_us` 设为 1000~1200，但 `STEPPER_MIN_DELAY_US = 5000`：
```c
Stepper_SetSpeed(&g_motor_pan, 1200);  // main.c:1752
Stepper_SetSpeed(&g_motor_tilt, 1500); // main.c:1753
```
`Stepper_SetSpeed()` 内部会 clamp 到 `[5000, 20000]`，所以实际不会低于 5000，但舞蹈效果和注释不符（注释说"快速"但实际还是 5ms/步）。

---

## 四、代码架构与可维护性问题

### ⚠️ P1 - main.c 2345 行，巨无霸文件

职责包含但不限于：
- 系统初始化（时钟/GPIO/串口/定时器）
- 串口命令解析（80+行 switch-case）
- 位置 PID 闭环控制
- K230 坐标帧解析 + 卡尔曼滤波
- PID 速度控制器 (K230_PID 全套)
- 按键扫描 + 8 种模式切换
- 舞蹈模式（6 阶段状态机）
- OLED 信息组织
- 蓝牙非阻塞发送 + 链路管理
- 回程差补偿
- 矩形循迹测试
- 遥测发送

建议拆分为：`pid.c`, `k230_protocol.c`, `gimbal_mode.c`, `bluetooth.c`, `telemetry.c`

### ⚠️ P1 - 全局变量泛滥

粗略统计有 **50+ 个全局/静态变量**：`g_motor_pan`, `g_motor_tilt`, `g_pid_pan`, `g_pid_tilt`, `g_pos_pid_pan`, `g_pos_pid_tilt`, `g_pos_target_pan`, `g_pos_target_tilt`, `g_k230_err_x`, `g_k230_err_y`, `g_k230_fstate`...

全为 `static` 全局，耦合度高，难以单元测试。

### ⚠️ P1 - K230_ControlAxis 函数过于复杂

```c
static void K230_ControlAxis(StepperMotor *motor, K230_PID *pid,
                              int16_t err, Kalman1D *kf,
                              uint8_t invert, const char *axis_label,
                              float *out_filter, int16_t deadzone_px,
                              int32_t pos_limit_steps, float max_speed_scale)
```

10 个参数，内部完成：卡尔曼滤波 → PID 更新 → 方向反转 → 减速区整形 → 速度映射 → 硬限位 → EMA 滤波 → 调试打印，**一个函数做太多事了**。

### ⚠️ P2 - `unsigned long` vs `%lu` 混用

代码中有些地方用 `(unsigned long)` 强制转换 + `%lu` 格式化（正确），有些直接用 `%d` 传 `uint32_t`（可能出警告），风格不统一。

---

## 五、文档与注释问题

### ⚠️ P2 - 注释与代码不一致

- main.c 文件头注释说"硬件连线 (ULN2003 驱动板)"但 GPIO 初始化注释还有"原 STBY"等 TB6612 残留
- OLED 调试注释说有 5 行 + 误差条，实际代码画了 8 行
- `oled_debug.h` 注释的布局（8 行）与 `oled_debug.c`（6 行）不一致

### ✅ 正常

大部分函数有关键注释，步进序列有表格说明，平台配置有详细解释，这点值得肯定。

---

## 六、Git 问题

### ⚠️ P2 - 只有 5 个提交

```
c213b41 蓝牙缓冲隔离 + 菜单版上位机
891bb5c 修复 printf float 不显示 + AUTO 模式锁定
14f1e6b CH340 非阻塞修复 + 矩形循迹测试模式
e77810f 回程差补偿 + 串口调参 backlash 命令
ce1a418 (初始) 完整位置闭环 + K230 多阈值跟踪
```

说明 git 仓库是在项目后期才初始化的，之前的大量开发历史丢失了。后续建议频繁提交，每个功能点独立 commit。

---

## 七、整改建议优先级

| 优先级 | 事项 | 影响 |
|--------|------|------|
| **🔴 立即** | 修复 `case 'I'` 死代码 → Tilt 方向反转可用 | Tilt 反转功能失效 |
| **🔴 立即** | 修复 OLED `mode_names` 数组越界 | 避免 HardFault |
| **🔴 立即** | 删除 `stm32_uln2003_28byj48_demo/k230_main.py`，统一用一个 K230 入口 | 改错代码风险 |
| **✅ 已完成** | 目录已重命名为 `stm32_uln2003_28byj48_demo`（原 `stm32_tb6612_28byj48_demo`） | 避免硬件接错 |
| **🟡 尽快** | 删除双份 firmware.hex，加到 .gitignore | 避免混淆 |
| **🟡 尽快** | 更新或删除过时的 `comms_architecture_review.md` | 避免误导 |
| **🟢 建议** | 拆分 main.c 为多个模块 | 可维护性 |
| **🟢 建议** | 统一两个 PID 控制器为一个 | 减少冗余 |
| **🟢 建议** | 舞蹈模式速度不要硬编码低于 STEPPER_MIN_DELAY_US | 预期一致 |
| **🟢 建议** | 全局变量封装到结构体，减少耦合 | 可测试性 |
