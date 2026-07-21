# K230 视觉端（庐山派 / CanMV）激光打靶追踪

> K230 识别**靶心**与**红色激光光斑**，通过串口把“靶心相对画面中心的误差”发给 STM32 舵机云台。
> 主程序 `main.py` 当前为 **OpenCV + RVV 加速 / 高帧率版**（约 35fps，旧版仅 1~2fps）。

---

## 一、文件清单

| 文件 | 说明 |
|------|------|
| `main.py` | **当前版本 v5.10（自包含，单文件）**：四线段外框循迹(BorderLineTrace) + 内框密集采样(RectTrajectory) + LAB 激光检测 + 矩形周长循迹（RECENTER / TRACE_BORDER / TRACE_INNER），三通道传感器。**不依赖任何本地模块，部署只需拷这一个文件。** |
| `rect_detector.py` | **已弃用 / 历史**：早期"基础要求"矩形检测模块（cv2.minAreaRect 旋转框）。v5.2 已内联矩形检测逻辑，不再 import 本文件。保留作参考。 |
| `backup/main_rect_legacy.py` | 历史版本：rect_detector 集成版 main.py（会 `from rect_detector import ...`，缺模块即 `ImportError`）|
| `backup/main_yolo_backup.py` | 历史版本：YOLO 检测靶心（已弃用，仅作参考）|
| `backup/main_noyolo_backup.py` | 历史版本：去 YOLO 但仍在 800×480 跑 find_rects（帧率慢，已弃用）|
| `deploy_config.json` | 早期 YOLO 方案的 kmodel 部署配置，**当前 `main.py` 不使用模型**，可忽略 |

---

## 二、部署步骤

1. **固件要求**：K230 必须烧录**带 OpenCV 的 CanMV daily build**（K230 的 cv2 是部分实现、部分算子 RVV 加速）。若烧的是不带 cv2 的固件，`main.py` 会回退到 `find_rects`/`find_blobs` 慢速路径。
2. 把 `main.py` 复制到 K230 的 **SD 卡根目录**（覆盖原有 `main.py`），重启。
   > ⚠️ **部署坑（踩过）**：CanMV 运行 `main.py` 时所有被 `import` 的**本地 `.py` 模块也必须放在 SD 卡根目录**，否则启动即 `ImportError: no module named 'xxx'`（报错行号指向 `import` 那一行，容易误以为是代码写错）。v5.2 是**单文件自包含**版本，只拷 `main.py` 即可；若以后换回多文件版本（如用到 `rect_detector.py`），务必把每个 `.py` 都拷到 SD 卡根目录。
3. 用串口工具（如 Putty / 串口助手，波特率 115200）监视 K230 REPL，正常应看到：
   ```
   [CV] cv2 可用, 启用 OpenCV+RVV 加速路径
   === K230 激光打靶追踪 ===
   ```
   若看到 `[CV] cv2 不可用`，说明固件不含 cv2，需换 daily build。
4. 把激光对准靶面、矩形进入画面后，STM32 侧按 **PA1** 进入矩形循迹：STM32 保持 AUTO 模式（按 K230 误差帧驱动云台），同时向 K230 发 `0x3C 0x3C [TRACE_BORDER]` 命令帧；K230 解析该命令后切到矩形循迹任务（task_id=2/3），沿矩形周长走 setpoint，并在接近四个角时置 **FLAG bit5 = NEAR_CORNER** 通知 STM32 减速防过冲。PA0 短按 = 回中 / 停止，长按 = 急停。

---

## 三、检测原理

### 靶心（目标）检测
- 传感器出 800×480 RGB888（显示清晰）。
- `to_numpy_ref()` 零拷贝取 RGB888 → `cv2.resize` 缩到 **320×240**（RVV 加速，计算量降到 1/6.25）。
- `cv2` 灰度 + 高斯 + 自适应阈值 → `findContours` 取最大轮廓：
  - 优先 **4 边形中心**（方框靶，approxPolyDP）；
  - 否则用轮廓**矩中心**（同心圆靶）。
- 检测坐标放大回 800×480 显示空间发包。

### 激光（红色光斑）检测
- 主路径：cv2 HSV `inRange`（小图）做可选加速。
- **兜底（已验证可靠）**：CanMV 原生 `img.find_blobs()` + **5 级渐进式 LAB 阈值表**（v5.7 新增 DIM 档，按优先级从高到低逐档放宽）：
  1. `RED_LASER_LAB_DIM` **`(L:58~84 / A:16~37 / B:-10~9)`** — 用户实测：激光**变暗/变小**时也能捕获（优先）
  2. `LASER_LAB_NARROW` `(L:56~100 / A:39~61 / B:2~18)` — 原默认窄范围
  3. `LASER_LAB_CENTER` 中范围
  4. `LASER_LAB_WIDE` 宽范围
  5. `LASER_LAB_HOT` 热区兜底
  - 激光掩膜最小像素 `LASER_MIN_PX` 由 3 降到 **2**（v5.7），激光变小也识别。
- 实测 LAB 阈值写在 `main.py` 顶部 `RED_L_*/RED_A_*/RED_B_*` 等常量，可按现场光环境微调。

### 矩形检测（E 题基础要求）

识别靶板**黑色外框矩形**（A4 边框），位置不固定、可倾斜，以**靶心为坐标原点**上报相对偏移。逻辑封装在 `rect_detector.py` 的 `detect_rect(small, img=None)`：

> **v5.10 四线段外框循迹（当前）**：
> - **外矩形（BorderLineTrace 四线段）**：画面边缘内缩 5% 的轴对齐固定矩形，循迹改为 **4 条独立直线段**（上→右→下→左），每段只动一个轴（Pan 或 Tilt 单轴）。启动时先从当前位置**直线对齐到最近角点**，再逐边画线。相比 v5.9 密集采样：更稳更快、无拐角过冲、K230 计算量更低。
> - **LOST 回退优化**：激光丢失时不再反转轨迹方向（避免死循环式 LOST/recovery），改为保持设定点不动，等激光回来后重同步到最近角点。
> - **内矩形（RectTrajectory 优化）**：取消拐角停留(`CORNER_DWELL_MS=0`)、适度放宽减速区，画线更流畅。
> - **速度调优**：外框速度 `OUTER_TRACE_SPEED` 从 80 降至 50（减少 LOST 频率）；STM32 端 PID 增益降低（Pan 0.006→0.005, Tilt 0.004→0.003），`TRACE_SPEED` 地板从 0.60 降至 0.45。
> - Task2 `TRACE_BORDER` 走**外框（四线段）**；Task3 `TRACE_INNER` 走**内框（胶带密集采样）**。

- **入口**：`main.py` 主循环每帧在 320×240 检测空间调用 `detect_rect(small)`（有 cv2 走 cv2 路径；无 cv2 传 `None` 走 `find_rects` 回退）。
- **cv2 路径**（`detect_rect_cv2`）：灰度 → `GaussianBlur(5×5)` → `adaptiveThreshold(THRESH_BINARY_INV, blockSize=11, C=5)` → `findContours` → 对每个轮廓取 `cv2.minAreaRect` 旋转外接矩形 → 过滤（面积 ≥ `RECT_MIN_AREA_CV`=200，宽高比 0.25~4.0）→ 按 `面积 × 1/(1+|宽高比−1|)` 打分取最优 → 角度归一化到 [−45°,45°)。
- **回退路径**（`detect_rect_legacy`）：cv2 不可用时用 CanMV `img.find_rects()`，帧率 1~2fps。
- **坐标系与平滑**：`main.py` 将检测坐标放大回 800×480 显示空间，做 EMA 平滑（`RECT_SMOOTH_ALPHA = 0.30`），并算 `rect_dx = 矩形中心 − 靶心`、`rect_dy`；仅当靶心有效时偏移有意义。
- **输出**：每 100ms（`RECT_FRAME_INTERVAL_MS`）发一次 16 字节矩形帧（见根目录 README 五.4），OSD 叠加紫色旋转框、绿十字靶心、偏移向量与文本。

> 常用阈值（`rect_detector.py` 顶部）：`RECT_ADAPT_BLK=11`、`RECT_ADAPT_C=5`、`RECT_MIN_AREA_CV=200`、`RECT_MIN_RATIO=0.25`、`RECT_MAX_RATIO=4.0`，可按边框尺寸/光照微调。

### 决策与抗抖
- **置信度门控**：靶心 + 激光均有效且连续 **2 帧**才驱动；任一丢失即 `valid=0`。
- **冻结**：`valid=0` 时 STM32 收到 `valid=0` 帧 → 停机 + 重置 PID，不急停、不误发大误差。
- **粘性命中锁**：误差 < `HIT_TOLERANCE_PX`（15px）判命中 → STM32 锁存停机；目标移动 > `HIT_RELEASE_PX`（40px）或丢帧才解锁重捕。
- 激光单帧位移限幅、EMA 平滑，避免抖动。

---

## 四、通信协议

坐标帧（K230 → STM32，10 字节）：

```
0x3C 0x3B [XH][XL] [YH][YL] [FLAG] [CRC8] 0x01 0x01
```

- **X / Y**：靶心相对画面中心误差（像素，显示空间 800×480；正 = 靶心在中心右侧 / 下方）
- **FLAG**：`bit0=valid`、`bit1=hit`、`bit2=hi_conf`、**`bit5=near_corner`（v5.7：接近矩形拐角，请求 STM32 减速防过冲）**
- **CRC8**：poly 0x07，覆盖 `[XH .. FLAG]`

命令帧（STM32 → K230，9 字节）：`0x3C 0x3C [CMD][PARAM] ...`（预留）。

> 协议、STM32 端参数与调参见根目录 **[`../README.md`](../README.md)** 与 **[`../stm32_servo_gimbal/README.md`](../stm32_servo_gimbal/README.md)**。

---

## 五、调试输出

K230 REPL 会周期打印（约每 30 帧）：

```
[TARGET] tgt=(484,220) valid=1
[LASER]  raw=1 lock=0 px=12 pos=(310,180)
```

- `px`：激光掩膜像素数；`px=0` 说明激光未入镜 / 太暗 / 阈值不匹配。
- `pos`：激光检测坐标（显示空间）。
- 若激光识别不稳定，调 `main.py` 顶部 `RED_L_*/RED_A_*/RED_B_*` 阈值或 `LASER_MIN_PX` / `LASER_WASHOUT_PX`。

---

## 六、帧率说明

- 当前版本（cv2 小图 + 原图 find_blobs）实测约 **35fps**。
- 若烧的是不带 cv2 的固件，会回退到 800×480 `find_rects` + `find_blobs`，帧率仅 **1~2fps**——务必使用带 OpenCV 的 CanMV daily build。
- 还觉得慢：可关 `to_ide=True`、或把 `DETECT_W/DETECT_H` 从 320×240 降到 160×120。

---

## 七、更新记录

- **v5.7（矩形循迹 / 抗过冲 / 抗亮度变化）**
  - **激光抗亮度变化**：LAB 阈值表升级为 5 级渐进式，新增 `RED_LASER_LAB_DIM (58,84,16,37,-10,9)` 作为最高优先级档，激光变暗/变小时优先捕获；`LASER_MIN_PX` 3→2。
  - **抗矩形识别小概率失误**：`RECT_HOLD_FRAMES` 5→6（检测丢失时保持上一框并外推）；`DETECT_INTERVAL` 2→3（矩形检测降频，让出算力给激光检测与串口传输，整体识别/传输频率更高、视觉闭环更流畅）。
  - **K230 解析 STM32 命令帧**：新增 `parse_host_command()`，识别 `0x3C 0x3C [CMD]`，响应 `TRACE_BORDER/TRACE_INNER/RECENTER/STOP/AIM`（PA1/PA0 按键驱动）。
  - **接近拐角减速**：循迹 setpoint 走到四个角附近（`d_corner < CORNER_SLOWDOWN_ARC`）时置 **FLAG bit5 = NEAR_CORNER**，通知 STM32 降速防过冲；直线段保持原速（提速）。
  - 通信坐标帧仍 `0x3C 0x3B ...`，FLAG 新增 bit5。

- **v5.8（循迹精度提升）**
  - **激光按圆形色块建模**：坐标取外接框几何中心（`_blob_geom_center`），替代原质心 `cx()/cy()`，对不对称光晕/卫星像素更鲁棒。
  - **检测门限拒反射误检**：`detect_laser` 非锁定分支（初始捕获 / 恢复中）若已有上次良好位置，要求候选在其 `LASER_TRACK_MAX_D` 内，否则判丢失——避免把远处反射/杂光当激光驱动云台冲飞（实测曾出现 700px 级假误差尖峰）。
  - 启动打印版本号同步为 v5.8。

- **v5.9（双矩形模型：外框固定 / 内框四点定位）**
  - **外矩形改为固定**：`fixed_outer_corners()` 取画面边缘内缩 5% 的轴对齐四角（左上→右上→右下→左下），不参与视觉检测；循迹外框时每个边只动一个轴（Pan/Tilt 单轴），更快更准，只需四点定位。
  - **内矩形改为视觉四点定位**：电工胶带矩形（黑胶带贴白 A4）由 cv2 `find_rect_corners` 检测 4 角并 EMA 吸附稳定，中心作“原点”。
  - **任务映射调整**：Task2 `TRACE_BORDER` → 走固定外框；Task3 `TRACE_INNER` → 走视觉检测的内框（胶带）。
  - **提速**：外框循迹速度 `OUTER_TRACE_SPEED`（默认 80，可用 `/data/ti_vision_tune.conf` 的 `outer_speed` 调）高于内框 `TRACE_SPEED`（40）。
  - 启动打印版本号同步为 v5.9；k230 与 stm32 两端版本号对齐为 v5.9。

- **v5.10（四线段外框循迹 + 稳定性提升）**
  - **BorderLineTrace 四线段循迹器**：外框从 `RectTrajectory` 密集采样改为 4 条独立直线段（上→右→下→左），每段只动单轴。启动时先直线对齐到最近角点再画线。无拐角减速/停留，自然过渡。
  - **LOST 回退不再反转方向**：保持设定点不动等激光回来，消除 LOST/reverse/LOST 死循环。
  - **内框优化**：取消拐角停留(`CORNER_DWELL_MS=0`)、放宽减速参数，画线更流畅。
  - **速度与 PID 调优**：`OUTER_TRACE_SPEED` 80→50；STM32 Pan 增益 0.006→0.005, Tilt 0.004→0.003；`TRACE_SPEED` 地板 0.60→0.45；`TRACE_NEAR_FLOOR` 0.06→0.03。
  - 启动打印版本号同步为 v5.10。

