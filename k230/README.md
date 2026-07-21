# K230 视觉端（庐山派 / CanMV）激光打靶追踪

> K230 识别**靶心**与**红色激光光斑**，通过串口把“靶心相对画面中心的误差”发给 STM32 舵机云台。
> 主程序 `main.py` 当前为 **OpenCV + RVV 加速 / 高帧率版**（约 35fps，旧版仅 1~2fps）。

---

## 一、文件清单

| 文件 | 说明 |
|------|------|
| `main.py` | **当前版本 v5.2（自包含，单文件）**：cv2 矩形角点检测 + LAB 激光检测 + 矩形周长循迹（RECENTER / TRACE_BORDER / TRACE_INNER），三通道传感器（YUV 显示 + 灰度矩形 + RGB565 激光）。**不依赖任何本地模块，部署只需拷这一个文件。** |
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
4. 把激光对准靶面、矩形进入画面后，STM32 侧发 `k` 进入自动打靶即可（v5.2 上电为 IDLE，需 STM32 进入 AUTO/K230 模式才会开始 nulling 误差；K230 按键发的是 `0x3C 0x3C` 命令帧，当前 STM32 固件未解析，仅作视觉循迹，STM32 始终按误差帧驱动云台）。

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
- **兜底（已验证可靠）**：CanMV 原生 `img.find_blobs()` + 用户实测 **LAB 阈值**：
  `L: 56~100 / A: 39~61 / B: 2~18`，在原图检测，坐标 ÷ 缩放比转到检测空间。
- 实测 LAB 阈值写在 `main.py` 顶部 `RED_L_MIN/MAX` 等常量，可按现场光环境微调。

### 矩形检测（E 题基础要求）

识别靶板**黑色外框矩形**（A4 边框），位置不固定、可倾斜，以**靶心为坐标原点**上报相对偏移。逻辑封装在 `rect_detector.py` 的 `detect_rect(small, img=None)`：

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
- **FLAG**：`bit0=valid`、`bit1=hit`、`bit2=hi_conf`
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
