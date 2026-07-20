# K230 视觉端（庐山派 / CanMV）激光打靶追踪

> K230 识别**靶心**与**红色激光光斑**，通过串口把“靶心相对画面中心的误差”发给 STM32 舵机云台。
> 主程序 `main.py` 当前为 **OpenCV + RVV 加速 / 高帧率版**（约 35fps，旧版仅 1~2fps）。

---

## 一、文件清单

| 文件 | 说明 |
|------|------|
| `main.py` | 打靶追踪主程序（当前版本，cv2 靶心轮廓 + find_blobs 激光）|
| `backup/main_yolo_backup.py` | 历史版本：YOLO 检测靶心（已弃用，仅作参考）|
| `backup/main_noyolo_backup.py` | 历史版本：去 YOLO 但仍在 800×480 跑 find_rects（帧率慢，已弃用）|
| `deploy_config.json` | 早期 YOLO 方案的 kmodel 部署配置，**当前 `main.py` 不使用模型**，可忽略 |

---

## 二、部署步骤

1. **固件要求**：K230 必须烧录**带 OpenCV 的 CanMV daily build**（K230 的 cv2 是部分实现、部分算子 RVV 加速）。若烧的是不带 cv2 的固件，`main.py` 会回退到 `find_rects`/`find_blobs` 慢速路径。
2. 把 `main.py` 复制到 K230 的 **SD 卡根目录**（覆盖原有 `main.py`），重启。
3. 用串口工具（如 Putty / 串口助手，波特率 115200）监视 K230 REPL，正常应看到：
   ```
   [CV] cv2 可用, 启用 OpenCV+RVV 加速路径
   === K230 激光打靶追踪 ===
   ```
   若看到 `[CV] cv2 不可用`，说明固件不含 cv2，需换 daily build。
4. 把激光对准靶面、靶心进入画面后，STM32 侧发 `k` 进入自动打靶即可。

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
