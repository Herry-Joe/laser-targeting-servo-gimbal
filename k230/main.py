# -*- coding: utf-8 -*-
# =============================================================
# 庐山派 K230 - 2023 电赛 E 题 · 激光打靶矩形循迹 v5.2
#
# === 核心改动 (v5.2) ===========================================
#   1. 开机自动回中 (TASK_BOOT): 上电即进入, 把激光对到矩形中心,
#      |err|<8 持续 500ms → 切 TASK_IDLE. 5s 超时每 2s 重试.
#      BOOT 期间按键无效, 必须等 BOOT 完成.
#   2. 激光零丢失: 多级 LAB 阈值回退 (NARROW→CENTER→WIDE→HOT),
#      ROI 失败立即全局兜底 (移除帧冷却). 丢失外推 3 帧.
#      丢失后 desired_point 保持, 输出 FLAG bit4 (HOLD),
#      STM32 收到 HOLD 维持速度 500ms 再停车.
#   3. 矩形稳定化: sort_corners_clockwise 强制顺时针排序,
#      RECT_HOLD_FRAMES 6→5, RECT_SNAP_ALPHA 0.4→0.35.
#   4. OSD BOOT 状态显示 + DEBUG_PRINT 开关 (长按 2s 切换).
#
# === 核心改动 (v5.0) ===========================================
#   1. 舍弃 YOLO, 改用 CanMV OpenCV (cv2) 做矩形检测:
#        二值化(THRESH_BINARY_INV) → GaussianBlur → Canny(30/90)
#        → findContours → approxPolyDP(4-point) → 评分选最优
#      兜底: 若 cv2 不可用/异常, 自动回退到 img.find_rects (MaixPy C).
#   2. 题目核心 = 矩形识别 + 激光点识别. 两者都做且都画到 OSD 上:
#        - 矩形角点用绿色四边形描出
#        - 矩形中心 (A4 纸中心) 用黄色十字标记 (即"原点")
#        - 激光点用红色十字标记
#        - 当前设定点 (循迹目标) 用蓝色十字标记
#   3. 原点 = A4 纸中心 = 检测到的矩形几何中心 (proc 坐标).
#      Task1 (RECENTER): 设定点 = 内框(胶带)中心 → 误差 = 中心 - 激光 → 对中.
#      Task2 (TRACE_BORDER): 设定点沿"外框"周长走. 外框 = 画面边缘内缩 5% 的
#            固定轴对齐矩形 (每边只动一个轴, 更快更准, 四点定位即可).
#      Task3 (TRACE_INNER): 设定点沿"内框"周长走. 内框 = 视觉四点定位检测到的
#            电工胶带矩形 (黑胶带贴白 A4 纸).
#   4. 视觉闭环: 误差 = 设定点 - 激光, 通过 UART 发给 STM32 PID 云台,
#      STM32 nulling 误差 → 激光跟随设定点移动 → 自动画出矩形.
#
# === 通信 (与 STM32 firmware.hex 一致) =======================
#   UART2 @115200, GPIO05(TX)→STM32 PB11, GPIO06(RX)←STM32 PB10
#   坐标帧: 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC8] 0x01 0x01  (10字节)
#           FLAG: bit0=valid, bit1=hit(完成), bit2=hi_conf
#   命令帧: 0x3C 0x3C [CMD][PARAM] 0x00 0x00 0x00 0x01 0x01  (9字节)
#           CMD: 0x01=AIM 0x02=RECENTER 0x03=TRACE_BORDER
#                  0x04=TRACE_INNER 0x05=STOP
# v5.2 新增 FLAG 位:
#   bit3 (0x08) = BOOT 模式 (开机回中阶段)
#   bit4 (0x10) = HOLD (激光丢失, 保持最后速度)
#
# === 按键 (Pin 53) ===========================================
#   单击 → 任务2 TRACE_BORDER (CMD 0x03)   [v5.2: 原 RECENTER 移除, 改由 BOOT 自动]
#   双击 → 任务3 TRACE_INNER  (CMD 0x04)
#   长按 < 1.2s → STOP            (CMD 0x05)
#   长按 >= 2s   → 切换 DEBUG_PRINT 开关  (BOOT 完成后生效)
#   注意: BOOT 模式期间按键无效, 必须等 BOOT 完成
#
# === 三通道传感器 =============================================
#   CHN0: 800x480 YUV420SP  → LCD 显示 (硬件直通)
#   CHN1: 320x192 GRAYSCALE → cv2 矩形检测
#   CHN2: 320x192 RGB565    → 激光点 LAB 检测
# =============================================================

import time
import os
import gc
import math
import image

from machine import UART
from machine import FPIOA
from machine import Pin
from media.sensor import *
from media.display import *
from media.media import *

# --- CanMV OpenCV (官方 cv2 绑定, 庐山派固件支持) ---
try:
    import cv2
    HAS_CV2 = True
except BaseException:
    cv2 = None
    HAS_CV2 = False


# ========================= 用户参数 =========================

FRAME_W = 800       # 显示通道宽度
FRAME_H = 480       # 显示通道高度
PROC_W  = 320       # 处理通道宽度 (矩形 + 激光检测)
PROC_H  = 192       # 处理通道高度
# OSD 校准偏移(显示像素): 若 OSD 框与目标错开, 上板量出偏移填入(可由 .conf 调)
OSD_X_OFFSET = 0
OSD_Y_OFFSET = 0

# 摄像头参数 (LCKFB K230 板级实测)
SENSOR_ID         = 2
SENSOR_INPUT_W    = 1280
SENSOR_INPUT_H    = 720
SENSOR_FPS        = 90
SENSOR_ANALOG_GAIN = 20
IDE_JPEG_QUALITY  = 50

# UART 接线: K230 GPIO5/6 <-> STM32 PB10/PB11
UART_TX_PIN   = 5
UART_RX_PIN   = 6
UART_BAUDRATE = 115200

# 图像翻转 (若画面上下颠倒/左右镜像, 改这里)
HMIRROR = False
VFLIP   = False

# ----- 矩形检测 (cv2, 灰度 320x192) -----
# 流水线: 灰度 → [二值化] → GaussianBlur → Canny → findContours
#          → approxPolyDP(4 点) → 评分选最优
USE_CV2 = True
CANNY_THRESH1     = 30   # 低阈值 (暗光下边缘淡, 比旧默认 50/150 更松、多抓边)
CANNY_THRESH2     = 90   # 高阈值
APPROX_EPSILON    = 0.04 # approxPolyDP 精度 = APPROX_EPSILON * 周长
GAUSSIAN_BLUR_SIZE = 3   # 高斯模糊核 (奇数)
# 兜底 find_rects 灰度阈值 (仅 cv2 不可用时走)
RECT_THRESHOLD    = 6000
# v5.1: 最小面积 500→4000 (过滤噪点小矩形, 320x192 画面上至少 ~63x63 像素)
RECT_MIN_AREA     = 4000
RECT_MAX_AREA     = PROC_W * PROC_H * 7 // 10
# v5.1 新增: 最小边长过滤 (短边 <40px 的小框直接拒收, 抗噪点)
RECT_MIN_EDGE      = 40
RECT_MAX_ASPECT   = 4.0
# v5.1: A4 纸长宽比 1.0→1.414 (A4 纸标准比例 √2), 让真正的 A4 矩形得分更高
RECT_TARGET_ASPECT     = 1.414   # "最标准"偏好: A4 纸长宽比 √2
# v5.1: 长宽比权重 0.02→0.05 (加强筛选, 拒绝非 A4 比例的杂框)
RECT_REGULARITY_WEIGHT = 0.05
# 二值化预处理: 暗框白纸 → 取低灰度段; 光照变化大就调 BINARY_LO/HI
RECT_BINARIZE = True
BINARY_LO = 0
BINARY_HI = 110
# 时间一致性权重 (抗误切换/抗丢帧): v5.1 0.2→0.4 (增强时序稳定性, 减少误切换)
RECT_TRACK_WEIGHT = 0.4
# 矩形边缘留白 (px): 矩形触碰画面边缘时视为不完整 → 拒识
RECT_EDGE_MARGIN = 0
# 边框吸附速度 (锁定后, 每帧把锁定角点朝真实角点 EMA 贴合)
RECT_SNAP_ALPHA         = 0.35   # 贴合速率 (0~1) [v5.2: 0.4→0.35]
RECT_MOVE_PX_PER_FRAME  = 1.5   # 边框中心帧间位移超过此值 → 判"在动", 强制每帧检测
RECT_HOLD_FRAMES         = 6     # [v5.7] 检测丢失时保持上一框并外推最多这么多帧 (抗小概率失误)
RECT_HOLD_EXTRAPOLATE    = True

# ----- 循迹参数 (矩形周长行走) -----
TRACE_ENABLE = True
# TRACE_REFERENCE = "laser": 误差 = 设定点 - 激光 (视觉闭环)
#                    "center": 误差 = 设定点 - 画面中心 (无激光时)
TRACE_REFERENCE = "laser"
TRACE_SPEED = 40.0           # 内框(胶带)周长行走速度 (proc px/s)
TRACE_LOOP  = "repeat"        # "once" 走一圈停 / "repeat" 循环 / "pingpong" 往返
TRACE_DIR   = 1               # 行走方向 +1 / -1
TRACE_SAMPLE_SPACING = 3.0    # 周长密集采样间距 (proc px)
TRACE_LOCK  = True            # 锁定后不重建轨迹 (除非边框漂移)
TRACE_REBUILD_DRIFT = 0.12    # 中心移动超过此比例才重建轨迹
# ----- 外矩形 (固定) / 内矩形 (视觉检测) -----
# 外矩形 = 画面边缘向内收缩 OUTER_INSET_RATIO, 轴对齐 (不依赖视觉检测).
#   循迹外框时每边只动一个轴 (Pan 或 Tilt 单轴), 更快更准, 用四点定位即可.
# 内矩形 = 电工胶带围成的矩形边框 (黑胶带贴白 A4 纸), 由视觉四点定位检测.
OUTER_INSET_RATIO = 0.05      # 外矩形内缩比例 (5%)
OUTER_TRACE_SPEED = 80.0      # 外框循迹速度 (固定轴对齐, 可更激进, proc px/s)

# 拐角处理 (让激光在 90° 拐角处不"切角")
CORNER_SLOWDOWN_ARC       = 25.0   # 拐角附近减速弧长 (proc px)
CORNER_SLOW_FACTOR        = 0.30   # 拐角处最低速度系数
CORNER_DWELL_MS           = 200    # 拐角停留时间 (ms, 0=不停)
CORNER_DWELL_ENTER        = 6.0    # 进入停留的弧长判定 (proc px)
CORNER_SMOOTH_DISABLE_ARC = 10.0   # 此弧长内禁用 EMA 平滑 (保证拐角锐利)

# 设定点 EMA 平滑 (让循迹流畅)
TRACE_SMOOTH       = True
TRACE_ALPHA_NEAR   = 0.5
TRACE_ALPHA_MIDDLE = 0.7
TRACE_ALPHA_FAR    = 0.9
TRACE_NEAR_DISTANCE = 2.0
TRACE_FAR_DISTANCE  = 10.0

# ----- 激光点检测 (RGB 320x192, LAB 红) -----
LASER_ENABLE = True
# 多级 LAB 阈值: 窄核 (初始捕获, 防误检) + 宽泛 (锁定后 ROI 追踪, 耐颜色漂移)
# v5.2: 改造为 4 级渐进式回退 (NARROW → CENTER → WIDE → HOT), 每级失败立即下一级
RED_LASER_LAB_MERGED = [(38, 80, 22, 72, -30, 8)]    # 饱和正红 (主力)
RED_LASER_LAB_WIDE   = [(40, 82, 18, 78, -34, 5)]    # 弱光/偏色
RED_LASER_LAB_HOT     = [(55, 95, 35, 88, -25, 25)]  # 过曝兜底
RED_LASER_LAB_USER    = [(66, 87, 37, 58, 7, 33)]    # 用户实测激光 LAB
RED_LASER_LAB_USER2   = [(39, 60, 58, 79, 19, 38)]   # 用户实测 LAB 2
# [v5.7] 用户实测阈值: 亮度偏低(激光变暗)时的红色范围 (L 58~84, A 16~37, B -10~9)
#   L 下限比其它集更低 → 暗激光也能抓到; 作为最高优先级先行尝试.
RED_LASER_LAB_DIM     = [(58, 84, 16, 37, -10, 9)]
# [v5.8] 用户实测阈值: 激光落在黑色矩形上, 因背景吸光导致色域偏移(更暗/偏色)时的范围
#   (L 25~37, A 33~62, B -5~32). 并入 DIM 级(OR), 优先捕获黑底上的暗红激光.
RED_LASER_LAB_DIM2    = [(25, 37, 33, 62, -5, 32)]
LASER_LAB_NARROW = ([(90, 100, -7, 23, -12, 25)] +  # 窄中心核
                    RED_LASER_LAB_USER2)
# v5.2: 4 级渐进式 LAB 阈值表 (每级是上一级的超集)
LASER_LAB_CENTER = (LASER_LAB_NARROW +
                    RED_LASER_LAB_MERGED)
LASER_LAB_WIDE   = (LASER_LAB_CENTER +
                    RED_LASER_LAB_WIDE)
LASER_LAB_HOT    = (LASER_LAB_WIDE +
                    RED_LASER_LAB_HOT +
                    RED_LASER_LAB_USER)
LASER_MIN_PX          = 2     # [v5.7] 降低: 激光变暗/变小(像素少)也能识别
LASER_WASHOUT_PX      = 1500      # 整屏泛红拒识
LASER_ACQUIRE_MAX_PX  = 1000      # 初始捕获最大像素
LASER_LOCK_FRAMES     = 2
LASER_LOST_FRAMES     = 3         # 连续 3 帧丢才掉锁 [v5.2: 8→3, 丢失外推 3 帧]
LASER_POS_ALPHA       = 0.5       # 激光位置 EMA 系数 (越大越跟手)
LASER_ROI_SIZE        = 120       # 锁定后 ROI 搜索窗口
LASER_TRACK_MAX_D     = 100       # 帧间许可移动距离 (proc px)
# 帧间运动预测 (dead-reckoning, 短暂丢失补齐)
LASER_PREDICT_ENABLE  = True
LASER_PREDICT_GAIN    = 0.7       # [v5.2: 1.0→0.7, 加大预测权重]
LASER_VEL_ALPHA       = 0.4
LASER_ROI_MISS_EXPAND = 2.5
LASER_GLOBAL_SWEEP_INTERVAL = 2   # 全局兜底扫描间隔帧 [v5.2: 保留常量但 detect_laser 不再使用]

# ----- 激光丢失回退机制 -----
# 激光丢失: 反转轨迹方向 → 云台主动"回退"到激光最后已知点附近
# 超时仍未捕获 → 放弃, 改发 valid=0, 由 STM32 超时停
LASER_RECOVER_ENABLE     = True
LASER_RECOVER_GIVEUP_MS  = 6000   # [v5.2: 4000→6000]

# ----- 输出协议 -----
# "binary" = 与 STM32 firmware 匹配 (默认)
# "ascii"  = "$cx,cy#" (TI 风格电机追点)
OUTPUT_PROTOCOL = "binary"

# ----- 循环节奏 -----
DETECT_INTERVAL        = 3    # [v5.7] 矩形检测降频 (1=每帧); 提高识别/传输频率, 视觉闭环更流畅
UART_SEND_INTERVAL     = 1
OSD_REFRESH_INTERVAL    = 1
DEBUG_PRINT_INTERVAL   = 30
GC_INTERVAL            = 1   # 每帧 GC (官方推荐防 K230 小堆碎片化卡死)
DETECT_TIMEOUT_MS      = 120  # 单次检测超时 → 永久回退 find_rects
LASER_INTERVAL         = 1

# ----- 状态文件 (上板无串口也能查) -----
STATUS_FILES_ENABLE  = True
STATUS_DIR           = "/data"
STATUS_RUNNING_FRAME = 30

# ----- 运行时调参 (/data/ti_vision_tune.conf, 免重烧) -----
TUNE_CONFIG_FILE       = STATUS_DIR + "/ti_vision_tune.conf"
TUNE_RELOAD_INTERVAL   = 15
_TUNE_BOUNDS = {
    "canny1": (1, 254), "canny2": (2, 255),
    "rect_thresh": (500, 60000), "reg_w": (0.0, 1.0),
    "reg_aspect": (0.1, 10.0), "detect_interval": (1, 30),
    "approx_eps": (0.01, 0.20),
    "laser_int": (1, 10), "laser_sweep": (5, 120),
    "osd_x": (-300, 300), "osd_y": (-300, 300),
    "track_w": (0.0, 3.0), "use_cv2": (0, 1),
    "bin": (0, 1), "bin_lo": (0, 254), "bin_hi": (1, 255),
    "snap_a": (0.05, 1.0), "move_px": (0.0, 20.0), "hold_f": (0, 30),
    "outer_speed": (5.0, 200.0),
    "trace_speed": (5.0, 200.0),
}
# 默认值快照 (删 .conf → 恢复)
_TUNE_DEFAULTS = {
    "canny1": CANNY_THRESH1, "canny2": CANNY_THRESH2,
    "rect_thresh": RECT_THRESHOLD, "reg_w": RECT_REGULARITY_WEIGHT,
    "reg_aspect": RECT_TARGET_ASPECT, "detect_interval": DETECT_INTERVAL,
    "approx_eps": APPROX_EPSILON,
    "laser_int": LASER_INTERVAL, "laser_sweep": LASER_GLOBAL_SWEEP_INTERVAL,
    "osd_x": OSD_X_OFFSET, "osd_y": OSD_Y_OFFSET,
    "track_w": RECT_TRACK_WEIGHT, "use_cv2": USE_CV2,
    "bin": RECT_BINARIZE, "bin_lo": BINARY_LO, "bin_hi": BINARY_HI,
    "snap_a": RECT_SNAP_ALPHA, "move_px": RECT_MOVE_PX_PER_FRAME,
    "hold_f": RECT_HOLD_FRAMES,
    "outer_speed": OUTER_TRACE_SPEED,
    "trace_speed": TRACE_SPEED,
}

# ----- 按键 (Pin 53) 多击状态机 -----
KEY_PIN              = 53
KEY_MULTI_WINDOW_MS  = 400    # 多击计数窗口
KEY_LONGPRESS_MS     = 1200   # 长按阈值 (STOP)
KEY_DEBUG_TOGGLE_MS  = 2000   # [v5.2] 长按切换 DEBUG_PRINT 阈值
KEY_DEBOUNCE_MS      = 30     # 抖动消除

# ----- 任务命令字节 (与 STM32 main.c 一致) -----
K230_CMD_AIM          = 0x01
K230_CMD_RECENTER     = 0x02
K230_CMD_TRACE_BORDER = 0x03
K230_CMD_TRACE_INNER  = 0x04
K230_CMD_STOP         = 0x05
K230_CMD_DEBUG_TOGGLE = 0x06   # [v5.2] 内部: 切换 DEBUG_PRINT (不发给 STM32)

# 任务模式 (K230 内部状态, 决定 desired_point 怎么算)
# 0=空闲(valid=0) / 1=RECENTER对中 / 2=外框循迹 / 3=内框循迹
TASK_IDLE     = 0
TASK_RECENTER = 1
TASK_BORDER   = 2
TASK_INNER    = 3
TASK_BOOT     = 4   # [v5.2] 开机自动回中 (上电即进入)

# [v5.2] BOOT 自动回中参数
BOOT_SETTLE_ERR    = 8      # 回中完成误差阈值 (像素)
BOOT_SETTLE_MS     = 500    # 回中稳定时长 (ms)
BOOT_FAIL_RETRY_MS = 2000   # 回中失败重试间隔 (ms)
BOOT_INIT_GUARD_MS = 3000   # 上电保护期 (ms)
BOOT_TIMEOUT_MS    = 5000   # 回中超时 (ms)
BOOT_DONE_DISPLAY_MS = 3000  # BOOT 完成后 OSD 显示 "DONE" 时长 (ms)


# ========================= 全局状态 =========================
sensor = None
uart = None
osd_img = None
_CV2_DISABLED = False   # cv2 不可用时置 True, 永久回退 find_rects

# [v5.2] DEBUG_PRINT 开关 (长按 2s 切换, 默认关闭)
DEBUG_PRINT = False


# ========================= 工具函数 =========================

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def sort_corners_clockwise(corners):
    """[v5.2] 将 4 个角点按顺时针排序 (参考 main_juxin.py)
    输入: [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]
    输出: 顺时针排序后的角点列表, 起点为左上角"""
    if len(corners) != 4:
        return corners
    cx = sum(c[0] for c in corners) / 4.0
    cy = sum(c[1] for c in corners) / 4.0
    def angle(c):
        return math.atan2(c[1] - cy, c[0] - cx)
    sorted_c = sorted(corners, key=angle)
    # 找左上角起点 (x+y 最小)
    start_idx = 0
    min_sum = sorted_c[0][0] + sorted_c[0][1]
    for i in range(1, 4):
        s = sorted_c[i][0] + sorted_c[i][1]
        if s < min_sum:
            min_sum = s
            start_idx = i
    return sorted_c[start_idx:] + sorted_c[:start_idx]


def status_path(name):
    return STATUS_DIR + "/ti_vision_" + name + ".txt"


def write_status(name, text):
    if not STATUS_FILES_ENABLE:
        return
    f = None
    try:
        f = open(status_path(name), "w")
        f.write(str(text))
    except BaseException:
        pass
    finally:
        if f is not None:
            try:
                f.close()
            except BaseException:
                pass


def clear_status_files():
    if not STATUS_FILES_ENABLE:
        return
    for name in ("boot", "camera_ok", "running", "error", "stopped"):
        try:
            os.remove(status_path(name))
        except BaseException:
            pass


def load_tune_config():
    """运行时调参 (免重烧): 读取 /data/ti_vision_tune.conf 的 key=value 行."""
    global CANNY_THRESH1, CANNY_THRESH2, RECT_THRESHOLD, RECT_REGULARITY_WEIGHT, \
        RECT_TARGET_ASPECT, DETECT_INTERVAL, APPROX_EPSILON, \
        LASER_INTERVAL, LASER_GLOBAL_SWEEP_INTERVAL, OSD_X_OFFSET, OSD_Y_OFFSET, \
        RECT_TRACK_WEIGHT, USE_CV2, RECT_BINARIZE, BINARY_LO, BINARY_HI, \
        RECT_SNAP_ALPHA, RECT_MOVE_PX_PER_FRAME, RECT_HOLD_FRAMES, \
        OUTER_TRACE_SPEED, TRACE_SPEED

    if not STATUS_FILES_ENABLE:
        return

    conf = None
    try:
        conf = open(TUNE_CONFIG_FILE, "r")
    except BaseException:
        conf = None

    if conf is None:
        applied = dict(_TUNE_DEFAULTS)
    else:
        applied = {}
        try:
            for raw_line in conf:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key = key.strip().lower()
                val = val.strip()
                if key not in _TUNE_BOUNDS:
                    continue
                try:
                    num = float(val)
                    if num.is_integer() and key not in (
                        "reg_w", "reg_aspect", "approx_eps", "track_w",
                        "snap_a", "move_px", "outer_speed", "trace_speed"
                    ):
                        num = int(num)
                except BaseException:
                    continue
                lo, hi = _TUNE_BOUNDS[key]
                if num < lo or num > hi:
                    continue
                applied[key] = num
        finally:
            try:
                conf.close()
            except BaseException:
                pass

    CANNY_THRESH1 = int(applied.get("canny1", CANNY_THRESH1))
    CANNY_THRESH2 = int(applied.get("canny2", CANNY_THRESH2))
    RECT_THRESHOLD = int(applied.get("rect_thresh", RECT_THRESHOLD))
    RECT_REGULARITY_WEIGHT = float(applied.get("reg_w", RECT_REGULARITY_WEIGHT))
    RECT_TARGET_ASPECT = float(applied.get("reg_aspect", RECT_TARGET_ASPECT))
    DETECT_INTERVAL = int(applied.get("detect_interval", DETECT_INTERVAL))
    APPROX_EPSILON = float(applied.get("approx_eps", APPROX_EPSILON))
    LASER_INTERVAL = int(applied.get("laser_int", LASER_INTERVAL))
    LASER_GLOBAL_SWEEP_INTERVAL = int(applied.get("laser_sweep", LASER_GLOBAL_SWEEP_INTERVAL))
    OSD_X_OFFSET = int(applied.get("osd_x", OSD_X_OFFSET))
    OSD_Y_OFFSET = int(applied.get("osd_y", OSD_Y_OFFSET))
    RECT_TRACK_WEIGHT = float(applied.get("track_w", RECT_TRACK_WEIGHT))
    RECT_SNAP_ALPHA = float(applied.get("snap_a", RECT_SNAP_ALPHA))
    RECT_MOVE_PX_PER_FRAME = float(applied.get("move_px", RECT_MOVE_PX_PER_FRAME))
    RECT_HOLD_FRAMES = int(applied.get("hold_f", RECT_HOLD_FRAMES))
    RECT_BINARIZE = bool(int(applied.get("bin", RECT_BINARIZE)))
    BINARY_LO = int(applied.get("bin_lo", BINARY_LO))
    BINARY_HI = int(applied.get("bin_hi", BINARY_HI))
    OUTER_TRACE_SPEED = float(applied.get("outer_speed", OUTER_TRACE_SPEED))
    TRACE_SPEED = float(applied.get("trace_speed", TRACE_SPEED))
    if "use_cv2" in applied:
        USE_CV2 = bool(int(applied["use_cv2"]))
    if CANNY_THRESH2 <= CANNY_THRESH1:
        CANNY_THRESH2 = CANNY_THRESH1 + 1

    write_status(
        "tune",
        "canny1={} canny2={} rect_thresh={} reg_w={} reg_aspect={} detect={} "
        "approx_eps={} laser_int={} laser_sweep={} osd=({},{}) "
        "track_w={} use_cv2={} bin={} bin_lo={} bin_hi={} snap_a={} move_px={} "
        "hold_f={} outer_speed={} trace_speed={}".format(
            CANNY_THRESH1, CANNY_THRESH2, RECT_THRESHOLD,
            RECT_REGULARITY_WEIGHT, RECT_TARGET_ASPECT, DETECT_INTERVAL,
            APPROX_EPSILON, LASER_INTERVAL, LASER_GLOBAL_SWEEP_INTERVAL,
            OSD_X_OFFSET, OSD_Y_OFFSET, RECT_TRACK_WEIGHT, USE_CV2,
            RECT_BINARIZE, BINARY_LO, BINARY_HI, RECT_SNAP_ALPHA,
            RECT_MOVE_PX_PER_FRAME, RECT_HOLD_FRAMES,
            OUTER_TRACE_SPEED, TRACE_SPEED,
        ),
    )


def elapsed_seconds(now_ms, previous_ms):
    dt = time.ticks_diff(now_ms, previous_ms) / 1000.0
    return clamp(dt, 0.005, 0.25)


# ========================= 通信协议 =========================

def crc8(data):
    """CRC8 (poly 0x07, init 0x00) — 与 STM32 K230_CRC8 一致."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def send_binary(uart, x, y, valid, hit, hi_conf, boot=False, hold=False, corner=False):
    """坐标帧: 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC8] 0x01 0x01 (10字节)
    [v5.2] FLAG 扩展: bit3=BOOT(开机回中), bit4=HOLD(激光丢失保持速度)
    [v5.7] bit5=NEAR_CORNER(接近矩形拐角, 请求 STM32 减速防过冲)"""
    if not uart:
        return 0
    buf = bytearray(10)
    buf[0] = 0x3C
    buf[1] = 0x3B
    x16 = int(clamp(x, -32768, 32767))
    y16 = int(clamp(y, -32768, 32767))
    buf[2] = (x16 >> 8) & 0xFF
    buf[3] = x16 & 0xFF
    buf[4] = (y16 >> 8) & 0xFF
    buf[5] = y16 & 0xFF
    f = 0
    if valid: f |= 1      # bit0 = valid
    if hit:   f |= 2      # bit1 = hit (完成)
    if hi_conf: f |= 4    # bit2 = hi_conf
    if boot:  f |= 8      # bit3 = BOOT  [v5.2 新增]
    if hold:  f |= 16     # bit4 = HOLD  [v5.2 新增]
    if corner: f |= 32    # bit5 = NEAR_CORNER [v5.7 新增]
    buf[6] = f
    buf[7] = crc8(bytes([buf[2], buf[3], buf[4], buf[5], buf[6]]))
    buf[8] = 1
    buf[9] = 1
    return uart.write(buf)


def send_ascii(uart, cx, cy, valid):
    if not uart:
        return 0
    if valid and cx is not None and cy is not None:
        uart.write("${},{}#".format(int(cx), int(cy)))
    else:
        uart.write("$-1,-1#")
    return 1


def send_cmd(uart, cmd, param=0):
    """命令帧: 0x3C 0x3C [CMD][PARAM] 0x00 0x00 0x00 0x01 0x01 (9字节)"""
    if not uart:
        return 0
    buf = bytearray(9)
    buf[0] = 0x3C; buf[1] = 0x3C
    buf[2] = cmd;  buf[3] = param
    for i in range(4, 7): buf[i] = 0
    buf[7] = 1; buf[8] = 1
    return uart.write(buf)


# ---- STM32 → K230 命令帧解析 (PA0/PA1 按键经 STM32 发给 K230 控制任务) ----
# 帧格式同 send_cmd: 0x3C 0x3C [CMD] [PARAM] 0x00 0x00 0x00 0x01 0x01
_host_fstate = 0   # 0=等待首 0x3C, 1=等待第二 0x3C, 2=等待 CMD
def parse_host_command(uart):
    """每帧调用, 返回解析到的 CMD 字节(0=无). 仅取 CMD, 忽略后续填充字节.
    [v5.7 修复] CanMV K230 的 UART 没有 readchar() 方法, 改用 read(1) 逐字节解析.
    """
    global _host_fstate
    if uart is None:
        return 0
    while uart.any():
        chunk = uart.read(1)
        if not chunk:
            break
        b = chunk[0]
        if _host_fstate == 0:
            if b == 0x3C:
                _host_fstate = 1
        elif _host_fstate == 1:
            if b == 0x3C:
                _host_fstate = 2
            else:
                _host_fstate = 0
        else:  # _host_fstate == 2: 收到 CMD
            _host_fstate = 0
            return b
    return 0


def send_trace(uart, desired_frame, laser_frame, valid, done,
               hold=False, boot=False, corner=False):
    """把当前循迹设定点发给云台.
    binary 模式: err = desired - reference (laser 或 画面中心)
    ascii  模式: 发设定点绝对坐标 (供追点电机用)
    [v5.2] 新增 hold/boot FLAG 位 (仅 binary 模式生效)
    [v5.7] 新增 corner(NEAR_CORNER) FLAG 位
    """
    if OUTPUT_PROTOCOL == "ascii":
        ok = desired_frame is not None
        return send_ascii(uart,
                          desired_frame[0] if desired_frame else None,
                          desired_frame[1] if desired_frame else None,
                          ok)
    # binary: err = desired - reference
    if desired_frame is not None and valid:
        if laser_frame is not None:
            ex = desired_frame[0] - laser_frame[0]
            ey = desired_frame[1] - laser_frame[1]
        else:
            ex = desired_frame[0] - FRAME_W // 2
            ey = desired_frame[1] - FRAME_H // 2
    else:
        ex = ey = 0
    return send_binary(uart, ex, ey, valid, done, valid, boot, hold, corner)


# ========================= 矩形几何 =========================

def order_corners(corners):
    """4 点按相对质心角度排序 → 一致绕向."""
    n = len(corners)
    cx = sum(p[0] for p in corners) / n
    cy = sum(p[1] for p in corners) / n
    return sorted(corners, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def rect_center_of(corners):
    n = len(corners)
    return (sum(p[0] for p in corners) / n, sum(p[1] for p in corners) / n)


def inset_corners(corners, ratio):
    """外框四角向中心缩 ratio 比例 → 内框四角 (兼容旧调用, 当前未使用)."""
    cx, cy = rect_center_of(corners)
    out = []
    for x, y in corners:
        nx = cx + (x - cx) * (1.0 - ratio)
        ny = cy + (y - cy) * (1.0 - ratio)
        out.append((nx, ny))
    return out


def fixed_outer_corners():
    """外矩形 = 画面边缘内缩 OUTER_INSET_RATIO, 轴对齐四角
    (顺时针: 左上→右上→右下→左下).
    固定不变, 不依赖视觉检测; 循迹时每个边只动一个轴 (Pan 或 Tilt 单轴),
    更快更准, 只需四点定位."""
    m = OUTER_INSET_RATIO
    x0 = int(round(PROC_W * m)); y0 = int(round(PROC_H * m))
    x1 = int(round(PROC_W * (1.0 - m))); y1 = int(round(PROC_H * (1.0 - m)))
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def choose_snap_alpha(jump, ref_size):
    """大跳变 → 整框贴合 (alpha=1.0); 否则 EMA 贴合."""
    if TRACE_REBUILD_DRIFT > 0 and jump > TRACE_REBUILD_DRIFT * ref_size:
        return 1.0
    return RECT_SNAP_ALPHA


def snap_corners_to_target(locked, detected, alpha):
    """角点 EMA 吸附: locked 朝 detected 贴合 alpha 比例."""
    return [
        (locked[i][0] + alpha * (detected[i][0] - locked[i][0]),
         locked[i][1] + alpha * (detected[i][1] - locked[i][1]))
        for i in range(4)
    ]


class RectTrajectory:
    """矩形周长密集采样, 让设定点按恒定弧长速度行走."""

    def __init__(self, corners, spacing):
        self.corners = corners
        self.spacing = spacing
        self.samples = []
        self.total_len = 0.0
        self.corner_arcs = []
        self.corner_points = []
        self._build()

    def _build(self):
        self.samples = []
        self.corner_arcs = []
        self.corner_points = []
        n = len(self.corners)
        idx = 0
        for i in range(n):
            x0, y0 = self.corners[i]
            x1, y1 = self.corners[(i + 1) % n]
            self.corner_arcs.append(idx * self.spacing)
            self.corner_points.append((x0, y0))
            seg_len = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
            steps = max(1, int(seg_len / self.spacing))
            for s in range(steps):
                t = s / steps
                self.samples.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
                idx += 1
        self.total_len = len(self.samples) * self.spacing

    def count(self):
        return len(self.samples)

    def point_at_index(self, idx):
        if not self.samples:
            return None
        return self.samples[idx % len(self.samples)]

    def point_at_arc(self, arc):
        if not self.samples:
            return None
        idx = int(arc / self.spacing) % len(self.samples)
        return self.samples[idx]

    def nearest_index(self, point):
        if not self.samples or point is None:
            return 0
        best = 0
        best_d = 1e18
        px, py = point
        for i, (sx, sy) in enumerate(self.samples):
            d = (sx - px) ** 2 + (sy - py) ** 2
            if d < best_d:
                best_d = d
                best = i
        return best


def arc_dist_to_nearest_corner(arc, corner_arcs, total):
    """环形 (wrap-aware) 弧距到最近拐角."""
    best = total
    for ca in corner_arcs:
        d = abs(arc - ca)
        if d > total * 0.5:
            d = total - d
        if d < best:
            best = d
    return best


def next_corner_ahead(arc, direction, corner_arcs, total, enter):
    """返回沿 direction 方向 enter 弧长内的最近拐角弧位置, 无则 None."""
    best_arc = None
    best_d = None
    for ca in corner_arcs:
        if direction > 0:
            d = (ca - arc) % total
        else:
            d = (arc - ca) % total
        if 0.0 <= d <= enter:
            if best_d is None or d < best_d:
                best_d = d
                best_arc = ca
    return best_arc


# ========================= 矩形检测 (cv2) =========================

def _corners_bbox(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs); y0 = min(ys)
    return x0, y0, max(xs) - x0, max(ys) - y0


def _score_corners(corners, prev_corners):
    """评分一个 4 点矩形 (抗误检). 镜像旧 choose_target_rect 逻辑."""
    x0, y0, w, h = _corners_bbox(corners)
    if w <= 0 or h <= 0:
        return None
    # v5.1: 最小边长过滤 (短边 < RECT_MIN_EDGE 的小框拒收, 抗噪点)
    if w < RECT_MIN_EDGE or h < RECT_MIN_EDGE:
        return None
    area = w * h
    if area < RECT_MIN_AREA or area > RECT_MAX_AREA:
        return None
    m = RECT_EDGE_MARGIN
    if x0 < m or y0 < m or (x0 + w) > (PROC_W - m) or (y0 + h) > (PROC_H - m):
        return None
    short = min(w, h); long = max(w, h)
    if short <= 0:
        return None
    aspect = long / short
    if aspect > RECT_MAX_ASPECT:
        return None
    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    frame_cx = PROC_W // 2
    frame_cy = PROC_H // 2
    center_penalty = abs(cx - frame_cx) + abs(cy - frame_cy)
    reg_penalty = abs(aspect - RECT_TARGET_ASPECT) * area * RECT_REGULARITY_WEIGHT
    track_penalty = 0.0
    if prev_corners is not None and RECT_TRACK_WEIGHT > 0:
        px0, py0, pw, ph = _corners_bbox(prev_corners)
        prev_area = pw * ph
        if prev_area > 0:
            prev_cx = px0 + pw / 2.0
            prev_cy = py0 + ph / 2.0
            jump = abs(cx - prev_cx) + abs(cy - prev_cy)
            size_ratio = area / prev_area
            size_pen = abs(math.log(max(size_ratio, 1e-3))) * 3000.0
            track_penalty = (jump + size_pen) * RECT_TRACK_WEIGHT
    return area - center_penalty * 2 - reg_penalty - track_penalty


def choose_target_corners(corner_cands, prev_corners=None):
    """从候选列表中选评分最高的 4 点矩形."""
    best = None
    best_score = -1e18
    for corners in corner_cands:
        score = _score_corners(corners, prev_corners)
        if score is None:
            continue
        if score > best_score:
            best_score = score
            best = corners
    # [v5.2] 返回前强制顺时针排序, 稳定角点顺序 (起点为左上角)
    if best is not None:
        best = sort_corners_clockwise(best)
    return best


def _approx_to_pts(approx):
    """把 cv2.approxPolyDP 结果 (N,1,2)/(N,2) 摊平为 [(x,y),...]."""
    pts = []
    for p in approx:
        try:
            sub = p[0]
            if isinstance(sub, (list, tuple)) or (
                hasattr(sub, "__len__") and len(sub) == 2
            ):
                pts.append((int(sub[0]), int(sub[1])))
            else:
                pts.append((int(p[0]), int(p[1])))
        except (TypeError, IndexError):
            try:
                pts.append((int(p[0]), int(p[1])))
            except (TypeError, IndexError):
                continue
    return pts


def find_rect_corners_cv2(img, prev_corners=None):
    """cv2 主路径: 灰度 → 二值化 → GaussianBlur → Canny → findContours
                  → approxPolyDP(4 点) → 评分选最优."""
    gray = img.to_numpy_ref()
    if hasattr(gray, "ndim") and gray.ndim == 3:
        gray = gray[:, :, 0]
    if RECT_BINARIZE:
        _, mask = cv2.threshold(gray, BINARY_HI, 255, cv2.THRESH_BINARY_INV)
    else:
        mask = gray
    if GAUSSIAN_BLUR_SIZE and GAUSSIAN_BLUR_SIZE > 1:
        k = GAUSSIAN_BLUR_SIZE if GAUSSIAN_BLUR_SIZE % 2 == 1 else GAUSSIAN_BLUR_SIZE + 1
        mask = cv2.GaussianBlur(mask, (k, k), 0)
    edges = cv2.Canny(mask, CANNY_THRESH1, CANNY_THRESH2)
    res = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = res[1] if len(res) == 3 else res[0]
    cands = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < RECT_MIN_AREA or area > RECT_MAX_AREA:
            continue
        # v5.1: 早期边长过滤 (boundingRect 比 approxPolyDP 快, 先用 bbox 短边过滤)
        bx, by, bw, bh = cv2.boundingRect(cnt)
        if bw < RECT_MIN_EDGE or bh < RECT_MIN_EDGE:
            continue
        peri = cv2.arcLength(cnt, True)
        if peri <= 0:
            continue
        approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
        pts = _approx_to_pts(approx)
        if len(pts) != 4:
            continue
        cands.append(order_corners(pts))
    return choose_target_corners(cands, prev_corners)


def find_rect_corners(img, prev_corners=None):
    """主路径 cv2; 兜底 img.find_rects (MaixPy C)."""
    global _CV2_DISABLED
    if USE_CV2 and HAS_CV2 and not _CV2_DISABLED:
        try:
            t0 = time.ticks_ms()
            corners = find_rect_corners_cv2(img, prev_corners)
            if time.ticks_diff(time.ticks_ms(), t0) > DETECT_TIMEOUT_MS:
                print("cv2 detect slow -> fallback to find_rects")
                _CV2_DISABLED = True
            else:
                return corners
        except BaseException as error:
            print("cv2 detect error:", error, "-> fallback to find_rects")
            _CV2_DISABLED = True
    detect_img = img.binary([(BINARY_LO, BINARY_HI)]) if RECT_BINARIZE else img
    rects = detect_img.find_rects(threshold=RECT_THRESHOLD)
    corner_cands = []
    for r in rects:
        try:
            c = r.corners()
            if c and len(c) == 4:
                corner_cands.append(order_corners([(int(p[0]), int(p[1])) for p in c]))
                continue
        except BaseException:
            pass
        x, y, w, h = r.rect()
        corner_cands.append([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
    return choose_target_corners(corner_cands, prev_corners)


# ========================= 激光点检测 (LAB) =========================

def _filter_laser_blobs(blobs, acquire=False):
    cands = []
    max_px = LASER_ACQUIRE_MAX_PX if acquire else LASER_WASHOUT_PX
    for b in blobs:
        px = b.pixels()
        if not (LASER_MIN_PX <= px <= max_px):
            continue
        bw, bh = b.w(), b.h()
        if bw < 1 or bh < 1:
            continue
        ratio = float(max(bw, bh)) / float(min(bw, bh))
        if ratio > 2.2:
            continue
        fill = float(px) / float(bw * bh)
        if fill < 0.45:
            continue
        cands.append((b, px, ratio, fill))
    return cands


def _blob_geom_center(b):
    """激光视为圆形色块: 以几何中心(外接框中心)为坐标.
    相比质心 cx()/cy()(像素质量中心), 外接框中心对不对称光晕/卫星像素更鲁棒,
    对规整圆形光斑二者重合; 这是"以几何中心为坐标"的实现."""
    return (b.x() + b.w() // 2, b.y() + b.h() // 2)


def _nearest_in_window(cands, ref_x, ref_y, max_d):
    best = None
    best_d2 = max_d * max_d
    for b, px, ratio, fill in cands:
        gx, gy = _blob_geom_center(b)
        dx = gx - ref_x
        dy = gy - ref_y
        d2 = dx * dx + dy * dy
        if d2 <= best_d2:
            best_d2 = d2
            best = (gx, gy)
    if best is None:
        return -1, -1, False
    return best[0], best[1], True


def detect_laser(laser_img, prev_x, prev_y, pred_x, pred_y,
                 locked, pred_ok, roi_expand=1.0, do_global=False,
                 last_good=None):
    """[v5.2] LAB 红激光检测: 多级 LAB 阈值回退 (NARROW→CENTER→WIDE→HOT),
    锁定后 ROI 搜索失败立即全局兜底 (无帧冷却)."""
    if not laser_img:
        return -1, -1, False

    # v5.7: 5 级渐进式 LAB 阈值表. DIM 集(L 下限更低)放最前, 优先捕获变暗的激光;
    #   后续各级是逐级放宽(超集), 直到 HOT 兜底.
    lab_levels = (RED_LASER_LAB_DIM + RED_LASER_LAB_DIM2, LASER_LAB_NARROW, LASER_LAB_CENTER,
                  LASER_LAB_WIDE, LASER_LAB_HOT)

    # 1) ROI 搜索 (锁定后, 在预测点附近小窗找)
    if locked and prev_x >= 0 and prev_y >= 0:
        cx = pred_x if pred_ok else prev_x
        cy = pred_y if pred_ok else prev_y
        size = int(LASER_ROI_SIZE * roi_expand)
        hs = size // 2
        rx = max(0, int(cx - hs))
        ry = max(0, int(cy - hs))
        rw = min(size, PROC_W - rx)
        rh = min(size, PROC_H - ry)
        if rw >= 10 and rh >= 10:
            roi = (rx, ry, rw, rh)
            # v5.2: 多级 LAB 回退, 每级失败立即下一级, 不等帧冷却
            for lab_set in lab_levels:
                blobs = laser_img.find_blobs(
                    lab_set, False, roi=roi,
                    x_stride=1, y_stride=1, pixels_threshold=LASER_MIN_PX,
                    merge=True, margin=True,
                )
                cands = _filter_laser_blobs(blobs)
                if cands:
                    rx2, ry2, ok = _nearest_in_window(
                        cands, cx, cy, LASER_TRACK_MAX_D * roi_expand)
                    if ok:
                        return rx2, ry2, True
                # 这级 ROI 没找到 → 立即下一级 (不冷却)

    # 2) 全局搜索 (初始捕获 或 ROI 失败后立即兜底)
    # v5.2: 移除 LASER_GLOBAL_SWEEP_INTERVAL 帧延迟, ROI 失败立即全局
    stride = 2 if locked else 1
    acquire = (not locked)
    for lab_set in lab_levels:
        blobs = laser_img.find_blobs(
            lab_set, False,
            x_stride=stride, y_stride=stride,
            pixels_threshold=LASER_MIN_PX,
            merge=True, margin=True,
        )
        cands = _filter_laser_blobs(blobs, acquire=acquire)
        if cands:
            if locked and prev_x >= 0 and prev_y >= 0:
                ref_x = pred_x if pred_ok else prev_x
                ref_y = pred_y if pred_ok else prev_y
                gx, gy, ok = _nearest_in_window(
                    cands, ref_x, ref_y, LASER_TRACK_MAX_D * roi_expand)
                if ok:
                    return gx, gy, True
                # 这级找到了但都不在窗口内 → 继续试下一级 (更宽泛)
            else:
                best = max(cands, key=lambda t: t[3] / (1.0 + t[1] * 0.01))
                bx, by = _blob_geom_center(best[0])
                # [v5.8] 非锁定(初始捕获/恢复中): 若已有上次良好位置, 要求候选在其
                #   LASER_TRACK_MAX_D 内, 否则拒绝(远处反射/杂光当激光会驱动云台冲飞).
                #   首次捕获(last_good=None)不门限, 正常选最圆 blob.
                if last_good is not None:
                    ddx = bx - last_good[0]
                    ddy = by - last_good[1]
                    if (ddx * ddx + ddy * ddy) > (LASER_TRACK_MAX_D * LASER_TRACK_MAX_D):
                        return -1, -1, False
                return bx, by, True
    return -1, -1, False


# ========================= 坐标缩放 =========================

def scale_point_to_frame(point):
    """proc 320x192 → frame 800x480."""
    if point is None:
        return None
    x = point[0] * FRAME_W / PROC_W + OSD_X_OFFSET
    y = point[1] * FRAME_H / PROC_H + OSD_Y_OFFSET
    return (int(round(x)), int(round(y)))


def scale_rect_to_frame(rect):
    if rect is None:
        return None
    x, y, w, h = rect
    sx = x * FRAME_W // PROC_W + OSD_X_OFFSET
    sy = y * FRAME_H // PROC_H + OSD_Y_OFFSET
    sw = w * FRAME_W // PROC_W
    sh = h * FRAME_H // PROC_H
    return (int(sx), int(sy), int(sw), int(sh))


# ========================= OSD =========================

def draw_osd(canvas, corner_frame, center_frame, start_frame, desired_frame,
             laser_frame, inner_corner_frame, progress, done, fps_value,
             recovering, task_mode, boot_state=None):
    """画 OSD: 矩形 (绿) + 矩形中心原点 (黄) + 内框 (浅绿) + 激光 (红)
              + 设定点 (蓝) + 任务状态文字.
    [v5.2] boot_state: None / "RECENTERING" / "DONE" / "FAIL"
    """
    canvas.clear()
    red    = (255, 0, 0)
    yellow = (255, 255, 0)
    white  = (255, 255, 255)
    green  = (0, 200, 0)
    lgreen = (60, 220, 60)
    blue   = (60, 180, 255)
    amber  = (255, 165, 0)

    # 画面中心参考十字
    canvas.draw_cross(FRAME_W // 2, FRAME_H // 2, color=(80, 80, 80),
                      size=10, thickness=1)

    # 外框矩形 (绿色四边形)
    if corner_frame is not None:
        n = len(corner_frame)
        for i in range(n):
            a = corner_frame[i]
            b = corner_frame[(i + 1) % n]
            canvas.draw_line(a[0], a[1], b[0], b[1], color=green, thickness=4)
        # 矩形中心 = A4 纸中心 = "原点" (黄色十字 + 圆)
        if center_frame is not None:
            canvas.draw_cross(center_frame[0], center_frame[1],
                              color=yellow, size=18, thickness=2)
            canvas.draw_circle(center_frame[0], center_frame[1],
                               5, yellow, 2)
        # 起点标记 (白色小圆)
        if start_frame is not None:
            canvas.draw_circle(start_frame[0], start_frame[1], 6, white, 2)

    # 内框 (浅绿虚线效果 — 用细线画)
    if inner_corner_frame is not None:
        n = len(inner_corner_frame)
        for i in range(n):
            a = inner_corner_frame[i]
            b = inner_corner_frame[(i + 1) % n]
            canvas.draw_line(a[0], a[1], b[0], b[1], color=lgreen, thickness=2)

    # 激光点 (红色十字)
    if laser_frame is not None:
        canvas.draw_cross(laser_frame[0], laser_frame[1],
                          color=red, size=14, thickness=2)
        canvas.draw_circle(laser_frame[0], laser_frame[1], 3, red, 2)

    # 设定点 (蓝色十字 — 循迹目标)
    if desired_frame is not None:
        canvas.draw_cross(desired_frame[0], desired_frame[1],
                          color=blue, size=12, thickness=2)

    # 状态文字
    # [v5.2] task_mode 包含 TASK_BOOT=4, 扩展任务名列表
    task_names = ["IDLE", "T1:RECENTER", "T2:BORDER", "T3:INNER", "BOOT"]
    task_name = task_names[task_mode] if task_mode < len(task_names) else "DONE"
    canvas.draw_string_advanced(10, 3, 28, "FPS:{:.1f}".format(fps_value),
                                color=white)
    canvas.draw_string_advanced(10, 39, 22, task_name, color=yellow)
    if task_mode in (2, 3):
        canvas.draw_string_advanced(180, 39, 22,
                                   "TRACE {:.0f}%".format(progress * 100.0),
                                   color=green)
    if done:
        canvas.draw_string_advanced(330, 39, 22, "DONE", color=red)

    if recovering:
        canvas.draw_string_advanced(450, 39, 24, "RECOVER", color=amber)
        canvas.draw_circle(FRAME_W // 2, FRAME_H // 2, 40, amber, 3)

    # [v5.2] BOOT 状态显示 (顶部, 居中)
    if boot_state == "RECENTERING":
        canvas.draw_string_advanced(FRAME_W // 2 - 90, 3, 24,
                                    "BOOT: RECENTERING", color=yellow)
    elif boot_state == "DONE":
        canvas.draw_string_advanced(FRAME_W // 2 - 60, 3, 24,
                                    "BOOT: DONE", color=green)
    elif boot_state == "FAIL":
        # 红色闪烁 (500ms 间隔)
        if (time.ticks_ms() // 500) % 2 == 0:
            canvas.draw_string_advanced(FRAME_W // 2 - 50, 3, 24,
                                        "BOOT: FAIL", color=red)


# ========================= 摄像头 / UART 初始化 =========================

def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    return UART(UART.UART2,
                baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


def camera_init():
    global sensor, uart, osd_img

    os.exitpoint(os.EXITPOINT_ENABLE)
    uart = init_uart()

    sensor = Sensor(id=SENSOR_ID,
                    width=SENSOR_INPUT_W,
                    height=SENSOR_INPUT_H,
                    fps=SENSOR_FPS)
    sensor.reset()
    try:
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
    except BaseException:
        pass

    # CHN0: 800x480 YUV420SP → LCD 直通
    sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # CHN1: 320x192 灰度 → cv2 矩形检测
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

    # CHN2: 320x192 RGB565 → 激光 LAB 检测
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)

    Display.init(Display.ST7701,
                  width=FRAME_W, height=FRAME_H,
                  to_ide=True, quality=IDE_JPEG_QUALITY,
                  osd_num=2)
    osd_img = image.Image(FRAME_W, FRAME_H, image.ARGB8888)
    MediaManager.init()
    sensor.run()

    try:
        gain = k_sensor_gain()
        gain.gain[0] = SENSOR_ANALOG_GAIN
        sensor.again(gain)
    except BaseException as error:
        print("sensor gain skipped:", error)


def camera_deinit():
    global sensor
    try:
        if isinstance(sensor, Sensor):
            sensor.stop()
    except BaseException as error:
        print("sensor stop error:", error)
    try:
        Display.deinit()
    except BaseException as error:
        print("display deinit error:", error)
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    try:
        MediaManager.deinit()
    except BaseException as error:
        print("media deinit error:", error)


# ========================= 按键多击状态机 =========================

class KeyStateMachine:
    """[v5.2] 按键多击/长按检测 (Pin 53).
    单击 → CMD 0x03 (TRACE_BORDER)   [v5.2: 原 RECENTER 移除]
    双击 → CMD 0x04 (TRACE_INNER)
    长按 < 1.2s 阈值 → (短按计数)
    长按 [1.2s, 2s) → CMD 0x05 (STOP)
    长按 >= 2s       → CMD 0x06 (DEBUG_TOGGLE, 内部不发给 STM32)
    """
    def __init__(self, pin_num):
        self.key = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
        self.click_cnt = 0
        self.last_release_ms = 0
        self.press_start_ms = 0
        self.was_pressed = False

    def update(self, uart):
        """每帧调用, 返回 (cmd_byte, param) 或 (0, 0) 表示无命令."""
        now_ms = time.ticks_ms()
        kv = self.key.value()
        cmd = 0
        param = 0

        if kv and not self.was_pressed:
            # 按下边沿
            self.was_pressed = True
            self.press_start_ms = now_ms
            time.sleep_ms(KEY_DEBOUNCE_MS)
        elif not kv and self.was_pressed:
            # 释放边沿
            self.was_pressed = False
            press_dur = now_ms - self.press_start_ms
            if press_dur < KEY_LONGPRESS_MS:
                # 短按: 累计多击
                if time.ticks_diff(now_ms, self.last_release_ms) < KEY_MULTI_WINDOW_MS:
                    self.click_cnt += 1
                else:
                    self.click_cnt = 1
                self.last_release_ms = now_ms
            elif press_dur < KEY_DEBUG_TOGGLE_MS:
                # 长按 [1.2s, 2s) → STOP
                cmd = K230_CMD_STOP
                param = 0
                self.click_cnt = 0
                print("[KEY] 长按 <2s → STOP (CMD 0x05)")
            else:
                # 长按 >= 2s → DEBUG_TOGGLE (内部, 不发给 STM32)
                cmd = K230_CMD_DEBUG_TOGGLE
                param = 0
                self.click_cnt = 0
                print("[KEY] 长按 >=2s → DEBUG_TOGGLE")
            time.sleep_ms(KEY_DEBOUNCE_MS)

        # 多击窗口超时 → 派发命令
        if self.click_cnt > 0 and \
           time.ticks_diff(now_ms, self.last_release_ms) >= KEY_MULTI_WINDOW_MS:
            if self.click_cnt == 1:
                # [v5.2] 单击 → BORDER (原双击)
                cmd = K230_CMD_TRACE_BORDER
                print("[KEY] 单击 → 任务2 TRACE_BORDER (CMD 0x03)")
            elif self.click_cnt >= 2:
                # [v5.2] 双击及以上 → INNER (原三击)
                cmd = K230_CMD_TRACE_INNER
                print("[KEY] 双击 → 任务3 TRACE_INNER (CMD 0x04)")
            param = 0
            self.click_cnt = 0

        # DEBUG_TOGGLE 是内部命令, 不发给 STM32
        if cmd != 0 and cmd != K230_CMD_DEBUG_TOGGLE and uart:
            send_cmd(uart, cmd, param)
        return cmd, param


# ========================= 主循环 =========================

def main():
    global sensor, uart, osd_img
    global DEBUG_PRINT

    camera_is_init = False
    frame_id = 0

    # 任务状态 (由按键命令切换)
    # [v5.4] 上电直接进入 IDLE 待命 — 移除 BOOT 状态机
    #   原方案 v5.2/v5.3 上电即进入 TASK_BOOT 自动回中, 但 K230 先启动后 STM32 后启动
    #   会因收不到 FLAG_BOOT 而卡在 BOOT, 逻辑过于复杂.
    #   参考 KarmanLine630: 无 BOOT 状态机, 上电直接 IDLE, 用户按键/串口命令触发任务.
    #   STM32 端同步: 上电 POSCTRL 待命, 收到 'k'/task1-3 命令才切 AUTO.
    task_mode = TASK_IDLE

    # [v5.2] BOOT 自动回中状态机变量 (v5.4 保留定义但不再使用, 兼容旧代码引用)
    boot_start_tick = time.ticks_ms()      # 开机时间戳
    boot_settle_tick = 0                    # 开始稳定时间戳
    boot_done = True                       # [v5.4] 默认已完成, 跳过 BOOT
    boot_fail_retry_tick = 0                # 上次失败重试时间戳
    boot_done_display_tick = 0              # BOOT 完成 OSD 显示截止时间
    boot_failed = False                     # BOOT 失败标志 (OSD 闪烁)

    # 轨迹状态 (Task2/3 用)
    # 外矩形: 固定轴对齐四角 (画面内缩, 不参与视觉检测)
    # 内矩形: 视觉检测到的电工胶带矩形四角 (四点定位)
    outer_corners_proc = fixed_outer_corners()   # 外框固定四角 (proc 空间)
    inner_corners_proc = None                    # 内框(胶带)检测四角 (proc 空间, None=未检测到)
    traj = None
    traj_corners_proc = None       # 当前建轨迹用的角点 (外框固定 / 内框检测)
    traj_use_inner = False         # True=走内框(胶带), False=走外框(固定)
    traj_needs_build = False       # 任务切换后置 True, 待对应角点建轨迹
    traj_corner_frame = [scale_point_to_frame(p) for p in outer_corners_proc]  # 外框(绿, 固定)
    traj_inner_corner_frame = None # 内框(胶带)四角 (frame 空间, OSD 浅绿)
    traj_center_frame = None      # 内框(胶带)中心 (frame 空间, "原点"黄色十字)
    traj_start_frame = None       # 起点标记
    traj_arc = 0.0
    traj_dir = TRACE_DIR
    traj_done = False
    traj_locked = False
    traj_dwell_until = 0
    traj_dwell_arc = 0.0
    missed_rect = 0
    last_traj_ms = time.ticks_ms()

    # 边框运动跟踪
    rect_moving = False
    rect_speed = 0.0
    rect_last_center = None
    rect_last_frame = 0
    rect_prev_corners = None
    rect_corner_vel = [(0.0, 0.0)] * 4

    # 设定点平滑
    smooth_x = 0.0
    smooth_y = 0.0
    smooth_init = False

    # 激光时序跟踪
    l_lock = False
    l_hit_cnt = 0
    l_miss_cnt = 0
    lx_s = 0
    ly_s = 0

    # 激光丢失回退
    recovering = False
    recover_start_ms = 0
    last_good_laser_proc = None

    # 激光帧间预测
    l_vx = 0.0
    l_vy = 0.0
    l_last_det_ms = 0
    l_prev_miss = False
    l_global_cd = 0

    # 按键状态机
    key_sm = KeyStateMachine(KEY_PIN)

    clock = time.clock()

    try:
        clear_status_files()
        write_status("boot", "main entered")
        camera_init()
        camera_is_init = True
        write_status("camera_ok", "camera initialized")
        print("=== ti_cup_e_rect_trace v5.9 (外框固定/内框四点定位) start ===")
        print("cv2 available:", HAS_CV2)

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1

            # 运行时调参重读 (免重烧)
            if TUNE_RELOAD_INTERVAL and frame_id % TUNE_RELOAD_INTERVAL == 0:
                load_tune_config()

            # ---- 按键多击/长按 → 任务命令 ----
            cmd, param = key_sm.update(uart)
            # [v5.7] STM32(PA0/PA1 按键)发来的命令帧 → 同样切换 K230 任务
            host_cmd = parse_host_command(uart)
            if cmd == 0 and host_cmd != 0:
                cmd = host_cmd
                param = 0
            # [v5.2] BOOT 模式期间按键无效 (必须等 BOOT 完成)
            if task_mode == TASK_BOOT and not boot_done:
                cmd = 0
                param = 0
            # [v5.2] DEBUG_TOGGLE 内部命令: 切换 DEBUG_PRINT 开关 (不切换任务)
            if cmd == K230_CMD_DEBUG_TOGGLE:
                DEBUG_PRINT = not DEBUG_PRINT
                print("[DEBUG] DEBUG_PRINT =", DEBUG_PRINT)
                cmd = 0
            if cmd == K230_CMD_RECENTER:
                task_mode = TASK_RECENTER
                traj_locked = False
                traj = None
                traj_done = False
                smooth_init = False
                print("[TASK] → RECENTER (激光对中到 A4 中心)")
            elif cmd == K230_CMD_TRACE_BORDER:
                task_mode = TASK_BORDER
                traj_use_inner = False
                traj_locked = False
                traj = None
                traj_done = False
                smooth_init = False
                traj_needs_build = True   # 用固定外框四角建轨迹
                print("[TASK] → TRACE_BORDER (激光绕外框/固定矩形一周, 单轴移动)")
            elif cmd == K230_CMD_TRACE_INNER:
                task_mode = TASK_INNER
                traj_use_inner = True
                traj_locked = False
                traj = None
                traj_done = False
                smooth_init = False
                traj_needs_build = True   # 用视觉检测到的内框(胶带)四角建轨迹
                print("[TASK] → TRACE_INNER (激光绕内框/电工胶带矩形一周)")
            elif cmd == K230_CMD_STOP:
                task_mode = TASK_IDLE
                traj_locked = False
                traj = None
                traj_done = False
                smooth_init = False
                print("[TASK] → STOP (回到空闲)")

            # 取帧
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            run_laser = LASER_ENABLE and (frame_id == 1 or frame_id % LASER_INTERVAL == 0)
            rgb = sensor.snapshot(chn=CAM_CHN_ID_2) if run_laser else None

            # ---- 激光检测 (每帧) + 帧间预测 + 丢失外推 ----
            laser_det = False
            lx = ly = -1
            if run_laser and rgb:
                now_ms = time.ticks_ms()
                dt = elapsed_seconds(now_ms, l_last_det_ms) if l_last_det_ms else 0.02
                if l_lock and LASER_PREDICT_ENABLE:
                    px_pred = clamp(lx_s + l_vx * dt * LASER_PREDICT_GAIN, 0, PROC_W - 1)
                    py_pred = clamp(ly_s + l_vy * dt * LASER_PREDICT_GAIN, 0, PROC_H - 1)
                    pred_ok = True
                else:
                    px_pred, py_pred, pred_ok = lx_s, ly_s, False
                roi_expand = LASER_ROI_MISS_EXPAND if l_prev_miss else 1.0
                if not l_lock:
                    do_global = True
                    l_global_cd = LASER_GLOBAL_SWEEP_INTERVAL
                elif l_global_cd <= 0:
                    do_global = True
                    l_global_cd = LASER_GLOBAL_SWEEP_INTERVAL
                else:
                    do_global = False
                    l_global_cd -= 1
                lx, ly, laser_det = detect_laser(
                    rgb,
                    lx_s if l_lock else -1,
                    ly_s if l_lock else -1,
                    px_pred, py_pred, l_lock, pred_ok,
                    roi_expand, do_global,
                    last_good=last_good_laser_proc,
                )
                if laser_det:
                    if l_lock:
                        inst_vx = (lx - lx_s) / dt if dt > 0 else 0.0
                        inst_vy = (ly - ly_s) / dt if dt > 0 else 0.0
                        l_vx = l_vx * (1 - LASER_VEL_ALPHA) + inst_vx * LASER_VEL_ALPHA
                        l_vy = l_vy * (1 - LASER_VEL_ALPHA) + inst_vy * LASER_VEL_ALPHA
                    l_hit_cnt = min(l_hit_cnt + 1, 32)
                    l_miss_cnt = 0
                    l_prev_miss = False
                    if not l_lock and l_hit_cnt == 1:
                        lx_s, ly_s = lx, ly
                        l_vx = l_vy = 0.0
                    else:
                        dx = lx - lx_s
                        dy = ly - ly_s
                        d2 = dx * dx + dy * dy
                        if d2 > LASER_TRACK_MAX_D * LASER_TRACK_MAX_D:
                            scale = LASER_TRACK_MAX_D / math.sqrt(d2)
                            lx = int(lx_s + dx * scale)
                            ly = int(ly_s + dy * scale)
                        lx_s = int(lx_s * (1 - LASER_POS_ALPHA) + lx * LASER_POS_ALPHA)
                        ly_s = int(ly_s * (1 - LASER_POS_ALPHA) + ly * LASER_POS_ALPHA)
                    if l_hit_cnt >= LASER_LOCK_FRAMES and not l_lock:
                        l_lock = True
                        # 激光重新捕获 → 轨迹点对齐到激光位置
                        if traj is not None:
                            start_idx = traj.nearest_index((lx_s, ly_s))
                            traj_arc = start_idx * TRACE_SAMPLE_SPACING
                            traj_dir = TRACE_DIR
                            traj_dwell_until = 0
                            smooth_init = False
                        recovering = False
                        last_good_laser_proc = None
                        print("laser: re-acquired, re-synced arc={:.1f}".format(traj_arc))
                else:
                    l_miss_cnt = min(l_miss_cnt + 1, 32)
                    l_hit_cnt = 0
                    l_prev_miss = True
                    if l_lock:
                        # 短暂丢失: 用预测速度外推, 控制参考连续
                        lx_s = int(clamp(lx_s + l_vx * dt * LASER_PREDICT_GAIN, 0, PROC_W - 1))
                        ly_s = int(clamp(ly_s + l_vy * dt * LASER_PREDICT_GAIN, 0, PROC_H - 1))
                    if l_lock and l_miss_cnt >= LASER_LOST_FRAMES:
                        l_lock = False
                        if LASER_RECOVER_ENABLE and traj_locked and traj is not None:
                            recovering = True
                            recover_start_ms = time.ticks_ms()
                            last_good_laser_proc = (lx_s, ly_s)
                            traj_dir = -traj_dir
                            print("laser: LOST -> entering recovery (rewind)")
                l_last_det_ms = now_ms

            # ---- 内矩形(电工胶带)检测: 四点定位 + 角点 EMA 吸附 + 丢失保持 ----
            # 外矩形 = 固定轴对齐 (画面内缩 OUTER_INSET_RATIO), 不参与视觉检测
            run_detection = (frame_id == 1) or (inner_corners_proc is None) or rect_moving \
                or (frame_id % DETECT_INTERVAL == 0)
            if run_detection and TRACE_ENABLE:
                corners = find_rect_corners(img, inner_corners_proc)
                if corners is not None:
                    missed_rect = 0
                    # 边框运动估计 (用于丢失外推)
                    rcx, rcy = rect_center_of(corners)
                    if rect_last_center is not None and rect_prev_corners is not None:
                        dframes = max(1, frame_id - rect_last_frame)
                        ddx = rcx - rect_last_center[0]
                        ddy = rcy - rect_last_center[1]
                        disp = math.sqrt(ddx * ddx + ddy * ddy) / dframes
                        rect_speed = rect_speed * 0.7 + disp * 0.3
                        rect_moving = rect_speed > RECT_MOVE_PX_PER_FRAME
                        for i in range(4):
                            vx = (corners[i][0] - rect_prev_corners[i][0]) / dframes
                            vy = (corners[i][1] - rect_prev_corners[i][1]) / dframes
                            rect_corner_vel[i] = (
                                rect_corner_vel[i][0] * 0.6 + vx * 0.4,
                                rect_corner_vel[i][1] * 0.6 + vy * 0.4,
                            )
                    rect_last_center = (rcx, rcy)
                    rect_last_frame = frame_id
                    rect_prev_corners = [tuple(c) for c in corners]

                    if inner_corners_proc is None:
                        # ---- 首次锁定内矩形 (胶带四点) ----
                        inner_corners_proc = [tuple(c) for c in corners]
                        traj_inner_corner_frame = [
                            scale_point_to_frame(p) for p in inner_corners_proc]
                        x0 = min(p[0] for p in inner_corners_proc)
                        y0 = min(p[1] for p in inner_corners_proc)
                        x1 = max(p[0] for p in inner_corners_proc)
                        y1 = max(p[1] for p in inner_corners_proc)
                        traj_center_frame = scale_point_to_frame(
                            ((x0 + x1) / 2.0, (y0 + y1) / 2.0))
                    else:
                        # ---- 已锁定: 角点 EMA 贴合 ----
                        old_c = rect_center_of(inner_corners_proc)
                        old_w = abs(inner_corners_proc[1][0] - inner_corners_proc[0][0]) or 1
                        old_h = abs(inner_corners_proc[2][1] - inner_corners_proc[1][1]) or 1
                        jdx = rcx - old_c[0]
                        jdy = rcy - old_c[1]
                        jump = math.sqrt(jdx * jdx + jdy * jdy)
                        alpha = choose_snap_alpha(jump, max(old_w, old_h))
                        inner_corners_proc = snap_corners_to_target(
                            inner_corners_proc, corners, alpha)
                        traj_inner_corner_frame = [
                            scale_point_to_frame(p) for p in inner_corners_proc]
                        rcx2, rcy2 = rect_center_of(inner_corners_proc)
                        traj_center_frame = scale_point_to_frame((rcx2, rcy2))
                        # 若当前正循迹内框且漂移, 重建轨迹并对齐弧长
                        if traj_use_inner and traj_locked and traj is not None:
                            cur = traj.point_at_arc(traj_arc)
                            traj = RectTrajectory(inner_corners_proc, TRACE_SAMPLE_SPACING)
                            if cur is not None:
                                traj_arc = traj.nearest_index(cur) * TRACE_SAMPLE_SPACING
                else:
                    # 检测丢失 → 保持上一内框并外推
                    missed_rect += 1
                    if (missed_rect <= RECT_HOLD_FRAMES and inner_corners_proc is not None
                            and RECT_HOLD_EXTRAPOLATE):
                        for i in range(4):
                            inner_corners_proc[i] = (
                                inner_corners_proc[i][0] + rect_corner_vel[i][0],
                                inner_corners_proc[i][1] + rect_corner_vel[i][1],
                            )
                            traj_inner_corner_frame[i] = scale_point_to_frame(
                                inner_corners_proc[i])
                        rcx2, rcy2 = rect_center_of(inner_corners_proc)
                        traj_center_frame = scale_point_to_frame((rcx2, rcy2))

            # ---- 轨迹建立: 外框固定 / 内框视觉检测 ----
            # 进入任务 (task_mode 改变) 时 traj_needs_build=True; 此处用对应角点建轨迹
            if traj_needs_build and not traj_locked:
                src = inner_corners_proc if traj_use_inner else outer_corners_proc
                if src is not None:
                    traj = RectTrajectory(src, TRACE_SAMPLE_SPACING)
                    traj_corners_proc = [tuple(c) for c in src]
                    # 起点 = 离参考最近的周长点
                    if TRACE_REFERENCE == "laser" and l_lock:
                        ref = (lx_s, ly_s)
                    elif recovering and last_good_laser_proc is not None:
                        ref = last_good_laser_proc
                    else:
                        rcx, rcy = rect_center_of(src)
                        ref = (rcx, rcy)
                    start_idx = traj.nearest_index(ref)
                    traj_arc = start_idx * TRACE_SAMPLE_SPACING
                    if not recovering:
                        traj_dir = TRACE_DIR
                    traj_dwell_until = 0
                    traj_dwell_arc = 0.0
                    traj_done = False
                    traj_start_frame = scale_point_to_frame(
                        traj.point_at_index(start_idx))
                    traj_locked = True
                    traj_needs_build = False
                    print("[TRAJ] built task={} src={} corners={}".format(
                        "BORDER" if not traj_use_inner else "INNER",
                        "fixed" if not traj_use_inner else "detected", src))

            # ---- 计算设定点 (desired_frame) ----
            desired_frame = None
            progress = 0.0
            near_corner = False   # [v5.7] 接近拐角标志, 每帧重置, 仅在循迹分支内置位
            if task_mode == TASK_BOOT:
                # [v5.2] BOOT 自动回中: 设定点 = 矩形中心 (类似 RECENTER)
                # |err_x|<8 且 |err_y|<8 持续 500ms → 切 TASK_IDLE + boot_done=True
                # 5 秒未锁定 → 每 2 秒重试 (清 traj_locked 让下次重新检测)
                if traj_center_frame is not None and l_lock:
                    desired_frame = traj_center_frame
                    # proc 空间误差
                    bx = int(traj_center_frame[0] * PROC_W / FRAME_W)
                    by = int(traj_center_frame[1] * PROC_H / FRAME_H)
                    err_bx = bx - lx_s
                    err_by = by - ly_s
                    if abs(err_bx) < BOOT_SETTLE_ERR and abs(err_by) < BOOT_SETTLE_ERR:
                        if boot_settle_tick == 0:
                            boot_settle_tick = time.ticks_ms()
                        elif time.ticks_diff(time.ticks_ms(), boot_settle_tick) >= BOOT_SETTLE_MS:
                            # 已稳定 500ms → 完成
                            boot_done = True
                            boot_done_display_tick = time.ticks_ms() + BOOT_DONE_DISPLAY_MS
                            boot_failed = False
                            task_mode = TASK_IDLE
                            traj_done = True
                            print("[BOOT] DONE: laser settled at center")
                    else:
                        boot_settle_tick = 0  # 误差大, 重新累计
                    # BOOT 超时检测 (5s)
                    if not boot_done and time.ticks_diff(time.ticks_ms(), boot_start_tick) >= BOOT_TIMEOUT_MS:
                        if time.ticks_diff(time.ticks_ms(), boot_fail_retry_tick) >= BOOT_FAIL_RETRY_MS:
                            boot_fail_retry_tick = time.ticks_ms()
                            boot_failed = True
                            # 重试: 清空轨迹锁定, 强制下次重新检测矩形
                            traj_locked = False
                            traj = None
                            traj_corners_proc = None
                            traj_center_frame = None
                            traj_corner_frame = None
                            traj_inner_corner_frame = None
                            print("[BOOT] FAIL: timeout, retrying...")
                else:
                    boot_settle_tick = 0
                    # BOOT 超时检测 (5s)
                    if not boot_done and time.ticks_diff(time.ticks_ms(), boot_start_tick) >= BOOT_TIMEOUT_MS:
                        if time.ticks_diff(time.ticks_ms(), boot_fail_retry_tick) >= BOOT_FAIL_RETRY_MS:
                            boot_fail_retry_tick = time.ticks_ms()
                            boot_failed = True
                            traj_locked = False
                            traj = None
                            print("[BOOT] FAIL: no rect/laser, retrying...")
            elif task_mode == TASK_RECENTER:
                # Task1: 设定点 = 矩形中心 (A4 中心)
                if traj_center_frame is not None:
                    desired_frame = traj_center_frame
                    progress = 1.0 if (l_lock and
                                       abs(lx_s - (traj_center_frame[0] * PROC_W / FRAME_W)) < 15
                                       and abs(ly_s - (traj_center_frame[1] * PROC_H / FRAME_H)) < 15) else 0.5
                    if progress >= 1.0:
                        traj_done = True
                else:
                    desired_frame = None
            elif task_mode in (TASK_BORDER, TASK_INNER) and traj_locked and traj is not None:
                # Task2/3: 设定点沿周长行走
                now_ms = time.ticks_ms()
                dt = elapsed_seconds(now_ms, last_traj_ms)
                last_traj_ms = now_ms

                total = traj.total_len
                # 外框固定轴对齐 → 用更激进的 OUTER_TRACE_SPEED 加快; 内框(胶带)用 TRACE_SPEED
                cur_trace_speed = OUTER_TRACE_SPEED if not traj_use_inner else TRACE_SPEED
                near_corner = False   # 接近拐角标志 → FLAG bit5, STM32 收到后减速防过冲
                if total > 0 and not traj_done:
                    if traj_dwell_until > 0 and now_ms < traj_dwell_until:
                        # 拐角停留: 冻结在顶点上
                        traj_arc = traj_dwell_arc
                    else:
                        if traj_dwell_until > 0:
                            # 停留刚结束: 微推过拐角避免重复触发
                            traj_arc = traj_dwell_arc + traj_dir * TRACE_SAMPLE_SPACING
                            traj_dwell_until = 0

                        # 拐角减速
                        d_corner = arc_dist_to_nearest_corner(
                            traj_arc, traj.corner_arcs, total)
                        near_corner = (d_corner < CORNER_SLOWDOWN_ARC)  # [v5.7] 接近拐角 → 通知 STM32 减速
                        if d_corner >= CORNER_SLOWDOWN_ARC:
                            speed_scale = 1.0
                        else:
                            t = d_corner / CORNER_SLOWDOWN_ARC
                            speed_scale = CORNER_SLOW_FACTOR + \
                                          (1.0 - CORNER_SLOW_FACTOR) * t
                        traj_arc += traj_dir * cur_trace_speed * speed_scale * dt

                        if TRACE_LOOP == "repeat":
                            # [v5.3] 绕一圈完成检测: 第一次跨过 total 边界时设置 traj_done
                            # 让 STM32 端通过 FLAG_HIT 检测任务完成, 而不是无限循环
                            prev_arc = traj_arc - traj_dir * cur_trace_speed * speed_scale * dt
                            if traj_dir > 0 and prev_arc < total <= traj_arc + 0.5:
                                traj_done = True
                                print("[T2/3] loop done (arc={:.1f})".format(traj_arc))
                            traj_arc %= total
                        elif TRACE_LOOP == "pingpong":
                            if traj_arc >= total:
                                traj_arc = total; traj_dir = -1
                            elif traj_arc <= 0:
                                traj_arc = 0; traj_dir = 1
                        else:  # once
                            if traj_arc >= total:
                                traj_arc = total; traj_done = True
                            elif traj_arc <= 0:
                                traj_arc = 0; traj_done = True

                        # 拐角停留触发
                        if CORNER_DWELL_MS > 0 and not traj_done:
                            ca = next_corner_ahead(traj_arc, traj_dir,
                                                  traj.corner_arcs, total,
                                                  CORNER_DWELL_ENTER)
                            if ca is not None:
                                traj_dwell_arc = ca
                                traj_dwell_until = now_ms + CORNER_DWELL_MS
                                traj_arc = ca

                desired_proc = traj.point_at_arc(traj_arc)
                if desired_proc is not None:
                    df = scale_point_to_frame(desired_proc)
                    if TRACE_SMOOTH:
                        d_corner = arc_dist_to_nearest_corner(
                            traj_arc, traj.corner_arcs, total)
                        if d_corner < CORNER_SMOOTH_DISABLE_ARC:
                            smooth_x, smooth_y = df
                            smooth_init = True
                            desired_frame = df
                        elif not smooth_init:
                            smooth_x, smooth_y = df
                            smooth_init = True
                            desired_frame = df
                        else:
                            d = max(abs(df[0] - smooth_x), abs(df[1] - smooth_y))
                            if d <= TRACE_NEAR_DISTANCE:
                                a = TRACE_ALPHA_NEAR
                            elif d <= TRACE_FAR_DISTANCE:
                                a = TRACE_ALPHA_MIDDLE
                            else:
                                a = TRACE_ALPHA_FAR
                            smooth_x += (df[0] - smooth_x) * a
                            smooth_y += (df[1] - smooth_y) * a
                            desired_frame = (int(round(smooth_x)),
                                             int(round(smooth_y)))
                    else:
                        desired_frame = df
                    progress = (traj_arc % total) / total if total > 0 else 0.0
            else:
                # TASK_IDLE: 无设定点
                desired_frame = None

            # ---- 激光丢失回退超时 ----
            if recovering and LASER_RECOVER_ENABLE:
                if time.ticks_diff(time.ticks_ms(), recover_start_ms) > LASER_RECOVER_GIVEUP_MS:
                    recovering = False
                    last_good_laser_proc = None
                    print("laser: recovery timeout -> give up (will safe-stop)")

            # ---- 决定 valid / 参考点 ----
            # [v5.2] HOLD: 激光丢失时 desired_point 保持, 用外推的 lx_s/ly_s 作参考,
            #        valid=True + FLAG bit4=1, STM32 收到后维持速度 500ms 再停车 (不要 err=0)
            hold_out = False
            if desired_frame is not None:
                if TRACE_REFERENCE == "laser":
                    if l_lock:
                        laser_frame = scale_point_to_frame((lx_s, ly_s))
                        valid_out = True
                    else:
                        if recovering and last_good_laser_proc is not None:
                            laser_frame = scale_point_to_frame(last_good_laser_proc)
                            valid_out = True
                        else:
                            # [v5.2] HOLD 模式: 用外推的激光位置作参考, 保持 desired 不变
                            laser_frame = scale_point_to_frame((lx_s, ly_s))
                            valid_out = True
                            hold_out = True
                else:
                    laser_frame = scale_point_to_frame((lx_s, ly_s)) if l_lock else None
                    valid_out = True
            else:
                # TASK_IDLE: 无显式设定点 -> 自动让云台响应激光
                #   [修复] 原逻辑 valid_out=False: 即使 K230 已检测到激光并在 OSD 标出,
                #          也不发 valid=1, STM32 永远冻结、无响应。
                #   现改为: 激光锁定(l_lock)即 valid_out=True, 并给一个设定点驱动云台:
                #     - 矩形已锁定 -> 设定点=矩形中心(自动 RECENTER, 把激光对到靶板中心)
                #     - 矩形未锁定 -> 设定点=画面中心(先把激光保持在视野中心, 给即时响应)
                if l_lock:
                    if traj_locked and traj_center_frame is not None:
                        desired_frame = traj_center_frame
                    else:
                        desired_frame = (FRAME_W // 2, FRAME_H // 2)
                    laser_frame = scale_point_to_frame((lx_s, ly_s))
                    valid_out = True
                else:
                    laser_frame = None
                    valid_out = False

            # [v5.2] BOOT 模式标志位 (发给 STM32 让其知道当前在回中阶段)
            # [v5.4] 上电不再进 TASK_BOOT, boot_out 永远为 False (FLAG_BOOT 不再发送)
            #   STM32 端不再依赖 FLAG_BOOT 握手, 上电直接 POSCTRL 待命.
            boot_out = False

            # ---- 发送 ----
            if frame_id % UART_SEND_INTERVAL == 0:
                # [v5.2] 传入 hold/boot FLAG 位 (l_lock=0 时 HOLD, task_mode=BOOT 时 BOOT)
                # [v5.7] 传入 corner(NEAR_CORNER) 标志 → STM32 接近拐角减速
                send_trace(uart, desired_frame, laser_frame, valid_out, traj_done,
                           hold=hold_out, boot=boot_out, corner=near_corner)

            # ---- Debug 打印 ----
            if DEBUG_PRINT_INTERVAL and frame_id % DEBUG_PRINT_INTERVAL == 0:
                if desired_frame is not None:
                    print("task={} desired={} laser={} prog={:.2f} done={} fps={:.1f}".format(
                        task_mode, desired_frame, laser_frame, progress,
                        traj_done, clock.fps()))
                else:
                    print("task={} idle fps={:.1f}".format(task_mode, clock.fps()))
            # [v5.2] DEBUG_PRINT 开启时每帧打印 (长按 2s 切换)
            if DEBUG_PRINT:
                _ex = _ey = 0
                if desired_frame is not None and laser_frame is not None:
                    _ex = desired_frame[0] - laser_frame[0]
                    _ey = desired_frame[1] - laser_frame[1]
                _lx = laser_frame[0] if laser_frame is not None else -1
                _ly = laser_frame[1] if laser_frame is not None else -1
                _sx = desired_frame[0] if desired_frame is not None else -1
                _sy = desired_frame[1] if desired_frame is not None else -1
                print("[frame] rect_lock=%d laser_lock=%d err=(%+d,%+d) laser=(%d,%d) setpoint=(%d,%d)" % (
                    1 if traj_locked else 0, 1 if l_lock else 0,
                    _ex, _ey, _lx, _ly, _sx, _sy))

            # ---- OSD ----
            if OSD_REFRESH_INTERVAL and frame_id % OSD_REFRESH_INTERVAL == 0:
                # [v5.2] 计算 BOOT 状态字符串传给 OSD
                boot_state = None
                if task_mode == TASK_BOOT and not boot_done:
                    boot_state = "RECENTERING"
                elif boot_done and time.ticks_ms() < boot_done_display_tick:
                    boot_state = "DONE"
                elif boot_failed and not boot_done:
                    boot_state = "FAIL"
                draw_osd(osd_img, traj_corner_frame, traj_center_frame,
                         traj_start_frame, desired_frame, laser_frame,
                         traj_inner_corner_frame, progress, traj_done,
                         clock.fps(), recovering, task_mode, boot_state)
                Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            if frame_id == STATUS_RUNNING_FRAME:
                write_status("running",
                             "frame={} fps={:.1f} task={}".format(
                                 frame_id, clock.fps(), task_mode))

            if GC_INTERVAL and frame_id % GC_INTERVAL == 0:
                gc.collect()

    except KeyboardInterrupt as error:
        write_status("stopped", repr(error))
        print("user stop:", error)
    except BaseException as error:
        error_text = repr(error)
        if "IDE interrupt" in error_text:
            write_status("stopped", error_text)
            print("ide stop:", error_text)
        else:
            write_status("error", error_text)
            print("exception:", error_text)
            raise
    finally:
        if camera_is_init:
            camera_deinit()


if __name__ == "__main__":
    main()
