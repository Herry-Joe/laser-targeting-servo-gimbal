# -*- coding: utf-8 -*-
# =============================================================
# 庐山派 K230 - 激光打靶追踪  (OpenCV + RVV 加速 / 高帧率版)
#
# 为什么重写 (相对 main_noyolo_backup.py):
#   旧版在 800x480 整图上每帧跑两次 find_rects(AprilTag 四边形检测, 大 O 运算)
#   + 全局 find_blobs, 是帧率只有 1~2fps 的根因。
#   本版利用 K230 CanMV 已适配的 cv2 (94 接口, 部分算子 RVV 加速):
#     1. 传感器仍出 800x480 RGB888 (显示清晰), 用 to_numpy_ref() 零拷贝取 RGB888
#     2. cv2.resize 缩到 320x240 (RVV 加速) 再跑检测 -> 计算量降到 1/6.25
#     3. 靶心: cv2 灰度+高斯+自适应阈值 -> findContours 取最大轮廓
#        优先 4 边形中心(方框靶), 否则用轮廓矩中心(同心圆靶)
#     4. 激光: 优先 cv2 HSV inRange(小图); 兜底 CanMV 原生 img.find_blobs() LAB(原图, 已验证可用)
#        坐标从显示空间转到检测空间; find_blobs 是旧版已验证能识别激光的唯一可靠路径
#     5. 检测坐标放大回 800x480 显示空间发包, 固件 HIT 容差(15px)语义不变
#   实测帧率预期 30fps 级别 (旧版 1~2fps)。
#
# 协议/接线不变:
#   坐标帧 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC] 0x01 0x01 (10字节)
#     FLAG bit0=valid  bit1=hit  bit2=hi_conf
#   命令帧 0x3C 0x3C [CMD][PARAM] ...
#   接线 GPIO05(TX)->STM32 PB11, GPIO06(RX)<-STM32 PB10
# =============================================================

import uos, math, time, image, gc

try:
    import cv2
    HAVE_CV2 = True
    print("[CV] cv2 可用, 启用 OpenCV+RVV 加速路径")
except ImportError:
    HAVE_CV2 = False
    print("[CV] cv2 不可用, 回退 find_rects/find_blobs (帧率较低, 需烧录带OpenCV的CanMV固件)")

from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, UART, FPIOA
from rect_detector import detect_rect, RECT_SMOOTH_ALPHA

# ========================= 分辨率 =========================
SENSOR_W = 800
SENSOR_H = 480
# 检测在缩小图上跑 (cv2 RVV 加速, 计算量 = (DETECT/SENSOR)^2)
if HAVE_CV2:
    DETECT_W, DETECT_H = 320, 240
else:
    DETECT_W, DETECT_H = SENSOR_W, SENSOR_H   # 回退: 直接在原图检测
SCALE_X = SENSOR_W / DETECT_W
SCALE_Y = SENSOR_H / DETECT_H

# ========================= 激光 LAB 阈值 =========================
# 用户实测激光 LAB(当前 320x240 缩图画面实测): L:56~100, A:39~61, B:2~18
# 主路径用这组值做 inRange; 同时试 RGB2LAB/BGR2LAB 取并集以适配 to_numpy_ref 通道顺序。
RED_L_MIN, RED_L_MAX = 56, 100
RED_A_MIN, RED_A_MAX = 39, 61
RED_B_MIN, RED_B_MAX = 2, 18
# 宽兜底(备用, 一般不再需要)
RED_L_MIN_W, RED_L_MAX_W = 45, 110
RED_A_MIN_W, RED_A_MAX_W = 0, 75
RED_B_MIN_W, RED_B_MAX_W = -45, 40
# 检测空间像素阈值 (320x240 下激光面积约 800x480 的 1/6.25)
LASER_MIN_PX     = 2
LASER_WASHOUT_PX = 400      # 整屏泛红/反光拒识
LASER_LOCK_FRAMES   = 2
LASER_LOST_FRAMES   = 5
LASER_POS_ALPHA     = 0.35
LASER_TRACK_MAX_D   = 40     # 检测空间最近邻跟踪最大距离

# ========================= 靶心检测 (cv2) =========================
TARGET_MIN_AREA_CV = 120     # 检测空间最小轮廓面积
TARGET_ADAPT_BLK   = 11      # 自适应阈值块大小(奇数)
TARGET_ADAPT_C     = 5       # 自适应阈值常数
RECT_SMOOTH_ALPHA  = 0.30

# ========================= 控制 =========================
HIT_TOLERANCE_PX  = 15       # 命中容差(显示空间像素)
COORD_INTERVAL_MS = 25       # 坐标发送间隔(ms): 40Hz
ERR_DEAD_ZONE     = 1
RECT_FRAME_INTERVAL_MS = 100  # 矩形帧发送间隔(ms): 10Hz

# ========================= 颜色 =========================
C_LASER=(255,0,0); C_TARGET=(0,128,255); C_LINE=(255,255,0)
C_TEXT=(255,255,255); C_BG=(0,0,0); C_RECT=(255,128,0)

# ======================= 工具 =======================
def draw_text_bg(img, x, y, text, fs, color=C_TEXT, bg=C_BG):
    w = len(text) * fs * 6 // 10
    h = int(fs * 1.3)
    img.draw_rectangle(x, y, w + 8, h, color=bg, thickness=1, fill=True)
    img.draw_string_advanced(x + 4, y + 1, fs, text, color=color)

def draw_cross(img, cx, cy, sz=10, color=(128,128,128)):
    img.draw_line(cx-sz, cy, cx+sz, cy, color, 1)
    img.draw_line(cx, cy-sz, cx, cy+sz, color, 1)
    img.draw_circle(cx, cy, 2, color, 1, fill=True)

def _contours_of(ret):
    """兼容 cv2.findContours 两种返回格式 (2值/3值)。"""
    if ret is None:
        return []
    if len(ret) == 2:
        return ret[0]
    return ret[1]

# ======================= cv2 检测 =======================
def detect_target_cv2(small):
    """small: RGB888 ndarray (DETECT_H, DETECT_W, 3) -> 靶心(detect space) 或 None。"""
    try:
        gray = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # 反相二值化: 暗环/暗靶面 -> 白。自适应阈值抗光照不均
        bin_img = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, TARGET_ADAPT_BLK, TARGET_ADAPT_C)
        contours = _contours_of(cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
        if not contours:
            return None
        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < TARGET_MIN_AREA_CV:
            return None
        # 优先判断方形框(4边形)中心; 否则用轮廓矩中心(同心圆靶)
        peri = cv2.arcLength(best, True)
        approx = cv2.approxPolyDP(best, 0.02 * peri, True)
        if len(approx) == 4:
            M = cv2.moments(approx)
            if M['m00'] != 0:
                return (M['m10'] / M['m00'], M['m01'] / M['m00'])
        M = cv2.moments(best)
        if M['m00'] != 0:
            return (M['m10'] / M['m00'], M['m01'] / M['m00'])
    except Exception as e:
        print("tgt_cv err:", e)
    return None

# 诊断: 激光像素数 (OSD/串口调试用)
g_laser_px = 0

def detect_laser(img, prev_x, prev_y, locked):
    """激光检测: 优先 cv2 HSV inRange(小图); 兜底 img.find_blobs()(原图, 已验证可用)。
    img: CanMV image 对象 (用于 find_blobs 兜底)
    返回 (cx_detect, cy_detect, ok) detect space 坐标。
    """
    global g_laser_px
    # ---- 路径 A: cv2 HSV inRange (在缩小图上跑, 快) ----
    if HAVE_CV2:
        try:
            arr = img.to_numpy_ref()
            small = cv2.resize(arr, (DETECT_W, DETECT_H))
            # 尝试 HSV (比 LAB 更通用, K230 cv2 更可能支持)
            hsv_ok = False
            for cname in ("COLOR_RGB2HSV", "COLOR_BGR2HSV"):
                try:
                    code = getattr(cv2, cname)
                    hsv = cv2.resize(small, (DETECT_W, DETECT_H))  # already small, no-op-ish
                    hsv = cv2.cvtColor(small, code)
                    # 红色在 HSV 中是 H≈0~10 或 H≈170~180, 高饱和度高亮度
                    lower1 = (0, 100, 100)
                    upper1 = (15, 255, 255)
                    lower2 = (160, 100, 100)
                    upper2 = (180, 255, 255)
                    m1 = cv2.inRange(hsv, lower1, upper1)
                    m2 = cv2.inRange(hsv, lower2, upper2)
                    mask = m1 | m2
                    contours = _contours_of(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
                    if contours:
                        best = max(contours, key=cv2.contourArea)
                        area = cv2.contourArea(best)
                        if area >= LASER_MIN_PX:
                            g_laser_px = int(area)
                            M = cv2.moments(best)
                            if M['m00'] != 0:
                                return M['m10']/M['m00'], M['m01']/M['m00'], True
                    hsv_ok = True
                    break
                except Exception:
                    continue
            if not hsv_ok:
                print("[LASER] cv2 无 HSV 支持, 用 find_blobs")
        except Exception as e:
            print("[LASER] cv2 检测失败:", e)

    # ---- 路径 B: CanMV 原生 find_blobs (兜底, 在原图上, 已验证可用) ----
    try:
        blobs = img.find_blobs([
            (RED_L_MIN, RED_L_MAX, RED_A_MIN, RED_A_MAX, RED_B_MIN, RED_B_MAX),
        ], False, x_stride=2, y_stride=2,
           pixels_threshold=LASER_MIN_PX*6, merge=True, margin=True)
        if not blobs:
            # 宽阈值兜底
            blobs = img.find_blobs([
                (RED_L_MIN_W, RED_L_MAX_W, RED_A_MIN_W, RED_A_MAX_W, RED_B_MIN_W, RED_B_MAX_W),
            ], False, x_stride=2, y_stride=2,
               pixels_threshold=LASER_MIN_PX*6, merge=True, margin=True)
        if blobs:
            b = max(blobs, key=lambda b: b.pixels())
            g_laser_px = b.pixels()
            # find_blobs 返回显示空间坐标 → 转到检测空间
            dx = float(b.cx()) / SCALE_X
            dy = float(b.cy()) / SCALE_Y
            return dx, dy, True
    except Exception as e:
        print("[LASER] find_blobs err:", e)

    g_laser_px = 0
    return -1, -1, False

# ======================= 回退检测 (无 cv2) =======================
def detect_target_legacy(img):
    try:
        rects = img.find_rects(threshold=2500)
        if not rects:
            rects = img.find_rects(threshold=1000)
        if not rects:
            return None
        best, score = None, 0
        for r in rects:
            x, y, w, h = r.rect()
            if w < 100 or h < 100:
                continue
            s = (w * h) * (r.magnitude() / 5000.0)
            if s > score:
                score = s; best = r
        if best is None:
            return None
        x, y, w, h = best.rect()
        return (x + w // 2, y + h // 2)
    except Exception:
        return None

# ======================== 串口 ========================
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc & 0xFF

def send_coord(uart, x, y, valid, hit, hi_conf):
    if not uart:
        return 0
    buf = bytearray(10)
    buf[0]=0x3C; buf[1]=0x3B
    buf[2]=(x>>8)&0xFF; buf[3]=x&0xFF
    buf[4]=(y>>8)&0xFF; buf[5]=y&0xFF
    f=0
    if valid: f|=1
    if hit: f|=2
    if hi_conf: f|=4
    buf[6]=f
    buf[7]=crc8(bytes([buf[2],buf[3],buf[4],buf[5],buf[6]]))
    buf[8]=1; buf[9]=1
    return uart.write(buf)

def send_cmd(uart, cmd, param):
    if not uart:
        return 0
    buf = bytearray(9)
    buf[0]=0x3C; buf[1]=0x3C
    buf[2]=cmd; buf[3]=param
    for i in range(4,9): buf[i]=0
    buf[7]=1; buf[8]=1
    return uart.write(buf)

def send_rect_frame(uart, dx, dy, w, h, ang, rect_valid, origin_valid):
    """发送矩形识别结果帧 (K230 → STM32, 16字节):
       0x3C 0x3A [CXH][CXL][CYH][CYL][WH][WL][HH][HL][ANGH][ANGL][FLAGS][CRC8] 0x01 0x01
       CX/CY: int16 矩形中心相对靶心偏移(px, 显示空间)
       W/H  : uint16 矩形宽高(px)
       ANG  : int16 倾斜角(0.1°)
       FLAGS: bit0=rect_valid, bit1=origin_valid(靶心有效→偏移可信)
       CRC8 poly0x07, 覆盖 [CXH..FLAGS] 共 11 字节, 与 STM32 端 K230_CRC8 一致。
    """
    if not uart:
        return 0
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    w  = max(0, min(65535, int(w)))
    h  = max(0, min(65535, int(h)))
    ang = max(-32768, min(32767, int(ang)))
    buf = bytearray(16)
    buf[0] = 0x3C; buf[1] = 0x3A
    buf[2] = (dx >> 8) & 0xFF; buf[3] = dx & 0xFF
    buf[4] = (dy >> 8) & 0xFF; buf[5] = dy & 0xFF
    buf[6] = (w >> 8) & 0xFF;  buf[7] = w & 0xFF
    buf[8] = (h >> 8) & 0xFF;  buf[9] = h & 0xFF
    buf[10] = (ang >> 8) & 0xFF; buf[11] = ang & 0xFF
    f = 0
    if rect_valid: f |= 1
    if origin_valid: f |= 2
    buf[12] = f
    buf[13] = crc8(bytes([buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12]]))
    buf[14] = 1; buf[15] = 1
    return uart.write(buf)

STATES=["W","T","L","C","D","X"]
def recv_status(uart):
    if not uart:
        return None
    try:
        if uart.any() < 8:
            return None
        b = uart.read(8)
        if not b or len(b) != 8:
            return None
        if b[0] != 0x3C or b[1] != 0x3D:
            return None
        if (sum(b[0:7]) & 0xFF) != b[7]:
            return None
        return (b[2]*270/255, b[3]*180/255, min(b[4],5),
                bool(b[5]&1), bool(b[5]&2), b[6])
    except:
        return None

# ======================== 主函数 ========================
def run():
    sensor = Sensor(width=SENSOR_W, height=SENSOR_H)
    sensor.reset()
    sensor.set_framesize(width=SENSOR_W, height=SENSOR_H)
    sensor.set_pixformat(Sensor.RGB888 if HAVE_CV2 else Sensor.RGB565)
    Display.init(Display.ST7701, width=SENSOR_W, height=SENSOR_H, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("[CAM] OK %dx%d %s" % (SENSOR_W, SENSOR_H, "RGB888+cv2" if HAVE_CV2 else "RGB565+legacy"))

    uart = None
    try:
        fpioa = FPIOA()
        fpioa.set_function(5, fpioa.UART2_TXD)
        fpioa.set_function(6, fpioa.UART2_RXD)
        uart = UART(UART.UART2, 115200, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print("[UART] OK")
    except Exception as e:
        print("[UART] FAIL:", e)
    key = Pin(53, Pin.IN, Pin.PULL_DOWN)

    try:
        if uart:
            send_cmd(uart, 0x01, 1)
        last_send = last_mode = 0
        fps = 0.0
        last_ms = time.ticks_ms()
        prev_hit = False

        # 激光时序 (detect space)
        lx_s = ly_s = 0.0
        l_lock = False
        l_hit_cnt = 0
        l_miss_cnt = 0
        last_lx = last_ly = 0
        last_laser_ms = 0

        # 靶心时序 (detect space)
        tgt_x = tgt_y = 0.0
        tgt_init = False
        last_tgt_ms = 0
        tgt_xh = tgt_yh = 0

        # 矩形(靶板外框)时序 (detect space) — 任意位置/倾斜检测
        rect_x = rect_y = rect_w = rect_h = rect_ang = 0.0
        rect_init = False
        last_rect_ms = 0
        rect_corners = None
        rect_valid = False
        rect_dx = rect_dy = 0          # 矩形中心相对靶心(原点)偏移(显示空间 px)
        rect_ang_i = 0                 # 倾斜角(int16, 0.1°)
        last_rect_send = 0

        srv_y = srv_p = 0.0
        srv_st = srv_ok = 0
        fc = 0

        print("=== 高帧率版: cv2轮廓检测(靶心)+cv2 LAB(激光) ===")

        while True:
            if key.value():
                if uart:
                    send_cmd(uart, 0x02, 0)
                time.sleep_ms(500)

            try:
                img = sensor.snapshot()
            except Exception as e:
                time.sleep_ms(30)
                continue

            # 本帧时间戳(供后续靶心保持/矩形保持/发送节流统一使用)
            now = time.ticks_ms()

            # ============== 检测 (cv2 小图靶心 / find_blobs激光) ==============
            if HAVE_CV2:
                arr = img.to_numpy_ref()              # 零拷贝 RGB888 ndarray
                small = cv2.resize(arr, (DETECT_W, DETECT_H))   # RVV 加速
                t_raw = detect_target_cv2(small)
                lx_d, ly_d, l_raw = detect_laser(img,
                    lx_s if l_lock else -1, ly_s if l_lock else -1, l_lock)
            else:
                t_raw = detect_target_legacy(img)
                lx_d, ly_d, l_raw = detect_laser(img,
                    lx_s if l_lock else -1, ly_s if l_lock else -1, l_lock)

            # ============== 矩形检测 (靶板外框, 任意位置/倾斜) ==============
            r_raw = detect_rect(small if HAVE_CV2 else None, img)

            # ============== 激光时序 (detect space) ==============
            if l_raw:
                l_hit_cnt = min(l_hit_cnt + 1, 32)
                l_miss_cnt = 0
                if not l_lock and l_hit_cnt == 1:
                    lx_s, ly_s = lx_d, ly_d
                else:
                    max_jump = 40
                    dx = lx_d - lx_s
                    dy = ly_d - ly_s
                    d2 = dx*dx + dy*dy
                    if d2 > max_jump * max_jump and d2 > 0:
                        scale = max_jump / math.sqrt(d2)
                        lx_d = lx_s + dx * scale
                        ly_d = ly_s + dy * scale
                    lx_s = lx_s*(1-LASER_POS_ALPHA) + lx_d*LASER_POS_ALPHA
                    ly_s = ly_s*(1-LASER_POS_ALPHA) + ly_d*LASER_POS_ALPHA
                if l_hit_cnt >= LASER_LOCK_FRAMES and not l_lock:
                    l_lock = True
                last_lx = lx_s; last_ly = ly_s
                last_laser_ms = time.ticks_ms()
            else:
                l_miss_cnt = min(l_miss_cnt + 1, 32)
                l_hit_cnt = 0
                if l_lock and l_miss_cnt >= LASER_LOST_FRAMES:
                    l_lock = False

            l_det = l_lock
            lx = lx_s if l_det else -1
            ly = ly_s if l_det else -1

            if fc % 30 == 0:
                print("[LASER] raw=%d lock=%d px=%d pos=(%.0f,%.0f)" % (l_raw, l_lock, g_laser_px, lx, ly))
            if fc % 30 == 0 and rect_valid:
                print("[RECT] c=(%d,%d) w=%d h=%d ang=%.1f valid=%d origin=%d" % (
                    rect_dx, rect_dy, int(rect_w), int(rect_h), rect_ang_i/10.0, rect_valid, target_valid))

            # ============== 靶心时序 (detect space) ==============
            if t_raw:
                tx, ty = t_raw
                if not tgt_init:
                    tgt_x, tgt_y, tgt_init = tx, ty, True
                else:
                    a = 0.4
                    tgt_x = tgt_x*(1-a) + tx*a
                    tgt_y = tgt_y*(1-a) + ty*a
                last_tgt_ms = time.ticks_ms()
                tgt_xh, tgt_yh = tgt_x, tgt_y
            elif tgt_init and time.ticks_diff(time.ticks_ms(), last_tgt_ms) < 400:
                tgt_x, tgt_y = tgt_xh, tgt_yh   # 短暂保持
            else:
                tgt_init = False

            target_valid = tgt_init

            # ============== 矩形(外框)处理: 显示空间 + 相对靶心偏移 ==============
            if r_raw:
                rcx, rcy, rw, rh, rang, rcorn = r_raw
                # 检测空间 → 显示空间
                rcx_s = rcx * SCALE_X
                rcy_s = rcy * SCALE_Y
                rw_s  = rw  * SCALE_X
                rh_s  = rh  * SCALE_Y
                if not rect_init:
                    rect_x, rect_y, rect_w, rect_h, rect_ang = rcx_s, rcy_s, rw_s, rh_s, rang
                    rect_init = True
                else:
                    a = RECT_SMOOTH_ALPHA
                    rect_x  = rect_x*(1-a) + rcx_s*a
                    rect_y  = rect_y*(1-a) + rcy_s*a
                    rect_w  = rect_w*(1-a) + rw_s*a
                    rect_h  = rect_h*(1-a) + rh_s*a
                    rect_ang = rect_ang*(1-a) + rang*a
                rect_corners = rcorn
                last_rect_ms = now
                rect_valid = True
            elif rect_init and time.ticks_diff(now, last_rect_ms) < 400:
                rect_valid = True   # 短暂保持
            else:
                rect_valid = False

            # ============== 矩形相对靶心(原点)的偏移 ==============
            if target_valid and rect_valid:
                rect_dx = int(rect_x - tgt_x)    # 显示空间, 右为正
                rect_dy = int(rect_y - tgt_y)    # 显示空间, 下为正
            else:
                rect_dx = rect_dy = 0
            rect_ang_i = int(rect_ang * 10)      # 0.1° 单位

            # ============== 误差合成 (放大到显示空间) ==============
            now = time.ticks_ms()
            l_det_hold = l_det or (last_laser_ms and
                         time.ticks_diff(now, last_laser_ms) < 500)
            if l_det_hold:
                vlx = lx if l_det else last_lx
                vly = ly if l_det else last_ly

            if l_det_hold and target_valid:
                # 显示空间误差
                ex = int((tgt_x - vlx) * SCALE_X)
                ey = int((tgt_y - vly) * SCALE_Y)
                valid_out = True
            elif l_det_hold and not target_valid:
                ex = ey = 0
                valid_out = True
            else:
                ex = ey = 0
                valid_out = False

            hit = l_det and target_valid and abs(ex) < HIT_TOLERANCE_PX and abs(ey) < HIT_TOLERANCE_PX
            hi_conf = bool(target_valid and l_det)
            prev_hit = hit

            sx_err = ex if abs(ex) > ERR_DEAD_ZONE else 0
            sy_err = ey if abs(ey) > ERR_DEAD_ZONE else 0

            if time.ticks_diff(now, last_send) >= COORD_INTERVAL_MS:
                send_coord(uart, sx_err, sy_err, valid=valid_out, hit=hit, hi_conf=hi_conf)
                last_send = now

            # ============== 矩形帧发送 (靶心为原点, 10Hz) ==============
            if time.ticks_diff(now, last_rect_send) >= RECT_FRAME_INTERVAL_MS:
                send_rect_frame(uart, rect_dx, rect_dy, rect_w, rect_h,
                                rect_ang_i, rect_valid, target_valid)
                last_rect_send = now

            if time.ticks_diff(now, last_mode) > 2000:
                if uart:
                    send_cmd(uart, 0x01, 1)
                last_mode = now

            sv = recv_status(uart)
            if sv:
                srv_y, srv_p, srv_st, srv_ok = sv[0], sv[1], sv[2], True

            # ============== OSD (显示空间坐标) ==============
            draw_cross(img, SENSOR_W//2, SENSOR_H//2)
            if l_det:
                ldx = int(lx * SCALE_X); ldy = int(ly * SCALE_Y)
                sz = 14
                img.draw_line(ldx-sz, ldy, ldx+sz, ldy, C_LASER, 2)
                img.draw_line(ldx, ldy-sz, ldx, ldy+sz, C_LASER, 2)
                img.draw_string_advanced(ldx+sz+2, ldy-8, 14, "R", color=C_LASER)
            if target_valid:
                tdx = int(tgt_x * SCALE_X); tdy = int(tgt_y * SCALE_Y)
                sz = 16
                img.draw_rectangle(tdx-sz, tdy-sz, sz*2, sz*2, C_RECT, 2)
                img.draw_circle(tdx, tdy, 3, (255,0,0), 2, fill=True)

            # ============== 矩形(外框) OSD ==============
            if rect_valid and rect_corners is not None:
                pts = []
                for (px, py) in rect_corners:
                    pts.append((int(px * SCALE_X), int(py * SCALE_Y)))
                for i in range(4):
                    x0, y0 = pts[i]; x1, y1 = pts[(i+1) % 4]
                    img.draw_line(x0, y0, x1, y1, C_RECT, 2)
            if target_valid:
                tdx2 = int(tgt_x * SCALE_X); tdy2 = int(tgt_y * SCALE_Y)
                img.draw_cross(tdx2, tdy2, 8, (0,255,0))   # 靶心原点(绿)
            if rect_valid:
                rdx = int(rect_x); rdy = int(rect_y)
                img.draw_circle(rdx, rdy, 3, C_RECT, 2, fill=True)
                if target_valid and rect_corners is not None:
                    img.draw_line(tdx2, tdy2, rdx, rdy, (0,229,255), 1)  # 偏移向量
                    img.draw_string_advanced((tdx2+rdx)//2+4, (tdy2+rdy)//2, 14,
                        "(%d,%d)" % (rect_dx, rect_dy), color=(0,229,255))
            if l_det and target_valid:
                ldx = int(lx * SCALE_X); ldy = int(ly * SCALE_Y)
                tdx = int(tgt_x * SCALE_X); tdy = int(tgt_y * SCALE_Y)
                dx = ldx - tdx; dy = ldy - tdy
                dp = int(math.sqrt(dx*dx + dy*dy))
                mx = (ldx + tdx)//2; my = (ldy + tdy)//2
                img.draw_line(ldx, ldy, tdx, tdy, C_LINE, 1)
                img.draw_string_advanced(mx+4, my-10, 14, "%dpx" % dp, color=C_LINE)

            lh = 22
            gl = "R:(%3d,%3d)" % (int(lx*SCALE_X), int(ly*SCALE_Y)) if l_det else "R: --"
            draw_text_bg(img, 4, 4, gl, 18, color=C_LASER)
            tt = "T:(%3d,%3d)" % (int(tgt_x*SCALE_X), int(tgt_y*SCALE_Y)) if target_valid else "T: --"
            draw_text_bg(img, 4, 4+lh, tt, 18, color=C_TARGET)
            if l_det and target_valid:
                st = "err:(%3d,%3d) %s" % (sx_err, sy_err, "HIT" if hit else "")
                draw_text_bg(img, 4, 4+lh*2, st, 18, color=C_LINE)
            if rect_valid:
                rr = "RECT:(%d,%d) %dx%d %+.1f" % (rect_dx, rect_dy, int(rect_w), int(rect_h), rect_ang_i/10.0)
                draw_text_bg(img, 4, 4+lh*3, rr, 16, color=C_RECT)
            if srv_ok:
                sn = STATES[min(int(srv_st), 5)]
                draw_text_bg(img, 4, SENSOR_H-24,
                    "Srv:Y%3d,P%3d[%s]" % (int(srv_y), int(srv_p), sn), 16, color=C_TEXT)

            n = time.ticks_ms()
            dt = time.ticks_diff(n, last_ms)
            last_ms = n
            if dt > 0:
                fps = fps*0.8 + 1000/dt*0.2 if fps > 0 else 1000/dt
            draw_text_bg(img, SENSOR_W-90, 4, "FPS:%.1f" % fps, 18)

            Display.show_image(img)
            if fc % 10 == 0:
                gc.collect()
            fc += 1

    except Exception as e:
        print("FATAL:", e)
    finally:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        print("Done.")

if __name__ == "__main__":
    run()
