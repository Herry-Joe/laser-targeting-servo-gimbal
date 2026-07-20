# -*- coding: utf-8 -*-
# =============================================================
# 庐山派 K230 - 激光打靶追踪  (非 YOLO 版 / 低延迟)
#
# 核心思路 (参考 庐山派 靶心识别 main_lushan_k230.py):
#   靶心: find_rects 边缘检测矩形边框 (外框中心 = 靶环中心), 不依赖神经网络
#   激光: 红色 LAB find_blobs (多阈值 + 局部ROI + 最近邻跟踪)
#   闭环: err = 靶心 - 激光 → UART → STM32 PID 云台
#
# 为什么去掉 YOLO:
#   原版每 3 帧跑一次 1920x1080 YOLO 推理, 靶心更新仅 ~3~5Hz, 是最大延迟源。
#   本版纯 CV (find_rects + find_blobs) 每帧可运行, 整环延迟从 >150ms 降到近帧级(<50ms),
#   且 STM32 端新增"命中粘滞锁存", 激光压到靶心即停稳, 不再抖动。
#
# 通信: UART2 @115200, GPIO05/06
#   坐标帧 0x3C 0x3B [XH][XL][YH][YL][FLAG] [CRC8] 0x01 0x01  (10字节)
#     FLAG bit0 = valid (靶心+激光都识别)
#     FLAG bit1 = hit   (误差 < 命中容差)
#     FLAG bit2 = hi_conf (靶心与激光都可靠识别 → STM32 用紧死区精确定中)
#   命令帧 0x3C 0x3C [CMD][PARAM] 0x00 0x00 0x00 0x01 0x01
#
# 接线: GPIO05(TX)→STM32 PB11, GPIO06(RX)←STM32 PB10
# =============================================================

import uos, math, time, image, gc
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, UART, FPIOA

# ========================= 参数 =========================
DISPLAY_W  = 800
DISPLAY_H  = 480

# 激光 (RGB565 800x480) — 红色 LAB 阈值
RED_LASER_LAB_MERGED = [(59, 71, 48, 70, 6, 25)]   # 用户实测激光 LAB: L=59~71, A=48~70, B=6~25
RED_LASER_LAB_WIDE   = [(45, 75, 0, 75, -45, 40)]  # 兜底宽阈值
LASER_MIN_PX        = 3
LASER_MAX_PX        = 600
LASER_WASHOUT_PX    = 6000      # 整屏泛红/反光拒识
LASER_LOCK_FRAMES   = 2          # 连续 N 帧命中才锁定
LASER_LOST_FRAMES   = 5          # 连续 N 帧丢失才释放
LASER_POS_ALPHA     = 0.35       # 位置 EMA
LASER_ROI_SIZE      = 81         # 锁定后局部ROI
LASER_TRACK_MAX_D   = 60         # 最近邻跟踪最大距离

# 矩形边框 (RGB565 800x480) — 靶环外黑框
RECT_LAB           = [(15, 28, -6, 10, -9, 9)]
RECT_MIN_AREA      = 5000
RECT_MIN_W         = 100
RECT_MIN_H         = 100
RECT_RATIO_MIN     = 0.3
RECT_RATIO_MAX     = 3.0
RECT_SMOOTH_ALPHA  = 0.30        # 靶心时序平滑(抑制边框检测跳变)

# 控制
HIT_TOLERANCE_PX  = 15           # 命中容差(像素): 误差 < 此值视为命中
COORD_INTERVAL_MS = 25           # 坐标发送间隔(ms): 40Hz (无YOLO, 每帧都能算)
ERR_DEAD_ZONE     = 1

# 颜色
C_LASER  = (255,0,0); C_TARGET = (0,128,255)
C_LINE   = (255,255,0); C_TEXT = (255,255,255); C_BG = (0,0,0)
C_RECT   = (255,128,0)

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

def detect_laser(laser_img, prev_x, prev_y, locked):
    """多阈值+局部ROI+最近邻跟踪的激光检测。

    返回: (raw_x, raw_y, ok)
    """
    if not laser_img:
        return -1, -1, False

    blobs = None
    roi = None
    search_mode = "global"

    # --- 锁定后先做局部 ROI 搜索 ---
    if locked and prev_x >= 0 and prev_y >= 0:
        hs = LASER_ROI_SIZE // 2
        rx = max(0, prev_x - hs)
        ry = max(0, prev_y - hs)
        rw = min(LASER_ROI_SIZE, DISPLAY_W - rx)
        rh = min(LASER_ROI_SIZE, DISPLAY_H - ry)
        if rw >= 10 and rh >= 10:
            roi = (rx, ry, rw, rh)
            search_mode = "roi"
            blobs = laser_img.find_blobs(RED_LASER_LAB_MERGED, False, roi=roi,
                x_stride=1, y_stride=1, pixels_threshold=LASER_MIN_PX,
                merge=True, margin=True)

    # --- ROI 未命中 → 全局合并阈值搜索 ---
    if not blobs:
        search_mode = "global"
        blobs = laser_img.find_blobs(RED_LASER_LAB_MERGED, False,
            x_stride=2, y_stride=2, pixels_threshold=LASER_MIN_PX,
            merge=True, margin=True)

    # --- 全局仍无 → 兜底放宽阈值 ---
    if not blobs:
        search_mode = "wide"
        blobs = laser_img.find_blobs(RED_LASER_LAB_WIDE, False,
            x_stride=1, y_stride=1, pixels_threshold=LASER_MIN_PX,
            merge=True, margin=True)

    if not blobs:
        return -1, -1, False

    # 面积筛选(去整屏反光/噪声)
    cands = []
    for b in blobs:
        px = b.pixels()
        if not (LASER_MIN_PX <= px <= LASER_WASHOUT_PX):
            continue
        bw, bh = b.w(), b.h()
        if bw < 1 or bh < 1:
            continue
        ratio = float(max(bw, bh)) / float(min(bw, bh))
        if ratio > 2.5:
            continue
        fill = float(px) / float(bw * bh)
        if fill < 0.3:
            continue
        cands.append((b, px, ratio, fill))

    if not cands:
        return -1, -1, False

    # 锁定后优先最近邻, 未锁定或跳变过大用综合评分(面积×紧凑度)
    if locked and prev_x >= 0 and prev_y >= 0:
        best_data = min(cands, key=lambda t: (t[0].cx()-prev_x)**2 + (t[0].cy()-prev_y)**2)
        d2 = (best_data[0].cx()-prev_x)**2 + (best_data[0].cy()-prev_y)**2
        if d2 > LASER_TRACK_MAX_D * LASER_TRACK_MAX_D:
            best_data = max(cands, key=lambda t: t[1] * t[3])
    else:
        best_data = max(cands, key=lambda t: t[1] * t[3])

    best = best_data[0]
    return best.cx(), best.cy(), True

def detect_rect_contours(laser_img):
    """检测矩形边框(靶环外黑框) → 返回靶心中心 (cx, cy) 或 None。

    使用 find_rects() (AprilTag 四边形算法) 基于边缘对比度找矩形,
    不依赖颜色, 不受光照/颜色漂移影响。外矩形中心即靶环中心(同心靶)。
    """
    try:
        rects = laser_img.find_rects(threshold=2500)
        if not rects:
            rects = laser_img.find_rects(threshold=1000)
        if not rects:
            return None

        # 筛选最佳矩形(面积×边缘置信度)
        best, best_score = None, 0
        ox = oy = ow = oh = 0
        for r in rects:
            x, y, w, h = r.rect()
            if w < RECT_MIN_W or h < RECT_MIN_H:
                continue
            ratio = max(w, h) / min(w, h)
            if ratio < RECT_RATIO_MIN or ratio > RECT_RATIO_MAX:
                continue
            mag = r.magnitude()
            score = (w * h) * (mag / 5000.0)
            if score > best_score:
                best_score = score
                best = r
                ox, oy, ow, oh = x, y, w, h

        if best is None:
            return None

        # 外框中心 = 靶环中心(同心靶近似)
        cx = ox + ow // 2
        cy = oy + oh // 2
        return (cx, cy, ow, oh)
    except Exception as e:
        print("rect err:", e)
    return None

# ==================== 串口 ====================
def crc8(data):
    """CRC8 (poly 0x07, init 0x00) — 与 STM32 端逐字节一致。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
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

STATES=["W","T","L","C","D","X"]
def recv_status(uart):
    if not uart:
        return None
    try:
        if uart.any()<8:
            return None
        b=uart.read(8)
        if not b or len(b)!=8:
            return None
        if b[0]!=0x3C or b[1]!=0x3D:
            return None
        if (sum(b[0:7])&0xFF)!=b[7]:
            return None
        return (b[2]*270/255, b[3]*180/255, min(b[4],5),
                bool(b[5]&1), bool(b[5]&2), b[6])
    except:
        return None

# ==================== 主函数 ====================
def run():
    # ---- 摄像头 (庐山派单通道 RGB565, 简洁可靠) ----
    sensor = Sensor(width=DISPLAY_W, height=DISPLAY_H)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_W, height=DISPLAY_H)
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("[CAM] OK")

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
        else:
            print("[WARN] no uart")

        last_send = last_mode = 0
        fps = 0.0
        last_ms = time.ticks_ms()
        prev_hit = False

        # 激光时序状态
        lx_s = ly_s = 0
        l_lock = False
        l_hit_cnt = 0
        l_miss_cnt = 0
        last_lx = last_ly = 0
        last_laser_ms = 0

        # 靶心时序状态
        rect_smooth = None
        tgt_x = tgt_y = 0
        tgt_init = False
        last_tgt_ms = 0
        tgt_xh = tgt_yh = 0

        srv_y = srv_p = 0.0
        srv_st = srv_ok = 0
        fc = 0

        print("=== 非YOLO 版: find_rects(靶环) + find_blobs(激光) ===")

        while True:
            if key.value():
                if uart:
                    send_cmd(uart, 0x02, 0)
                time.sleep_ms(500)

            # --- 取帧 (单通道 RGB565) ---
            try:
                img = sensor.snapshot()
            except Exception as e:
                time.sleep_ms(30)
                continue

            # ============== 激光检测 (每帧) ==============
            raw_det = False
            rlx = rly = -1
            rlx, rly, raw_det = detect_laser(img, lx_s if l_lock else -1,
                                             ly_s if l_lock else -1, l_lock)

            if raw_det:
                l_hit_cnt = min(l_hit_cnt + 1, 32)
                l_miss_cnt = 0
                if not l_lock and l_hit_cnt == 1:
                    lx_s, ly_s = rlx, rly
                else:
                    max_jump = 50
                    dx = rlx - lx_s
                    dy = rly - ly_s
                    d2 = dx*dx + dy*dy
                    if d2 > max_jump * max_jump:
                        scale = max_jump / math.sqrt(d2)
                        rlx = int(lx_s + dx * scale)
                        rly = int(ly_s + dy * scale)
                    lx_s = int(lx_s*(1-LASER_POS_ALPHA) + rlx*LASER_POS_ALPHA)
                    ly_s = int(ly_s*(1-LASER_POS_ALPHA) + rly*LASER_POS_ALPHA)
                if l_hit_cnt >= LASER_LOCK_FRAMES:
                    if not l_lock:
                        l_lock = True
                    last_lx = lx_s
                    last_ly = ly_s
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
                print("[LASER] det=%d lock=%d hit=%d miss=%d pos=(%d,%d)" % (
                    raw_det, l_lock, l_hit_cnt, l_miss_cnt,
                    lx if l_det else -1, ly if l_det else -1))

            # ============== 靶心检测 (非YOLO: find_rects, 每帧) ==============
            r_raw = detect_rect_contours(img)
            if r_raw:
                cx, cy, ow, oh = r_raw
                if rect_smooth is None:
                    rect_smooth = (cx, cy, ow, oh)
                else:
                    a = RECT_SMOOTH_ALPHA
                    scx = int(rect_smooth[0]*(1-a) + cx*a)
                    scy = int(rect_smooth[1]*(1-a) + cy*a)
                    sow = int(rect_smooth[2]*(1-a) + ow*a)
                    soh = int(rect_smooth[3]*(1-a) + oh*a)
                    rect_smooth = (scx, scy, sow, soh)
                target_x = rect_smooth[0]
                target_y = rect_smooth[1]
                target_valid = True
            else:
                rect_smooth = None
                target_valid = False

            # 靶心 EMA 平滑(轻量, 防边框检测抖)
            if target_valid:
                if not tgt_init:
                    tgt_x = target_x
                    tgt_y = target_y
                    tgt_init = True
                else:
                    dx = target_x - tgt_x
                    dy = target_y - tgt_y
                    a = 0.4   # 较激进(低延迟), 比 YOLO 版更快收敛
                    tgt_x = int(tgt_x*(1-a) + target_x*a)
                    tgt_y = int(tgt_y*(1-a) + target_y*a)

            # ============== 误差合成 ==============
            now = time.ticks_ms()
            # 靶心保持(短暂丢失沿用上次)
            TARGET_HOLD_MS = 400
            if target_valid:
                last_tgt_ms = now
                tgt_xh, tgt_yh = tgt_x, tgt_y
            elif last_tgt_ms and time.ticks_diff(now, last_tgt_ms) < TARGET_HOLD_MS:
                target_valid = True
                tgt_x, tgt_y = tgt_xh, tgt_yh

            # 激光保持
            LASER_HOLD_MS = 500
            l_det_hold = l_det or (last_laser_ms and
                         time.ticks_diff(now, last_laser_ms) < LASER_HOLD_MS)
            if l_det_hold:
                vlx = lx if l_det else last_lx
                vly = ly if l_det else last_ly

            # 只有激光+靶心都到位才发误差
            if l_det_hold and target_valid:
                ex = tgt_x - vlx
                ey = tgt_y - vly
                valid_out = True
            elif l_det_hold and not target_valid:
                ex = 0
                ey = 0
                valid_out = True      # 激光在但无靶心 → 保持位置
            else:
                ex = 0
                ey = 0
                valid_out = False      # 无激光 → 不发误差, STM32 冻结

            hit = l_det and target_valid and abs(ex) < HIT_TOLERANCE_PX and abs(ey) < HIT_TOLERANCE_PX
            # 高置信度: 靶心与激光都可靠识别
            hi_conf = bool(target_valid and l_det)
            prev_hit = hit

            sx_err = ex if abs(ex) > ERR_DEAD_ZONE else 0
            sy_err = ey if abs(ey) > ERR_DEAD_ZONE else 0

            if time.ticks_diff(now, last_send) >= COORD_INTERVAL_MS:
                send_coord(uart, sx_err, sy_err,
                           valid=valid_out, hit=hit, hi_conf=hi_conf)
                last_send = now

            # 每2s重发AIM
            if time.ticks_diff(now, last_mode) > 2000:
                if uart:
                    send_cmd(uart, 0x01, 1)
                last_mode = now

            # STM32 回传
            sv = recv_status(uart)
            if sv:
                srv_y, srv_p, srv_st, srv_ok = sv[0], sv[1], sv[2], True

            # ============== OSD ==============
            draw_cross(img, DISPLAY_W//2, DISPLAY_H//2)

            if l_det:
                sz = 14
                img.draw_line(lx-sz, ly, lx+sz, ly, C_LASER, 2)
                img.draw_line(lx, ly-sz, lx, ly+sz, C_LASER, 2)
                img.draw_string_advanced(lx+sz+2, ly-8, 14, "R", color=C_LASER)
                if not raw_det:
                    img.draw_circle(lx, ly, 6, C_LASER, 1)

            if target_valid:
                sz = 16
                img.draw_rectangle(tgt_x-sz, tgt_y-sz, sz*2, sz*2, C_RECT, 2)
                img.draw_circle(tgt_x, tgt_y, 3, (255,0,0), 2, fill=True)

            if l_det and target_valid:
                dx = lx - tgt_x
                dy = ly - tgt_y
                dp = int(math.sqrt(dx*dx + dy*dy))
                mx = (lx + tgt_x)//2
                my = (ly + tgt_y)//2
                img.draw_line(lx, ly, tgt_x, tgt_y, C_LINE, 1)
                img.draw_string_advanced(mx+4, my-10, 14, "%dpx" % dp, color=C_LINE)

            lh = 22
            gl = "R:(%3d,%3d)" % (lx, ly) if l_det else "R: --"
            draw_text_bg(img, 4, 4, gl, 18, color=C_LASER)
            tt = "T:(%3d,%3d)" % (tgt_x, tgt_y) if target_valid else "T: --"
            draw_text_bg(img, 4, 4+lh, tt, 18, color=C_TARGET)
            if l_det and target_valid:
                st = "err:(%3d,%3d) %s" % (sx_err, sy_err, "HIT" if hit else "")
                draw_text_bg(img, 4, 4+lh*2, st, 18, color=C_LINE)
            if srv_ok:
                sn = STATES[min(int(srv_st), 5)]
                draw_text_bg(img, 4, DISPLAY_H-24,
                    "Srv:Y%3d,P%3d[%s]" % (int(srv_y), int(srv_p), sn), 16, color=C_TEXT)

            n = time.ticks_ms()
            dt = time.ticks_diff(n, last_ms)
            last_ms = n
            if dt > 0:
                fps = fps*0.8 + 1000/dt*0.2 if fps > 0 else 1000/dt
            draw_text_bg(img, DISPLAY_W-90, 4, "FPS:%.1f" % fps, 18)

            Display.show_image(img)

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
