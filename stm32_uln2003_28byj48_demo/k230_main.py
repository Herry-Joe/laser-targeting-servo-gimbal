# -*- coding: utf-8 -*-
# =============================================================
# 庐山派 K230 - 激光打靶追踪 v4.11 (打靶优化版)
#
# 核心思路:
#   YOLO 检测圆靶 (主力, 每 3 帧) + 矩形边框 (仅兜底, 每 12 帧)
#   激光闭环: ex = 靶心 - 激光 → UART → STM32 PID 云台
#   v4.11: 极致精简 - 砍掉所有非打靶必需的 OSD 描边/遮罩/GC
#          提升有效帧率, 专注打靶实时性
#
# 靶心: YOLO > 边框+偏移(BDR+O) > 边框中心(BORDER)
# 激光: 红色 LAB find_blobs
# 控制: 闭环误差 → UART → STM32 USART3 (PB10/PB11)
#
# 通信: UART2 @115200, GPIO05/06
#   坐标帧 0x3C 0x3B [XH][XL][YH][YL][FLAG] [CRC8] 0x01 0x01  (10字节)
#   命令帧 0x3C 0x3C [CMD][PARAM] 0x00 0x00 0x00 0x01 0x01
#
# 按键: Pin(53) 短按 → RECENTER
# 接线: GPIO05(TX)→STM32 PB11, GPIO06(RX)←STM32 PB10
# =============================================================

import uos, math, ujson, aicube, time, image, gc
from media.sensor import *
from media.display import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
from machine import Pin, UART, FPIOA

# ========================= 参数 =========================
DISPLAY_W  = 800
DISPLAY_H  = 480
OUT_W      = 1920
OUT_H      = 1080

# 激光 (CHN1 RGB565 800x480) — 红色 LAB 阈值 (v4.9: 用户实测优化)
RED_LASER_LAB      = [(52, 67, 16, 63, -36, 24)]
LASER_MIN_PX       = 3
LASER_MAX_PX       = 200
LASER_MASK_PAD     = 8

# 矩形边框 (CHN1 RGB565 800x480) — 2025电赛E题靶子边框
# v4.1: 外轮廓用 find_rects 边缘检测, RECT_LAB 仅用于 ROI 内白色边框厚度估算
RECT_LAB           = [(15, 28, -6, 10, -9, 9)]
RECT_MIN_AREA      = 5000
RECT_MIN_W         = 100
RECT_MIN_H         = 100
RECT_RATIO_MIN     = 0.3
RECT_RATIO_MAX     = 3.0
RECT_SMOOTH_ALPHA  = 0.30     # v4.0: 矩形框位置时序平滑系数

# v3.1: 偏移动态学习
#   offset = EMA(yolo_center - white_center), 只在两者同时有效时更新
OFFSET_LEARN_ALPHA = 0.05    # EMA 学习率 (越小越慢但越稳, 0.05≈收敛500ms)
OFFSET_MAX_PX      = 40      # 偏移绝对值上限 (像素, 防异常值)

# YOLO
YOLO_CANDIDATE_DIRS = ["/sdcard/yolo/","/sd/yolo/","/sdcard/Kmodel/","/sd/Kmodel/","/root/yolo25e/"]
YOLO_SKIP_FRAMES = 3    # v4.11: 每 3 帧跑一次 YOLO (约 8Hz), K230 KPU 负载降 33%
YOLO_STALE_MAX  = 7     # 连续 7 帧未更新 → 失效 (3 跳帧×7≈21 帧 ≈ 0.8s)
RECT_SKIP_FRAMES = 12   # v4.11: 每 12 帧检测一次矩形 (仅辅助兜底)

# 控制
HIT_TOLERANCE_PX  = 15
COORD_INTERVAL_MS = 40
ERR_DEAD_ZONE     = 1

# 颜色
C_LASER  = (255,0,0); C_TARGET = (0,128,255)
C_LINE   = (255,255,0); C_TEXT = (255,255,255); C_BG = (0,0,0)
C_RECT   = (255,128,0); C_OFFSET = (0,255,255)
C_RECT_OUTER = (0,255,0)       # v4.0: 外轮廓 — 亮绿色
C_RECT_INNER = (0,200,0)       # v4.0: 内轮廓(黑白分界线) — 中绿色

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

def find_yolo():
    for d in YOLO_CANDIDATE_DIRS:
        cfg = d + "deploy_config.json"
        try:
            uos.stat(cfg)
            with open(cfg) as f: conf = ujson.load(f)
            km = conf.get("kmodel_path","")
            kp = d + km if "/" not in km else km
            for k in [kp, d+"model_3356.cvimodel", d+"model_3356.kmodel",
                       d+"best_AnchorBaseDet_can2_10_s_20250730105226.cvimodel",
                       d+"best_AnchorBaseDet_can2_10_s_20250730105226.kmodel"]:
                try: uos.stat(k); return d, conf, k
                except OSError: continue
        except OSError: continue
    return None, None, None

# ==================== 检测函数 ====================
def detect_rect_contours(laser_img):
    """检测矩形边框的内外轮廓 (v4.1: 边缘检测)
    
    使用 find_rects() (AprilTag 四边形算法) 基于边缘对比度找矩形,
    不受颜色漂移影响, 不会误抓白色区域.
    
    返回: (cx, cy, ow, oh, ix, iy, iw, ih, thickness) 或 None
      cx,cy,ow,oh = 外矩形中心与宽高 (兼容旧 r_det[0..3])
      ix,iy,iw,ih = 内矩形左上角与宽高 (黑白分界线)
      thickness    = 边框估算厚度 (px)
    
    策略:
      1. find_rects() 基于边缘检测定位矩形 (黑框 vs 白色背景)
      2. 最优矩形 → 外轮廓
      3. 在外矩形 ROI 内找白色边框 blob → 估算厚度 → 内轮廓
    """
    try:
        # --- 第一步: 边缘检测找矩形 (AprilTag 算法) ---
        # 基于对比度, 不依赖颜色; threshold 越高越严格
        rects = laser_img.find_rects(threshold=2500)
        
        # fallback: 降低阈值再试一次
        if not rects:
            rects = laser_img.find_rects(threshold=1000)
        
        if not rects: return None
        
        # --- 第二步: 筛选最佳矩形 ---
        best, best_score = None, 0
        ox = oy = ow = oh = 0
        for r in rects:
            x, y, w, h = r.rect()
            if w < RECT_MIN_W or h < RECT_MIN_H: continue
            ratio = max(w,h) / min(w,h)
            if ratio < RECT_RATIO_MIN or ratio > RECT_RATIO_MAX: continue
            # 评分: 面积 × 归一化置信度
            mag = r.magnitude()
            score = (w * h) * (mag / 5000.0)
            if score > best_score:
                best_score = score; best = r
                ox, oy, ow, oh = x, y, w, h
        
        if best is None: return None
        
        cx = ox + ow // 2
        cy = oy + oh // 2
        
        # --- 第三步: 在外矩形 ROI 内找白色边框, 估算厚度 ---
        # 利用 LAB 阈值, 但限制 ROI 在外矩形内沿附近, 避免干扰
        roi_margin = max(2, min(ow, oh) // 10)
        inner_roi = (ox + roi_margin, oy + roi_margin,
                     ow - 2 * roi_margin, oh - 2 * roi_margin)
        
        thickness = max(2, min(ow, oh) // 20)  # 默认值兜底
        
        try:
            white_blobs = laser_img.find_blobs(RECT_LAB,
                roi=inner_roi, pixels_threshold=200, merge=True)
            if white_blobs:
                best_w = max(white_blobs, key=lambda b: b.area())
                _,_,bw,bh = best_w.rect()
                if bw > 10 and bh > 10:
                    thick_est = best_w.pixels() // (2 * (bw + bh))
                    thickness = max(2, min(thick_est, min(ow, oh) // 7))
        except:
            pass  # 用默认厚度
        
        # --- 第四步: 内轮廓 (边框内沿 = 黑白分界线) ---
        ix = ox + thickness
        iy = oy + thickness
        iw = ow - 2 * thickness
        ih = oh - 2 * thickness
        
        if iw < 10 or ih < 10: return None
        
        return (cx, cy, ow, oh, ix, iy, iw, ih, thickness)
    
    except Exception as e:
        print("rect err:", e)
    return None

# ==================== 串口 ====================
def crc8(data):
    """CRC8 (poly 0x07, init 0x00) — 与 STM32 端 K230_CRC8 逐位一致。
    用于坐标帧校验, 抵抗步进电机+TB6612 紧邻串口线带来的 EMI 误码。"""
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
    if not uart: return 0
    # 帧: 0x3C 0x3B [XH][XL][YH][YL][FLAG] [CRC8] 0x01 0x01  (10字节)
    # CRC8 覆盖 XH XL YH YL FLAG (5 字节), STM32 端逐字节重算并比对。
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
    if not uart: return 0
    buf = bytearray(9)
    buf[0]=0x3C; buf[1]=0x3C
    buf[2]=cmd; buf[3]=param
    for i in range(4,9): buf[i]=0
    buf[7]=1; buf[8]=1
    return uart.write(buf)

STATES=["W","T","L","C","D","X"]
def recv_status(uart):
    if not uart: return None
    try:
        if uart.any()<8: return None
        b=uart.read(8)
        if not b or len(b)!=8: return None
        if b[0]!=0x3C or b[1]!=0x3D: return None
        if (sum(b[0:7])&0xFF)!=b[7]: return None
        return (b[2]*270/255, b[3]*180/255, min(b[4],5),
                bool(b[5]&1), bool(b[5]&2), b[6])
    except: return None

# ==================== 主函数 ====================
def run():
    root, conf, kmodel = find_yolo()
    if not conf:
        print("ERROR: no model found"); return
    labels=conf["categories"]; conf_th=conf["confidence_threshold"]
    nms_th=conf["nms_threshold"]; isz=conf["img_size"]
    nc=conf["num_classes"]; nms_opt=conf["nms_option"]
    mtype=conf["model_type"]
    anchors = conf["anchors"][0]+conf["anchors"][1]+conf["anchors"][2] \
              if mtype=="AnchorBaseDet" else None

    fr_sz=[OUT_W,OUT_H]; strides=[8,16,32]
    w,h=isz[0],isz[1]; r=min(float(w)/OUT_W,float(h)/OUT_H)
    nw,nh=int(r*OUT_W),int(r*OUT_H)
    dw,dh=(w-nw)/2,(h-nh)/2
    top=int(round(dh-0.1)); bot=int(round(dh+0.1))
    left=int(round(dw-0.1)); rig=int(round(dw+0.1))

    kpu=nn.kpu(); ai2d=nn.ai2d()
    kpu.load_kmodel(kmodel)
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0,top,bot,left,rig], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_b=ai2d.build([1,3,OUT_H,OUT_W],[1,3,h,w])

    # 摄像头
    sensor=Sensor(id=2)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_W, height=DISPLAY_H)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=OUT_W, height=OUT_H, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    sensor.set_framesize(width=DISPLAY_W, height=DISPLAY_H, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)

    Display.bind_layer(**sensor.bind_info(x=0,y=0,chn=CAM_CHN_ID_0),
                       layer=Display.LAYER_VIDEO1)
    Display.init(Display.ST7701, to_ide=True)
    osd=image.Image(DISPLAY_W, DISPLAY_H, image.ARGB8888)

    uart=None
    try:
        fpioa=FPIOA()
        fpioa.set_function(5, fpioa.UART2_TXD)
        fpioa.set_function(6, fpioa.UART2_RXD)
        uart=UART(UART.UART2, 115200, bits=UART.EIGHTBITS,
                   parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print("[UART] OK")
    except Exception as e: print("[UART] FAIL:", e)
    key=Pin(53, Pin.IN, Pin.PULL_DOWN)

    try:
        MediaManager.init(); sensor.run()
        if uart: send_cmd(uart, 0x01, 1)
        else: print("[WARN] no uart")

        data=np.ones((1,3,h,w), dtype=np.uint8)
        ai2d_out=nn.from_numpy(data)
        last_send=last_mode=0; fps=0.0; last_ms=time.ticks_ms()
        prev_hit=False
        srv_y=0.0; srv_p=0.0; srv_st=0; srv_ok=False

        # EMA (v3.2: 下限 0.22→0.30, 减少靶心追踪滞后)
        def ema_alpha(e):
            if e>120: return 0.55
            elif e>40: return 0.40
            elif e>10: return 0.35
            else: return 0.30

        tgt_x=tgt_y=0; tgt_init=False

        # v4.1: find_rects 边缘检测, 每3帧跑一次省性能
        rect_smooth = None  # (cx, cy, ow, oh, thickness) 平滑后状态
        rect_skip_cnt = 0   # 矩形跳帧计数器
        yolo_skip_cnt = 0   # YOLO 跳帧计数器
        off_x=0; off_y=0; off_learned=False; border_seen=0
        last_lx=0; last_ly=0; last_laser_ms=0  # 激光保持: 短暂丢失时用上次位置

        fc=0; yolo_skip=YOLO_SKIP_FRAMES-1; yolo_stale=99; yolo_ok=False; yx=0; yy=0
        print("=== v4.11 打靶优化版 ===")
        print("Model:", labels[0], "conf=%.2f  YOLO=1/%df  Rect=1/%df" %
              (conf_th, YOLO_SKIP_FRAMES, RECT_SKIP_FRAMES))

        while True:
            if key.value():
                if uart: send_cmd(uart, 0x02, 0)
                time.sleep_ms(500)

            # --- 取帧 ---
            try: laser_img=sensor.snapshot(chn=CAM_CHN_ID_1)
            except: laser_img=None
            try:
                rgb=sensor.snapshot(chn=CAM_CHN_ID_2)
                if rgb.format()!=image.RGBP888: continue
            except: time.sleep_ms(30); continue

            osd.clear()
            draw_cross(osd, DISPLAY_W//2, DISPLAY_H//2)

            # --- 激光检测 (800x480) ---
            l_det=False; lx=ly=-1
            if laser_img:
                lbs=laser_img.find_blobs(RED_LASER_LAB, False, x_stride=2,
                    y_stride=2, pixels_threshold=LASER_MIN_PX, merge=True, margin=True)
                if lbs:
                    bl=max(lbs, key=lambda b: b.pixels())
                    if LASER_MIN_PX <= bl.pixels() <= LASER_MAX_PX:
                        bx,by,bw,bh = bl.rect()
                        lx=bl.cx(); ly=bl.cy(); l_det=True
                        last_lx=lx; last_ly=ly; last_laser_ms=time.ticks_ms()
                        sz=14
                        osd.draw_line(lx-sz,ly,lx+sz,ly,C_LASER,2)
                        osd.draw_line(lx,ly-sz,lx,ly+sz,C_LASER,2)
                        osd.draw_string_advanced(lx+sz+2,ly-8,14,"R",color=C_LASER)

            # --- YOLO 推理 (每 YOLO_SKIP_FRAMES 帧跑一次, 节省 KPU) ---
            dets=[]; yolo_skip_cnt += 1
            run_yolo = (yolo_skip_cnt >= YOLO_SKIP_FRAMES)
            if not run_yolo:
                # 复用上一帧结果: yolo_ok/yx/yy 保持原值不变
                pass
            else:
                yolo_skip_cnt = 0
                yolo_ok=False; yx=yy=0   # 本帧跑 YOLO, 先清空, 检测到才置 True
                try:
                    inp=rgb.to_numpy_ref()
                    it=nn.from_numpy(inp)
                    ai2d_b.run(it, ai2d_out)
                    kpu.set_input_tensor(0, ai2d_out)
                    kpu.run()
                    res=[]
                    for i in range(kpu.outputs_size()):
                        od=kpu.get_output_tensor(i); rv=od.to_numpy()
                        rv=rv.reshape((rv.shape[0]*rv.shape[1]*rv.shape[2]*rv.shape[3]))
                        del od; res.append(rv)
                    if mtype=="AnchorBaseDet":
                        dets=aicube.anchorbasedet_post_process(res[0],res[1],res[2],
                            isz,fr_sz,strides,nc,conf_th,nms_th,anchors,nms_opt)
                    else:
                        dets=aicube.anchorfreedet_post_process(res[0],res[1],res[2],
                            isz,fr_sz,strides,nc,conf_th,nms_th,nms_opt)
                    del res
                except Exception as e: print("YOLO err:", e)
                gc.collect()

                if dets:
                    bd=max(dets,key=lambda b:b[1]) if len(dets)>1 else dets[0]
                    x1,y1,x2,y2=bd[2],bd[3],bd[4],bd[5]
                    sx=(x2-x1)*DISPLAY_W/OUT_W; sy=(y2-y1)*DISPLAY_H/OUT_H
                    yx=int(x1*DISPLAY_W/OUT_W + sx//2)
                    yy=int(y1*DISPLAY_H/OUT_H + sy//2)
                    yolo_ok=True; yolo_stale=0
            if yolo_ok:
                yolo_stale += 1
                if yolo_stale > YOLO_STALE_MAX:  # 连续跳过太多帧 → 失效
                    yolo_ok = False

            # --- 矩形边框检测 (v4.11: 降频到每12帧, 仅辅助兜底) ---
            rect_skip_cnt += 1
            if rect_skip_cnt >= RECT_SKIP_FRAMES or rect_smooth is None:
                rect_skip_cnt = 0
                r_raw = detect_rect_contours(laser_img)
            else:
                r_raw = (rect_smooth[0], rect_smooth[1],
                         rect_smooth[2], rect_smooth[3],
                         0, 0, 0, 0, rect_smooth[4]) if rect_smooth else None
            
            if r_raw:
                border_seen = min(border_seen+1, 255)
                cx, cy, ow, oh, _, _, _, _, thick = r_raw
                # 矩形框时序平滑 (减少抖动)
                if rect_smooth is None:
                    rect_smooth = (cx, cy, ow, oh, thick)
                else:
                    a = RECT_SMOOTH_ALPHA
                    scx = int(rect_smooth[0]*(1-a) + cx*a)
                    scy = int(rect_smooth[1]*(1-a) + cy*a)
                    sow = int(rect_smooth[2]*(1-a) + ow*a)
                    soh = int(rect_smooth[3]*(1-a) + oh*a)
                    sthick = rect_smooth[4]  # 厚度不做平滑
                    rect_smooth = (scx, scy, sow, soh, sthick)
                # r_det 保持 (cx,cy,w,h) 兼容偏移学习
                r_det = (rect_smooth[0], rect_smooth[1],
                         rect_smooth[2], rect_smooth[3])
            else:
                border_seen = 0
                r_det = None
                rect_smooth = None

            # ============== v3.5: 双源定位 + 偏移学习 ==============
            # 策略: YOLO > 边框+偏移(BDR+O) > 边框中心(BORDER)
            if yolo_ok:
                target_x=yx; target_y=yy
                target_valid=True; target_from="YOLO"
                # 学习 边框→YOLO 偏移
                if r_det and border_seen>=3:
                    rx, ry = r_det[0], r_det[1]
                    b_ox = yx - rx; b_oy = yy - ry
                    if abs(b_ox) <= OFFSET_MAX_PX and abs(b_oy) <= OFFSET_MAX_PX:
                        if not off_learned:
                            off_x = b_ox; off_y = b_oy; off_learned = True
                        else:
                            off_x = int(off_x*(1-OFFSET_LEARN_ALPHA) + b_ox*OFFSET_LEARN_ALPHA)
                            off_y = int(off_y*(1-OFFSET_LEARN_ALPHA) + b_oy*OFFSET_LEARN_ALPHA)
            elif r_det and off_learned:
                target_x = r_det[0] + off_x
                target_y = r_det[1] + off_y
                target_valid = True; target_from = "BDR+O"
            elif r_det:
                target_x = r_det[0]; target_y = r_det[1]
                target_valid = True; target_from = "BORDER"
            else:
                target_x = target_y = 0
                target_valid = False; target_from = "LOST"

            # EMA 平滑靶心
            if target_valid:
                if not tgt_init:
                    tgt_x=target_x; tgt_y=target_y; tgt_init=True
                else:
                    a=ema_alpha(max(abs(target_x-tgt_x), abs(target_y-tgt_y)))
                    tgt_x=int(tgt_x*(1-a)+target_x*a)
                    tgt_y=int(tgt_y*(1-a)+target_y*a)

            # --- OSD 靶心 (v4.11: 精简版, 只画靶心方框) ---
            if target_valid:
                tc = C_TARGET if target_from=="YOLO" else C_RECT
                sz=16
                osd.draw_rectangle(tgt_x-sz,tgt_y-sz,sz*2,sz*2,tc,2)
                osd.draw_circle(tgt_x,tgt_y,3,(255,0,0),2,fill=True)
                osd.draw_string_advanced(tgt_x+sz+2,tgt_y-8,14,target_from,color=tc)

            # --- 连线 + 距离 ---
            if l_det and target_valid:
                dx=l_det and (lx-tgt_x) or 0
                dy=l_det and (ly-tgt_y) or 0
                dp=int(math.sqrt(dx*dx+dy*dy)) if (l_det and target_valid) else 0
                mx=(lx+tgt_x)//2; my=(ly+tgt_y)//2
                osd.draw_line(lx,ly,tgt_x,tgt_y,C_LINE,1)
                osd.draw_string_advanced(mx+4,my-10,14,"%dpx"%dp,color=C_LINE)

            # --- 误差 → 串口 (激光闭环 + 激光保持 + 绝对靶心兜底) ---
            now=time.ticks_ms()
            # 激光保持: 短暂丢失 (<500ms) 时虚拟激光位置 = 上次有效位置
            # 避免激光一丢就切到绝对靶心导致云台猛偏
            LASER_HOLD_MS = 500
            l_det_hold = l_det or (last_laser_ms and
                         time.ticks_diff(now, last_laser_ms) < LASER_HOLD_MS)
            if l_det_hold:
                vlx = lx if l_det else last_lx
                vly = ly if l_det else last_ly
            if l_det_hold and target_valid:
                ex=tgt_x-vlx; ey=tgt_y-vly          # 闭环: 激光追靶心
            elif target_valid:
                ex=tgt_x-DISPLAY_W//2; ey=tgt_y-DISPLAY_H//2  # 绝对靶心
            else:
                ex=ey=0
            hit=l_det and target_valid and abs(ex)<HIT_TOLERANCE_PX and abs(ey)<HIT_TOLERANCE_PX
            prev_hit=hit

            # v3.2: 误差直发 — 靶心已做EMA平滑, STM32 PID自带deriv/out滤波
            #        去掉二次EMA, 消除 ~500ms 小误差响应延迟
            sx_err = ex if abs(ex)>ERR_DEAD_ZONE else 0
            sy_err = ey if abs(ey)>ERR_DEAD_ZONE else 0

            if time.ticks_diff(now, last_send) >= COORD_INTERVAL_MS:
                send_coord(uart, sx_err, sy_err,
                          valid=target_valid or l_det,
                          hit=hit, hi_conf=target_valid)
                last_send=now

            # 每2s重发AIM
            if time.ticks_diff(now, last_mode)>2000:
                if uart: send_cmd(uart, 0x01, 1)
                last_mode=now

            # STM32回传
            sv=recv_status(uart)
            if sv: srv_y,srv_p,srv_st,srv_ok=sv[0],sv[1],sv[2],True

            # 调试
            if fc%60==0:
                print("[DBG] FPS=%.1f T=(%d,%d)[%s] err=(%d,%d)" % (
                    fps, tgt_x, tgt_y, target_from,
                    sx_err, sy_err))

            # OSD 信息
            lh=22
            gl="R:(%3d,%3d)"%(lx,ly) if l_det else "R: --"
            draw_text_bg(osd,4,4,gl,18,color=C_LASER)
            tt="T:(%3d,%3d) %s"%(tgt_x,tgt_y,target_from) if target_valid else "T: --"
            color_t = C_TARGET if target_from=="YOLO" else C_RECT
            draw_text_bg(osd,4,4+lh,tt,18,color=color_t)
            if l_det and target_valid:
                st="err:(%3d,%3d) %s"%(sx_err,sy_err,"HIT" if hit else "")
                draw_text_bg(osd,4,4+lh*2,st,18,color=C_LINE)
            if srv_ok:
                sn=STATES[min(srv_st,5)]
                draw_text_bg(osd,4,DISPLAY_H-24,
                    "Srv:Y%3d,P%3d[%s]"%(int(srv_y),int(srv_p),sn),16,color=C_TEXT)

            # FPS
            n=time.ticks_ms(); dt=time.ticks_diff(n,last_ms); last_ms=n
            if dt>0: fps=fps*0.8+1000/dt*0.2 if fps>0 else 1000/dt
            draw_text_bg(osd,DISPLAY_W-90,4,"FPS:%.1f"%fps,18)

            Display.show_image(osd, 0, 0, Display.LAYER_OSD3)

            fc+=1
            # v4.11: 移除每 30 帧的 gc.collect(), 避免 GC 卡顿;
            # 内存由 MicroPython 自动管理, KPU dets/res 已在 YOLO 阶段 del+gc

    except Exception as e: print("FATAL:", e)
    finally:
        if 'ai2d_out' in locals(): del ai2d_out
        sensor.stop(); Display.deinit(); MediaManager.deinit()
        gc.collect(); time.sleep(1); nn.shrink_memory_pool()
        print("Done.")

if __name__=="__main__": run()
