# -*- coding: utf-8 -*-
# =============================================================
# K230 矩形检测模块 (庐山派 / CanMV, OpenCV + RVV 加速)
#
# 用于 2023 电赛 E 题: 识别靶板上的黑色矩形外框(A4 框),
# 支持任意位置、任意倾斜。以 cv2 最小面积旋转矩形(minAreaRect)
# 一次得到 中心 / 宽 / 高 / 倾斜角 / 四角, 天然支持旋转矩形。
#
# 输入 : detect space 的 RGB888 ndarray (DETECT_H, DETECT_W, 3)
# 输出 : (cx, cy, w, h, angle, corners) 检测空间; 无则返回 None
#   - cx,cy    : 矩形中心(检测空间像素)
#   - w,h      : 矩形宽高(检测空间像素, 已做长边交换归一化)
#   - angle    : 倾斜角(度, 归一化到 [-45,45))
#   - corners  : 4x2 ndarray, 矩形四角(检测空间)
#
# 无 cv2 时回退到 CanMV find_rects (AprilTag 四边形检测)。
# 参考: 01Studio K230 笔记《基于矩形透射变换的矩形检测及其中心坐标计算》
# =============================================================

import math

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

# ========================= 可调参数 =========================
RECT_ADAPT_BLK   = 11      # 自适应阈值块大小(奇数)
RECT_ADAPT_C     = 5       # 自适应阈值常数
RECT_MIN_AREA_CV = 200     # 检测空间最小轮廓面积(外框较大)
RECT_MIN_RATIO   = 0.25    # 宽高比下限(过扁/过窄排除)
RECT_MAX_RATIO   = 4.0     # 宽高比上限
RECT_SMOOTH_ALPHA = 0.30   # 主程序对矩形量做 EMA 平滑的系数(参考)

# 无 cv2 回退: find_rects 阈值
RECT_LEGACY_THRESH_HI = 2500
RECT_LEGACY_THRESH_LO = 1000
RECT_LEGACY_MIN_W = 60
RECT_LEGACY_MIN_H = 60


def _contours_of(ret):
    """兼容 cv2.findContours 两种返回格式 (2值/3值)。"""
    if ret is None:
        return []
    if len(ret) == 2:
        return ret[0]
    return ret[1]


def detect_rect_cv2(small):
    """cv2 路径: 灰度阈值 + findContours + minAreaRect。
    返回 (cx,cy,w,h,angle,corners) 检测空间, 或 None。"""
    if not HAVE_CV2:
        return None
    try:
        gray = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # 外框为暗线 → 反相二值化(暗区变白)
        bin_img = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, RECT_ADAPT_BLK, RECT_ADAPT_C)
        contours = _contours_of(cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
        if not contours:
            return None

        best = None
        best_score = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < RECT_MIN_AREA_CV:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), ang = rect
            if w <= 0 or h <= 0:
                continue
            ratio = max(w, h) / min(w, h)
            if ratio < RECT_MIN_RATIO or ratio > RECT_MAX_RATIO:
                continue
            # 评分: 面积 × 宽高比合理性(越接近1越好)
            score = area * (1.0 / (1.0 + abs(ratio - 1.0)))
            if score > best_score:
                best_score = score
                best = rect

        if best is None:
            return None

        (cx, cy), (w, h), ang = best
        # 归一化角度到 [-45,45): 若接近 ±90 则交换 w/h
        if ang < -45.0:
            ang += 90.0
            w, h = h, w
        corners = cv2.boxPoints(((cx, cy), (w, h), ang))
        return (float(cx), float(cy), float(w), float(h), float(ang), corners)
    except Exception as e:
        print("rect_cv err:", e)
        return None


def detect_rect_legacy(img):
    """无 cv2 回退: CanMV find_rects(AprilTag 四边形检测)。
    img: CanMV image 对象(原图)。返回 (cx,cy,w,h,angle,None) 或 None。"""
    try:
        rects = img.find_rects(threshold=RECT_LEGACY_THRESH_HI)
        if not rects:
            rects = img.find_rects(threshold=RECT_LEGACY_THRESH_LO)
        if not rects:
            return None
        best = None
        best_score = 0
        for r in rects:
            x, y, w, h = r.rect()
            if w < RECT_LEGACY_MIN_W or h < RECT_LEGACY_MIN_H:
                continue
            ratio = max(w, h) / min(w, h)
            if ratio < RECT_MIN_RATIO or ratio > RECT_MAX_RATIO:
                continue
            mag = r.magnitude()
            score = (w * h) * (mag / 5000.0)
            if score > best_score:
                best_score = score
                best = r
        if best is None:
            return None
        x, y, w, h = best.rect()
        ang = best.rotation() if hasattr(best, 'rotation') else 0.0
        # find_rects 角度符号/单位与 cv2 不同, 仅作回退
        return (float(x + w // 2), float(y + h // 2), float(w), float(h), float(ang), None)
    except Exception as e:
        print("rect_legacy err:", e)
        return None


def detect_rect(small, img=None):
    """统一对外接口: 优先 cv2, 否则回退 find_rects。
    small: 检测空间 RGB888 ndarray (cv2 用)
    img  : CanMV image 对象 (回退用, 可 None)
    返回 : (cx,cy,w,h,angle,corners) 或 None
    """
    if HAVE_CV2 and small is not None:
        return detect_rect_cv2(small)
    if img is not None:
        return detect_rect_legacy(img)
    return None
