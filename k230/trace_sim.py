# -*- coding: utf-8 -*-
"""
矩形循迹闭环数值仿真 (复现 K230 v5.7 行为, 用于离线调参, 不烧板)
==================================================================
架构:
  K230 侧: RectTrajectory 沿矩形周长行走 -> desired 设定点 (frame 800x480 坐标)
           NEAR_CORNER 标志在接近四角时置位
  STM32 侧: K230_ControlAxis 逐帧处理 err = desired - laser, 输出带符号速度指令,
           经步进节拍(STEPPER_MIN/MAX_DELAY_US)换算成每帧实际位移(px)
 检测模型:  激光真值 = 云台位置; 检测值 = 真值 + 噪声; 高速/大误差/随机变暗 -> LOST
            LOST 时按模式决定 rewind(回退) 或 hold(原地保持)

目的: 量化"稳态偏差 / 极限环幅度 / LOST 次数 / 大误检次数", 对比 current vs improved.
"""
import math, random

# ---------------- STM32 控制环常量 (与 main.c 一致) ----------------
PAN_GAIN      = 0.006
TILT_GAIN     = 0.004
CTRL_KD       = 0.0015
PAN_DEC_ZONE  = 120.0
TILT_DEC_ZONE = 90.0
PID_DEADZONE      = 3
PID_DEADZONE_TILT = 4
TRACE_SPEED   = 0.60      # v5.7 直线段恒定速度地板
CORNER_FACTOR = 0.35
VEL_STOP_BAND = 0.015
EMA_A = 0.6; EMA_B = 0.4  # 速度 EMA
STEPPER_MIN_DELAY_US = 800
STEPPER_MAX_DELAY_US = 20000
RANGE = STEPPER_MAX_DELAY_US - STEPPER_MIN_DELAY_US
PAN_MAX_SCALE  = 1.0
TILT_SPEED_SCALE = 0.6
PAN_INVERT = 1
TILT_INVERT = 0
KALMAN_Q = 20.0; KALMAN_R = 25.0
K230_ERROR_MAX_PX = 400

# ---------------- 仿真标定参数 (使稳态偏差/LOST 量级接近实测日志) ----------------
PX_PER_STEP = 3.0     # 每步对应画面像素 (标定: vel=0.6 -> ~12px/frame ≈ 行走速度)
FPS = 30.0
WALK_SPEED = 10.0     # 周长行走速度 px/frame (从日志 prog 增量反推)
CORNER_SLOWDOWN_ARC = 25
CORNER_SLOW_FACTOR  = 0.30

# ---------------- 检测模型参数 ----------------
ROI_HALF       = 120    # 跟踪 ROI 半窗(安全值, 仅作检测门限参考)
DIM_P_BASE     = 0.05   # 基础变暗丢失概率/帧 (贴合日志 ~9 次/50帧)
DIM_P_SPEED    = 0.012  # 每 (px/frame) 速度附加丢失概率 (运动模糊)
MISDET_P       = 0.010  # 大误检(反射)概率/帧
RECOVER_FRAMES = 3      # LOST 后恢复帧数

random.seed(12345)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Kalman1D:
    def __init__(self):
        self.q = KALMAN_Q; self.r = KALMAN_R; self.init = 0; self.x = 0.0; self.p = 1.0
    def update(self, z):
        if not self.init:
            self.x = z; self.p = 1.0; self.init = 1; return self.x
        p_pred = self.p + self.q
        k = p_pred / (p_pred + self.r)
        self.x += k * (z - self.x)
        self.p = (1.0 - k) * p_pred
        return self.x


class PID:
    def __init__(self, kp, kd):
        self.kp = kp; self.kd = kd; self.prev = 0.0; self.last_tick = 0.0; self.out_max = 1.0; self.out_min = -1.0
    def reset(self):
        self.prev = 0.0; self.last_tick = 0.0
    def update(self, err, dt):
        if dt < 0.001: dt = 0.001
        if dt > 1.0: dt = 1.0
        out = self.kp * err
        derr = (err - self.prev) / dt
        self.prev = err
        out += self.kd * derr
        return clamp(out, self.out_min, self.out_max)


def delay_to_disp(vel_mag):
    """vel_mag[0,1] -> 本帧位移(px). 与 main.c: delay = MAX - vel*RANGE; 位移 = PX*1e6/(fps*delay)"""
    vel_mag = clamp(vel_mag, 0.0, 1.0)
    delay = STEPPER_MAX_DELAY_US - vel_mag * RANGE
    delay = clamp(delay, STEPPER_MIN_DELAY_US, STEPPER_MAX_DELAY_US)
    steps = 1e6 / delay / FPS
    return PX_PER_STEP * steps


def control_axis(err, pid, kf, invert, scale, tracing, corner, prev_vel, mode, floor_thresh):
    """复现 K230_ControlAxis 速度支路. 返回 (signed_vel_ema, disp_px 带符号).
    仿真直接积分激光位置: 位移方向 = sign(vel_out) = 朝减小误差方向 (负反馈),
    invert 已被正确反馈吸收, 此处只取 vel_out 符号, 不二次取反."""
    if abs(err) > K230_ERROR_MAX_PX:
        err = 0
    if tracing:
        pid.reset()  # 循迹不锁靶 (简化: 清微分历史近似)
    # 卡尔曼
    kf_err = kf.update(float(err))
    vel_out = pid.update(kf_err, 1.0 / FPS)
    vel_out = clamp(vel_out, -1.0, 1.0)
    # 减速区 (循迹时拉到 1e6, 限幅~0 -> 速度由地板决定)
    dec_zone = (PAN_DEC_ZONE if scale == PAN_MAX_SCALE else TILT_DEC_ZONE)
    if dec_zone <= PID_DEADZONE: dec_zone = PID_DEADZONE + 1.0
    if tracing: dec_zone = 1.0e6
    a = abs(kf_err)
    shape = (a - PID_DEADZONE) / (dec_zone - PID_DEADZONE)
    shape = clamp(shape, 0.0, 1.0)
    mag = abs(vel_out)
    if mag > shape: mag = shape
    mag *= scale
    mag = max(mag, 0.0)
    # 方向: 朝减小误差 (sign(vel_out)); err≈0 时用上次方向避免卡死
    sgn_base = 1.0 if vel_out >= 0 else (-1.0 if vel_out < 0 else (1.0 if prev_vel >= 0 else -1.0))
    signed_cmd = mag * sgn_base

    # ---- 循迹速度整形 ----
    if tracing:
        if mode == "current":
            tspeed = TRACE_SPEED * (CORNER_FACTOR if corner else 1.0)
            if mag < tspeed:
                signed_cmd = sgn_base * tspeed
        else:  # improved: 误差相关地板 —— 远则保持跟线速度, 近则放开允许归零(消除极限环)
            if abs(err) > floor_thresh:
                tspeed = TRACE_SPEED * (CORNER_FACTOR if corner else 1.0)
            else:
                tspeed = 0.06
            if mag < tspeed:
                signed_cmd = sgn_base * tspeed

    # 速度 EMA (带符号)
    vel_ema = signed_cmd if prev_vel == 0.0 else (prev_vel * EMA_A + signed_cmd * EMA_B)
    vel_mag = abs(vel_ema)
    sgn = 1.0 if vel_ema >= 0 else -1.0
    disp = sgn * delay_to_disp(vel_mag)
    return vel_ema, disp


# ---------------- 矩形周长行走 (复现 K230 RectTrajectory) ----------------
# 四角 (顺时针): TL -> TR -> BR -> BL, 与日志吻合
CORNERS = [(110, 40), (617, 40), (617, 471), (110, 471)]
PERIM = 0.0
_EDGES = []
for i in range(4):
    x0, y0 = CORNERS[i]; x1, y1 = CORNERS[(i + 1) % 4]
    L = math.hypot(x1 - x0, y1 - y0)
    _EDGES.append((x0, y0, x1, y1, L))
    PERIM += L

def point_at(prog):
    """prog in [0,1) -> (x,y) 周长上的点"""
    d = prog * PERIM
    for (x0, y0, x1, y1, L) in _EDGES:
        if d <= L or L == 0:
            t = (d / L) if L > 0 else 0.0
            return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        d -= L
    return CORNERS[0]

def corner_dist(prog):
    """到最近四角的距离(px), 用于 NEAR_CORNER 标志"""
    p = point_at(prog)
    best = 1e9
    for c in CORNERS:
        best = min(best, math.hypot(p[0] - c[0], p[1] - c[1]))
    return best


def run(mode, floor_thresh=25, detect_gate=True, rewind=True, n_frames=1200):
    # 状态
    prog = 0.0
    laser = list(point_at(0.0))   # 循迹从激光已在矩形起点附近开始(真实场景)
    pid_x = PID(PAN_GAIN, CTRL_KD); pid_y = PID(TILT_GAIN, CTRL_KD)
    kf_x = Kalman1D(); kf_y = Kalman1D()
    vel_x = 0.0; vel_y = 0.0
    lost = False; recover = 0
    stats = dict(errs=[], lost_cnt=0, misdet_cnt=0, max_err=0)
    last_disp = [0.0, 0.0]

    for f in range(n_frames):
        # --- K230: 推进设定点 (LOST 时按模式决定是否回退) ---
        if not (lost and rewind):
            cd = corner_dist(prog)
            step = WALK_SPEED * (CORNER_SLOW_FACTOR if cd < CORNER_SLOWDOWN_ARC else 1.0)
            if not lost:
                prog = (prog + step / PERIM) % 1.0
        desired = point_at(prog)
        corner = corner_dist(prog) < CORNER_SLOWDOWN_ARC

        # --- 检测模型: LOST 原因 = 变暗(随机, 速度相关) + 大误检(反射) ---
        if lost:
            recover -= 1
            if recover <= 0:
                lost = False
            detected = None
        else:
            true_x, true_y = laser
            nx = random.gauss(0, 2.0); ny = random.gauss(0, 2.0)
            dx, dy = true_x + nx, true_y + ny
            speed = (abs(vel_x) + abs(vel_y))
            p_dim = DIM_P_BASE + DIM_P_SPEED * speed * 10.0
            # 大误检(反射到画面边缘)
            if random.random() < MISDET_P:
                if detect_gate:
                    # 门限拒绝远处假点 -> 视为 LOST(hold), 不驱动错方向
                    lost = True; stats["lost_cnt"] += 1; recover = RECOVER_FRAMES
                    if rewind: prog = (prog - 0.03) % 1.0
                    detected = None
                else:
                    dx, dy = random.uniform(700, 795), random.uniform(30, 120)
                    stats["misdet_cnt"] += 1
                    detected = (dx, dy)
            elif random.random() < p_dim:
                # 变暗丢失
                lost = True; stats["lost_cnt"] += 1; recover = RECOVER_FRAMES
                if rewind: prog = (prog - 0.03) % 1.0
                detected = None
            else:
                detected = (dx, dy)

        if detected is not None:
            ex = desired[0] - detected[0]
            ey = desired[1] - detected[1]
            stats["errs"].append((ex, ey))
            stats["max_err"] = max(stats["max_err"], math.hypot(ex, ey))
        else:
            ex = ey = 0

        # --- STM32: 控制环 ---
        if detected is not None:
            vel_x, disp_x = control_axis(ex, pid_x, kf_x, PAN_INVERT, PAN_MAX_SCALE, True, corner, vel_x, mode, floor_thresh)
            vel_y, disp_y = control_axis(ey, pid_y, kf_y, TILT_INVERT, TILT_SPEED_SCALE, True, corner, vel_y, mode, floor_thresh)
            last_disp = [disp_x, disp_y]
            laser[0] += disp_x
            laser[1] += disp_y
        else:
            # valid=0 -> 冻结(停转, 重置 PID), 与固件一致; 激光原地不动
            vel_x = vel_y = 0.0
            pid_x.reset(); pid_y.reset(); kf_x.init = 0; kf_y.init = 0
            laser[0] += last_disp[0] * 0.0  # 停转, 不积分

    # 指标
    if stats["errs"]:
        es = [math.hypot(e[0], e[1]) for e in stats["errs"]]
        stats["err_mean"] = sum(es) / len(es)
        stats["err_rms"] = math.sqrt(sum(x * x for x in es) / len(es))
        es_s = sorted(es)
        stats["err_p95"] = es_s[int(len(es_s) * 0.95)]
        stats["n"] = len(es)
    else:
        stats["n"] = 0; stats["err_mean"] = 0; stats["err_rms"] = 0; stats["err_p95"] = 0
    return stats


if __name__ == "__main__":
    print("=== 矩形循迹闭环仿真 (复现 v5.7 行为) ===")
    print("标定: PX_PER_STEP=%.1f  WALK_SPEED=%.1f px/frame  FPS=%.0f\n" % (PX_PER_STEP, WALK_SPEED, FPS))

    print("[当前 v5.7] 恒定速度地板 TRACE_SPEED=0.60, rewind, 接受大误检:")
    s1 = run("current", detect_gate=False, rewind=True)
    print("  有效帧=%d  平均误差=%.1fpx  RMS=%.1fpx  P95=%.1fpx  最大=%.1fpx  LOST=%d  大误检=%d"
          % (s1["n"], s1["err_mean"], s1["err_rms"], s1["err_p95"], s1["max_err"], s1["lost_cnt"], s1["misdet_cnt"]))

    print("\n[改进A] 误差相关地板(far=0.60/near=0.06) + 检测门限 + 不rewind:")
    s2 = run("improved", floor_thresh=25, detect_gate=True, rewind=False)
    print("  有效帧=%d  平均误差=%.1fpx  RMS=%.1fpx  P95=%.1fpx  最大=%.1fpx  LOST=%d  大误检=%d"
          % (s2["n"], s2["err_mean"], s2["err_rms"], s2["err_p95"], s2["max_err"], s2["lost_cnt"], s2["misdet_cnt"]))

    print("\n[地板取值扫描] 改进方案下 TRACE_SPEED(远线地板) 扫参 (near地板=0.06, 不rewind, 门限开):")
    best = None
    for ts in (0.30, 0.40, 0.50, 0.60):
        # 临时覆盖 TRACE_SPEED
        globals()["TRACE_SPEED"] = ts
        s = run("improved", floor_thresh=20, detect_gate=True, rewind=False)
        print("  TRACE_SPEED=%.2f -> 平均=%.1fpx RMS=%.1fpx P95=%.1fpx 最大=%.1fpx LOST=%d"
              % (ts, s["err_mean"], s["err_rms"], s["err_p95"], s["max_err"], s["lost_cnt"]))
        if best is None or s["err_rms"] < best[1]["err_rms"]:
            best = (ts, s)
    globals()["TRACE_SPEED"] = 0.60
    print("  => 最优远线地板 ≈ %.2f (RMS 最小)" % best[0])

    print("\n结论: 当前->最优(TRACE_SPEED=%.2f, 误差相关地板, 检测门限, 不rewind)" % best[0])
    print("  平均误差 %.1f -> %.1f px, RMS %.1f -> %.1f px, 最大 %.1f -> %.1f px, LOST %d -> %d, 大误检 %d -> %d"
          % (s1["err_mean"], best[1]["err_mean"], s1["err_rms"], best[1]["err_rms"],
             s1["max_err"], best[1]["max_err"], s1["lost_cnt"], best[1]["lost_cnt"],
             s1["misdet_cnt"], best[1]["misdet_cnt"]))
