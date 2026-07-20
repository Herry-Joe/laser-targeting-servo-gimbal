#!/usr/bin/env python3
"""
舵机云台 上位机控制软件
STM32F103 + PA8/PA11 舵机二维云台
串口: 115200 baud, 8N1
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import re
from queue import Queue

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ── 常量 ──
BAUD = 115200
TELE_REFRESH_MS = 100  # 遥测刷新匹配 STM32 的 100ms

# 默认 PID 范围
PID_LIMITS = {
    'kp': (0.001, 0.05),
    'ki': (10.0, 150.0),
    'kd': (0.0, 0.01),
}


class GimbalControl:
    """云台控制面板"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("舵机云台 控制系统")
        self.root.geometry("900x680")
        self.root.minsize(800, 600)
        if HAS_SERIAL:
            self.root.iconbitmap(default='')  # 可选: 设置图标
        try:
            self.root.tk.call('tk', 'scaling', 1.2)
        except Exception:
            pass

        # ── 状态变量 ──
        self.ser = None
        self.serial_thread = None
        self.running = False
        self.tx_queue = Queue()
        self.connected = tk.BooleanVar(value=False)

        # 遥测数据
        self.telem = {
            'pan_pos': 0.0, 'pan_target': 0.0, 'pan_err': 0.0, 'pan_out': 0.0,
            'tilt_pos': 0.0, 'tilt_target': 0.0, 'tilt_err': 0.0, 'tilt_out': 0.0,
            'mode': '---',
        }
        self.last_telem_time = 0

        # ── 构建界面 ──
        self._build_ui()
        self._poll_serial()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ═══════════════════════════════════════════════
    #  界面构建
    # ═══════════════════════════════════════════════

    def _build_ui(self):
        # ── 主布局: 左控制面板 + 右终端 ──
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        left_frame = ttk.Frame(main_pane, width=440)
        main_pane.add(left_frame, weight=0)

        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        # ── 串口连接 ──
        conn_frame = ttk.LabelFrame(left_frame, text="串口连接", padding=8)
        conn_frame.pack(fill=tk.X, pady=(0, 6))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=18, state='readonly')
        self.port_combo.pack(side=tk.LEFT, padx=(0, 6))
        self._refresh_ports()

        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self._toggle_connect, width=8)
        self.connect_btn.pack(side=tk.LEFT, padx=2)

        ttk.Button(conn_frame, text="刷新", command=self._refresh_ports, width=5).pack(side=tk.LEFT, padx=2)

        self.conn_indicator = tk.Canvas(conn_frame, width=16, height=16, highlightthickness=0)
        self.conn_indicator.pack(side=tk.LEFT, padx=(8, 0))
        self._indicator = self.conn_indicator.create_oval(2, 2, 14, 14, fill='red', outline='')

        # ── 位置控制 ──
        pos_frame = ttk.LabelFrame(left_frame, text="位置控制", padding=8)
        pos_frame.pack(fill=tk.X, pady=6)

        # Pan
        row_pan = ttk.Frame(pos_frame)
        row_pan.pack(fill=tk.X, pady=3)
        ttk.Label(row_pan, text="Pan:", width=5).pack(side=tk.LEFT)
        self.pan_var = tk.DoubleVar(value=0.0)
        self.pan_scale = ttk.Scale(row_pan, from_=-135, to=135, variable=self.pan_var,
                                    command=lambda v: self._update_pos_label(self.pan_label, v))
        self.pan_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.pan_label = ttk.Label(row_pan, text="0.0°", width=7, anchor='center')
        self.pan_label.pack(side=tk.LEFT)
        ttk.Button(row_pan, text="发送", width=4, command=lambda: self._send_pos('x')).pack(side=tk.LEFT, padx=(4, 0))

        # Tilt
        row_tilt = ttk.Frame(pos_frame)
        row_tilt.pack(fill=tk.X, pady=3)
        ttk.Label(row_tilt, text="Tilt:", width=5).pack(side=tk.LEFT)
        self.tilt_var = tk.DoubleVar(value=0.0)
        self.tilt_scale = ttk.Scale(row_tilt, from_=-40, to=40, variable=self.tilt_var,
                                     command=lambda v: self._update_pos_label(self.tilt_label, v))
        self.tilt_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.tilt_label = ttk.Label(row_tilt, text="0.0°", width=7, anchor='center')
        self.tilt_label.pack(side=tk.LEFT)
        ttk.Button(row_tilt, text="发送", width=4, command=lambda: self._send_pos('y')).pack(side=tk.LEFT, padx=(4, 0))

        # 快捷按钮
        btn_row = ttk.Frame(pos_frame)
        btn_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(btn_row, text="归零 (0,0)", command=self._cmd_home, width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_row, text="停止", command=self._cmd_stop, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_row, text="当前位置归零", command=self._cmd_zero, width=12).pack(side=tk.LEFT, padx=2)

        # ── 状态显示 ──
        status_frame = ttk.LabelFrame(left_frame, text="实时状态", padding=8)
        status_frame.pack(fill=tk.X, pady=6)

        self.status_text = tk.StringVar(value="等待连接...")
        ttk.Label(status_frame, textvariable=self.status_text, font=('Consolas', 9), justify='left',
                  background='white', relief='sunken', padding=6).pack(fill=tk.X)

        # ── 模式切换 ──
        mode_frame = ttk.LabelFrame(left_frame, text="工作模式", padding=8)
        mode_frame.pack(fill=tk.X, pady=6)

        mode_row = ttk.Frame(mode_frame)
        mode_row.pack()
        self.mode_var = tk.StringVar(value='posctrl')
        ttk.Radiobutton(mode_row, text="位置闭环", variable=self.mode_var, value='posctrl',
                        command=lambda: self._send('m')).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(mode_row, text="K230 打靶", variable=self.mode_var, value='k230',
                        command=lambda: self._send('k')).pack(side=tk.LEFT, padx=4)
        self.mode_var.set('posctrl')

        # ── PID 调参 ──
        pid_frame = ttk.LabelFrame(left_frame, text="PID 调参 (位置环)", padding=8)
        pid_frame.pack(fill=tk.X, pady=6)

        pid_grid = ttk.Frame(pid_frame)
        pid_grid.pack()

        self.pid_vars = {}
        headers = ['', 'Kp', 'Ki', 'Kd']
        for j, h in enumerate(headers):
            ttk.Label(pid_grid, text=h, width=10, anchor='center', font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=j, padx=1)

        for i, axis in enumerate(['Pan', 'Tilt']):
            ttk.Label(pid_grid, text=axis, width=5).grid(row=i+1, column=0)
            for j, param in enumerate(['kp', 'ki', 'kd']):
                key = f"{axis}_{param}"
                var = tk.StringVar(value=self._pid_default(param))
                self.pid_vars[key] = var
                entry = ttk.Entry(pid_grid, textvariable=var, width=9, justify='center')
                entry.grid(row=i+1, column=j+1, padx=1, pady=2)
                entry.bind('<Return>', lambda e, a=axis[0].lower(), p=param, v=var: self._send_pid(a, p, v))

        # ── 遥测开关 ──
        tele_row = ttk.Frame(left_frame)
        tele_row.pack(fill=tk.X, pady=(6, 0))
        self.tele_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tele_row, text="遥测数据 (100ms)", variable=self.tele_var,
                        command=self._toggle_tele).pack(side=tk.LEFT)
        ttk.Button(tele_row, text="刷新状态", command=self._cmd_status).pack(side=tk.RIGHT)

        # ── 右侧: 串口监视器 ──
        term_frame = ttk.LabelFrame(right_frame, text="串口监视器", padding=6)
        term_frame.pack(fill=tk.BOTH, expand=True)

        self.term = scrolledtext.ScrolledText(term_frame, font=('Consolas', 9), wrap=tk.WORD,
                                               state='disabled', height=20, bg='#1e1e1e', fg='#d4d4d4',
                                               insertbackground='white')
        self.term.pack(fill=tk.BOTH, expand=True)

        # 颜色标签
        self.term.tag_config('ok', foreground='#4ec9b0')
        self.term.tag_config('err', foreground='#f44747')
        self.term.tag_config('info', foreground='#569cd6')
        self.term.tag_config('tele', foreground='#808080')
        self.term.tag_config('bold', font=('Consolas', 9, 'bold'))

        cmd_row = ttk.Frame(term_frame)
        cmd_row.pack(fill=tk.X, pady=(4, 0))
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        cmd_entry.bind('<Return>', lambda e: self._send_cmd())
        ttk.Button(cmd_row, text="发送", command=self._send_cmd, width=6).pack(side=tk.RIGHT)

    # ═══════════════════════════════════════════════
    #  串口操作
    # ═══════════════════════════════════════════════

    def _refresh_ports(self):
        if not HAS_SERIAL:
            self.port_combo['values'] = ['(无 pyserial 库)']
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.connected.get():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not HAS_SERIAL:
            self._log("ERR 未安装 pyserial (pip install pyserial)\n", 'err')
            return
        port = self.port_var.get()
        if not port:
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=0.1)
            self.connected.set(True)
            self.connect_btn.config(text="断开")
            self.conn_indicator.itemconfig(self._indicator, fill='#4ec9b0')
            self._log(f"OK 已连接 {port} @ {BAUD}\n", 'ok')
            self.running = True
            self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            self._log(f"ERR 连接失败: {e}\n", 'err')

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected.set(False)
        self.connect_btn.config(text="连接")
        self.conn_indicator.itemconfig(self._indicator, fill='red')
        self._log("--- 已断开 ---\n", 'info')

    def _serial_reader(self):
        buf = ''
        while self.running and self.ser and self.ser.is_open:
            try:
                # 发送队列
                while not self.tx_queue.empty():
                    cmd = self.tx_queue.get_nowait()
                    self.ser.write((cmd + '\r\n').encode())
                    self.tx_queue.task_done()
                # 读取
                data = self.ser.read(256)
                if data:
                    buf += data.decode('utf-8', errors='replace')
                    # 按行处理
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip('\r')
                        if line:
                            self.root.after(0, self._process_line, line + '\n')
            except Exception as e:
                if self.running:
                    self.root.after(0, self._log, f"ERR 串口: {e}\n", 'err')
                break
        if self.running:
            self.root.after(0, self._disconnect)

    def _process_line(self, line):
        # 显示到终端
        self._append_term(line)

        # 解析遥测 POS,pan_pos,pan_target,pan_err,pan_out,tilt_pos,tilt_target,tilt_err,tilt_out
        m = re.match(r'POS,([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)', line)
        if m:
            self.telem['pan_pos'] = float(m.group(1))
            self.telem['pan_target'] = float(m.group(2))
            self.telem['pan_err'] = float(m.group(3))
            self.telem['pan_out'] = float(m.group(4))
            self.telem['tilt_pos'] = float(m.group(5))
            self.telem['tilt_target'] = float(m.group(6))
            self.telem['tilt_err'] = float(m.group(7))
            self.telem['tilt_out'] = float(m.group(8))
            self.last_telem_time = time.time()
            return

        # 解析模式
        m = re.search(r'K230 自动打靶|自动打靶', line)
        if m:
            self.telem['mode'] = 'K230 打靶'
            self.root.after(0, lambda: self.mode_var.set('k230'))
        m = re.search(r'位置闭环|手动模式', line)
        if m:
            self.telem['mode'] = '位置闭环'
            self.root.after(0, lambda: self.mode_var.set('posctrl'))

    def _send(self, cmd):
        if self.connected.get():
            self.tx_queue.put(cmd)

    def _send_cmd(self):
        cmd = self.cmd_var.get().strip()
        if cmd:
            self._send(cmd)
            self.cmd_var.set('')

    def _log(self, msg, tag=''):
        self._append_term(msg, tag)

    def _append_term(self, text, tag=''):
        try:
            self.term.config(state='normal')
            if tag:
                self.term.insert(tk.END, text, tag)
            else:
                # 自动判断
                if text.startswith('# OK'):
                    self.term.insert(tk.END, text, 'ok')
                elif text.startswith('# ERR') or text.startswith('ERR'):
                    self.term.insert(tk.END, text, 'err')
                elif text.startswith('POS,'):
                    self.term.insert(tk.END, text, 'tele')
                elif text.startswith('#'):
                    self.term.insert(tk.END, text, 'info')
                else:
                    self.term.insert(tk.END, text)
            self.term.see(tk.END)
            self.term.config(state='disabled')
        except Exception:
            pass

    # ═══════════════════════════════════════════════
    #  命令快捷操作
    # ═══════════════════════════════════════════════

    def _send_pos(self, axis):
        if axis == 'x':
            angle = self.pan_var.get()
            self._send(f"x{angle:.1f}")
        else:
            angle = self.tilt_var.get()
            self._send(f"y{angle:.1f}")

    def _update_pos_label(self, label, val):
        try:
            label.config(text=f"{float(val):.1f}°")
        except Exception:
            pass

    def _cmd_home(self):
        self.pan_var.set(0.0)
        self.tilt_var.set(0.0)
        self._update_pos_label(self.pan_label, 0.0)
        self._update_pos_label(self.tilt_label, 0.0)
        self._send('home')

    def _cmd_stop(self):
        self._send('stop')

    def _cmd_zero(self):
        self._send('z')

    def _cmd_status(self):
        self._send('status')

    def _send_pid(self, axis, param, var):
        try:
            val = float(var.get().strip())
            self._send(f"pid {axis} {param} {val}")
        except ValueError:
            pass

    def _pid_default(self, param):
        defaults = {'kp': '0.0080', 'ki': '0.0', 'kd': '0.00005'}
        return defaults.get(param, '0.0')

    def _toggle_tele(self):
        if self.tele_var.get():
            self._send('tele on')
        else:
            self._send('tele off')

    # ═══════════════════════════════════════════════
    #  轮询刷新
    # ═══════════════════════════════════════════════

    def _poll_serial(self):
        # 更新状态显示
        now = time.time()
        if now - self.last_telem_time < 1.0:
            s = (
                f"Pan: {self.telem['pan_pos']:+.1f}°  → {self.telem['pan_target']:+.1f}°"
                f"  err={self.telem['pan_err']:+.1f}\n"
                f"Tilt:{self.telem['tilt_pos']:+.1f}°  → {self.telem['tilt_target']:+.1f}°"
                f"  err={self.telem['tilt_err']:+.1f}\n"
                f"PID_out: Pan={self.telem['pan_out']:.3f}  Tilt={self.telem['tilt_out']:.3f}\n"
                f"模式: {self.telem['mode']}"
            )
        elif self.connected.get():
            s = "已连接，等待遥测数据...\n发 tele on 开启遥测"
        else:
            s = "等待连接...\n选择串口后点击「连接」"
        self.status_text.set(s)

        self.root.after(200, self._poll_serial)

    def _on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


def main():
    if not HAS_SERIAL:
        root = tk.Tk()
        root.title("提示")
        root.geometry("300x100")
        ttk.Label(root, text="需要 pyserial 库\n\n安装: pip install pyserial").pack(expand=True)
        ttk.Button(root, text="确定", command=root.destroy).pack()
        root.mainloop()
        return
    app = GimbalControl()
    app.root.mainloop()


if __name__ == '__main__':
    main()
