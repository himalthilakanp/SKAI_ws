#!/usr/bin/env python3

import os
import math
import time
import struct
import threading

import can
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import customtkinter as ctk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageDraw

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers
from skai_interfaces.msg import TargetPose


# ── Joint configuration ────────────────────────────────────────────────────────

JOINT_NAMES  = ['ROT_1', 'PITCH_1', 'PITCH_2', 'PITCH_3', 'ROT_2', 'ROT_3']
JOINT_LABELS = ['J1 Base', 'J2 Shoulder', 'J3 Elbow', 'J4 Wrist 1', 'J5 Wrist 2', 'J6 Wrist 3']
NODE_IDS     = [21, 22, 23, 24, 25, 26]

# Preset fractional-turn offsets converted to radians (from ARM_REDO_8.py)
# J2 and J3 signs are inverted relative to ARM_REDO_8 to match ros2_control axis direction
def _t2r(turns): return [t * 2 * math.pi for t in turns]

HOME_RAD   = _t2r([0.959, 0.548, 0.655, 0.160, 0.614, 0.000])
EXT_RAD    = _t2r([0.959, 0.764, 0.060, 0.117, 0.614, 0.000])
UP_DELTA   = _t2r([ 0,  0.02, -0.04,  0.1,   0.0,   0.0])
DOWN_DELTA = _t2r([ 0, -0.02,  0.04, -0.1,   0.0,   0.0])

# ODrive CAN monitoring (read-only, from ARM_REDO_8.py)
_CAN_BUS      = 'can0'
_CAN_IFACE    = 'socketcan'
_CAN_HB       = 0x001
_CAN_SDO_RX   = 0x04
_CAN_SDO_TX   = 0x05
_CAN_ENC_EST  = 0x009
_STATE_IDLE   = 1
_STATE_CLOSED = 8

# URDF joint definitions for FK  (xyz_m, rpy_rad, axis)
JOINT_DEFS = [
    ((-0.067,  -0.065,   0.082),  (0.0,     0.0,     0.0    ), (0,  0, -1)),
    (( 0.000,  -0.046,   0.194),  (1.5708,  1.2217,  3.1416 ), (0,  0, -1)),
    ((-0.37152, 0.2145,  0.107),  (1.5708,  0.0,     0.87266), (0,  1,  0)),
    (( 0.3542,  0.012,   0.2045), (1.5708,  0.68579, 0.0    ), (0,  0,  1)),
    (( 0.023514,0.062204,-0.0725),(-1.5708, 0.3614,  2.7802 ), (0,  0, -1)),
    ((-0.12262,-0.3244,  -0.0725),(1.5708, -0.69813,-0.3614 ), (0,  0, -1)),
]

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


# ── FK math ────────────────────────────────────────────────────────────────────

def _trans4(x, y, z):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M

def _rpy4(roll, pitch, yaw):
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    M = np.eye(4)
    M[0, :3] = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr]
    M[1, :3] = [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr]
    M[2, :3] = [-sp,   cp*sr,            cp*cr            ]
    return M

def _axis_angle4(axis, angle):
    ax, ay, az = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    M = np.eye(4)
    M[0, :3] = [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay]
    M[1, :3] = [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax]
    M[2, :3] = [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ]
    return M

def calculate_fk(joint_angles_rad):
    points = [np.zeros(3)]
    T = np.eye(4)
    for (xyz, rpy, axis), q in zip(JOINT_DEFS, joint_angles_rad):
        T = T @ _trans4(*xyz) @ _rpy4(*rpy) @ _axis_angle4(axis, q)
        points.append(T[:3, 3].copy())
    return np.array(points)


def nearest_turn_target(current_rad, offset_rad):
    """Navigate to the nearest turn that has the given fractional offset (ARM_REDO_8 logic)."""
    offset_turns  = offset_rad  / (2 * math.pi)
    current_turns = current_rad / (2 * math.pi)
    integer_turns = round(current_turns - offset_turns)
    return (integer_turns + offset_turns) * 2 * math.pi


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class DashboardNode(Node):

    def __init__(self):
        super().__init__('skai_dashboard')

        self._lock = threading.Lock()
        self._joint_positions     = {n: 0.0 for n in JOINT_NAMES}
        self._joint_last_ts       = 0.0
        self._joint_update_counts = {n: 0   for n in JOINT_NAMES}
        self._controller_active   = False

        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        self._can_enable_pub = self.create_publisher(Bool,           '/can_send_enable',                    10)
        self._target_pub     = self.create_publisher(TargetPose,     '/target_pose',                        10)
        self._traj_pub       = self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',    10)

        self._switch_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self._list_cli   = self.create_client(ListControllers,  '/controller_manager/list_controllers')

        self.create_timer(1.0, self._poll_controller_state)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_positions:
                    self._joint_positions[name] = pos
                    self._joint_update_counts[name] += 1
            self._joint_last_ts = time.time()

    def _poll_controller_state(self):
        if not self._list_cli.service_is_ready():
            return
        self._list_cli.call_async(ListControllers.Request()).add_done_callback(self._on_list_done)

    def _on_list_done(self, future):
        try:
            result = future.result()
            active = any(c.name == 'arm_controller' and c.state == 'active' for c in result.controller)
            with self._lock:
                self._controller_active = active
        except Exception:
            pass

    # ── thread-safe getters ────────────────────────────────────────────────────

    def get_joint_positions(self):
        with self._lock:
            return dict(self._joint_positions), self._joint_last_ts

    def get_joint_update_counts(self):
        with self._lock:
            return dict(self._joint_update_counts)

    def get_controller_active(self):
        with self._lock:
            return self._controller_active

    # ── commands ───────────────────────────────────────────────────────────────

    def _set_can_send(self, enabled: bool):
        msg = Bool(); msg.data = enabled
        self._can_enable_pub.publish(msg)

    def send_ik_goal(self, x, y, z):
        self._set_can_send(True)
        msg = TargetPose()
        msg.x, msg.y, msg.z = x, y, z
        msg.roll = msg.pitch = msg.yaw = 0.0
        self._target_pub.publish(msg)

    def send_joint_trajectory(self, positions_rad: dict):
        self._set_can_send(True)
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [positions_rad.get(n, 0.0) for n in JOINT_NAMES]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def estop(self):
        self._set_can_send(False)
        if not self._switch_cli.service_is_ready():
            return
        req = SwitchController.Request()
        req.deactivate_controllers = ['arm_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._switch_cli.call_async(req)

    def resume(self):
        self._set_can_send(True)
        if not self._switch_cli.service_is_ready():
            return
        req = SwitchController.Request()
        req.activate_controllers = ['arm_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._switch_cli.call_async(req)


# ── GUI ────────────────────────────────────────────────────────────────────────

class SKAIDashboard(ctk.CTk):

    def __init__(self, ros_node: DashboardNode):
        super().__init__()
        self.node = ros_node
        self.title("SKAI Arm Controller  v2.0  (MoveIt)")
        self.geometry("1540x900")

        self._glow_idx            = 0
        self._entries_initialized = False

        # Per-node live CAN state (ARM_REDO_8 style)
        self._can_hb_counts = {n: 0   for n in NODE_IDS}
        self._can_states    = {n: {'error': 0, 'state': 0, 'last_update': 0.0} for n in NODE_IDS}
        self._can_est_enc   = {n: None for n in NODE_IDS}
        self._can_raw_enc   = {n: None for n in NODE_IDS}
        self._can_bus       = None
        self._can_running   = True

        self._precompute_leds()
        self._build_ui()
        self._setup_can_monitor()
        self._update_loop()
        self._animate_leds()
        self._show_splash()

    # ── splash / logo ──────────────────────────────────────────────────────────

    def _logo_path(self):
        here = os.path.dirname(os.path.abspath(__file__))
        p = os.path.join(here, 'logo_suriNova_Center.png')
        return p if os.path.exists(p) else None

    def _show_splash(self):
        self.withdraw()
        splash = ctk.CTkToplevel(self)
        splash.overrideredirect(True)
        splash.configure(fg_color='#101010')
        w = splash.winfo_screenwidth()
        h = splash.winfo_screenheight()
        splash.geometry(f'{w}x{h}+0+0')
        lp = self._logo_path()
        if lp:
            try:
                img = Image.open(lp)
                new_w = 800
                new_h = int(img.size[1] * new_w / img.size[0])
                img   = img.resize((new_w, new_h), Image.Resampling.LANCZOS)
                self._splash_img = ctk.CTkImage(light_image=img, dark_image=img, size=(new_w, new_h))
                ctk.CTkLabel(splash, image=self._splash_img, text='').pack(expand=True)
            except Exception as e:
                print(f"Splash warning: {e}")
        self.after(2500, lambda: (splash.destroy(), self.deiconify()))

    # ── LED precompute (matches ARM_REDO_8 style) ──────────────────────────────

    def _precompute_leds(self):
        self._leds = {'green': [], 'blue': [], 'red': [], 'yellow': [], 'gray': []}

        def make(color, pulse):
            sz  = 64
            img  = Image.new('RGBA', (sz, sz), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img, 'RGBA')
            cx = cy = sz // 2
            r, g, b = color
            gr = 20 + int(pulse * 8)
            for i in range(8):
                a   = int(max(0, 22 - i * 2) * 2.55)
                rad = gr - i * 2
                if rad > 0:
                    draw.ellipse([cx-rad, cy-rad, cx+rad, cy+rad], fill=(r, g, b, a))
            for i in range(5):
                a   = int(max(0, 50 - i * 8) * 2.55)
                rad = 12 - i
                if rad > 0:
                    draw.ellipse([cx-rad, cy-rad, cx+rad, cy+rad], fill=(r, g, b, a))
            draw.ellipse([cx-5, cy-5, cx+5,  cy+5 ], fill=(240, 255, 240, 255))
            draw.ellipse([cx-3, cy-3, cx-1,  cy-1 ], fill=(255, 255, 255, 180))
            return ctk.CTkImage(light_image=img, dark_image=img, size=(36, 36))

        for i in range(16):
            p = (math.sin(i * math.pi / 8) + 1) / 2
            self._leds['green'].append(make((80,  255, 120), p))
            self._leds['blue'].append( make((0,   170, 255), p))
            self._leds['red'].append(  make((255,  50,  50), p))
            self._leds['yellow'].append(make((255, 200,   0), p))
        self._leds['gray'].append(make((100, 100, 100), 0))

    # ── top-level layout ───────────────────────────────────────────────────────

    def _build_ui(self):
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=3)
        self.grid_columnconfigure(2, weight=0)
        self._build_left()
        self._build_center()
        self._build_right()

    # ── LEFT PANEL ─────────────────────────────────────────────────────────────

    def _build_left(self):
        frame = ctk.CTkScrollableFrame(self, width=280, corner_radius=0)
        frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)

        lp = self._logo_path()
        if lp:
            try:
                img   = Image.open(lp)
                new_w = 200
                new_h = int(img.size[1] * new_w / img.size[0])
                img   = img.resize((new_w, new_h), Image.Resampling.LANCZOS)
                self._left_logo = ctk.CTkImage(light_image=img, dark_image=img, size=(new_w, new_h))
                ctk.CTkLabel(frame, image=self._left_logo, text='').pack(pady=(10, 20))
            except Exception:
                self._fallback_title(frame)
        else:
            self._fallback_title(frame)

        # Heartbeat counters — driven by live CAN
        self._hb_labels = {}
        for i, name in enumerate(JOINT_NAMES):
            lbl = ctk.CTkLabel(frame,
                               text=f'J{i+1} : 0',
                               font=ctk.CTkFont(family='Consolas', size=22, weight='bold'),
                               text_color='#E74C3C')
            lbl.pack(anchor='center', pady=2)
            self._hb_labels[name] = lbl

        ctk.CTkFrame(frame, height=2, fg_color='#444').pack(fill='x', padx=10, pady=20)

        # Quick Adjustments
        ctk.CTkLabel(frame, text='Quick Adjustments',
                     font=ctk.CTkFont(size=16, weight='bold')).pack(pady=(0, 10))

        for text, fg, hover, cmd in [
            ('HOME',          '#27AE60', '#1E8449', self._preset_home),
            ('EXT',           '#8E44AD', '#732D91', self._preset_ext),
            ('UP  (Preset)',  '#3498DB', '#2980B9', self._preset_up),
            ('DOWN (Preset)', '#E67E22', '#D35400', self._preset_down),
        ]:
            ctk.CTkButton(frame, text=text, height=40,
                          font=ctk.CTkFont(weight='bold', size=14),
                          fg_color=fg, hover_color=hover,
                          command=cmd).pack(fill='x', padx=15, pady=5)

    def _fallback_title(self, frame):
        ctk.CTkLabel(frame, text='SKAI ARM',
                     font=ctk.CTkFont(size=22, weight='bold')).pack(pady=(20, 2))
        ctk.CTkLabel(frame, text='Dashboard  v2.0',
                     font=ctk.CTkFont(size=11), text_color='gray').pack(pady=(0, 20))

    # ── CENTER PANEL ───────────────────────────────────────────────────────────

    def _build_center(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=0, column=1, sticky='nsew', padx=10, pady=10)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_rowconfigure(1, weight=0)
        frame.grid_columnconfigure(0, weight=1)

        # 3-D plot
        plot_bg = ctk.CTkFrame(frame, fg_color='#101010')
        plot_bg.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        self._fig = plt.Figure(figsize=(7, 5), dpi=100, facecolor='#101010')
        self._ax  = self._fig.add_subplot(111, projection='3d')
        self._init_axes()
        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_bg)
        self._canvas.get_tk_widget().pack(fill='both', expand=True)

        # Controls row
        ctrl = ctk.CTkFrame(frame)
        ctrl.grid(row=1, column=0, sticky='ew', padx=8, pady=(0, 8))

        # ── Incremental joint control (ARM_REDO_8 style) ──
        ctk.CTkLabel(ctrl, text='Set Incremental Steps  (degrees)',
                     font=ctk.CTkFont(size=13, weight='bold')).pack(pady=(10, 4))

        grid = ctk.CTkFrame(ctrl, fg_color='transparent')
        grid.pack()

        self._led_widgets = {}
        self._jvars       = {}

        for i, name in enumerate(JOINT_NAMES):
            # row 0: LED indicator — driven by live CAN heartbeat
            led = ctk.CTkLabel(grid, text='', image=self._leds['gray'][0])
            led.grid(row=0, column=i, pady=(0, 4))
            self._led_widgets[name] = led

            # row 1: joint label
            ctk.CTkLabel(grid, text=f'J{i+1}:',
                         font=ctk.CTkFont(size=12)).grid(row=1, column=i, padx=8)

            # row 2: delta entry
            var = ctk.StringVar(value='0.000')
            ctk.CTkEntry(grid, textvariable=var, width=70,
                         font=ctk.CTkFont(family='Consolas', size=11)
                         ).grid(row=2, column=i, padx=8, pady=(0, 4))
            self._jvars[name] = var

            # row 3: per-joint Send button
            ctk.CTkButton(grid, text='Send', width=70, height=24,
                          font=ctk.CTkFont(size=11, weight='bold'),
                          fg_color='#2980B9', hover_color='#1F618D',
                          command=lambda idx=i: self._send_single(idx)
                          ).grid(row=3, column=i, padx=8, pady=(0, 8))

        ctk.CTkButton(ctrl, text='SEND ALL',
                      font=ctk.CTkFont(weight='bold'),
                      fg_color='#2980B9', hover_color='#1F618D',
                      command=self._send_all).pack(pady=(0, 6))

        ctk.CTkFrame(ctrl, height=1, fg_color='#444').pack(fill='x', padx=20, pady=6)

        # ── Cartesian IK ──
        ctk.CTkLabel(ctrl, text='Cartesian Target  →  MoveIt IK',
                     font=ctk.CTkFont(size=13, weight='bold')).pack(pady=(0, 4))

        xyz_row = ctk.CTkFrame(ctrl, fg_color='transparent')
        xyz_row.pack()
        self._xyz = {}
        for axis, default in [('X', '0.30'), ('Y', '0.00'), ('Z', '0.40')]:
            col = ctk.CTkFrame(xyz_row, fg_color='transparent')
            col.pack(side='left', padx=12)
            ctk.CTkLabel(col, text=axis, font=ctk.CTkFont(weight='bold', size=14)).pack()
            var = ctk.StringVar(value=default)
            ctk.CTkEntry(col, textvariable=var, width=80,
                         font=ctk.CTkFont(family='Consolas')).pack()
            self._xyz[axis] = var

        ctk.CTkButton(ctrl, text='SEND IK GOAL',
                      fg_color='#27AE60', hover_color='#1E8449',
                      font=ctk.CTkFont(weight='bold'),
                      command=self._send_ik).pack(pady=(8, 6))

        ctk.CTkFrame(ctrl, height=1, fg_color='#444').pack(fill='x', padx=20, pady=6)

        # ── Safety ──
        safety = ctk.CTkFrame(ctrl, fg_color='transparent')
        safety.pack(pady=(0, 12))

        ctk.CTkButton(safety, text='E-STOP',
                      width=90, height=90, corner_radius=45,
                      font=ctk.CTkFont(weight='bold', size=18),
                      fg_color='#E74C3C', hover_color='#922B21',
                      command=self.node.estop).pack(side='left', padx=20)

        ctk.CTkButton(safety, text='CLEAR ERRORS\n& RESUME',
                      height=60,
                      font=ctk.CTkFont(weight='bold'),
                      fg_color='#27AE60', hover_color='#1E8449',
                      command=self.node.resume).pack(side='left', padx=20)

    # ── RIGHT PANEL ────────────────────────────────────────────────────────────

    def _build_right(self):
        frame = ctk.CTkScrollableFrame(self, width=320, corner_radius=0)
        frame.grid(row=0, column=2, sticky='nsew', padx=10, pady=10)

        ctk.CTkLabel(frame, text='Encoder Estimates',
                     font=ctk.CTkFont(size=20, weight='bold')).pack(pady=(20, 4))

        self._enc_ts_label = ctk.CTkLabel(frame, text='CAN: offline',
                                          font=ctk.CTkFont(family='Consolas', size=11),
                                          text_color='#E74C3C')
        self._enc_ts_label.pack(pady=(0, 8))

        self._enc_labels = {}
        for i, (name, label) in enumerate(zip(JOINT_NAMES, JOINT_LABELS)):
            card = ctk.CTkFrame(frame, fg_color='#2b2b2b', corner_radius=8)
            card.pack(fill='x', padx=10, pady=5)

            ctk.CTkLabel(card, text=f'{label}  (Node {NODE_IDS[i]})',
                         font=ctk.CTkFont(weight='bold', size=12)
                         ).pack(anchor='w', padx=10, pady=(6, 0))

            row = ctk.CTkFrame(card, fg_color='transparent')
            row.pack(fill='x', padx=10, pady=(0, 6))

            lbl_est = ctk.CTkLabel(row, text='Est: NO DATA',
                                   text_color='#E74C3C',
                                   font=ctk.CTkFont(family='Consolas', size=12))
            lbl_est.pack(side='left', anchor='w')

            lbl_raw = ctk.CTkLabel(row, text='Raw: NO DATA',
                                   text_color='#E74C3C',
                                   font=ctk.CTkFont(family='Consolas', size=12))
            lbl_raw.pack(side='right', anchor='e')

            self._enc_labels[name] = (lbl_est, lbl_raw)

        ctk.CTkFrame(frame, height=2, fg_color='#555').pack(fill='x', padx=15, pady=(24, 10))

        ctk.CTkLabel(frame, text='Drive / Controller Status',
                     font=ctk.CTkFont(size=16, weight='bold')).pack(pady=(0, 8))

        self._status_panel = ctk.CTkFrame(frame, fg_color='#331414',
                                          corner_radius=8,
                                          border_width=1, border_color='#E74C3C')
        self._status_panel.pack(fill='x', padx=10, pady=(0, 20))

        self._status_label = ctk.CTkLabel(self._status_panel,
                                          text='Connecting...',
                                          font=ctk.CTkFont(weight='bold', size=14),
                                          text_color='#F1C40F')
        self._status_label.pack(padx=10, pady=20)

    # ── 3-D axes ───────────────────────────────────────────────────────────────

    def _init_axes(self):
        self._ax.set_facecolor('#101010')
        for ax in [self._ax.xaxis, self._ax.yaxis, self._ax.zaxis]:
            ax.pane.fill = False
            ax.pane.set_edgecolor('gray')
            ax.set_tick_params(colors='gray')
            ax.label.set_color('white')
        self._ax.set_xlabel('X (m)')
        self._ax.set_ylabel('Y (m)')
        self._ax.set_zlabel('Z (m)')
        lim = 1.2
        self._ax.set_xlim([-lim, lim])
        self._ax.set_ylim([-lim, lim])
        self._ax.set_zlim([-0.1, lim * 1.2])
        self._ax.view_init(elev=30, azim=135)

    def _redraw_arm(self, angles_rad):
        pts = calculate_fk(angles_rad)
        for artist in list(self._ax.lines) + list(self._ax.collections):
            artist.remove()
        s = 0.8
        X, Y = np.meshgrid([-s, s], [-s, s])
        self._ax.plot_surface(X, Y, np.zeros_like(X), color='#2c3e50', alpha=0.4)
        link_colors = ['#555555', '#3498DB', '#3498DB', '#ECF0F1', '#ECF0F1', '#E74C3C']
        th = 12
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            c = link_colors[i] if i < len(link_colors) else 'white'
            self._ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                          color=c, linewidth=th, solid_capstyle='round')
            self._ax.scatter(p1[0], p1[1], p1[2], color='black', s=80, edgecolors='white', zorder=10)
            if th > 5:
                th -= 1.2
        ee = pts[-1]
        self._ax.scatter(ee[0], ee[1], ee[2], color='#E74C3C', s=180, marker='*', zorder=11)
        self._canvas.draw_idle()

    # ── CAN monitoring (ARM_REDO_8 style, read-only) ───────────────────────────

    def _setup_can_monitor(self):
        self._sdo_rr_idx = 0
        try:
            self._can_bus = can.interface.Bus(
                channel=_CAN_BUS, interface=_CAN_IFACE, bitrate=250000)
            t = threading.Thread(target=self._can_rx_thread, daemon=True)
            t.start()
            self.after(100, self._poll_sdo)
            print(f"CAN monitor connected: {_CAN_BUS}")
        except Exception as e:
            print(f"CAN monitor offline: {e}")
            self.after(3000, self._setup_can_monitor)

    def _can_rx_thread(self):
        consecutive_errors = 0
        while self._can_running:
            if not self._can_bus:
                time.sleep(1)
                continue
            try:
                msg = self._can_bus.recv(timeout=0.1)
                consecutive_errors = 0
                if msg is None:
                    continue
                node_id = msg.arbitration_id >> 5
                cmd_id  = msg.arbitration_id & 0x1F
                if node_id not in NODE_IDS:
                    continue
                if cmd_id == _CAN_HB and len(msg.data) >= 5:
                    try:
                        err, state = struct.unpack('<IB', msg.data[:5])
                        self._can_hb_counts[node_id] += 1
                        self._can_states[node_id] = {
                            'error': err, 'state': state, 'last_update': time.time()}
                    except struct.error:
                        pass
                elif cmd_id == _CAN_ENC_EST and len(msg.data) >= 8:
                    try:
                        pos, _vel = struct.unpack('<ff', msg.data)
                        self._can_est_enc[node_id] = (pos, time.time())
                    except struct.error:
                        pass
                elif cmd_id == _CAN_SDO_TX and len(msg.data) >= 8:
                    try:
                        opcode, ep_id, _ = struct.unpack('<BHB', msg.data[:4])
                        if opcode == 0:
                            if (node_id in [21, 22, 23] and ep_id == 704) or \
                               (node_id in [24, 25, 26] and ep_id == 644):
                                raw_val = struct.unpack('<f', msg.data[4:8])[0]
                                self._can_raw_enc[node_id] = (raw_val, time.time())
                    except struct.error:
                        pass
            except can.CanError:
                consecutive_errors += 1
                if consecutive_errors >= 20:
                    print("CAN monitor: bus error — reconnecting...")
                    try:
                        self._can_bus.shutdown()
                    except Exception:
                        pass
                    self._can_bus = None
                    consecutive_errors = 0
                    time.sleep(2.0)
                    try:
                        self._can_bus = can.interface.Bus(
                            channel=_CAN_BUS, interface=_CAN_IFACE, bitrate=250000)
                        print("CAN monitor: reconnected")
                    except Exception as e:
                        print(f"CAN monitor: reconnect failed: {e}")
                else:
                    time.sleep(0.1)

    def _poll_sdo(self):
        if not self._can_running:
            return
        if self._can_bus:
            # Round-robin: one node per tick to avoid flooding the TX queue
            node_id = NODE_IDS[self._sdo_rr_idx % len(NODE_IDS)]
            self._sdo_rr_idx += 1
            ep     = 704 if node_id in [21, 22, 23] else 644
            arb_id = (node_id << 5) | _CAN_SDO_RX
            try:
                data = struct.pack('<BHB4s', 0, ep, 0, b'\x00\x00\x00\x00')
                msg  = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
                self._can_bus.send(msg)
            except can.CanError:
                pass
        self.after(100, self._poll_sdo)

    # ── 100 ms update loop ─────────────────────────────────────────────────────

    def _update_loop(self):
        now = time.time()

        positions, last_ts = self.node.get_joint_positions()
        ctrl_active        = self.node.get_controller_active()
        online = last_ts > 0 and (now - last_ts) < 1.0

        if online and not self._entries_initialized:
            for name in JOINT_NAMES:
                self._jvars[name].set('0.000')
            self._entries_initialized = True

        angles = [positions[n] for n in JOINT_NAMES]

        # Heartbeat counters — from live CAN
        any_can_live = False
        for i, name in enumerate(JOINT_NAMES):
            node_id = NODE_IDS[i]
            cnt  = self._can_hb_counts[node_id]
            st   = self._can_states[node_id]
            live = cnt > 0 and (now - st['last_update']) < 2.0
            if live:
                any_can_live = True
            color = '#00FF00' if live else '#E74C3C'
            self._hb_labels[name].configure(text=f'J{i+1} : {cnt}', text_color=color)

        # LED indicators — per-node, from live CAN heartbeat state only
        for i, name in enumerate(JOINT_NAMES):
            node_id = NODE_IDS[i]
            st   = self._can_states[node_id]
            live = self._can_hb_counts[node_id] > 0 and (now - st['last_update']) < 2.0
            if live:
                if st['error'] != 0:
                    key = 'red'
                elif st['state'] == _STATE_CLOSED:
                    key = 'green'
                elif st['state'] == _STATE_IDLE:
                    key = 'blue'
                else:
                    key = 'yellow'
                img = self._leds[key][self._glow_idx]
            else:
                img = self._leds['gray'][0]
            self._led_widgets[name].configure(image=img)

        # CAN status timestamp
        if any_can_live:
            self._enc_ts_label.configure(text='CAN: live', text_color='#2ECC71')
        else:
            self._enc_ts_label.configure(text='CAN: offline', text_color='#E74C3C')

        # Encoder right panel — raw CAN values (ARM_REDO_8 style)
        for i, name in enumerate(JOINT_NAMES):
            node_id          = NODE_IDS[i]
            lbl_est, lbl_raw = self._enc_labels[name]
            est_data = self._can_est_enc[node_id]
            raw_data = self._can_raw_enc[node_id]

            if est_data is not None and (now - est_data[1]) < 2.0 \
                    and not math.isnan(est_data[0]):
                lbl_est.configure(text=f'Est: {est_data[0]:+.3f} rev',
                                  text_color='#00FF00')
            else:
                lbl_est.configure(text='Est: NO DATA', text_color='#E74C3C')

            if raw_data is not None and (now - raw_data[1]) < 2.0 \
                    and not math.isnan(raw_data[0]):
                lbl_raw.configure(text=f'Raw: {raw_data[0]:+.3f} rev',
                                  text_color='#3498DB')
            else:
                lbl_raw.configure(text='Raw: NO DATA', text_color='#E74C3C')

        # Controller status panel
        if ctrl_active:
            self._status_panel.configure(fg_color='#143320', border_color='#2ECC71')
            self._status_label.configure(text='arm_controller  ACTIVE',   text_color='#2ECC71')
        elif online:
            self._status_panel.configure(fg_color='#332B14', border_color='#F1C40F')
            self._status_label.configure(text='arm_controller  INACTIVE', text_color='#F1C40F')
        else:
            self._status_panel.configure(fg_color='#331414', border_color='#E74C3C')
            self._status_label.configure(text='ros2_control  OFFLINE',    text_color='#E74C3C')

        self._redraw_arm(angles)
        self.after(100, self._update_loop)

    def _animate_leds(self):
        self._glow_idx = (self._glow_idx + 1) % 16
        self.after(50, self._animate_leds)

    # ── button callbacks ───────────────────────────────────────────────────────

    def _send_single(self, joint_idx: int):
        name = JOINT_NAMES[joint_idx]
        try:
            delta_rad = math.radians(float(self._jvars[name].get()))
        except ValueError:
            return
        positions, _ = self.node.get_joint_positions()
        target = dict(positions)
        target[name] = positions[name] + delta_rad
        self.node.send_joint_trajectory(target)

    def _send_all(self):
        try:
            deltas_rad = {n: math.radians(float(self._jvars[n].get())) for n in JOINT_NAMES}
        except ValueError:
            return
        positions, _ = self.node.get_joint_positions()
        target = {n: positions[n] + deltas_rad[n] for n in JOINT_NAMES}
        self.node.send_joint_trajectory(target)

    def _send_ik(self):
        try:
            x = float(self._xyz['X'].get())
            y = float(self._xyz['Y'].get())
            z = float(self._xyz['Z'].get())
        except ValueError:
            return
        self.node.send_ik_goal(x, y, z)

    # ── presets (nearest-turn logic from ARM_REDO_8) ───────────────────────────

    def _preset_home(self):
        positions, _ = self.node.get_joint_positions()
        target = {n: nearest_turn_target(positions[n], HOME_RAD[i])
                  for i, n in enumerate(JOINT_NAMES)}
        self.node.send_joint_trajectory(target)

    def _preset_ext(self):
        positions, _ = self.node.get_joint_positions()
        target = {n: nearest_turn_target(positions[n], EXT_RAD[i])
                  for i, n in enumerate(JOINT_NAMES)}
        self.node.send_joint_trajectory(target)

    def _preset_up(self):
        positions, _ = self.node.get_joint_positions()
        target = {n: positions[n] + UP_DELTA[i] for i, n in enumerate(JOINT_NAMES)}
        self.node.send_joint_trajectory(target)

    def _preset_down(self):
        positions, _ = self.node.get_joint_positions()
        target = {n: positions[n] + DOWN_DELTA[i] for i, n in enumerate(JOINT_NAMES)}
        self.node.send_joint_trajectory(target)

    # ── cleanup ────────────────────────────────────────────────────────────────

    def destroy(self):
        self._can_running = False
        if self._can_bus:
            try:
                self._can_bus.shutdown()
            except Exception:
                pass
        super().destroy()


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    ros_node = DashboardNode()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = SKAIDashboard(ros_node)
    app.protocol('WM_DELETE_WINDOW', app.destroy)
    app.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
