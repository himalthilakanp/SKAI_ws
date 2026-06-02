#!/usr/bin/env python3

import math
import time
import threading

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

JOINT_NAMES = ['ROT_1', 'PITCH_1', 'PITCH_2', 'PITCH_3', 'ROT_2', 'ROT_3']
JOINT_LABELS = ['J1 Base', 'J2 Shoulder', 'J3 Elbow', 'J4 Wrist 1', 'J5 Wrist 2', 'J6 Wrist 3']

# URDF joint origins — (xyz_m, rpy_rad, axis)
# Source: arm_rviz/urdf/skai_arm.urdf
JOINT_DEFS = [
    # ROT_1:   BASE  → J_1
    ((-0.067, -0.065,  0.082), (0.0,      0.0,     0.0    ), (0,  0, -1)),
    # PITCH_1: J_1   → J_2
    (( 0.000, -0.046,  0.194), (1.5708,   1.2217,  3.1416 ), (0,  0, -1)),
    # PITCH_2: J_2   → J_3
    ((-0.37152, 0.2145, 0.107), (1.5708,  0.0,     0.87266), (0,  1,  0)),
    # PITCH_3: J_3   → J_4
    (( 0.3542,  0.012,  0.2045),(1.5708,  0.68579, 0.0    ), (0,  0,  1)),
    # ROT_2:   J_4   → J_5
    (( 0.023514, 0.062204, -0.0725),(-1.5708, 0.3614, 2.7802),(0, 0, -1)),
    # ROT_3:   J_5   → J_6
    ((-0.12262, -0.3244, -0.0725), (1.5708, -0.69813, -0.3614),(0, 0, -1)),
]

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


# ── FK math (exact URDF kinematics) ───────────────────────────────────────────

def _trans4(x, y, z):
    M = np.eye(4)
    M[:3, 3] = [x, y, z]
    return M


def _rpy4(roll, pitch, yaw):
    """URDF RPY convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    M = np.eye(4)
    M[0, 0] = cy * cp
    M[0, 1] = cy * sp * sr - sy * cr
    M[0, 2] = cy * sp * cr + sy * sr
    M[1, 0] = sy * cp
    M[1, 1] = sy * sp * sr + cy * cr
    M[1, 2] = sy * sp * cr - cy * sr
    M[2, 0] = -sp
    M[2, 1] = cp * sr
    M[2, 2] = cp * cr
    return M


def _axis_angle4(axis, angle):
    """Rodrigues rotation around unit axis by angle."""
    ax, ay, az = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    M = np.eye(4)
    M[0, 0] = t*ax*ax + c
    M[0, 1] = t*ax*ay - s*az
    M[0, 2] = t*ax*az + s*ay
    M[1, 0] = t*ax*ay + s*az
    M[1, 1] = t*ay*ay + c
    M[1, 2] = t*ay*az - s*ax
    M[2, 0] = t*ax*az - s*ay
    M[2, 1] = t*ay*az + s*ax
    M[2, 2] = t*az*az + c
    return M


def calculate_fk(joint_angles_rad):
    """
    Forward kinematics from URDF joint definitions.
    Returns (7,3) array: [BASE origin, J1, J2, J3, J4, J5, J6] in metres.
    """
    points = [np.zeros(3)]
    T = np.eye(4)

    for (xyz, rpy, axis), q in zip(JOINT_DEFS, joint_angles_rad):
        T_origin = _trans4(*xyz) @ _rpy4(*rpy)
        T_joint  = _axis_angle4(axis, q)
        T = T @ T_origin @ T_joint
        points.append(T[:3, 3].copy())

    return np.array(points)


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class DashboardNode(Node):

    def __init__(self):
        super().__init__('skai_dashboard')

        self._lock = threading.Lock()
        self._joint_positions  = {n: 0.0 for n in JOINT_NAMES}
        self._joint_last_ts    = 0.0
        self._controller_active = False

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        self._can_enable_pub = self.create_publisher(
            Bool, '/can_send_enable', 10)

        self._target_pub = self.create_publisher(
            TargetPose, '/target_pose', 10)

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self._switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        self._list_cli = self.create_client(
            ListControllers, '/controller_manager/list_controllers')

        self.create_timer(1.0, self._poll_controller_state)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_positions:
                    self._joint_positions[name] = pos
            self._joint_last_ts = time.time()

    def _poll_controller_state(self):
        if not self._list_cli.service_is_ready():
            return
        future = self._list_cli.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_done)

    def _on_list_done(self, future):
        try:
            result = future.result()
            active = any(
                c.name == 'arm_controller' and c.state == 'active'
                for c in result.controller
            )
            with self._lock:
                self._controller_active = active
        except Exception:
            pass

    # ── Thread-safe getters ────────────────────────────────────────────────────

    def get_joint_positions(self):
        with self._lock:
            return dict(self._joint_positions), self._joint_last_ts

    def get_controller_active(self):
        with self._lock:
            return self._controller_active

    # ── Commands ───────────────────────────────────────────────────────────────

    def _set_can_send(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self._can_enable_pub.publish(msg)

    def send_ik_goal(self, x, y, z):
        self._set_can_send(True)
        msg = TargetPose()
        msg.x, msg.y, msg.z = x, y, z
        msg.roll = msg.pitch = msg.yaw = 0.0
        self._target_pub.publish(msg)

    def send_joint_trajectory(self, positions_rad):
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
        self.title("SKAI Arm Dashboard")
        self.geometry("1540x840")

        self._led_frame_idx = 0
        self._entries_initialized = False
        self._precompute_leds()
        self._build_ui()
        self._update_loop()
        self._animate_leds()

    # ── LED precompute ─────────────────────────────────────────────────────────

    def _precompute_leds(self):
        self._leds = {'green': [], 'red': [], 'yellow': [], 'gray': []}

        def make(color, pulse):
            sz = 64
            img = Image.new("RGBA", (sz, sz), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img, "RGBA")
            cx, cy = sz // 2, sz // 2
            r, g, b = color
            gr = 20 + int(pulse * 8)
            for i in range(8):
                a = int(max(0, 22 - i * 2) * 2.55)
                rad = gr - i * 2
                if rad > 0:
                    draw.ellipse([cx-rad, cy-rad, cx+rad, cy+rad], fill=(r, g, b, a))
            for i in range(5):
                a = int(max(0, 50 - i * 8) * 2.55)
                rad = 12 - i
                if rad > 0:
                    draw.ellipse([cx-rad, cy-rad, cx+rad, cy+rad], fill=(r, g, b, a))
            draw.ellipse([cx-5, cy-5, cx+5, cy+5], fill=(240, 255, 240, 255))
            draw.ellipse([cx-3, cy-3, cx-1, cy-1], fill=(255, 255, 255, 180))
            return ctk.CTkImage(light_image=img, dark_image=img, size=(30, 30))

        for i in range(16):
            p = (math.sin(i * math.pi / 8) + 1) / 2
            self._leds['green'].append(make((80, 255, 120), p))
            self._leds['red'].append(make((255, 50, 50), p))
            self._leds['yellow'].append(make((255, 200, 0), p))
        self._leds['gray'].append(make((100, 100, 100), 0))

    # ── UI layout ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=3)
        self.grid_columnconfigure(2, weight=0)
        self._build_left()
        self._build_center()
        self._build_right()

    # ── Left panel ─────────────────────────────────────────────────────────────

    def _build_left(self):
        frame = ctk.CTkScrollableFrame(self, width=250, corner_radius=0)
        frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)

        ctk.CTkLabel(
            frame, text="SKAI ARM",
            font=ctk.CTkFont(size=22, weight='bold')
        ).pack(pady=(20, 2))

        ctk.CTkLabel(
            frame, text="Dashboard  v1.0",
            font=ctk.CTkFont(size=11), text_color='gray'
        ).pack(pady=(0, 20))

        ctk.CTkLabel(
            frame, text="Joint Status",
            font=ctk.CTkFont(size=14, weight='bold')
        ).pack(pady=(0, 8))

        self._led_widgets  = {}
        self._status_texts = {}

        for name, label in zip(JOINT_NAMES, JOINT_LABELS):
            row = ctk.CTkFrame(frame, fg_color='transparent')
            row.pack(fill='x', padx=12, pady=3)

            led = ctk.CTkLabel(row, text='', image=self._leds['gray'][0])
            led.pack(side='left', padx=(0, 8))

            lbl = ctk.CTkLabel(
                row, text=label,
                font=ctk.CTkFont(family='Consolas', size=13)
            )
            lbl.pack(side='left')

            self._led_widgets[name]  = led
            self._status_texts[name] = lbl

        ctk.CTkFrame(frame, height=2, fg_color='#444').pack(
            fill='x', padx=10, pady=20)

        ctk.CTkLabel(
            frame, text="Presets",
            font=ctk.CTkFont(size=14, weight='bold')
        ).pack(pady=(0, 10))

        for text, fg, hover, cmd in [
            ('HOME',       '#27AE60', '#1E8449', self._preset_home),
            ('EXTENDED',   '#8E44AD', '#732D91', self._preset_extended),
            ('ZERO ALL',   '#2980B9', '#1F618D', self._preset_zero),
        ]:
            ctk.CTkButton(
                frame, text=text, height=38,
                font=ctk.CTkFont(weight='bold'),
                fg_color=fg, hover_color=hover,
                command=cmd
            ).pack(fill='x', padx=15, pady=5)

    # ── Center panel ───────────────────────────────────────────────────────────

    def _build_center(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=0, column=1, sticky='nsew', padx=10, pady=10)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_rowconfigure(1, weight=0)
        frame.grid_columnconfigure(0, weight=1)

        # 3D plot
        plot_bg = ctk.CTkFrame(frame, fg_color='#101010')
        plot_bg.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)

        self._fig = plt.Figure(figsize=(7, 5), dpi=100, facecolor='#101010')
        self._ax  = self._fig.add_subplot(111, projection='3d')
        self._init_axes()

        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_bg)
        self._canvas.get_tk_widget().pack(fill='both', expand=True)

        # Controls
        ctrl = ctk.CTkFrame(frame)
        ctrl.grid(row=1, column=0, sticky='ew', padx=8, pady=(0, 8))

        # Cartesian IK
        ctk.CTkLabel(
            ctrl, text="Cartesian Target  →  MoveIt IK",
            font=ctk.CTkFont(size=13, weight='bold')
        ).pack(pady=(10, 6))

        xyz_row = ctk.CTkFrame(ctrl, fg_color='transparent')
        xyz_row.pack()
        self._xyz = {}
        for axis, default in [('X', '0.30'), ('Y', '0.00'), ('Z', '0.40')]:
            col = ctk.CTkFrame(xyz_row, fg_color='transparent')
            col.pack(side='left', padx=12)
            ctk.CTkLabel(
                col, text=axis,
                font=ctk.CTkFont(weight='bold', size=14)
            ).pack()
            var = ctk.StringVar(value=default)
            ctk.CTkEntry(
                col, textvariable=var, width=80,
                font=ctk.CTkFont(family='Consolas')
            ).pack()
            self._xyz[axis] = var

        ctk.CTkButton(
            ctrl, text="SEND IK GOAL",
            fg_color='#2980B9', hover_color='#1F618D',
            font=ctk.CTkFont(weight='bold'),
            command=self._send_ik
        ).pack(pady=(8, 4))

        ctk.CTkFrame(ctrl, height=1, fg_color='#444').pack(
            fill='x', padx=20, pady=10)

        # Direct joint angles
        ctk.CTkLabel(
            ctrl, text="Direct Joint Angles  (radians)",
            font=ctk.CTkFont(size=13, weight='bold')
        ).pack(pady=(0, 6))

        jrow = ctk.CTkFrame(ctrl, fg_color='transparent')
        jrow.pack()
        self._jvars = {}
        for i, name in enumerate(JOINT_NAMES):
            col = ctk.CTkFrame(jrow, fg_color='transparent')
            col.pack(side='left', padx=6)
            ctk.CTkLabel(
                col, text=f'J{i+1}',
                font=ctk.CTkFont(size=11, weight='bold')
            ).pack()
            var = ctk.StringVar(value='0.00')
            ctk.CTkEntry(
                col, textvariable=var, width=70,
                font=ctk.CTkFont(family='Consolas', size=11)
            ).pack()
            self._jvars[name] = var

        ctk.CTkButton(
            ctrl, text="SEND JOINT POSITIONS",
            fg_color='#16A085', hover_color='#0E6655',
            font=ctk.CTkFont(weight='bold'),
            command=self._send_joints
        ).pack(pady=(6, 8))

        # Safety row
        safety = ctk.CTkFrame(ctrl, fg_color='transparent')
        safety.pack(pady=(0, 10))

        ctk.CTkButton(
            safety, text="E-STOP",
            width=100, height=80, corner_radius=40,
            font=ctk.CTkFont(weight='bold', size=16),
            fg_color='#E74C3C', hover_color='#922B21',
            command=self.node.estop
        ).pack(side='left', padx=20)

        ctk.CTkButton(
            safety, text="RESUME",
            width=110, height=50,
            font=ctk.CTkFont(weight='bold'),
            fg_color='#27AE60', hover_color='#1E8449',
            command=self.node.resume
        ).pack(side='left', padx=20)

    # ── Right panel ────────────────────────────────────────────────────────────

    def _build_right(self):
        frame = ctk.CTkScrollableFrame(self, width=290, corner_radius=0)
        frame.grid(row=0, column=2, sticky='nsew', padx=10, pady=10)

        ctk.CTkLabel(
            frame, text="Encoder Feedback",
            font=ctk.CTkFont(size=18, weight='bold')
        ).pack(pady=(20, 4))

        self._enc_ts_label = ctk.CTkLabel(
            frame, text='Last update: --',
            font=ctk.CTkFont(family='Consolas', size=11),
            text_color='gray'
        )
        self._enc_ts_label.pack(pady=(0, 8))

        self._jstate_labels = {}

        for name, label in zip(JOINT_NAMES, JOINT_LABELS):
            card = ctk.CTkFrame(frame, fg_color='#2b2b2b', corner_radius=8)
            card.pack(fill='x', padx=10, pady=6)

            ctk.CTkLabel(
                card, text=f"{label}  ({name})",
                font=ctk.CTkFont(weight='bold', size=12)
            ).pack(anchor='w', padx=10, pady=(6, 0))

            lbl_rad = ctk.CTkLabel(
                card, text='enc rad:   ---',
                font=ctk.CTkFont(family='Consolas', size=12),
                text_color='#E74C3C'
            )
            lbl_rad.pack(anchor='w', padx=16)

            lbl_turns = ctk.CTkLabel(
                card, text='enc turns: ---',
                font=ctk.CTkFont(family='Consolas', size=12),
                text_color='#E74C3C'
            )
            lbl_turns.pack(anchor='w', padx=16, pady=(0, 6))

            self._jstate_labels[name] = (lbl_rad, lbl_turns)

        ctk.CTkFrame(frame, height=2, fg_color='#444').pack(
            fill='x', padx=10, pady=18)

        ctk.CTkLabel(
            frame, text="Controller Status",
            font=ctk.CTkFont(size=16, weight='bold')
        ).pack(pady=(0, 8))

        self._ctrl_panel = ctk.CTkFrame(
            frame, fg_color='#331414',
            corner_radius=8, border_width=1, border_color='#E74C3C'
        )
        self._ctrl_panel.pack(fill='x', padx=10, pady=(0, 20))

        self._ctrl_label = ctk.CTkLabel(
            self._ctrl_panel,
            text="Connecting...",
            font=ctk.CTkFont(weight='bold', size=13),
            text_color='#F1C40F'
        )
        self._ctrl_label.pack(padx=10, pady=16)

    # ── FK / plot ──────────────────────────────────────────────────────────────

    def _init_axes(self):
        self._ax.set_facecolor('#101010')
        for axis in [self._ax.xaxis, self._ax.yaxis, self._ax.zaxis]:
            axis.pane.fill = False
            axis.pane.set_edgecolor('gray')
            axis.set_tick_params(colors='gray')
            axis.label.set_color('white')
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

        # Ground plane
        s = 0.8
        X, Y = np.meshgrid([-s, s], [-s, s])
        self._ax.plot_surface(X, Y, np.zeros_like(X),
                              color='#2c3e50', alpha=0.4)

        link_colors = ['#555555', '#3498DB', '#3498DB',
                       '#ECF0F1', '#ECF0F1', '#E74C3C']
        thickness = 12

        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            c = link_colors[i] if i < len(link_colors) else 'white'
            self._ax.plot(
                [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color=c, linewidth=thickness, solid_capstyle='round'
            )
            self._ax.scatter(p1[0], p1[1], p1[2],
                             color='black', s=80, edgecolors='white', zorder=10)
            if thickness > 5:
                thickness -= 1.2

        ee = pts[-1]
        self._ax.scatter(ee[0], ee[1], ee[2],
                         color='#E74C3C', s=180, marker='*', zorder=11)

        self._canvas.draw_idle()

    # ── Periodic update (100 ms) ───────────────────────────────────────────────

    def _update_loop(self):
        positions, last_ts = self.node.get_joint_positions()
        ctrl_active = self.node.get_controller_active()
        online = last_ts > 0 and (time.time() - last_ts) < 1.0

        angles = [positions[n] for n in JOINT_NAMES]

        # Seed joint entry fields once from encoder so pressing SEND
        # without editing a joint doesn't snap it to 0.0.
        if online and not self._entries_initialized:
            for name in JOINT_NAMES:
                self._jvars[name].set(f'{positions[name]:.3f}')
            self._entries_initialized = True

        # Right panel — encoder feedback cards + timestamp
        if online:
            age = time.time() - last_ts
            self._enc_ts_label.configure(
                text=f'Last update: {age:.2f}s ago', text_color='#2ECC71')
        else:
            self._enc_ts_label.configure(
                text='Last update: no data', text_color='#E74C3C')

        for name in JOINT_NAMES:
            lbl_rad, lbl_turns = self._jstate_labels[name]
            rad   = positions[name]
            turns = rad / (2 * math.pi)
            color = '#00FF00' if online else '#E74C3C'
            lbl_rad.configure(
                text=f'enc rad:   {rad:+.4f}', text_color=color)
            lbl_turns.configure(
                text=f'enc turns: {turns:+.4f}', text_color=color)

        # Right panel — controller status
        if ctrl_active:
            self._ctrl_panel.configure(
                fg_color='#143320', border_color='#2ECC71')
            self._ctrl_label.configure(
                text='arm_controller  ACTIVE', text_color='#2ECC71')
        elif online:
            self._ctrl_panel.configure(
                fg_color='#332B14', border_color='#F1C40F')
            self._ctrl_label.configure(
                text='arm_controller  INACTIVE', text_color='#F1C40F')
        else:
            self._ctrl_panel.configure(
                fg_color='#331414', border_color='#E74C3C')
            self._ctrl_label.configure(
                text='ros2_control  OFFLINE', text_color='#E74C3C')

        # Left panel — LED color
        if online and ctrl_active:
            key = 'green'
        elif online:
            key = 'yellow'
        else:
            key = 'gray'

        idx = self._led_frame_idx if key != 'gray' else 0
        for name in JOINT_NAMES:
            img = self._leds[key][idx] if key != 'gray' else self._leds['gray'][0]
            self._led_widgets[name].configure(image=img)

        # 3D FK redraw
        self._redraw_arm(angles)

        self.after(100, self._update_loop)

    def _animate_leds(self):
        self._led_frame_idx = (self._led_frame_idx + 1) % 16
        self.after(50, self._animate_leds)

    # ── Button callbacks ───────────────────────────────────────────────────────

    def _send_ik(self):
        try:
            x = float(self._xyz['X'].get())
            y = float(self._xyz['Y'].get())
            z = float(self._xyz['Z'].get())
        except ValueError:
            return
        self.node.send_ik_goal(x, y, z)

    def _send_joints(self):
        try:
            positions = {
                n: float(self._jvars[n].get())
                for n in JOINT_NAMES
            }
        except ValueError:
            return
        self.node.send_joint_trajectory(positions)

    def _preset_home(self):
        self.node.send_joint_trajectory({n: 0.0 for n in JOINT_NAMES})

    def _preset_extended(self):
        self.node.send_joint_trajectory({
            'ROT_1':   0.0,
            'PITCH_1': -1.57,
            'PITCH_2':  1.57,
            'PITCH_3':  0.0,
            'ROT_2':    0.0,
            'ROT_3':    0.0,
        })

    def _preset_zero(self):
        self.node.send_joint_trajectory({n: 0.0 for n in JOINT_NAMES})


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    ros_node = DashboardNode()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = SKAIDashboard(ros_node)
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    app.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
