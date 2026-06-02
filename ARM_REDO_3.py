import os
import time
import struct
import threading
import math
import numpy as np
import customtkinter as ctk
import can
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageDraw

# --- Configuration ---
CAN_BUS = 'can0'
CAN_BUSTYPE = 'socketcan'
NODE_IDS = [21, 22, 23, 24, 25, 26]  # J1 to J6

# ODrive CAN Command IDs
MSG_ID_HEARTBEAT = 0x001
MSG_ID_RX_SDO = 0x04                 
MSG_ID_TX_SDO = 0x05                 
MSG_ID_SET_AXIS_STATE = 0x007
MSG_ID_GET_ENCODER_ESTIMATES = 0x009
MSG_ID_SET_INPUT_POS = 0x00C
MSG_ID_CLEAR_ERRORS = 0x018

# ODrive State Definitions
AXIS_STATE_IDLE = 1                  
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# Standard appearance setup
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


class RobotArmControlPanel(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("RPi5 ODrive Arm Controller v2.0 (UR10 Visualizer)")
        self.geometry("1500x800")  

        # Data storage
        self.heartbeat_counts = {node: 0 for node in NODE_IDS}
        self.encoder_data = {node: None for node in NODE_IDS}  
        self.raw_encoder_data = {node: None for node in NODE_IDS}  
        self.node_states = {node: {'error': 0, 'state': 0, 'last_update': 0} for node in NODE_IDS} 
        self.joint_vars = []
        
        # Tracks which node's error we are currently viewing if multiple errors occur
        self.selected_error_node = None 
        
        # Absolute position tracking 
        self.absolute_positions = [0.5, 0.4, 0.6, 0.5, 0.5, 0.5] 

        # Generate the Neon LED Animation Frames
        self.precompute_neon_leds()

        # CAN setup
        self.bus = None
        self.setup_can_bus()

        # Build UI and 3D Visualizer
        self.build_ui()

        # Start background threads
        self.running = True
        self.can_thread = threading.Thread(target=self.can_rx_loop, daemon=True)
        self.can_thread.start()

        # Request Closed Loop Control on Startup
        self.request_closed_loop_all()

        # Start GUI periodic updates & polling
        self.update_gui_status()
        self.animate_leds()
        self.poll_sdo_endpoints()  

        # Trigger Fullscreen Splash Screen
        self.show_splash_screen()

    def get_image_path(self, filename):
        """Helper to safely find the image regardless of where the script is run from"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(current_dir, filename)
        if not os.path.exists(path):
            return filename
        return path

    def show_splash_screen(self):
        """Displays a borderless, fullscreen splash screen with ONLY the logo image on startup"""
        self.withdraw() # Temporarily hide the primary controller panel
        self.splash = ctk.CTkToplevel(self)
        self.splash.overrideredirect(True)
        self.splash.configure(fg_color="#101010")
        
        # Pull environment screen hardware dimensions
        screen_width = self.splash.winfo_screenwidth()
        screen_height = self.splash.winfo_screenheight()
        self.splash.geometry(f"{screen_width}x{screen_height}+0+0")

        try:
            img_path = self.get_image_path("logo_suriNova_Center.png")
            logo_image = Image.open(img_path)
            
            # Scale logo optimally for fullscreen views (Width = 800px)
            w_percent = (800 / float(logo_image.size[0]))
            h_size = int((float(logo_image.size[1]) * float(w_percent)))
            logo_image = logo_image.resize((800, h_size), Image.Resampling.LANCZOS)
            
            self.splash_img = ctk.CTkImage(light_image=logo_image, dark_image=logo_image, size=(800, h_size))
            
            lbl = ctk.CTkLabel(self.splash, image=self.splash_img, text="")
            lbl.pack(expand=True)
        except Exception as e:
            print(f"Splash Screen Warning: Could not execute asset build: {e}")

        # Kill splash layout and transition to the control grid after 2.5 seconds
        self.after(2500, self.close_splash)

    def close_splash(self):
        """Destroys startup splash top-level window and brings back main UI grid"""
        self.splash.destroy()
        self.deiconify()

    def precompute_neon_leds(self):
        """Generates multi-layered alpha gradient frames mimicking the PyQt5 Neon look"""
        self.led_frames = {'green': [], 'blue': [], 'red': [], 'yellow': [], 'gray': []}
        
        def create_glow_frame(color, pulse_strength):
            size = 64
            img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img, "RGBA")
            cx, cy = size // 2, size // 2
            r, g, b = color

            # Outer huge glow
            glow_radius = 20 + (pulse_strength * 8)
            for i in range(8):
                alpha = int(max(0, 22 - i * 2) * 2.55) # Scale to 255
                rad = glow_radius - i * 2
                if rad > 0:
                    draw.ellipse([cx - rad, cy - rad, cx + rad, cy + rad], fill=(r, g, b, alpha))

            # Mid concentrated glow
            for i in range(5):
                alpha = int(max(0, 50 - i * 8) * 2.55)
                rad = 12 - i
                if rad > 0:
                    draw.ellipse([cx - rad, cy - rad, cx + rad, cy + rad], fill=(r, g, b, alpha))

            # Solid Core
            draw.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(240, 255, 240, 255))
            
            # Glass Reflection (small offset)
            draw.ellipse([cx - 3, cy - 3, cx - 1, cy - 1], fill=(255, 255, 255, 180))

            return ctk.CTkImage(light_image=img, dark_image=img, size=(36, 36))

        # Generate 16 frames of breathing animation
        for i in range(16):
            pulse_strength = (math.sin(i * (math.pi / 8)) + 1) / 2 # Oscillates 0 to 1
            self.led_frames['green'].append(create_glow_frame((80, 255, 120), pulse_strength))
            self.led_frames['blue'].append(create_glow_frame((0, 170, 255), pulse_strength))
            self.led_frames['red'].append(create_glow_frame((255, 50, 50), pulse_strength))
            self.led_frames['yellow'].append(create_glow_frame((255, 200, 0), pulse_strength))
            
        # Offline/Gray LED doesn't pulse
        self.led_frames['gray'].append(create_glow_frame((100, 100, 100), 0))
        self.current_glow_idx = 0

    def setup_can_bus(self):
        """Initializes the CAN bus via python-can"""
        try:
            self.bus = can.interface.Bus(channel=CAN_BUS, bustype=CAN_BUSTYPE)
            print(f"Connected to CAN bus: {CAN_BUS}")
        except Exception as e:
            print(f"CAN Bus Error (Hardware Offline): {e}")
            self.bus = None

    def request_closed_loop_all(self):
        """Sends Enter Closed Loop command to all nodes on startup"""
        if not self.bus:
            print("ERROR: CAN bus hardware offline. Cannot request closed loop.")
            return

        print("--- Transmitting Enter Closed Loop (State 8) ---")
        for node_id in NODE_IDS:
            try:
                arbitration_id = (node_id << 5) | MSG_ID_SET_AXIS_STATE
                data = struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL)
                msg = can.Message(
                    arbitration_id=arbitration_id,
                    data=data,
                    is_extended_id=False
                )
                self.bus.send(msg)
                print(f"Node {node_id} -> Requested CLOSED_LOOP_CONTROL")
            except can.CanError as e:
                print(f"CAN Send Error on Node {node_id} (Closed Loop): {e}")
            
            time.sleep(0.01)  

    def estop_all(self):
        """Sends IDLE state to all joints instantly to cut motor power"""
        if not self.bus:
            print("ERROR: CAN bus offline. Cannot send E-STOP.")
            return

        print("!!! E-STOP TRIGGERED: Sending IDLE state to all nodes !!!")
        for node_id in NODE_IDS:
            try:
                arbitration_id = (node_id << 5) | MSG_ID_SET_AXIS_STATE
                data = struct.pack('<I', AXIS_STATE_IDLE)
                msg = can.Message(
                    arbitration_id=arbitration_id,
                    data=data,
                    is_extended_id=False
                )
                self.bus.send(msg)
                print(f"Node {node_id} -> Force IDLE (E-STOP)")
            except can.CanError as e:
                print(f"CAN Send Error on Node {node_id} (E-STOP): {e}")

    def clear_errors_and_resume(self):
        """Clears errors on all drives, then re-requests closed loop control"""
        self.selected_error_node = None 
        
        if not self.bus:
            print("ERROR: CAN bus offline. Cannot clear errors.")
            return

        print("--- Clearing Errors on All Nodes ---")
        for node_id in NODE_IDS:
            try:
                arbitration_id = (node_id << 5) | MSG_ID_CLEAR_ERRORS
                msg = can.Message(
                    arbitration_id=arbitration_id,
                    data=b'',
                    is_extended_id=False
                )
                self.bus.send(msg)
                print(f"Node {node_id} -> Cleared Errors")
            except can.CanError as e:
                print(f"CAN Send Error on Node {node_id} (Clear Errors): {e}")
        
        print("Waiting 100ms for errors to clear...")
        time.sleep(0.1) 
        
        self.request_closed_loop_all()

    def build_ui(self):
        """Constructs the CTK GUI layout"""
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)  
        self.grid_columnconfigure(1, weight=3)  
        self.grid_columnconfigure(2, weight=0)  

        joint_names = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist 1", "J5 Wrist 2", "J6 Wrist 3"]

        # ==================== LEFT PANEL ====================
        self.left_frame = ctk.CTkScrollableFrame(self, width=280, corner_radius=0)
        self.left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # 1. Add Logo
        try:
            img_path = self.get_image_path("logo_suriNova_Center.png")
            logo_image = Image.open(img_path)
            w_percent = (200 / float(logo_image.size[0]))
            h_size = int((float(logo_image.size[1]) * float(w_percent)))
            logo_image = logo_image.resize((200, h_size), Image.Resampling.LANCZOS)
            self.logo_img = ctk.CTkImage(light_image=logo_image, dark_image=logo_image, size=(200, h_size))
            ctk.CTkLabel(self.left_frame, image=self.logo_img, text="").pack(pady=(10, 20))
        except Exception as e:
            print(f"Warning: Could not load logo_suriNova_Center.png: {e}")

        # 2. Simplified "J1 : 1" Status Text Counter
        self.heartbeat_labels = {}
        for i, node in enumerate(NODE_IDS):
            frame = ctk.CTkFrame(self.left_frame, fg_color="transparent")
            frame.pack(fill="x", padx=15, pady=2)

            lbl_count = ctk.CTkLabel(frame, text=f"J{i+1} : 0", font=ctk.CTkFont(family="Consolas", size=22, weight="bold"))
            lbl_count.pack(anchor="center")

            self.heartbeat_labels[node] = lbl_count

        # 3. QUICK ADJUSTMENTS TAB (PLACED BELOW NODE COUNTERS)
        preset_frame = ctk.CTkFrame(self.left_frame, fg_color="transparent")
        preset_frame.pack(pady=(30, 10), fill="x")
        
        ctk.CTkLabel(preset_frame, text="Quick Adjustments", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(0, 10))
        
        self.btn_home = ctk.CTkButton(preset_frame, text="HOME", font=ctk.CTkFont(weight="bold", size=14), 
                                      fg_color="#27AE60", hover_color="#1E8449", height=40,
                                      command=self.move_preset_home)
        self.btn_home.pack(pady=(5, 5), padx=20, fill="x")

        self.btn_ext = ctk.CTkButton(preset_frame, text="EXT", font=ctk.CTkFont(weight="bold", size=14), 
                                      fg_color="#8E44AD", hover_color="#732D91", height=40,
                                      command=self.move_preset_ext)
        self.btn_ext.pack(pady=(5, 15), padx=20, fill="x")
        
        self.btn_up = ctk.CTkButton(preset_frame, text="UP (Preset)", font=ctk.CTkFont(weight="bold"), 
                                    fg_color="#3498DB", hover_color="#2980B9", height=40,
                                    command=self.move_preset_up)
        self.btn_up.pack(pady=5, padx=20, fill="x")
        
        self.btn_down = ctk.CTkButton(preset_frame, text="DOWN (Preset)", font=ctk.CTkFont(weight="bold"), 
                                      fg_color="#E67E22", hover_color="#D35400", height=40,
                                      command=self.move_preset_down)
        self.btn_down.pack(pady=5, padx=20, fill="x")


        # ==================== CENTER PANEL (Visualizer & Control) ====================
        self.center_frame = ctk.CTkFrame(self)
        self.center_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.center_frame.grid_rowconfigure(0, weight=1)  
        self.center_frame.grid_rowconfigure(1, weight=0)  
        self.center_frame.grid_columnconfigure(0, weight=1)

        # --- 3D Plot Area ---
        self.plot_frame = ctk.CTkFrame(self.center_frame, fg_color="#101010")
        self.plot_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        self.figure = plt.Figure(figsize=(7, 6), dpi=100, facecolor='#101010')
        self.ax = self.figure.add_subplot(111, projection='3d')

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # --- Controls Area (Bottom) ---
        self.controls_frame = ctk.CTkFrame(self.center_frame)
        self.controls_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))

        ctk.CTkLabel(self.controls_frame, text="Set Incremental Steps (-1.00 to +1.00)", font=ctk.CTkFont(weight="bold")).pack(
            pady=10)
        
        input_container = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        input_container.pack(pady=(0, 10))

        self.led_indicators = {}

        for i, node in enumerate(NODE_IDS):
            var = ctk.StringVar(value="0.00")
            self.joint_vars.append(var)

            # Row 0: Glowing Neon LEDs
            led = ctk.CTkLabel(input_container, text="", image=self.led_frames['gray'][0], cursor="hand2")
            led.grid(row=0, column=i, pady=(0, 5))
            led.bind("<Button-1>", lambda event, n=node: self.on_led_click(n))
            self.led_indicators[node] = led

            # Row 1: Labels
            lbl_j = ctk.CTkLabel(input_container, text=f"J{i + 1}:")
            lbl_j.grid(row=1, column=i, padx=10)

            # Row 2: Text Entries
            entry = ctk.CTkEntry(input_container, textvariable=var, width=65, font=ctk.CTkFont(family="Consolas"))
            entry.grid(row=2, column=i, padx=10, pady=(0, 5))

            # Row 3: Individual Send Buttons
            btn_single = ctk.CTkButton(input_container, text="Send", width=65, height=24,
                                       font=ctk.CTkFont(size=12, weight="bold"),
                                       fg_color="#2980B9", hover_color="#1F618D",
                                       command=lambda idx=i: self.send_single_position(idx))
            btn_single.grid(row=3, column=i, padx=10, pady=(0, 10))

        self.btn_send_all = ctk.CTkButton(self.controls_frame, text="SEND ALL OVER CAN", font=ctk.CTkFont(weight="bold"),
                                      fg_color="#2980B9", hover_color="#1F618D", command=self.send_all_positions)
        self.btn_send_all.pack(pady=(0, 10))

        # --- Safety & Recovery Controls ---
        safety_container = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        safety_container.pack(pady=(0, 15))

        self.btn_estop = ctk.CTkButton(
            safety_container, 
            text="E-STOP", 
            font=ctk.CTkFont(weight="bold", size=18),
            width=90, 
            height=90, 
            corner_radius=45, 
            fg_color="#E74C3C", 
            hover_color="#922B21", 
            command=self.estop_all
        )
        self.btn_estop.pack(side="left", padx=20)

        self.btn_clear_resume = ctk.CTkButton(
            safety_container, 
            text="CLEAR ERRORS\n& RESUME", 
            font=ctk.CTkFont(weight="bold"),
            height=60,
            fg_color="#27AE60", 
            hover_color="#1E8449", 
            command=self.clear_errors_and_resume
        )
        self.btn_clear_resume.pack(side="left", padx=20)

        self.link_colors = ['#555555', '#3498DB', '#3498DB', '#ECf0F1', '#ECf0F1', '#E74C3C']  


        # ==================== RIGHT PANEL ====================
        self.right_frame = ctk.CTkScrollableFrame(self, width=320, corner_radius=0) 
        self.right_frame.grid(row=0, column=2, sticky="nsew", padx=10, pady=10)

        ctk.CTkLabel(self.right_frame, text="Encoder Estimates", font=ctk.CTkFont(size=20, weight="bold")).pack(
            pady=(20, 10))

        self.encoder_labels = {}
        self.raw_labels = {}

        for i, node in enumerate(NODE_IDS):
            frame = ctk.CTkFrame(self.right_frame, fg_color="#2b2b2b", corner_radius=8)
            frame.pack(fill="x", padx=10, pady=8)

            lbl_title = ctk.CTkLabel(frame, text=f"{joint_names[i]} (Node {node})", font=ctk.CTkFont(weight="bold"))
            lbl_title.pack(anchor="w", padx=10, pady=(5, 0))

            data_container = ctk.CTkFrame(frame, fg_color="transparent")
            data_container.pack(fill="x", padx=10, pady=(0, 5))

            lbl_est = ctk.CTkLabel(data_container, text="Est: NO DATA", text_color="#E74C3C",
                                   font=ctk.CTkFont(family="Consolas", size=12))
            lbl_est.pack(side="left", anchor="w")
            
            lbl_raw = ctk.CTkLabel(data_container, text="Raw: NO DATA", text_color="#E74C3C",
                                   font=ctk.CTkFont(family="Consolas", size=12))
            lbl_raw.pack(side="right", anchor="e")

            self.encoder_labels[node] = lbl_est
            self.raw_labels[node] = lbl_raw

        # 4. DRIVE ERROR STATUS PANEL (MOVED BELOW ENCODER ESTIMATES)
        ctk.CTkFrame(self.right_frame, height=2, fg_color="#555555").pack(fill="x", padx=15, pady=(40, 10))
        ctk.CTkLabel(self.right_frame, text="Drive Error Status", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(0, 5))

        self.error_panel = ctk.CTkFrame(self.right_frame, fg_color="#143320", corner_radius=8, border_width=1, border_color="#2ECC71")
        self.error_panel.pack(fill="x", padx=15, pady=5)

        self.lbl_error_out = ctk.CTkLabel(self.error_panel, text="SYSTEM NORMAL", text_color="#2ECC71", font=ctk.CTkFont(weight="bold", size=14), justify="center")
        self.lbl_error_out.pack(padx=10, pady=20)


        self.initialize_view()
        self.update_arm_plot()

    # ==================== CAN LOGIC ====================
    
    def get_error_string(self, err_code):
        """Translates ODrive bitmask error codes into Plain-Text Strings"""
        if err_code == 0:
            return "NONE"
            
        known_errors = {
            0x01: "INVALID STATE",
            0x02: "MOTOR FAILED",
            0x04: "SENSORLESS ERR",
            0x08: "ENCODER FAILED",
            0x10: "CONTROLLER ERR",
            0x40: "WATCHDOG TIMEOUT",
            0x80: "OVER CURRENT",          
            0x100: "MIN ENDSTOP",
            0x200: "MAX ENDSTOP",
            0x400: "ESTOP REQUESTED",
            0x800: "VEL LIMIT REACHED",    
            0x1000: "HOMING ERR"
        }
        
        for bit in sorted(known_errors.keys(), reverse=True):
            if err_code & bit:
                return known_errors[bit]
                
        return f"UNKNOWN (0x{err_code:04X})"

    def on_led_click(self, node_id):
        """Callback for clicking a node's LED in the center panel"""
        if self.node_states[node_id]['error'] != 0:
            self.selected_error_node = node_id
            self.update_gui_status() 

    def poll_sdo_endpoints(self):
        """Actively requests specific raw absolute encoder data via TxSDO"""
        if self.bus:
            for node_id in NODE_IDS:
                endpoint_id = 704 if node_id in [21, 22, 23] else 644
                arbitration_id = (node_id << 5) | MSG_ID_RX_SDO
                
                data = struct.pack('<BHB4s', 0, endpoint_id, 0, b'\x00\x00\x00\x00')
                try:
                    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
                    self.bus.send(msg)
                except can.CanError:
                    pass
                    
        self.after(100, self.poll_sdo_endpoints)

    def can_rx_loop(self):
        """Background thread strictly listening for REAL heartbeats and Encoders"""
        while self.running:
            if self.bus:
                try:
                    msg = self.bus.recv(timeout=0.1)
                    if msg is not None:
                        node_id = msg.arbitration_id >> 5
                        cmd_id = msg.arbitration_id & 0x01F

                        if cmd_id == MSG_ID_HEARTBEAT and node_id in self.heartbeat_counts:
                            self.heartbeat_counts[node_id] += 1
                            if len(msg.data) >= 5:
                                try:
                                    axis_error, axis_state = struct.unpack('<IB', msg.data[:5])
                                    self.node_states[node_id] = {
                                        'error': axis_error, 
                                        'state': axis_state, 
                                        'last_update': time.time()
                                    }
                                except struct.error:
                                    pass
                        
                        elif cmd_id == MSG_ID_GET_ENCODER_ESTIMATES and node_id in self.encoder_data:
                            try:
                                pos, vel = struct.unpack('<ff', msg.data)
                                self.encoder_data[node_id] = (pos, time.time())
                            except struct.error:
                                pass 
                                
                        elif cmd_id == MSG_ID_TX_SDO:
                            if len(msg.data) >= 8:
                                try:
                                    opcode, ep_id, _ = struct.unpack('<BHB', msg.data[:4])
                                    if opcode == 0:  
                                        if (node_id in [21, 22, 23] and ep_id == 704) or \
                                           (node_id in [24, 25, 26] and ep_id == 644):
                                            
                                            raw_val = struct.unpack('<f', msg.data[4:8])[0]
                                            self.raw_encoder_data[node_id] = (raw_val, time.time())
                                except struct.error:
                                    pass

                except can.CanError:
                    time.sleep(0.1)
            else:
                time.sleep(1)

    def animate_leds(self):
        """Applies the current frame of the neon breathing animation (Runs at 20fps)"""
        self.current_glow_idx = (self.current_glow_idx + 1) % 16
        current_time = time.time()

        for node, led in self.led_indicators.items():
            state_data = self.node_states[node]
            
            if self.heartbeat_counts[node] > 0 and (current_time - state_data['last_update']) < 1.0:
                if state_data['error'] != 0:
                    img = self.led_frames['red'][self.current_glow_idx]
                elif state_data['state'] == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    img = self.led_frames['green'][self.current_glow_idx]
                elif state_data['state'] == AXIS_STATE_IDLE:
                    img = self.led_frames['blue'][self.current_glow_idx]
                else:
                    img = self.led_frames['yellow'][self.current_glow_idx]
            else:
                img = self.led_frames['gray'][0] 
                
            led.configure(image=img)

        self.after(50, self.animate_leds)

    def update_gui_status(self):
        """Updates text labels (Heartbeats, Encoders, Error Tab) every 100ms"""
        current_time = time.time()
        active_error_nodes = []

        # Simplified J1 : Count
        for i, node in enumerate(NODE_IDS):
            count = self.heartbeat_counts[node]
            if count > 0:
                self.heartbeat_labels[node].configure(text=f"J{i+1} : {count}", text_color="#00FF00")
            else:
                self.heartbeat_labels[node].configure(text=f"J{i+1} : {count}", text_color="#E74C3C")
                
            # Log active errors for the tab below
            state_data = self.node_states[node]
            if count > 0 and (current_time - state_data['last_update']) < 1.0 and state_data['error'] != 0:
                active_error_nodes.append(node)

        # --- Update Error Tab Logic ---
        if len(active_error_nodes) == 0:
            self.error_panel.configure(fg_color="#143320", border_color="#2ECC71")
            self.lbl_error_out.configure(text="SYSTEM NORMAL", text_color="#2ECC71")
            self.selected_error_node = None 
            
        elif len(active_error_nodes) == 1:
            err_node = active_error_nodes[0]
            err_code = self.node_states[err_node]['error']
            err_str = self.get_error_string(err_code)
            self.error_panel.configure(fg_color="#331414", border_color="#E74C3C")
            self.lbl_error_out.configure(text=f"NODE {err_node} ERROR:\n{err_str}", text_color="#E74C3C")
            
        else:
            if self.selected_error_node in active_error_nodes:
                err_code = self.node_states[self.selected_error_node]['error']
                err_str = self.get_error_string(err_code)
                self.error_panel.configure(fg_color="#331414", border_color="#E74C3C")
                self.lbl_error_out.configure(text=f"NODE {self.selected_error_node} ERROR:\n{err_str}", text_color="#E74C3C")
            else:
                self.error_panel.configure(fg_color="#332B14", border_color="#F1C40F")
                self.lbl_error_out.configure(text="MULTIPLE ERRORS!\nClick Red LED to View", text_color="#F1C40F")

        # Update Encoders
        for node in NODE_IDS:
            lbl_est = self.encoder_labels[node]
            lbl_raw = self.raw_labels[node]
            
            enc_data = self.encoder_data[node]
            raw_data = self.raw_encoder_data[node]

            if enc_data is not None and (current_time - enc_data[1]) < 1.0:
                if math.isnan(enc_data[0]):
                    lbl_est.configure(text="Est: ERR NAN", text_color="#F1C40F") 
                else:
                    lbl_est.configure(text=f"Est: {enc_data[0]:.3f} rev", text_color="#00FF00")
            else:
                lbl_est.configure(text="Est: NO DATA", text_color="#E74C3C")
                
            if raw_data is not None and (current_time - raw_data[1]) < 1.0:
                if math.isnan(raw_data[0]):
                    lbl_raw.configure(text="Raw: ERR NAN", text_color="#F1C40F") 
                else:
                    lbl_raw.configure(text=f"Raw: {raw_data[0]:.3f} rev", text_color="#3498DB")
            else:
                lbl_raw.configure(text="Raw: NO DATA", text_color="#E74C3C")

        self.after(100, self.update_gui_status)

    def send_single_position(self, joint_index):
        try:
            delta = float(self.joint_vars[joint_index].get())
            deltas = [0.0] * 6
            deltas[joint_index] = delta
            self.send_incremental_array(deltas)
            
        except ValueError:
            print(f"Warning: Invalid value at Joint J{joint_index + 1}")

    def send_all_positions(self):
        print("--- Transmitting ALL Incremental Arm Positions ---")
        try:
            deltas = [float(var.get()) for var in self.joint_vars]
            self.send_incremental_array(deltas)
        except ValueError:
            print("Warning: One or more fields contain invalid text.")

    def move_preset_home(self):
        print("--- Executing HOME Preset ---")
        # ---> UPDATED DESIGN: Configured HOME parameters dynamically matching your first calibration snapshot data layer <---
        home_offsets = [0.959, 0.545, 0.656, 0.000, 0.614, 0.000]
        current_time = time.time()
        deltas = [0.0] * 6
        
        for i, node_id in enumerate(NODE_IDS):
            enc_data = self.encoder_data[node_id]
            raw_data = self.raw_encoder_data[node_id]
            
            has_raw = raw_data is not None and (current_time - raw_data[1]) < 1.0 and not math.isnan(raw_data[0])
            has_est = enc_data is not None and (current_time - enc_data[1]) < 1.0 and not math.isnan(enc_data[0])
            
            if has_raw and has_est:
                turns = round(enc_data[0] - raw_data[0])
                current_pos = turns + raw_data[0]
            elif has_est:
                current_pos = enc_data[0]
            elif has_raw:
                current_pos = raw_data[0]
            else:
                current_pos = self.absolute_positions[i]
                
            turns = round(current_pos - home_offsets[i])
            target_pos = turns + home_offsets[i]
            
            deltas[i] = target_pos - current_pos

        self.send_incremental_array(deltas)

    def move_preset_ext(self):
        print("--- Executing EXT Preset ---")
        # ---> UPDATED DESIGN: Configured EXT parameters dynamically matching your second calibration snapshot data layer <---
        ext_offsets = [0.959, 0.764, 0.993, 0.000, 0.614, 0.000]
        current_time = time.time()
        deltas = [0.0] * 6
        
        for i, node_id in enumerate(NODE_IDS):
            enc_data = self.encoder_data[node_id]
            raw_data = self.raw_encoder_data[node_id]
            
            has_raw = raw_data is not None and (current_time - raw_data[1]) < 1.0 and not math.isnan(raw_data[0])
            has_est = enc_data is not None and (current_time - enc_data[1]) < 1.0 and not math.isnan(enc_data[0])
            
            if has_raw and has_est:
                turns = round(enc_data[0] - raw_data[0])
                current_pos = turns + raw_data[0]
            elif has_est:
                current_pos = enc_data[0]
            elif has_raw:
                current_pos = raw_data[0]
            else:
                current_pos = self.absolute_positions[i]
                
            turns = round(current_pos - ext_offsets[i])
            target_pos = turns + ext_offsets[i]
            
            deltas[i] = target_pos - current_pos

        self.send_incremental_array(deltas)

    def move_preset_up(self):
        print("--- Executing UP Preset ---")
        up_deltas = [0.02, -0.02, 0.04, 0.2, 0.5, 0.0]
        self.send_incremental_array(up_deltas)

    def move_preset_down(self):
        print("--- Executing DOWN Preset ---")
        down_deltas = [-0.02, 0.02, -0.04, -0.2, -0.5, 0.0]
        self.send_incremental_array(down_deltas)

    def send_incremental_array(self, deltas):
        current_time = time.time()
        
        for i, node_id in enumerate(NODE_IDS):
            delta = deltas[i]
            if delta == 0.0:
                continue  
                
            enc_data = self.encoder_data[node_id]
            raw_data = self.raw_encoder_data[node_id]
            
            has_raw = raw_data is not None and (current_time - raw_data[1]) < 1.0 and not math.isnan(raw_data[0])
            has_est = enc_data is not None and (current_time - enc_data[1]) < 1.0 and not math.isnan(enc_data[0])
            
            if has_raw and has_est:
                turns = round(enc_data[0] - raw_data[0])
                current_pos = turns + raw_data[0]
            elif has_est:
                current_pos = enc_data[0]
            elif has_raw:
                current_pos = raw_data[0]
            else:
                current_pos = self.absolute_positions[i]

            new_pos = current_pos + delta
            
            new_pos = max(-1.0, min(1.0, new_pos))
            self.absolute_positions[i] = new_pos

            if self.bus:
                try:
                    arbitration_id = (node_id << 5) | MSG_ID_SET_INPUT_POS
                    data = struct.pack('<fhh', new_pos, 0, 0)
                    msg = can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=False
                    )
                    self.bus.send(msg)
                    print(f"-> J{i + 1} (Node {node_id}) Step {delta:+.3f} -> Abs Target: {new_pos:.3f}")
                except can.CanError as e:
                    print(f"CAN Send Error on Node {node_id}: {e}")
            else:
                print(f"[MOCK MODE] J{i + 1} (Node {node_id}) step {delta:+.3f} -> Target: {new_pos:.3f}")
                
        self.update_arm_plot()

    # ==================== VISUALIZER LOGIC ====================

    def initialize_view(self):
        self.ax.set_facecolor('#101010')
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.pane.fill = False
            axis.pane.set_edgecolor('gray')
            axis.set_tick_params(colors='gray')
            axis.label.set_color('white')

        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        scale = 1300
        self.ax.set_xlim([-scale, scale])
        self.ax.set_ylim([-scale, scale])
        self.ax.set_zlim([0, scale * 1.3])

        self.ax.view_init(elev=30, azim=135)

    def rot_x(self, theta):
        return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

    def rot_y(self, theta):
        return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

    def rot_z(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    def calculate_fk(self, positions):
        angles = [p * 2 * np.pi for p in positions]

        L_BASE_H = 127
        L_UPPER_ARM = 612
        L_FOREARM = 572
        L_WRIST_1_O = 163  
        L_WRIST_2_O = 115  
        L_WRIST_3_H = 92  

        points = [np.array([0, 0, 0])]  

        T_accum = np.eye(4)  

        T_accum = T_accum @ self.trans_matrix(0, 0, L_BASE_H) @ self.rot_z_4x4(angles[0])
        points.append(T_accum[:3, 3])

        T_accum = T_accum @ self.rot_x_4x4(np.pi / 2) @ self.rot_z_4x4(angles[1]) @ self.trans_matrix(0, -L_UPPER_ARM, 0)
        points.append(T_accum[:3, 3])

        T_accum = T_accum @ self.rot_z_4x4(angles[2]) @ self.trans_matrix(0, -L_FOREARM, 0)
        points.append(T_accum[:3, 3])

        T_accum = T_accum @ self.trans_matrix(0, 0, L_WRIST_1_O) @ self.rot_z_4x4(angles[3])
        points.append(T_accum[:3, 3])

        T_accum = T_accum @ self.trans_matrix(0, 0, L_WRIST_2_O) @ self.rot_z_4x4(-np.pi / 2) @ self.rot_z_4x4(angles[4])
        points.append(T_accum[:3, 3])

        T_accum = T_accum @ self.trans_matrix(0, 0, L_WRIST_3_H) @ self.rot_z_4x4(angles[5])
        points.append(T_accum[:3, 3])

        return np.array(points)

    def trans_matrix(self, x, y, z):
        M = np.eye(4)
        M[:3, 3] = [x, y, z]
        return M

    def rot_x_4x4(self, theta):
        M = np.eye(4)
        M[:3, :3] = self.rot_x(theta)
        return M

    def rot_y_4x4(self, theta):
        M = np.eye(4)
        M[:3, :3] = self.rot_y(theta)
        return M

    def rot_z_4x4(self, theta):
        M = np.eye(4)
        M[:3, :3] = self.rot_z(theta)
        return M

    def update_arm_plot(self):
        positions = self.absolute_positions
        points = self.calculate_fk(positions)

        for artist in self.ax.lines + self.ax.collections:
            artist.remove()

        table_size = 800
        x = np.linspace(-table_size, table_size, 2)
        y = np.linspace(-table_size, table_size, 2)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)  
        self.ax.plot_surface(X, Y, Z, color='#2c3e50', alpha=0.6, shade=True)

        thickness = 14
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]

            color = self.link_colors[i] if i < len(self.link_colors) else 'white'
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                         color=color, linewidth=thickness, solid_capstyle='round')

            self.ax.scatter(p1[0], p1[1], p1[2], color='black', s=120, edgecolors='white', zorder=10)

            if thickness > 6: thickness -= 1.5

        self.canvas.draw_idle()  

    def destroy(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
        super().destroy()


if __name__ == "__main__":
    app = RobotArmControlPanel()
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    app.mainloop()