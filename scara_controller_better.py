import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import re
import os
import queue
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import sys
import logging
from matplotlib.patches import Rectangle, Circle

# --- Cấu hình Logging ---
script_dir = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(script_dir, 'scara_controller.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(log_file_path, mode='a', encoding='utf-8'), # UTF-8 for file
        logging.StreamHandler(sys.stdout) # Console encoding
    ]
)
logging.info(f"Log file path: {log_file_path}")


class ScaraRobot:
    def __init__(self):
        self.robot_params = {
            'x1': 0.0, 'y1': 0.0, 'x5': -20.0, 'y5': 0.0,
            'L1': 15.0, 'L2': 25.0, 'L3': 25.0, 'L4': 15.0,
        }
        self.angle_limits = {
            'theta1_min': -120.0, 'theta1_max': 150.0,
            'theta2_min': 30.0, 'theta2_max': 300.0
        }
        self.home_angles = {'theta1': 90.0, 'theta2': 90.0}
        self.calculate_workspace()
        logging.info("Khởi tạo ScaraRobot với thông số: %s", self.robot_params)
        logging.info("Giới hạn góc: %s", self.angle_limits)

    def calculate_workspace(self):
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1, L2 = self.robot_params['L1'], self.robot_params['L2']
        L3, L4 = self.robot_params['L3'], self.robot_params['L4']
        self.r1_min = max(0, abs(L1 - L2)) + 0.5
        self.r1_max = (L1 + L2) - 0.5
        self.r2_min = max(0, abs(L4 - L3)) + 0.5
        self.r2_max = (L4 + L3) - 0.5
        self.workspace_bounds = {
            'left': min(x1 - self.r1_max, x5 - self.r2_max),
            'right': max(x1 + self.r1_max, x5 + self.r2_max),
            'bottom': min(y1 - self.r1_max, y5 - self.r2_max), # Keep bottom for bounds calculation
            'top': max(y1 + self.r1_max, y5 + self.r2_max)
        }
        logging.info("Tính toán vùng làm việc: R1=[%.2f, %.2f], R2=[%.2f, %.2f], Bounds=%s",
                     self.r1_min, self.r1_max, self.r2_min, self.r2_max, self.workspace_bounds)

    def forward_kinematics(self, theta1_deg, theta2_deg):
        theta1 = np.radians(theta1_deg); theta2 = np.radians(theta2_deg)
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1, L2 = self.robot_params['L1'], self.robot_params['L2']
        L3, L4 = self.robot_params['L3'], self.robot_params['L4']
        x2 = x1 + L1 * np.cos(theta1); y2 = y1 + L1 * np.sin(theta1)
        x4 = x5 + L4 * np.cos(theta2); y4 = y5 + L4 * np.sin(theta2)
        dx = x4 - x2; dy = y4 - y2; d_sq = dx*dx + dy*dy
        if d_sq < 1e-12: return None, None, None
        d = np.sqrt(d_sq)
        epsilon = 1e-6
        if d < epsilon or d > L2 + L3 + epsilon or d < abs(L2 - L3) - epsilon: return None, None, None
        try:
            div = 2.0 * L2 * d
            if abs(div) < 1e-9: return None, None, None # Avoid division by zero
            cos_angle_at_j1 = np.clip((L2**2 + d_sq - L3**2) / div, -1.0, 1.0)
            angle_at_j1 = np.arccos(cos_angle_at_j1)
            angle_j1_to_j2 = np.arctan2(dy, dx)
            angle_L2 = angle_j1_to_j2 - angle_at_j1 # Elbow down assumed
            x3 = x2 + L2 * np.cos(angle_L2); y3 = y2 + L2 * np.sin(angle_L2)
            return (x2, y2), (x3, y3), (x4, y4)
        except (ValueError, ZeroDivisionError) as e:
             logging.error(f"Lỗi tính toán FK: {e}"); return None, None, None

    def inverse_kinematics(self, x3, y3, current_config="elbow_down_down"):
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1, L2 = self.robot_params['L1'], self.robot_params['L2']
        L3, L4 = self.robot_params['L3'], self.robot_params['L4']
        th1_min, th1_max = self.angle_limits['theta1_min'], self.angle_limits['theta1_max']
        th2_min, th2_max = self.angle_limits['theta2_min'], self.angle_limits['theta2_max']
        epsilon = 1e-6; solutions = []
        try:
            dx13, dy13 = x3 - x1, y3 - y1; L13_sq = dx13**2 + dy13**2
            if L13_sq < epsilon**2: L13 = epsilon
            else: L13 = np.sqrt(L13_sq)
            if L13 > L1 + L2 + epsilon or L13 < abs(L1 - L2) - epsilon: return None, None, None
            beta1 = np.arctan2(dy13, dx13); denominator1 = 2 * L1 * L13
            if denominator1 < epsilon: return None, None, None
            cos_alpha1 = np.clip((L1**2 + L13_sq - L2**2) / denominator1, -1.0, 1.0)
            alpha1 = np.arccos(cos_alpha1)
            theta1_sol1 = np.degrees(beta1 - alpha1); theta1_sol2 = np.degrees(beta1 + alpha1)

            dx53, dy53 = x3 - x5, y3 - y5; L53_sq = dx53**2 + dy53**2
            if L53_sq < epsilon**2: L53 = epsilon
            else: L53 = np.sqrt(L53_sq)
            if L53 > L4 + L3 + epsilon or L53 < abs(L4 - L3) - epsilon: return None, None, None
            beta5 = np.arctan2(dy53, dx53); denominator5 = 2 * L4 * L53
            if denominator5 < epsilon: return None, None, None
            cos_alpha5 = np.clip((L4**2 + L53_sq - L3**2) / denominator5, -1.0, 1.0)
            alpha5 = np.arccos(cos_alpha5)
            theta2_sol1 = np.degrees(beta5 + alpha5); theta2_sol2 = np.degrees(beta5 - alpha5)

            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol1 <= th2_max: solutions.append(("elbow_up_up", theta1_sol1, theta2_sol1))
            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol2 <= th2_max: solutions.append(("elbow_up_down", theta1_sol1, theta2_sol2))
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol1 <= th2_max: solutions.append(("elbow_down_up", theta1_sol2, theta2_sol1))
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol2 <= th2_max: solutions.append(("elbow_down_down", theta1_sol2, theta2_sol2))

            if not solutions: return None, None, None
            preferred = [s for s in solutions if s[0] == current_config]
            if preferred: return preferred[0][1], preferred[0][2], preferred[0][0]
            down_down = [s for s in solutions if s[0] == "elbow_down_down"]
            if down_down: return down_down[0][1], down_down[0][2], down_down[0][0]
            return solutions[0][1], solutions[0][2], solutions[0][0] # Return first available
        except (ValueError, ZeroDivisionError) as e:
            logging.error(f"Lỗi tính toán IK: {e}"); return None, None, None

    def check_point_in_workspace(self, x, y):
        if y <= 0: return False # Ensure Y is positive
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        epsilon = 1e-6
        d1_sq = (x - x1)**2 + (y - y1)**2; d2_sq = (x - x5)**2 + (y - y5)**2
        r1_min_sq = self.r1_min**2; r1_max_sq = self.r1_max**2
        r2_min_sq = self.r2_min**2; r2_max_sq = self.r2_max**2
        if not (r1_min_sq - epsilon <= d1_sq <= r1_max_sq + epsilon): return False
        if not (r2_min_sq - epsilon <= d2_sq <= r2_max_sq + epsilon): return False
        theta1, theta2, _ = self.inverse_kinematics(x, y) # Check if IK solvable within limits
        if theta1 is None or theta2 is None: return False
        return True

    def create_workspace_visualization_points(self, resolution=0.5):
        bounds = self.workspace_bounds
        y_start = max(bounds['bottom'], resolution) # Start Y slightly above 0
        x_coords = np.arange(bounds['left'], bounds['right'] + resolution, resolution)
        y_coords = np.arange(y_start, bounds['top'] + resolution, resolution)
        grid_x, grid_y = np.meshgrid(x_coords, y_coords)
        test_points = np.vstack([grid_x.ravel(), grid_y.ravel()]).T
        logging.info(f"Tạo visualization workspace ({len(test_points)} điểm)...")
        start_time = time.time()
        valid_points = [p for p in test_points if self.check_point_in_workspace(p[0], p[1])]
        elapsed = time.time() - start_time
        logging.info(f"Hoàn thành tạo workspace sau {elapsed:.2f}s, {len(valid_points)} điểm hợp lệ.")
        return valid_points

class ScaraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Controller - Flipped X")
        try: self.root.state('zoomed')
        except tk.TclError: self.root.geometry("1200x800")

        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 90.0, 'theta2': 90.0}

        fk_result = self.robot.forward_kinematics(90.0, 90.0)
        fk_x_internal, fk_y = fk_result[1] if fk_result and fk_result[1] else (10.0, 30.0) # Safe fallback
        self.current_position = {'x': fk_x_internal, 'y': fk_y}
        self.pen_is_down = False
        self.current_config = "elbow_down_down"

        x_display = -self.current_position['x']
        logging.info(f"Vị trí XY nội bộ ban đầu: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
        logging.info(f"Vị trí XY hiển thị ban đầu: ({x_display:.2f}, {self.current_position['y']:.2f})")

        self.trace_segments = []; self.current_segment = {}
        self.workspace_points = []; self._workspace_visualization_active = False
        self.gcode_lines = []; self.gcode_running = False; self.gcode_paused = False
        self.gcode_line_num = 0; self.gcode_total_lines = 0; self.stop_gcode_flag = False

        self.command_queue = queue.Queue()
        self.command_processing = False
        self.serial_lock = threading.Lock()
        self.last_response_time = time.time()
        self.command_timeout = 10.0
        self.move_timeout_factor = 2.5 # Factor for move commands

        self.create_widgets()
        self.update_ports()

        self.process_command_thread = threading.Thread(target=self._process_command_queue, daemon=True)
        self.process_command_thread.start()

        self.root.after(100, self.update_animation)
        logging.info("Khởi tạo ScaraGUI hoàn tất.")

    def _process_command_queue(self):
        while True:
            try:
                command_item = self.command_queue.get(block=True, timeout=None)
                if command_item is None: break
                command, wait_response, is_move_cmd = command_item
                self.command_processing = True
                success = self._execute_command(command, wait_response, is_move_cmd)
                self.command_processing = False
                self.command_queue.task_done()
                logging.debug(f"Xử lý xong lệnh '{command}', Success={success}")
                time.sleep(0.01)
            except queue.Empty: continue
            except Exception as e:
                logging.exception("Lỗi nghiêm trọng trong _process_command_queue:")
                self.command_processing = False; time.sleep(0.1)

    def create_widgets(self):
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        control_panel = ttk.Frame(main_frame, padding=10); main_frame.add(control_panel, weight=35)
        display_panel = ttk.Frame(main_frame, padding=10); main_frame.add(display_panel, weight=65)
        control_panel.columnconfigure(0, weight=1)

        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(control_panel, text="Kết nối", padding=10)
        conn_frame.grid(row=0, column=0, sticky=tk.EW, pady=(0, 10)); conn_frame.columnconfigure(1, weight=1)
        ttk.Label(conn_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar(); self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.EW)
        ttk.Button(conn_frame, text="⟳", width=3, command=self.update_ports).grid(row=0, column=2, padx=(0, 5), pady=5)
        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn_frame, textvariable=self.baud_var, width=15, state="readonly", values=["9600", "19200", "38400", "57600", "115200", "250000"]).grid(row=1, column=1, padx=5, pady=5, sticky=tk.EW)
        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(0, 5), pady=5)

        # --- Manual Control Frame ---
        manual_frame = ttk.LabelFrame(control_panel, text="Điều khiển thủ công", padding=10)
        manual_frame.grid(row=1, column=0, sticky=tk.EW, pady=(0, 10)); manual_frame.columnconfigure(0, weight=1)
        pen_frame = ttk.Frame(manual_frame); pen_frame.grid(row=0, column=0, columnspan=2, sticky=tk.EW, pady=5); pen_frame.columnconfigure(0, weight=1); pen_frame.columnconfigure(1, weight=1)
        self.pen_up_btn = ttk.Button(pen_frame, text="Nâng bút (U)", command=self.pen_up); self.pen_up_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.root.bind('<u>', lambda e=None: self.pen_up()); self.root.bind('<U>', lambda e=None: self.pen_up())
        self.pen_down_btn = ttk.Button(pen_frame, text="Hạ bút (D)", command=self.pen_down); self.pen_down_btn.grid(row=0, column=1, padx=(5, 0), sticky=tk.EW)
        self.root.bind('<d>', lambda e=None: self.pen_down()); self.root.bind('<D>', lambda e=None: self.pen_down())
        xy_frame = ttk.Frame(manual_frame); xy_frame.grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=5); xy_frame.columnconfigure(1, weight=1); xy_frame.columnconfigure(3, weight=1); xy_frame.columnconfigure(4, weight=2)
        ttk.Label(xy_frame, text="X:").grid(row=0, column=0, padx=(0, 2)); self.x_entry = ttk.Entry(xy_frame, width=7); self.x_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        x_display_init = -self.current_position.get('x', 0.0); self.x_entry.insert(0, f"{x_display_init:.2f}")
        ttk.Label(xy_frame, text="Y:").grid(row=0, column=2, padx=(5, 2)); self.y_entry = ttk.Entry(xy_frame, width=7); self.y_entry.grid(row=0, column=3, padx=(0, 5), sticky=tk.EW)
        y_init = self.current_position.get('y', 0.0); self.y_entry.insert(0, f"{y_init:.2f}")
        self.go_xy_btn = ttk.Button(xy_frame, text="Di chuyển XY", command=self.move_to_xy); self.go_xy_btn.grid(row=0, column=4, sticky=tk.EW)
        angle_frame = ttk.Frame(manual_frame); angle_frame.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=5); angle_frame.columnconfigure(1, weight=1); angle_frame.columnconfigure(3, weight=1); angle_frame.columnconfigure(4, weight=2)
        ttk.Label(angle_frame, text="θ1:").grid(row=0, column=0, padx=(0, 2)); self.angle1_entry = ttk.Entry(angle_frame, width=7); self.angle1_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        self.angle1_entry.insert(0, f"{self.current_angles['theta1']:.2f}")
        ttk.Label(angle_frame, text="θ2:").grid(row=0, column=2, padx=(5, 2)); self.angle2_entry = ttk.Entry(angle_frame, width=7); self.angle2_entry.grid(row=0, column=3, padx=(0, 5), sticky=tk.EW)
        self.angle2_entry.insert(0, f"{self.current_angles['theta2']:.2f}")
        self.go_angle_btn = ttk.Button(angle_frame, text="Di chuyển Góc", command=self.move_to_angle); self.go_angle_btn.grid(row=0, column=4, sticky=tk.EW)
        speed_accel_frame = ttk.Frame(manual_frame); speed_accel_frame.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=5); speed_accel_frame.columnconfigure(1, weight=1); speed_accel_frame.columnconfigure(4, weight=1)
        ttk.Label(speed_accel_frame, text="Tốc độ:").grid(row=0, column=0, padx=(0, 2), sticky=tk.W); self.speed_var = tk.StringVar(value="8000")
        ttk.Entry(speed_accel_frame, textvariable=self.speed_var, width=6).grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        self.set_speed_button = ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_speed); self.set_speed_button.grid(row=0, column=2, padx=(0, 10))
        ttk.Label(speed_accel_frame, text="Gia tốc:").grid(row=0, column=3, padx=(10, 2), sticky=tk.W); self.accel_var = tk.StringVar(value="3000")
        ttk.Entry(speed_accel_frame, textvariable=self.accel_var, width=6).grid(row=0, column=4, padx=(0, 5), sticky=tk.EW)
        self.set_accel_button = ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_acceleration); self.set_accel_button.grid(row=0, column=5, padx=(0, 0))
        func_frame = ttk.Frame(manual_frame); func_frame.grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=5); func_frame.columnconfigure(0, weight=1); func_frame.columnconfigure(1, weight=1); func_frame.columnconfigure(2, weight=1)
        self.home_btn = ttk.Button(func_frame, text="HOME (H)", command=self.home); self.home_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.root.bind('<h>', lambda e=None: self.home()); self.root.bind('<H>', lambda e=None: self.home())
        self.calibrate_btn = ttk.Button(func_frame, text="Calib 90,90", command=self.calibrate_position); self.calibrate_btn.grid(row=0, column=1, padx=5, sticky=tk.EW)
        ttk.Button(func_frame, text="DỪNG KHẨN CẤP", command=self.emergency_stop, style="Emergency.TButton").grid(row=0, column=2, padx=(5, 0), sticky=tk.EW)

        # --- G-Code Frame ---
        gcode_frame = ttk.LabelFrame(control_panel, text="Điều khiển G-Code", padding=10)
        gcode_frame.grid(row=2, column=0, sticky=tk.NSEW, pady=(0, 10)); gcode_frame.columnconfigure(0, weight=1)
        load_frame = ttk.Frame(gcode_frame); load_frame.grid(row=0, column=0, sticky=tk.EW, pady=(0, 5)); load_frame.columnconfigure(1, weight=1)
        ttk.Button(load_frame, text="Tải G-Code", command=self.load_gcode).grid(row=0, column=0, padx=(0, 10))
        self.file_label = ttk.Label(load_frame, text="Chưa tải file", anchor=tk.W, relief="sunken", padding=(2, 1)); self.file_label.grid(row=0, column=1, sticky=tk.EW)
        code_view_frame = ttk.Frame(gcode_frame); code_view_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(0, 5)); code_view_frame.rowconfigure(0, weight=1); code_view_frame.columnconfigure(0, weight=1)
        self.gcode_text = scrolledtext.ScrolledText(code_view_frame, height=8, wrap=tk.NONE, borderwidth=1, relief="sunken", font=("Courier New", 9)); self.gcode_text.grid(row=0, column=0, sticky=tk.NSEW)
        gcode_ctrl_frame = ttk.Frame(gcode_frame); gcode_ctrl_frame.grid(row=3, column=0, sticky=tk.EW, pady=(5, 5)); gcode_ctrl_frame.columnconfigure(0, weight=1); gcode_ctrl_frame.columnconfigure(1, weight=1); gcode_ctrl_frame.columnconfigure(2, weight=1)
        self.run_btn = ttk.Button(gcode_ctrl_frame, text="Chạy", command=self.run_gcode, state=tk.DISABLED); self.run_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.pause_btn = ttk.Button(gcode_ctrl_frame, text="Tạm dừng", command=self.pause_gcode, state=tk.DISABLED); self.pause_btn.grid(row=0, column=1, padx=5, sticky=tk.EW)
        self.stop_gcode_btn = ttk.Button(gcode_ctrl_frame, text="Dừng", command=self.stop_gcode, state=tk.DISABLED); self.stop_gcode_btn.grid(row=0, column=2, padx=(5, 0), sticky=tk.EW)
        self.progress_var = tk.DoubleVar(); ttk.Progressbar(gcode_frame, variable=self.progress_var, maximum=100, mode='determinate').grid(row=4, column=0, sticky=tk.EW, pady=(0, 2))
        self.progress_label = ttk.Label(gcode_frame, text="0.0% hoàn thành"); self.progress_label.grid(row=5, column=0, sticky=tk.EW)
        delay_frame = ttk.Frame(gcode_frame); delay_frame.grid(row=6, column=0, sticky=tk.EW, pady=(5, 0)); ttk.Label(delay_frame, text="Chờ giữa lệnh (s):").pack(side=tk.LEFT, padx=(0, 5))
        self.gcode_delay_var = tk.DoubleVar(value=0.01); ttk.Spinbox(delay_frame, from_=0.00, to=2.0, increment=0.01, textvariable=self.gcode_delay_var, width=5).pack(side=tk.LEFT)

        # --- Log Frame ---
        log_frame = ttk.LabelFrame(control_panel, text="Nhật ký", padding=10)
        log_frame.grid(row=3, column=0, sticky=tk.NSEW, pady=(0, 0)); log_frame.rowconfigure(0, weight=1); log_frame.columnconfigure(0, weight=1); control_panel.rowconfigure(3, weight=1)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=5, wrap=tk.WORD, borderwidth=1, relief="sunken", font=("Segoe UI", 9), state=tk.DISABLED); self.log_text.grid(row=0, column=0, sticky=tk.NSEW)
        self.log_text.tag_configure("INFO", foreground="black"); self.log_text.tag_configure("WARNING", foreground="orange"); self.log_text.tag_configure("ERROR", foreground="red", font=("Segoe UI", 9, "bold"))
        self.log_text.tag_configure("DEBUG", foreground="gray"); self.log_text.tag_configure("SENT", foreground="blue"); self.log_text.tag_configure("RECEIVED", foreground="green"); self.log_text.tag_configure("SUCCESS", foreground="dark green")

        # --- Display Panel ---
        display_panel.rowconfigure(0, weight=1); display_panel.columnconfigure(0, weight=1)
        canvas_ui_frame = ttk.Frame(display_panel); canvas_ui_frame.grid(row=0, column=0, sticky=tk.NSEW); canvas_ui_frame.rowconfigure(0, weight=1); canvas_ui_frame.columnconfigure(0, weight=1)
        self.setup_plot(canvas_ui_frame); self.canvas_widget.grid(row=0, column=0, sticky=tk.NSEW) # Grid canvas widget
        vis_ctrl_frame = ttk.Frame(canvas_ui_frame); vis_ctrl_frame.grid(row=1, column=0, sticky=tk.EW, pady=(5, 0))
        self.workspace_btn = ttk.Button(vis_ctrl_frame, text="Hiện/Ẩn Vùng làm việc", command=self.toggle_workspace_visualization); self.workspace_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(vis_ctrl_frame, text="Xóa Hình Vẽ", command=self.clear_drawing).pack(side=tk.LEFT, padx=5)
        status_frame = ttk.LabelFrame(display_panel, text="Trạng thái Robot", padding=10); status_frame.grid(row=1, column=0, sticky=tk.EW, pady=(10, 0)); status_frame.columnconfigure(1, weight=1); status_frame.columnconfigure(3, weight=1)
        ttk.Label(status_frame, text="Vị trí (X,Y):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2); self.pos_label = ttk.Label(status_frame, text="X=???, Y=???", width=20, font=("Segoe UI", 10, "bold")); self.pos_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        ttk.Label(status_frame, text="Góc (θ1,θ2):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2); self.angle_label = ttk.Label(status_frame, text="θ1=???, θ2=???", width=20, font=("Segoe UI", 10, "bold")); self.angle_label.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        ttk.Label(status_frame, text="Bút:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2); self.pen_label = ttk.Label(status_frame, text="Nâng", width=10, font=("Segoe UI", 10)); self.pen_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        ttk.Label(status_frame, text="Cấu hình IK:").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2); self.config_label = ttk.Label(status_frame, text=self.current_config, width=15, font=("Segoe UI", 10)); self.config_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)
        style = ttk.Style(); style.configure("Emergency.TButton", foreground="white", background="red", font=("Segoe UI", 10, "bold"))
        self.update_position_display()

    def setup_plot(self, master_frame):
        if hasattr(self, 'canvas_widget'): self.canvas_widget.destroy()
        if hasattr(self, 'fig'): plt.close(self.fig)
        self.fig = plt.figure(figsize=(7, 7)); self.ax = self.fig.add_subplot(111); self.ax.clear()
        bounds = self.robot.workspace_bounds; padding = 5
        x_min_internal = bounds['left'] - padding; x_max_internal = bounds['right'] + padding; x_range_internal = x_max_internal - x_min_internal
        y_min_visible = 0 - padding; y_max_visible = bounds['top'] + padding; y_range_visible = y_max_visible - y_min_visible
        max_range = max(x_range_internal, y_range_visible); x_center_internal = (x_min_internal + x_max_internal) / 2; y_center_visible = (0 + bounds['top']) / 2
        new_x_min_internal = x_center_internal - max_range / 2; new_x_max_internal = x_center_internal + max_range / 2
        new_y_min = y_center_visible - max_range / 2; new_y_max = y_center_visible + max_range / 2
        self.ax.set_xlim(-(new_x_max_internal), -(new_x_min_internal)); self.ax.set_ylim(new_y_min, new_y_max)
        self.ax.set_xlabel("X (cm) - Flipped Display"); self.ax.set_ylabel("Y (cm)"); self.ax.set_title("SCARA Robot Visualization")
        self.ax.grid(True, linestyle='--', alpha=0.6); self.ax.set_aspect('equal', adjustable='box')
        x1_vis = -self.robot.robot_params['x1']; y1 = self.robot.robot_params['y1']; x5_vis = -self.robot.robot_params['x5']; y5 = self.robot.robot_params['y5']
        self.ax.plot(x1_vis, y1, 'ks', markersize=8, label='Motor 1 (Y)'); self.ax.plot(x5_vis, y5, 'ks', markersize=8, label='Motor 2 (X)')
        self.ax.text(x1_vis + 1, y1 + 1, "M1(Y)", fontsize=8, color='black'); self.ax.text(x5_vis + 1, y5 + 1, "M2(X)", fontsize=8, color='black')
        self.arm1_line, = self.ax.plot([], [], 'b-', lw=5, alpha=0.7, label='Arm 1-2', solid_capstyle='round')
        self.arm2_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, label='Arm 2-3', solid_capstyle='round')
        self.arm3_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, solid_capstyle='round')
        self.arm4_line, = self.ax.plot([], [], 'c-', lw=5, alpha=0.7, label='Arm 4-5', solid_capstyle='round')
        self.joints, = self.ax.plot([], [], 'o', color='darkred', markersize=8, label='Joints')
        self.effector, = self.ax.plot([], [], 'o', color='magenta', markersize=10, label='Effector')
        self.trace_line, = self.ax.plot([], [], '-', color='red', lw=1.5, label='Drawing Trace')
        text_x_pos_vis = -(new_x_max_internal) + max_range * 0.02; text_y_pos_vis = new_y_max - max_range * 0.02
        self.angle_text = self.ax.text(text_x_pos_vis, text_y_pos_vis, "", fontsize=9, ha='left', va='top', bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none'))
        self.pen_status_text = self.ax.text(text_x_pos_vis, text_y_pos_vis - 3.0, "", fontsize=9, ha='left', va='top', bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none'))
        self.fig.tight_layout(); self.canvas = FigureCanvasTkAgg(self.fig, master=master_frame); self.canvas_widget = self.canvas.get_tk_widget(); self.canvas.draw()
        self._workspace_visualization_active = False
        if hasattr(self, 'workspace_scatter'):
            try: self.workspace_scatter.remove()
            except ValueError: pass
            del self.workspace_scatter

    def toggle_workspace_visualization(self):
        if self._workspace_visualization_active:
            if hasattr(self, 'workspace_scatter'):
                try: self.workspace_scatter.remove()
                except ValueError: pass
                del self.workspace_scatter; self._workspace_visualization_active = False
                self.log("Đã ẩn visualization vùng làm việc."); self.canvas.draw_idle(); logging.info("Đã ẩn workspace visualization.")
            else: self._workspace_visualization_active = False; logging.warning("Trạng thái workspace không nhất quán, đã reset.")
        else:
            if not self.workspace_points:
                self.log("Đang tính toán các điểm vùng làm việc..."); logging.info("Bắt đầu tạo điểm workspace visualization...")
                self.workspace_btn.config(state=tk.DISABLED); threading.Thread(target=self._generate_and_draw_workspace, daemon=True).start()
            else: self._draw_workspace_points()

    def _generate_and_draw_workspace(self):
        generated_points = []
        try: generated_points = self.robot.create_workspace_visualization_points(resolution=0.5)
        except Exception as e: logging.exception("Lỗi tạo visualization workspace:"); self.log(f"Lỗi tạo vùng làm việc: {e}", tag="ERROR")
        self.workspace_points = generated_points
        def _task():
            self.workspace_btn.config(state=tk.NORMAL)
            if self.workspace_points: self._draw_workspace_points()
            else: self.log("Không tạo được điểm nào cho vùng làm việc.", tag="WARNING")
        if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists(): self.root.after(0, _task)

    def _draw_workspace_points(self):
        if not self.workspace_points: logging.warning("Không có điểm workspace để vẽ."); return
        if hasattr(self, 'workspace_scatter'):
             try: self.workspace_scatter.remove()
             except ValueError: pass
             del self.workspace_scatter
        try:
            internal_x_coords, y_coords = zip(*self.workspace_points); vis_x_coords = [-x for x in internal_x_coords]
            self.workspace_scatter = self.ax.scatter(vis_x_coords, y_coords, s=1, c='lightblue', alpha=0.3, label='Workspace', zorder=0)
            self._workspace_visualization_active = True; self.log(f"Đã hiển thị {len(self.workspace_points)} điểm vùng làm việc.")
            self.canvas.draw_idle(); logging.info("Đã vẽ workspace visualization.")
        except Exception as e: logging.exception("Lỗi vẽ điểm workspace:"); self.log(f"Lỗi vẽ vùng làm việc: {e}", tag="ERROR")

    def update_animation(self):
        theta1_deg = self.current_angles.get('theta1', 90.0); theta2_deg = self.current_angles.get('theta2', 90.0)
        try:
            fk_result = self.robot.forward_kinematics(theta1_deg, theta2_deg)
            if fk_result is None: return
            joint1_pos, effector_pos_internal, joint3_pos = fk_result
            if joint1_pos is None or effector_pos_internal is None or joint3_pos is None: return
            x1_vis, y1 = -self.robot.robot_params['x1'], self.robot.robot_params['y1']; x5_vis, y5 = -self.robot.robot_params['x5'], self.robot.robot_params['y5']
            x2_vis, y2 = -joint1_pos[0], joint1_pos[1]; x3_vis, y3 = -effector_pos_internal[0], effector_pos_internal[1]; x4_vis, y4 = -joint3_pos[0], joint3_pos[1]
            self.arm1_line.set_data([x1_vis, x2_vis], [y1, y2]); self.arm2_line.set_data([x2_vis, x3_vis], [y2, y3])
            self.arm3_line.set_data([x3_vis, x4_vis], [y3, y4]); self.arm4_line.set_data([x4_vis, x5_vis], [y4, y5])
            self.joints.set_data([x2_vis, x4_vis], [y2, y4]); self.effector.set_data([x3_vis], [y3])
            if self.pen_is_down:
                if not self.current_segment or not self.current_segment.get('x'): self.current_segment = {'x': [x3_vis], 'y': [y3]}
                else:
                    dx = x3_vis - self.current_segment['x'][-1]; dy = y3 - self.current_segment['y'][-1]
                    if dx*dx + dy*dy > 1e-6: self.current_segment['x'].append(x3_vis); self.current_segment['y'].append(y3)
            all_trace_x, all_trace_y = [], []
            for segment in self.trace_segments:
                 if segment and segment.get('x'): all_trace_x.extend(segment['x']); all_trace_y.extend(segment['y']); all_trace_x.append(None); all_trace_y.append(None)
            if self.current_segment and self.current_segment.get('x'): all_trace_x.extend(self.current_segment['x']); all_trace_y.extend(self.current_segment['y'])
            self.trace_line.set_data(all_trace_x, all_trace_y)
            self.angle_text.set_text(f"θ1={theta1_deg:.2f}° | θ2={theta2_deg:.2f}°"); pen_text = f"Bút: {'Hạ' if self.pen_is_down else 'Nâng'}"
            self.pen_status_text.set_text(pen_text); self.pen_status_text.set_color('red' if self.pen_is_down else 'black')
            if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists(): self.canvas.draw_idle()
        except Exception as e: logging.exception("Lỗi trong update_animation:")

    def clear_drawing(self):
        if not self.trace_segments and not self.current_segment.get('x'): self.log("Không có hình vẽ nào để xóa."); return
        self.trace_segments = []; self.current_segment = {}; self.trace_line.set_data([], [])
        self.log("Đã xóa hình vẽ trên mô phỏng."); logging.info("Đã xóa hình vẽ."); self.canvas.draw_idle()

    def update_ports(self):
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                current_port = self.port_var.get()
                if current_port not in ports: self.port_var.set(ports[0])
            else: self.port_var.set("")
            logging.info(f"Cập nhật cổng COM: {ports}")
        except Exception as e: logging.error(f"Lỗi cập nhật cổng COM: {e}"); self.port_combo['values'] = []; self.port_var.set("")

    def toggle_connection(self):
        if self.is_connected: self.disconnect()
        else: self.connect()

    def connect(self):
        port = self.port_var.get(); baud = self.baud_var.get()
        if not port: messagebox.showerror("Lỗi Kết Nối", "Vui lòng chọn cổng COM!"); return
        if not baud.isdigit(): messagebox.showerror("Lỗi Kết Nối", "Baud rate không hợp lệ!"); return
        baud = int(baud)
        try:
            if self.serial and self.serial.is_open:
                try: self.serial.close(); time.sleep(0.5)
                except Exception as close_err: logging.warning(f"Lỗi đóng cổng cũ: {close_err}")
            self.log(f"Đang kết nối đến {port} @ {baud} baud..."); logging.info(f"Đang kết nối đến {port} @ {baud} baud...")
            self.serial = serial.Serial(port, baud, timeout=0.1, write_timeout=1.0)
            self.log("Chờ Arduino gửi tín hiệu sẵn sàng (tối đa 5s)..."); start_wait = time.time()
            initial_ready = False; initial_buffer = ""; max_wait = 5.0
            while time.time() - start_wait < max_wait:
                try:
                    bytes_in = 0
                    if self.serial.in_waiting > 0: bytes_in = self.serial.in_waiting
                    if bytes_in > 0:
                        data = self.serial.read(bytes_in).decode('utf-8', errors='ignore'); initial_buffer += data
                        while '\n' in initial_buffer:
                            line, initial_buffer = initial_buffer.split('\n', 1); line = line.strip()
                            if line:
                                logging.debug(f"Nhận phản hồi khởi tạo: '{line}'"); self.log(f"<- {line}")
                                line_upper = line.upper()
                                if "READY" in line_upper or "SCARA_READY" in line_upper or "OK" in line_upper:
                                    initial_ready = True; logging.info("Nhận READY/OK từ Arduino."); break
                    if initial_ready: break
                except serial.SerialException as ser_e: logging.warning(f"Lỗi đọc serial khi chờ READY: {ser_e}"); self.log(f"(!) Lỗi đọc Serial khi chờ: {ser_e}", tag="ERROR"); break
                except Exception as e: logging.exception("Lỗi chờ READY:"); break
                time.sleep(0.05)
            if initial_ready:
                self.is_connected = True; self.connect_btn.configure(text="Ngắt kết nối")
                self.log(f"✓ Kết nối thành công với {port}", tag="SUCCESS"); logging.info(f"Kết nối thành công đến {port}")
                self.serial.reset_input_buffer(); self.serial.reset_output_buffer()
                self.root.after(200, self.set_speed); self.root.after(400, self.set_acceleration); self.root.after(700, self.home)
            else:
                self.log(f"❌ Không nhận được READY/OK từ Arduino sau {max_wait}s.", tag="ERROR"); logging.warning("Kết nối thất bại, không nhận READY/OK.")
                self.disconnect(silent=True); messagebox.showwarning("Kết Nối Thất Bại", "Không nhận được phản hồi hợp lệ từ Arduino.\nKiểm tra firmware, baud rate.")
        except serial.SerialException as e:
            logging.exception("Lỗi Serial khi kết nối:"); self.log(f"❌ Lỗi Serial: {e}", tag="ERROR"); messagebox.showerror("Lỗi Kết Nối", f"Lỗi Serial:\n{e}\nKiểm tra cổng COM."); self.disconnect(silent=True)
        except Exception as e: logging.exception("Lỗi kết nối:"); self.log(f"❌ Lỗi không xác định: {e}", tag="ERROR"); messagebox.showerror("Lỗi Kết Nối", f"{e}"); self.disconnect(silent=True)

    def disconnect(self, silent=False):
        if not self.is_connected and not silent: logging.info("Đã ngắt kết nối."); return
        was_connected = self.is_connected; self.is_connected = False
        if self.gcode_running: self.stop_gcode(is_emergency=True)
        self._clear_queues()
        serial_closed = False
        if self.serial and self.serial.is_open:
            try:
                with self.serial_lock: self.serial.write(b"disable\n"); logging.info("Gửi disable trước khi đóng.")
                time.sleep(0.1); self.serial.close(); serial_closed = True; logging.info("Đã đóng cổng Serial.")
            except Exception as e: logging.error(f"Lỗi đóng cổng Serial: {e}");
            if not silent: self.log(f"Lỗi đóng cổng: {e}", tag="ERROR")
        self.serial = None; self.connect_btn.configure(text="Kết nối")
        if was_connected and not silent: self.log("Đã ngắt kết nối."); logging.info("Đã ngắt kết nối.")
        elif serial_closed and not silent: self.log("Đã đóng cổng Serial.")

    def _clear_queues(self):
        cleared = 0
        while not self.command_queue.empty():
            try: self.command_queue.get_nowait(); self.command_queue.task_done(); cleared += 1
            except queue.Empty: break
        if cleared > 0: logging.debug(f"Đã xóa {cleared} lệnh khỏi command queue.")

    def send_command(self, command, wait_for_response=True):
        if not self.is_connected: self.log("(!) Lỗi: Chưa kết nối!", tag="ERROR"); logging.warning("send_command: Chưa kết nối."); return False
        cmd_upper = command.strip().upper(); is_move = (',' in command) or cmd_upper == "H" or cmd_upper == "HOME" or cmd_upper.startswith(("G0", "G1"))
        self.command_queue.put((command, wait_for_response, is_move)); logging.debug(f"Queue: '{command}', Wait={wait_for_response}, Move={is_move}")
        return True

    def _execute_command(self, command, wait_for_response, is_move_cmd):
        """Thực thi lệnh và chờ MOVE_COMPLETE AT:X,Y + OK cho lệnh di chuyển."""
        start_time = time.time(); success = False
        try:
            self.log(f"-> {command}", tag="SENT"); cmd_bytes = (command.strip() + '\n').encode('utf-8')
            with self.serial_lock:
                if not self.serial or not self.serial.is_open: logging.error("Lỗi gửi: Serial không mở."); self.log("Lỗi gửi: Serial không mở.", tag="ERROR"); return False
                self.serial.write(cmd_bytes); logging.debug(f"Đã gửi: {cmd_bytes}")
            if wait_for_response:
                timeout = self.command_timeout * self.move_timeout_factor if is_move_cmd else self.command_timeout
                wait_msg = "MOVE_COMPLETE và OK" if is_move_cmd else "OK"
                logging.debug(f"Chờ '{wait_msg}' cho '{command}' (Timeout: {timeout:.1f}s)")
                responses = []; move_complete_parsed = False; ok_received = False; final_x, final_y = None, None
                while time.time() - start_time < timeout:
                    line = "";
                    try:
                        with self.serial_lock:
                            if self.serial and self.serial.is_open and self.serial.in_waiting > 0: line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                            elif not self.serial or not self.serial.is_open: raise serial.SerialException("Serial đóng khi chờ.")
                    except serial.SerialException as e: logging.error(f"Lỗi đọc Serial chờ '{command}': {e}"); self.log(f"(!) Lỗi đọc Serial: {e}", tag="ERROR"); return False
                    except Exception as e: logging.error(f"Lỗi đọc Serial khác cho '{command}': {e}"); self.log(f"(!) Lỗi đọc Serial (khác): {e}", tag="ERROR"); return False

                    if line:
                        self.log(f"<- {line}", tag="RECEIVED"); responses.append(line); self.last_response_time = time.time(); line_upper = line.upper()
                        # 1. Parse MOVE_COMPLETE AT:X,Y
                        if is_move_cmd and not move_complete_parsed and "MOVE_COMPLETE AT:" in line_upper:
                            match = re.search(r"MOVE_COMPLETE AT:(-?\d+\.?\d*),(-?\d+\.?\d*)", line, re.IGNORECASE)
                            if match:
                                try:
                                    final_x = float(match.group(1)); final_y = float(match.group(2))
                                    logging.info(f"Parse MOVE_COMPLETE XY=({final_x:.2f}, {final_y:.2f})")
                                    move_complete_parsed = True
                                    if ok_received: success = True; break # OK đã đến trước
                                except (ValueError, IndexError): logging.error(f"Lỗi parse XY từ MOVE_COMPLETE: '{line}'"); success = False; break
                            elif "FK_ERR" in line_upper: logging.warning(f"Arduino báo lỗi FK: '{line}'"); move_complete_parsed = True; # Vẫn coi là hoàn thành logic
                            if ok_received: success = True; break
                            else: logging.warning(f"MOVE_COMPLETE sai định dạng XY: '{line}'")
                        # 2. Check OK/READY/COMPLETE
                        elif not ok_received and ("OK" in line_upper or "READY" in line_upper or "COMPLETE" in line_upper):
                            ok_received = True
                            if (is_move_cmd and move_complete_parsed) or (not is_move_cmd): success = True; break
                        # 3. Other status updates
                        elif "PEN_UP" in line_upper: self.root.after(0, lambda: self._update_pen_status(False))
                        elif "PEN_DOWN" in line_upper: self.root.after(0, lambda: self._update_pen_status(True))
                        elif "CONFIG:" in line_upper: cfg = line.split(":")[-1].strip(); self.root.after(0, lambda c=cfg: self._update_ik_config(c))
                        # 4. Check ERROR
                        elif "ERROR" in line_upper: logging.error(f"Arduino Error cho '{command}': {line}"); self.log(f"(!) Arduino Error: {line}", tag="ERROR"); success = False; break
                    else: # No line
                        time.sleep(0.01)
                        if self.gcode_running and self.stop_gcode_flag: logging.info(f"Chờ '{command}' bị ngắt do dừng G-code."); success = False; break
                # --- After wait loop ---
                if not success:
                    if time.time() - start_time >= timeout:
                        log_msg = f"(!) Timeout ({timeout:.1f}s) chờ '{wait_msg}' cho: '{command}'"
                        if is_move_cmd and not move_complete_parsed: log_msg += " (Chưa nhận MOVE_COMPLETE)"
                        elif not ok_received: log_msg += " (Chưa nhận OK)"
                        self.log(log_msg, tag="ERROR"); logging.warning(log_msg)
                else: # Success
                    logging.debug(f"Lệnh '{command}' hoàn thành (Nhận {wait_msg}).")
                    if is_move_cmd and move_complete_parsed and final_x is not None and final_y is not None:
                        self.root.after(0, self.update_position_from_xy, final_x, final_y)
                    elif is_move_cmd and not move_complete_parsed: logging.warning(f"'{command}' thành công nhưng thiếu MOVE_COMPLETE?")
            else: success = True # Not waiting
            return success
        except serial.SerialTimeoutException: self.log(f"(!) Lỗi Serial Timeout GỬI: '{command}'", tag="ERROR"); logging.error(f"Timeout GỬI: '{command}'"); return False
        except serial.SerialException as e: self.log(f"❌ Lỗi Serial GỬI '{command}': {e}", tag="ERROR"); logging.exception(f"Lỗi Serial GỬI:"); return False
        except Exception as e: self.log(f"❌ Lỗi GỬI '{command}': {e}", tag="ERROR"); logging.exception(f"Lỗi GỬI:"); return False

    def _update_ik_config(self, config_str):
        if config_str and config_str != self.current_config:
            self.current_config = config_str
            try:
                if hasattr(self, 'config_label') and self.config_label.winfo_exists(): self.config_label.config(text=config_str)
            except tk.TclError: logging.warning("Lỗi cập nhật config_label.")
            logging.info(f"Cập nhật cấu hình IK: {config_str}")

    def _update_pen_status(self, is_down):
        if self.pen_is_down == is_down: return
        self.pen_is_down = is_down
        try:
            if hasattr(self, 'pen_label') and self.pen_label.winfo_exists(): self.pen_label.configure(text="Hạ" if is_down else "Nâng")
        except tk.TclError: logging.warning("Lỗi cập nhật pen_label.")
        logging.debug(f"Cập nhật bút: {'Hạ' if is_down else 'Nâng'}")
        if is_down:
            x_int = self.current_position.get('x', float('nan')); y_now = self.current_position.get('y', float('nan'))
            if not math.isnan(x_int): x_vis = -x_int; self.current_segment = {'x': [x_vis], 'y': [y_now]}; logging.debug(f"Bắt đầu segment vẽ tại Vis({x_vis:.2f}, {y_now:.2f})")
            else: logging.warning("Không thể bắt đầu segment vẽ (tọa độ nan)."); self.current_segment = {}
        else:
            if self.current_segment and self.current_segment.get('x'):
                if len(self.current_segment['x']) > 1: self.trace_segments.append(self.current_segment.copy()); logging.debug(f"Lưu segment vẽ ({len(self.current_segment['x'])} điểm).")
                else: logging.debug("Bỏ qua segment 1 điểm.")
                self.current_segment = {}
        self.update_animation()

    def move_to_angle(self):
        if not self.is_connected: messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối!"); return
        try:
            t1 = float(self.angle1_entry.get()); t2 = float(self.angle2_entry.get()); lim = self.robot.angle_limits
            if not (lim['theta1_min'] <= t1 <= lim['theta1_max']): messagebox.showerror("Góc Không Hợp Lệ", f"θ1 ngoài giới hạn [{lim['theta1_min']}°, {lim['theta1_max']}°]."); return
            if not (lim['theta2_min'] <= t2 <= lim['theta2_max']): messagebox.showerror("Góc Không Hợp Lệ", f"θ2 ngoài giới hạn [{lim['theta2_min']}°, {lim['theta2_max']}°]."); return
            cmd = f"{t1:.2f},{t2:.2f}"; self.log(f"Yêu cầu góc: θ1={t1:.2f}, θ2={t2:.2f}"); self.send_command(cmd, wait_for_response=True)
        except ValueError: messagebox.showerror("Lỗi Nhập Liệu", "Góc không hợp lệ!")
        except Exception as e: logging.exception("Lỗi move_to_angle:"); messagebox.showerror("Lỗi", f"{e}")

    def move_to_xy(self):
        if not self.is_connected: messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối!"); return
        try:
            x_disp = float(self.x_entry.get()); y = float(self.y_entry.get()); x_int = -x_disp
            if not self.robot.check_point_in_workspace(x_int, y): messagebox.showwarning("Ngoài Vùng Làm Việc", f"(X={x_disp:.2f}, Y={y:.2f}) ngoài vùng."); return
            ik_res = self.robot.inverse_kinematics(x_int, y, self.current_config)
            if ik_res is None: messagebox.showerror("Lỗi Tính Toán", f"Không thể tính góc cho (X={x_disp:.2f}, Y={y:.2f})."); return
            t1, t2, cfg = ik_res; cmd = f"{t1:.2f},{t2:.2f}"
            self.log(f"Yêu cầu XY (Display): ({x_disp:.2f}, {y:.2f}) -> Góc ({t1:.2f}, {t2:.2f}), Config: {cfg}")
            self.send_command(cmd, wait_for_response=True)
        except ValueError: messagebox.showerror("Lỗi Nhập Liệu", "Tọa độ không hợp lệ!")
        except Exception as e: logging.exception("Lỗi move_to_xy:"); messagebox.showerror("Lỗi", f"{e}")

    def update_position_from_xy(self, final_x, final_y):
        self.current_position['x'] = final_x; self.current_position['y'] = final_y
        logging.info(f"Cập nhật từ Arduino XY: Pos=({final_x:.2f}, {final_y:.2f})")
        ik_res = self.robot.inverse_kinematics(final_x, final_y, self.current_config)
        if ik_res:
            calc_t1, calc_t2, sel_cfg = ik_res
            self.current_angles['theta1'] = calc_t1; self.current_angles['theta2'] = calc_t2
            if sel_cfg != self.current_config: self._update_ik_config(sel_cfg)
            logging.info(f"Góc tính từ XY: Angles=({calc_t1:.2f}, {calc_t2:.2f}), Config={sel_cfg}")
        else: logging.warning(f"Không thể tính IK cho XY cuối ({final_x:.2f}, {final_y:.2f}).")
        self.update_position_display(); self.update_animation()

    def update_position_display(self):
        x_int = self.current_position.get('x', float('nan')); y = self.current_position.get('y', float('nan'))
        t1 = self.current_angles.get('theta1', float('nan')); t2 = self.current_angles.get('theta2', float('nan'))
        x_disp = -x_int if not math.isnan(x_int) else float('nan')
        try:
            if hasattr(self, 'pos_label') and self.pos_label.winfo_exists(): pos_txt = f"X={x_disp:.2f}, Y={y:.2f}" if not math.isnan(x_disp) else "X=???, Y=???"; self.pos_label.config(text=pos_txt)
            if hasattr(self, 'angle_label') and self.angle_label.winfo_exists(): ang_txt = f"θ1={t1:.2f}°, θ2={t2:.2f}°" if not math.isnan(t1) else "θ1=???, θ2=???"; self.angle_label.config(text=ang_txt)
        except tk.TclError: pass
        try:
            focus = self.root.focus_get()
            if hasattr(self, 'x_entry') and self.x_entry.winfo_exists() and focus != self.x_entry: self.x_entry.delete(0, tk.END); self.x_entry.insert(0, f"{x_disp:.2f}")
            if hasattr(self, 'y_entry') and self.y_entry.winfo_exists() and focus != self.y_entry: self.y_entry.delete(0, tk.END); self.y_entry.insert(0, f"{y:.2f}")
            if hasattr(self, 'angle1_entry') and self.angle1_entry.winfo_exists() and focus != self.angle1_entry: self.angle1_entry.delete(0, tk.END); self.angle1_entry.insert(0, f"{t1:.2f}")
            if hasattr(self, 'angle2_entry') and self.angle2_entry.winfo_exists() and focus != self.angle2_entry: self.angle2_entry.delete(0, tk.END); self.angle2_entry.insert(0, f"{t2:.2f}")
        except tk.TclError: logging.warning("Lỗi cập nhật entry.")
        except Exception as e: logging.exception("Lỗi cập nhật entry:")

    def pen_down(self):
        if self.is_connected: self.log("Yêu cầu hạ bút (d)"); self.send_command("d", wait_for_response=True)
    def pen_up(self):
        if self.is_connected: self.log("Yêu cầu nâng bút (u)"); self.send_command("u", wait_for_response=True)
    def home(self):
        if not self.is_connected: messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối!"); return
        self.log("Yêu cầu về HOME (h)"); self.send_command("h", wait_for_response=True)

    def calibrate_position(self):
         if not self.is_connected: messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối!"); return
         if messagebox.askokcancel("Xác nhận Hiệu Chuẩn", "Đặt lại góc firmware về (90, 90)?\nRobot SẼ KHÔNG DI CHUYỂN."):
             self.log("Yêu cầu hiệu chuẩn vị trí về (90, 90)...")
             self.send_command('calibrate', wait_for_response=True) # Arduino sẽ gửi MOVE_COMPLETE+OK
             # Python state sẽ được cập nhật trong update_position_from_xy khi nhận MOVE_COMPLETE

    def emergency_stop(self):
        self.log("!!! DỪNG KHẨN CẤP !!!", tag="ERROR"); logging.warning("Dừng khẩn cấp!")
        if self.gcode_running: self.stop_gcode(is_emergency=True)
        self._clear_queues()
        if self.is_connected and self.serial and self.serial.is_open:
            try:
                self.log("Gửi disable trực tiếp...", tag="WARNING"); disable_cmd = b"disable\n"
                with self.serial_lock: self.serial.write(disable_cmd); logging.info("Gửi disable khẩn cấp.")
            except Exception as e: self.log(f"Lỗi gửi disable khẩn cấp: {e}", tag="ERROR"); logging.exception("Lỗi disable khẩn cấp:")
        else: self.log("Không thể disable (chưa kết nối).", tag="WARNING")
        self.command_processing = False

    def set_speed(self):
        if not self.is_connected: return
        try:
            spd = int(self.speed_var.get());
            if spd > 0: self.log(f"Cài đặt tốc độ: {spd} steps/sec"); self.send_command(f"f{spd}", wait_for_response=True)
            else: messagebox.showerror("Giá Trị Không Hợp Lệ", "Tốc độ phải dương.")
        except ValueError: messagebox.showerror("Lỗi Nhập Liệu", "Tốc độ không hợp lệ!")
        except Exception as e: logging.exception("Lỗi set_speed:"); messagebox.showerror("Lỗi", f"{e}")

    def set_acceleration(self):
        if not self.is_connected: return
        try:
            acc = int(self.accel_var.get());
            if acc > 0: self.log(f"Cài đặt gia tốc: {acc} steps/sec^2"); self.send_command(f"a{acc}", wait_for_response=True)
            else: messagebox.showerror("Giá Trị Không Hợp Lệ", "Gia tốc phải dương.")
        except ValueError: messagebox.showerror("Lỗi Nhập Liệu", "Gia tốc không hợp lệ!")
        except Exception as e: logging.exception("Lỗi set_acceleration:"); messagebox.showerror("Lỗi", f"{e}")

    def log(self, message, tag="INFO"):
        def _log():
            try:
                if not self.root.winfo_exists() or not hasattr(self, 'log_text') or not self.log_text.winfo_exists(): return
                self.log_text.config(state=tk.NORMAL); num = int(self.log_text.index('end - 1 line').split('.')[0]); max_lines = 500
                if num > max_lines: self.log_text.delete('1.0', f'{num - max_lines + 1}.0')
                ts = time.strftime("[%H:%M:%S]"); safe_msg = message.replace('⚠️', '(!)') # Replace emoji
                start = self.log_text.index(tk.END + "-1c"); self.log_text.insert(tk.END, f"{ts} {safe_msg}\n"); end = self.log_text.index(tk.END + "-1c")
                if tag in self.log_text.tag_names(): self.log_text.tag_add(tag, start, end)
                self.log_text.see(tk.END); self.log_text.config(state=tk.DISABLED)
            except tk.TclError as e: logging.warning(f"TclError log_text: {e}")
            except Exception as e: logging.exception("Lỗi log:")
        try:
            if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists(): self.root.after(0, _log)
            else: print(f"LOG (no root): {message}")
        except Exception: print(f"LOG (root error): {message}")

    def load_gcode(self):
        if self.gcode_running:
            if messagebox.askyesno("G-Code Đang Chạy", "Dừng và tải file mới?"): self.stop_gcode()
            else: return
        fpath = filedialog.askopenfilename(title="Chọn G-Code", filetypes=(("G-Code", "*.gcode *.ngc *.nc *.tap"), ("All", "*.*")))
        if not fpath: return
        try:
            with open(fpath, "r", encoding='utf-8', errors='ignore') as f: raw = f.readlines()
            proc_gcode, disp_content = [], []
            for line in raw:
                orig = line.strip(); cleaned = orig; cmt_idx = cleaned.find(';')
                if cmt_idx != -1: cleaned = cleaned[:cmt_idx].strip()
                cleaned = re.sub(r'\([^)]*\)', '', cleaned).strip()
                if not cleaned: disp_content.append(orig); continue
                proc_line = cleaned.upper(); proc_gcode.append(proc_line); disp_content.append(orig)
            self.gcode_text.config(state=tk.NORMAL); self.gcode_text.delete(1.0, tk.END); self.gcode_text.insert(tk.END, "\n".join(disp_content)); self.gcode_text.config(state=tk.DISABLED)
            self.gcode_lines = proc_gcode; self.gcode_total_lines = len(proc_gcode); fname = os.path.basename(fpath)
            self.file_label.config(text=fname); self.log(f"Đã tải: {fname} ({self.gcode_total_lines} lệnh)"); logging.info(f"Tải G-code: {fname}, {self.gcode_total_lines} lệnh.")
            self.progress_var.set(0); self.progress_label.config(text="0.0% hoàn thành")
            self.run_btn.config(state=tk.NORMAL); self.pause_btn.config(state=tk.DISABLED); self.stop_gcode_btn.config(state=tk.DISABLED)
            self.clear_drawing()
        except Exception as e:
            logging.exception("Lỗi tải/xử lý G-code:"); messagebox.showerror("Lỗi Tải File", f"{e}"); self.log(f"Lỗi tải G-code: {e}", tag="ERROR")
            self.file_label.config(text="Lỗi tải file"); self.gcode_lines = []; self.gcode_total_lines = 0; self.run_btn.config(state=tk.DISABLED)

    def run_gcode(self):
        if not self.is_connected: messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối!"); return
        if not self.gcode_lines: messagebox.showwarning("Chưa Tải File", "Vui lòng tải file!"); return
        if self.gcode_running and self.gcode_paused: self.gcode_paused = False; self.pause_btn.config(text="Tạm dừng"); self.log("Tiếp tục G-code..."); logging.info("Tiếp tục G-code."); return
        if self.gcode_running and not self.gcode_paused: self.log("G-code đang chạy."); return
        self.gcode_running = True; self.gcode_paused = False; self.stop_gcode_flag = False; self.gcode_line_num = 0; self.update_progress(0.0)
        delay = self.gcode_delay_var.get(); self.log(f"Bắt đầu G-code (delay {delay:.3f}s)."); logging.info(f"Bắt đầu G-code (delay={delay:.3f}s).")
        self.run_btn.config(state=tk.DISABLED); self.pause_btn.config(text="Tạm dừng", state=tk.NORMAL); self.stop_gcode_btn.config(state=tk.NORMAL)
        self.direct_controls_enable(False)
        if hasattr(self, '_gcode_thread') and self._gcode_thread.is_alive(): logging.warning("Thread G-code cũ vẫn chạy?"); return
        self._gcode_thread = threading.Thread(target=self._process_gcode_thread, daemon=True); self._gcode_thread.start()

    def _process_gcode_thread(self):
        logging.info("Thread G-code bắt đầu.")
        try:
            while self.gcode_line_num < self.gcode_total_lines and not self.stop_gcode_flag:
                while self.gcode_paused and not self.stop_gcode_flag: time.sleep(0.1)
                if self.stop_gcode_flag: logging.info("Thread G-code: Nhận stop flag."); break
                idx = self.gcode_line_num; line = self.gcode_lines[idx].strip()
                logging.debug(f"G-code Line {idx + 1}/{self.gcode_total_lines}: {line}")
                self.root.after(0, lambda ln=idx: self.highlight_current_line(ln))
                success = self.process_gcode_line_and_wait(line) # Waits internally
                if not success or self.stop_gcode_flag:
                     logging.warning(f"Dừng G-code dòng {idx + 1} do lỗi/stop."); self.root.after(0, lambda e=not success: self.stop_gcode(is_emergency=e)); break
                if self.gcode_line_num == idx and not self.stop_gcode_flag: # Ensure progress only if line succeeded and not stopped
                    self.gcode_line_num += 1; prog = (self.gcode_line_num / self.gcode_total_lines) * 100.0; self.root.after(0, lambda p=prog: self.update_progress(p))
                delay = self.gcode_delay_var.get()
                if delay > 0 and not self.stop_gcode_flag: time.sleep(delay)
            if not self.stop_gcode_flag: logging.info("Hoàn thành G-code (hết file)."); self.root.after(0, self.complete_gcode)
        except Exception as e:
            logging.exception("Lỗi thread G-code:"); self.log(f"❌ Lỗi G-code: {e}", tag="ERROR")
            if self.gcode_running: self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
        finally: logging.info("Thread G-code kết thúc.")

    def process_gcode_line_and_wait(self, line):
        """Phân tích, gửi lệnh G-code và chờ hoàn thành."""
        if not line: return True  # Empty line is success

        cmd_to_send = None;
        is_move = False;
        action_py = False
        parts = line.split();
        cmd_code = parts[0] if parts else ""
        x, y, z, f, p = None, None, None, None, None;
        hasX, hasY = False, False

        # --- THÊM DÒNG KHỞI TẠO NÀY ---
        code = None  # Initialize 'code' to None before the loop

        for part in parts[1:]:
            # Skips if part is empty or too short
            if not part or len(part) < 2: continue;
            # Assigns 'code' if the loop runs and the 'if' passes
            code = part[0]
            try:
                val = float(part[1:])
                # Uses 'code' here
                if code == 'X':
                    x = val; hasX = True
                elif code == 'Y':
                    y = val; hasY = True
                elif code == 'Z':
                    z = val
                elif code == 'F':
                    f = val
                elif code == 'P':
                    p = val
            except ValueError:
                logging.warning(f"Bỏ qua G-code part không hợp lệ: '{part}'");
                continue

        # Process Z (Pen) - sends OK internally
        if z is not None:
            pen_cmd = 'd' if z <= 0 and not self.pen_is_down else ('u' if z > 0 and self.pen_is_down else None)
            if pen_cmd:
                 if not self.send_command(pen_cmd, wait_for_response=True): return False
                 if not self.wait_for_command_completion(): return False # Wait for pen cmd
                 if self.stop_gcode_flag: return False

        # Process F (Speed) - sends OK internally
        if f is not None and f > 0:
            spd_cmd = f"f{int(f)}"
            if not self.send_command(spd_cmd, wait_for_response=True): return False
            if not self.wait_for_command_completion(): return False # Wait for speed cmd
            if self.stop_gcode_flag: return False

        # Process XY (Move) - waits for MOVE_COMPLETE + OK
        if hasX and hasY:
            if x is None or y is None: self.log(f"Lỗi G-code: Thiếu X/Y '{line}'", tag="ERROR"); return False
            x_int, y_int = -x, y # Flip X from G-code
            if not self.robot.check_point_in_workspace(x_int, y_int): self.log(f"Lỗi G-code: Điểm ({x}, {y}) ngoài vùng!", tag="ERROR"); return False
            ik_res = self.robot.inverse_kinematics(x_int, y_int, self.current_config)
            if ik_res is None: self.log(f"Lỗi G-code: Không tính được IK cho ({x}, {y})!", tag="ERROR"); return False
            t1, t2, _ = ik_res; cmd_to_send = f"{t1:.2f},{t2:.2f}"; is_move = True
        # Other G/M codes
        elif cmd_code == 'G4':
             if p is not None and p >= 0:
                 dwell = p / 1000.0; self.log(f"Dừng G-code {dwell:.3f}s (G4)"); start = time.time()
                 while time.time() - start < dwell:
                     if self.stop_gcode_flag: return False; time.sleep(0.05)
                 action_py = True # Handled in Python
             else: self.log(f"Lỗi G-code: Thiếu P cho G4 '{line}'", tag="ERROR"); return False
        elif cmd_code == 'G28': cmd_to_send = "h"; is_move = True # Home waits for MOVE_COMPLETE+OK
        elif cmd_code.startswith(("M3", "M03")): cmd_to_send = ('d' if not self.pen_is_down else None) # Pen sends OK internally
        elif cmd_code.startswith(("M5", "M05")): cmd_to_send = ('u' if self.pen_is_down else None) # Pen sends OK internally
        elif cmd_code == 'M17': cmd_to_send = "enable" # Sends OK internally
        elif cmd_code.startswith(("M18", "M84")): cmd_to_send = "disable" # Sends OK internally
        elif cmd_code.startswith(("G90", "G91", "G20", "G21")): logging.debug(f"Bỏ qua G-code: {line}"); action_py = True # Ignore, treat as success
        else: logging.warning(f"Bỏ qua G-code không xác định: {line}"); return True # Unknown is success

        # Send command if needed and wait
        if cmd_to_send:
            if not self.send_command(cmd_to_send, wait_for_response=True): return False
            # Crucially wait for the command queue processing to finish
            if not self.wait_for_command_completion(): return False
            if self.stop_gcode_flag: return False
            return True # Command sent and waited for successfully
        elif action_py: return True # Handled purely in Python (G4, G90 etc.)
        else: return True # No command needed (e.g., M3 when already down)

    def wait_for_command_completion(self):
        """Waits until the command queue is empty and processing is done."""
        start = time.time(); max_wait = self.command_timeout * (self.move_timeout_factor + 1.0)
        while time.time() - start < max_wait:
            q_empty = self.command_queue.empty(); proc_done = not self.command_processing
            if q_empty and proc_done: return True
            if self.stop_gcode_flag: logging.info("wait_for_completion: Stop flag."); return False
            time.sleep(0.02)
        logging.warning(f"Timeout ({max_wait:.1f}s) chờ lệnh hoàn thành!"); self.log(f"(!) Timeout chờ lệnh!", tag="WARNING"); return False

    def highlight_current_line(self, line_num):
        try:
            if not hasattr(self, 'gcode_text') or not self.gcode_text.winfo_exists(): return
            self.gcode_text.tag_remove("current_line", "1.0", tk.END); start = f"{line_num + 1}.0"; end = f"{line_num + 1}.end"
            self.gcode_text.tag_add("current_line", start, end)
            if "current_line" not in self.gcode_text.tag_names(): self.gcode_text.tag_config("current_line", background="yellow", relief="raised")
            self.gcode_text.see(start)
        except tk.TclError: logging.warning(f"Lỗi highlight dòng {line_num + 1}.")
        except Exception as e: logging.exception(f"Lỗi highlight dòng {line_num + 1}:")

    def update_progress(self, progress):
        try:
            if not hasattr(self, 'progress_var') or not self.root.winfo_exists(): return
            clamped = max(0.0, min(100.0, progress)); self.progress_var.set(clamped); self.progress_label.config(text=f"{clamped:.1f}% hoàn thành")
        except tk.TclError: logging.warning("Lỗi cập nhật progress bar.")
        except Exception as e: logging.exception("Lỗi cập nhật progress:")

    def pause_gcode(self):
        if not self.gcode_running: return
        self.gcode_paused = not self.gcode_paused; state = "Tiếp tục" if self.gcode_paused else "Tạm dừng"; log = "Đã tạm dừng G-code." if self.gcode_paused else "Tiếp tục G-code."
        self.pause_btn.config(text=state); self.log(log); logging.info(log)

    def stop_gcode(self, is_emergency=False):
        if not self.gcode_running: return
        self.stop_gcode_flag = True; self.gcode_paused = False; self.gcode_running = False
        tag = "WARNING" if is_emergency else "INFO"; msg = "Đang dừng G-code (khẩn cấp)..." if is_emergency else "Đang dừng G-code..."
        self.log(msg, tag=tag); logging.warning(msg) if is_emergency else logging.info(msg)
        self.root.after(50, self._reset_gcode_ui); self.log("Đã dừng G-code.")

    def complete_gcode(self):
        if self.gcode_running: self.gcode_running = False
        self.log("✓ Hoàn thành G-code.", tag="SUCCESS"); logging.info("G-code hoàn thành.")
        self._reset_gcode_ui(); self.update_progress(100.0)

    def _reset_gcode_ui(self):
        try:
            if not self.root.winfo_exists(): return
            run_st = tk.NORMAL if self.gcode_lines else tk.DISABLED
            if hasattr(self, 'run_btn') and self.run_btn.winfo_exists(): self.run_btn.config(state=run_st)
            if hasattr(self, 'pause_btn') and self.pause_btn.winfo_exists(): self.pause_btn.config(text="Tạm dừng", state=tk.DISABLED)
            if hasattr(self, 'stop_gcode_btn') and self.stop_gcode_btn.winfo_exists(): self.stop_gcode_btn.config(state=tk.DISABLED)
            if hasattr(self, 'gcode_text') and self.gcode_text.winfo_exists(): self.gcode_text.tag_remove("current_line", "1.0", tk.END)
            self.direct_controls_enable(True)
        except tk.TclError: logging.warning("Lỗi reset G-code UI.")
        except Exception as e: logging.exception("Lỗi reset G-code UI:")

    def direct_controls_enable(self, enabled=True):
         state = tk.NORMAL if enabled else tk.DISABLED
         widgets = [getattr(self, w, None) for w in ['go_xy_btn', 'go_angle_btn', 'home_btn', 'pen_up_btn', 'pen_down_btn', 'calibrate_btn', 'set_speed_button', 'set_accel_button']]
         for w in widgets:
             try:
                 if w and hasattr(w, 'config') and w.winfo_exists(): w.config(state=state)
             except tk.TclError: logging.warning("Lỗi đổi trạng thái widget.")
             except Exception as e: logging.exception("Lỗi đổi trạng thái widget:")

    def reconnect(self):
        tag = "WARNING"; msg = "(!) Mất kết nối? Thử lại..." if not self.is_connected else "(!) Lỗi? Thử ngắt và kết nối lại..."
        self.log(msg, tag=tag); logging.warning(msg)
        if self.is_connected: self.disconnect(); self.root.after(1500, self.connect)
        else: self.connect()

    def on_closing(self):
        logging.info("Đóng ứng dụng..."); self.log("Đang đóng ứng dụng...")
        self.stop_gcode_flag = True; self.command_queue.put(None)
        threads = [getattr(self, t, None) for t in ['_gcode_thread', 'process_command_thread']]
        for t in threads:
             if t and t.is_alive():
                 try: t.join(timeout=0.5)
                 except RuntimeError: pass
        if self.is_connected: self.emergency_stop(); time.sleep(0.1); self.disconnect(silent=True)
        self.root.destroy(); logging.info("Ứng dụng đã đóng.")

def main():
    root = tk.Tk()
    try:
        style = ttk.Style(root); themes = style.theme_names(); logging.info(f"Themes: {themes}")
        if 'vista' in themes: style.theme_use('vista'); logging.info(f"Using theme: {style.theme_use()}")
    except Exception as e: logging.warning(f"Không thể đặt theme: {e}")
    app = ScaraGUI(root); root.protocol("WM_DELETE_WINDOW", app.on_closing)
    try: root.mainloop()
    except KeyboardInterrupt: logging.info("KBInterrupt, đóng..."); app.on_closing()

if __name__ == "__main__":
    main()