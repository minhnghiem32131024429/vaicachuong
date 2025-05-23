import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import re
import os
import queue
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch


class ScaraRobot:
    def __init__(self):
        self.robot_params = {
            'x1': -20.0,  # Tọa độ X motor 1 (motor Y) - Đã điều chỉnh
            'y1': 0.0,  # Tọa độ Y motor 1
            'x5': 0.0,  # Tọa độ X motor 2 (motor X) - Đã điều chỉnh
            'y5': 0.0,  # Tọa độ Y motor 2
            'L1': 15.0,  # Chiều dài cánh tay 1 từ motor 1
            'L2': 25.0,  # Chiều dài cánh tay 2 từ joint 1 đến đầu vẽ
            'L3': 25.0,  # Chiều dài cánh tay 3 từ đầu vẽ đến joint 2
            'L4': 15.0,  # Chiều dài cánh tay 4 từ joint 2 đến motor 2
            'reach_radius1': 40.0,  # Tầm với từ motor 1 (L1+L2)
            'reach_radius2': 40.0  # Tầm với từ motor 2 (L3+L4)
        }

        # Vùng làm việc cố định - tham khảo từ song_song.py
        self.workspace_bounds = {
            'left': -20.0,
            'right': 0.0,
            'bottom': 15.0,
            'top': 35.0
        }

        self.home_position = {'x': -10.0, 'y': 33.0}  # Vị trí home

        # Tính bán kính vùng làm việc từ mỗi động cơ (cho các tính toán khác)
        self.r1_min = max(0, abs(self.robot_params['L1'] - self.robot_params['L2'])) + 0.5
        self.r1_max = (self.robot_params['L1'] + self.robot_params['L2']) - 0.5
        self.r2_min = max(0, abs(self.robot_params['L4'] - self.robot_params['L3'])) + 0.5
        self.r2_max = (self.robot_params['L4'] + self.robot_params['L3']) - 0.5

    def calculate_workspace(self):
        """Tính vùng làm việc cân đối cho cả hai động cơ"""
        # Thông số
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Tính bán kính vùng làm việc từ mỗi động cơ
        self.r1_min = max(0, abs(L1 - L2)) + 0.5
        self.r1_max = (L1 + L2) - 0.5
        self.r2_min = max(0, abs(L4 - L3)) + 0.5
        self.r2_max = (L4 + L3) - 0.5

        # Tính hình chữ nhật bao quanh khu vực làm việc - cân đối cho cả hai bên
        max_radius = max(self.r1_max, self.r2_max)

        # Đảm bảo kích thước hai bên bằng nhau
        self.workspace_bounds = {
            'left': min(x1 - max_radius, x5 - max_radius),
            'right': max(x1 + max_radius, x5 + max_radius),
            'bottom': min(y1 - max_radius, y5 - max_radius),  # Đã loại bỏ hệ số 0.7
            'top': max(y1 + max_radius, y5 + max_radius)
        }

        # Log thông tin về workspace
        print(f"Vùng làm việc: X={self.workspace_bounds['left']:.1f} đến {self.workspace_bounds['right']:.1f}, "
              f"Y={self.workspace_bounds['bottom']:.1f} đến {self.workspace_bounds['top']:.1f}")
        print(f"Bán kính từ motor 1: {self.r1_min:.1f} đến {self.r1_max:.1f}")
        print(f"Bán kính từ motor 2: {self.r2_min:.1f} đến {self.r2_max:.1f}")

    def forward_kinematics(self, theta1, theta2):
        x1 = self.robot_params['x1']
        y1 = self.robot_params['y1']
        x5 = self.robot_params['x5']
        y5 = self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Tính tọa độ các điểm
        x2 = x1 + L1 * np.cos(np.radians(theta1))
        y2 = y1 + L1 * np.sin(np.radians(theta1))

        x4 = x5 + L4 * np.cos(np.radians(theta2))
        y4 = y5 + L4 * np.sin(np.radians(theta2))

        two_to_four = np.sqrt((x2 - x4) ** 2 + (y2 - y4) ** 2)
        two_to_h = (L2 ** 2 - L3 ** 2 + two_to_four ** 2) / (2 * two_to_four)

        yh = y2 + (two_to_h / two_to_four) * (y4 - y2)
        xh = x2 + (two_to_h / two_to_four) * (x4 - x2)

        three_to_h = np.sqrt(abs(L2 ** 2 - two_to_h ** 2))  # Thêm abs để tránh lỗi căn bậc âm

        x3 = xh + (three_to_h / two_to_four) * (y4 - y2)
        y3 = yh - (three_to_h / two_to_four) * (x4 - x2)

        return (x2, y2), (x3, y3), (x4, y4)

    def inverse_kinematics(self, x3, y3, current_config=None):
        # Lấy thông số robot
        x1 = self.robot_params['x1']
        y1 = self.robot_params['y1']
        x5 = self.robot_params['x5']
        y5 = self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Giới hạn góc
        THETA1_MIN, THETA1_MAX = -120, 150
        THETA2_MIN, THETA2_MAX = 30, 300

        # Tính khoảng cách từ điểm vẽ đến các động cơ
        L13_sq = (x3 - x1) ** 2 + (y3 - y1) ** 2
        L13 = np.sqrt(L13_sq)
        L53_sq = (x3 - x5) ** 2 + (y3 - y5) ** 2
        L53 = np.sqrt(L53_sq)

        # Kiểm tra điểm có nằm trong phạm vi không
        min_dist1 = abs(L1 - L2) + 0.1
        max_dist1 = L1 + L2 - 0.1
        min_dist2 = abs(L4 - L3) + 0.1
        max_dist2 = L4 + L3 - 0.1

        if (L13 < min_dist1 or L13 > max_dist1 or L53 < min_dist2 or L53 > max_dist2):
            return None, None, None

        # Tính góc cho motor 1 (Motor Y)
        cos_alpha1 = (L1 ** 2 + L13_sq - L2 ** 2) / (2 * L1 * L13)
        if abs(cos_alpha1) > 1.0 + 1e-10:
            return None, None, None

        cos_alpha1 = np.clip(cos_alpha1, -1.0, 1.0)
        alpha1 = np.arccos(cos_alpha1)
        beta1 = np.arctan2(y3 - y1, x3 - x1)

        theta1_elbow_down = np.degrees(beta1 + alpha1)
        theta1_elbow_up = np.degrees(beta1 - alpha1)

        # Tính góc cho motor 2 (Motor X)
        cos_alpha5 = (L4 ** 2 + L53_sq - L3 ** 2) / (2 * L4 * L53)
        if abs(cos_alpha5) > 1.0 + 1e-10:
            return None, None, None

        cos_alpha5 = np.clip(cos_alpha5, -1.0, 1.0)
        alpha5 = np.arccos(cos_alpha5)
        beta5 = np.arctan2(y3 - y5, x3 - x5)

        theta2_elbow_down = np.degrees(beta5 - alpha5)
        theta2_elbow_up = np.degrees(beta5 + alpha5)

        # Chuẩn hóa góc về khoảng [-180, 180]
        def normalize_angle(angle):
            while angle > 180: angle -= 360
            while angle < -180: angle += 360
            return angle

        theta1_elbow_down = normalize_angle(theta1_elbow_down)
        theta1_elbow_up = normalize_angle(theta1_elbow_up)
        theta2_elbow_down = normalize_angle(theta2_elbow_down)
        theta2_elbow_up = normalize_angle(theta2_elbow_up)

        # Kiểm tra các giải pháp có thỏa mãn giới hạn góc không
        # Thay đổi: Thử cả hai cấu hình và chọn giải pháp tốt nhất
        solutions = []

        # Kiểm tra cấu hình khuỷu xuống
        if (THETA1_MIN <= theta1_elbow_down <= THETA1_MAX and
                THETA2_MIN <= theta2_elbow_down <= THETA2_MAX):
            solutions.append(("elbow_down", theta1_elbow_down, theta2_elbow_down))

        # Kiểm tra cấu hình khuỷu lên
        if (THETA1_MIN <= theta1_elbow_up <= THETA1_MAX and
                THETA2_MIN <= theta2_elbow_up <= THETA2_MAX):
            solutions.append(("elbow_up", theta1_elbow_up, theta2_elbow_up))

        if not solutions:
            return None, None, None

        # Ưu tiên giải pháp gần với cấu hình hiện tại
        if current_config is not None and len(solutions) > 1:
            for config, theta1, theta2 in solutions:
                if config == current_config:
                    return theta1, theta2, config

        # Đặc biệt ưu tiên giải pháp cho Y âm khi cần thiết
        if y3 < 0 and len(solutions) > 1:
            # Tìm giải pháp tốt nhất cho Y âm
            best_solution = solutions[0]
            for solution in solutions:
                config, theta1, theta2 = solution
                if (config == "elbow_up" and y3 < -5) or (config == "elbow_down" and y3 > -5):
                    best_solution = solution
                    break
            return best_solution[1], best_solution[2], best_solution[0]

        # Nếu không có ưu tiên đặc biệt, lấy giải pháp đầu tiên
        config, theta1, theta2 = solutions[0]
        return theta1, theta2, config

    def check_point_in_workspace(self, x, y):
        """Kiểm tra điểm có nằm trong vùng làm việc cố định không"""
        bounds = self.workspace_bounds

        # Kiểm tra điểm có nằm trong hình chữ nhật phạm vi làm việc
        in_bounds = (bounds['left'] <= x <= bounds['right'] and
                     bounds['bottom'] <= y <= bounds['top'])

        if not in_bounds:
            return False

        # Nếu điểm nằm trong vùng làm việc cố định, kiểm tra xem có giải pháp động học ngược không
        theta1, theta2, _ = self.inverse_kinematics(x, y)
        return theta1 is not None and theta2 is not None

    def create_workspace_visualization(self):
        """Tạo dữ liệu trực quan hóa vùng làm việc cố định"""
        points = []
        resolution = 0.3  # Độ phân giải cao hơn

        # Sử dụng khu vực làm việc cố định
        x_range = np.arange(self.workspace_bounds['left'], self.workspace_bounds['right'] + resolution, resolution)
        y_range = np.arange(self.workspace_bounds['bottom'], self.workspace_bounds['top'] + resolution, resolution)

        for x in x_range:
            for y in y_range:
                # Chỉ kiểm tra giải pháp động học ngược, không cần kiểm tra khoảng cách
                theta1, theta2, _ = self.inverse_kinematics(x, y)
                if theta1 is not None and theta2 is not None:
                    points.append((x, y))

        return points


class ScaraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Controller")
        self.root.geometry("1000x700")
        self.current_config = None

        # Khởi tạo robot và Serial
        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 0.0, 'theta2': 0.0}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.pen_is_down = False

        self.gcode_delay = 0.5  # Mặc định 0.5 giây

        # Animation variables
        self.animation = None
        self.animation_speed = 50
        self.trace_segments = []
        self.current_segment = {'x': [], 'y': []}
        self.robot_parts = []
        self.paused = False

        # Biến cho G-code
        self.gcode_lines = []
        self.gcode_running = False
        self.gcode_paused = False
        self.gcode_line_num = 0
        self.gcode_total_lines = 0

        # Hệ thống lệnh
        self.command_queue = queue.Queue()
        self.command_processing = False
        self.serial_lock = threading.Lock()
        self.process_command_thread = threading.Thread(target=self._process_command_queue, daemon=True)
        self.process_command_thread.start()

        # Tạo giao diện
        self.create_widgets()

        # Cập nhật danh sách cổng COM
        self.update_ports()

    def _process_command_queue(self):
        while True:
            try:
                if not self.command_processing and not self.command_queue.empty():
                    command, wait_response = self.command_queue.get()
                    self.command_processing = True
                    self._execute_command(command, wait_response)
                    self.command_processing = False
                    time.sleep(0.05)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.log(f"Lỗi trong queue lệnh: {str(e)}")
                self.command_processing = False
                time.sleep(0.1)

    def create_widgets(self):
        # Frame chính
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Panel điều khiển
        control_frame = ttk.Frame(main_frame, padding=5)
        main_frame.add(control_frame, weight=40)

        # Panel hiển thị
        canvas_frame = ttk.Frame(main_frame, padding=5)
        main_frame.add(canvas_frame, weight=60)

        # --- Panel kết nối ---
        conn_frame = ttk.LabelFrame(control_frame, text="Kết nối", padding=5)
        conn_frame.pack(fill=tk.X, pady=5)

        ttk.Label(conn_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        refresh_btn = ttk.Button(conn_frame, text="⟳", width=3, command=self.update_ports)
        refresh_btn.grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="9600")
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=15,
                                  values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=5, pady=5)

        # --- Panel điều khiển thủ công ---
        manual_frame = ttk.LabelFrame(control_frame, text="Điều khiển thủ công", padding=5)
        manual_frame.pack(fill=tk.X, pady=5)

        # Điều khiển bút
        pen_frame = ttk.Frame(manual_frame)
        pen_frame.pack(fill=tk.X, pady=5)

        pen_up_btn = ttk.Button(pen_frame, text="Nâng bút", command=self.pen_up)
        pen_up_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        pen_down_btn = ttk.Button(pen_frame, text="Hạ bút", command=self.pen_down)
        pen_down_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # Tọa độ XY
        pos_frame = ttk.Frame(manual_frame)
        pos_frame.pack(fill=tk.X, pady=5)

        ttk.Label(pos_frame, text="X:").pack(side=tk.LEFT, padx=5)
        self.x_entry = ttk.Entry(pos_frame, width=8)
        self.x_entry.pack(side=tk.LEFT, padx=5)
        self.x_entry.insert(0, "0.0")

        ttk.Label(pos_frame, text="Y:").pack(side=tk.LEFT, padx=5)
        self.y_entry = ttk.Entry(pos_frame, width=8)
        self.y_entry.pack(side=tk.LEFT, padx=5)
        self.y_entry.insert(0, "30.0")

        go_xy_btn = ttk.Button(pos_frame, text="Di chuyển XY", command=self.move_to_xy)
        go_xy_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        # Góc
        angle_frame = ttk.Frame(manual_frame)
        angle_frame.pack(fill=tk.X, pady=5)

        ttk.Label(angle_frame, text="Góc 1:").pack(side=tk.LEFT, padx=5)
        self.angle1_entry = ttk.Entry(angle_frame, width=8)
        self.angle1_entry.pack(side=tk.LEFT, padx=5)
        self.angle1_entry.insert(0, "0.0")

        ttk.Label(angle_frame, text="Góc 2:").pack(side=tk.LEFT, padx=5)
        self.angle2_entry = ttk.Entry(angle_frame, width=8)
        self.angle2_entry.pack(side=tk.LEFT, padx=5)
        self.angle2_entry.insert(0, "0.0")

        go_angle_btn = ttk.Button(angle_frame, text="Di chuyển Góc", command=self.move_to_angle)
        go_angle_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        # Nút hiển thị vùng làm việc
        workspace_btn = ttk.Button(manual_frame, text="Hiển thị vùng làm việc", command=self.show_workspace)
        workspace_btn.pack(fill=tk.X, padx=5, pady=5)

        # Chức năng bổ sung
        func_frame = ttk.Frame(manual_frame)
        func_frame.pack(fill=tk.X, pady=5)

        home_btn = ttk.Button(func_frame, text="HOME", command=self.home)
        home_btn.grid(row=0, column=0, padx=5, pady=5, sticky=tk.EW)

        test_btn = ttk.Button(func_frame, text="Test", command=self.test_motors)
        test_btn.grid(row=0, column=1, padx=5, pady=5, sticky=tk.EW)

        stop_btn = ttk.Button(func_frame, text="DỪNG", command=self.emergency_stop,
                              style="Emergency.TButton")
        stop_btn.grid(row=0, column=2, padx=5, pady=5, sticky=tk.EW)

        # Tốc độ và gia tốc
        speed_frame = ttk.Frame(manual_frame)
        speed_frame.pack(fill=tk.X, pady=5)

        ttk.Label(speed_frame, text="Tốc độ:").grid(row=0, column=0, padx=5)
        self.speed_var = tk.StringVar(value="600")
        speed_entry = ttk.Entry(speed_frame, textvariable=self.speed_var, width=8)
        speed_entry.grid(row=0, column=1, padx=5)

        ttk.Button(speed_frame, text="Set", command=self.set_speed).grid(row=0, column=2, padx=5)

        ttk.Label(speed_frame, text="Gia tốc:").grid(row=1, column=0, padx=5, pady=5)
        self.accel_var = tk.StringVar(value="200")
        accel_entry = ttk.Entry(speed_frame, textvariable=self.accel_var, width=8)
        accel_entry.grid(row=1, column=1, padx=5, pady=5)

        ttk.Button(speed_frame, text="Set", command=self.set_acceleration).grid(row=1, column=2, padx=5, pady=5)

        # --- Panel G-code ---
        gcode_frame = ttk.LabelFrame(control_frame, text="Điều khiển G-Code", padding=5)
        gcode_frame.pack(fill=tk.X, pady=5)

        # Nút tải file G-code
        load_frame = ttk.Frame(gcode_frame)
        load_frame.pack(fill=tk.X, pady=5)

        load_btn = ttk.Button(load_frame, text="Tải file G-Code", command=self.load_gcode)
        load_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Hiển thị tên file
        self.file_label = ttk.Label(load_frame, text="Chưa tải file")
        self.file_label.pack(side=tk.RIGHT, padx=5)

        # Khung hiển thị G-code
        code_frame = ttk.Frame(gcode_frame)
        code_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.gcode_text = tk.Text(code_frame, height=6, wrap=tk.NONE)
        self.gcode_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        gcode_yscroll = ttk.Scrollbar(code_frame, command=self.gcode_text.yview)
        gcode_yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.gcode_text.config(yscrollcommand=gcode_yscroll.set)

        gcode_xscroll = ttk.Scrollbar(gcode_frame, orient=tk.HORIZONTAL, command=self.gcode_text.xview)
        gcode_xscroll.pack(fill=tk.X)
        self.gcode_text.config(xscrollcommand=gcode_xscroll.set)

        # Nút điều khiển
        ctrl_frame = ttk.Frame(gcode_frame)
        ctrl_frame.pack(fill=tk.X, pady=5)

        self.run_btn = ttk.Button(ctrl_frame, text="Chạy G-Code", command=self.run_gcode, state=tk.DISABLED)
        self.run_btn.grid(row=0, column=0, padx=5, pady=5)

        self.pause_btn = ttk.Button(ctrl_frame, text="Tạm dừng", command=self.pause_gcode, state=tk.DISABLED)
        self.pause_btn.grid(row=0, column=1, padx=5, pady=5)

        self.stop_btn = ttk.Button(ctrl_frame, text="Dừng", command=self.stop_gcode, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=2, padx=5, pady=5)

        # Thanh tiến trình
        self.progress_var = tk.IntVar()
        progress_bar = ttk.Progressbar(gcode_frame, variable=self.progress_var, maximum=100)
        progress_bar.pack(fill=tk.X, pady=5)

        # Nhãn tiến trình
        self.progress_label = ttk.Label(gcode_frame, text="0% hoàn thành")
        self.progress_label.pack(pady=2)

        delay_frame = ttk.Frame(gcode_frame)
        delay_frame.pack(fill=tk.X, pady=5)

        ttk.Label(delay_frame, text="Thời gian chờ giữa các lệnh (giây):").pack(side=tk.LEFT, padx=5)
        self.delay_var = tk.StringVar(value="0.5")
        delay_entry = ttk.Entry(delay_frame, textvariable=self.delay_var, width=5)
        delay_entry.pack(side=tk.LEFT, padx=5)

        apply_btn = ttk.Button(delay_frame, text="Áp dụng", command=self.apply_delay)
        apply_btn.pack(side=tk.LEFT, padx=5)

        # Panel thông báo
        log_frame = ttk.LabelFrame(control_frame, text="Nhật ký", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = tk.Text(log_frame, height=10, wrap=tk.WORD)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        log_scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scroll.set)

        # --- Panel Hiển thị ---
        # Create the figure and canvas for visualization
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Thông tin trạng thái
        status_frame = ttk.Frame(canvas_frame)
        status_frame.pack(fill=tk.X, pady=5)

        # Vị trí hiện tại
        pos_status = ttk.Frame(status_frame)
        pos_status.pack(fill=tk.X, pady=5)

        ttk.Label(pos_status, text="Vị trí:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.pos_label = ttk.Label(pos_status, text="X=0.0, Y=0.0")
        self.pos_label.grid(row=0, column=1, sticky=tk.W, padx=5)

        ttk.Label(pos_status, text="Góc:").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.angle_label = ttk.Label(pos_status, text="θ1=0.0°, θ2=0.0°")
        self.angle_label.grid(row=1, column=1, sticky=tk.W, padx=5)

        # Bút
        ttk.Label(pos_status, text="Bút:").grid(row=2, column=0, sticky=tk.W, padx=5)
        self.pen_label = ttk.Label(pos_status, text="Nâng")
        self.pen_label.grid(row=2, column=1, sticky=tk.W, padx=5)

        # Style cho nút dừng khẩn cấp
        style = ttk.Style()
        style.configure("Emergency.TButton", foreground="white", background="red", font=("Arial", 10, "bold"))

        # Khởi tạo visualization
        self.setup_plot()

    def setup_plot(self):
        """Khởi tạo visualization"""
        # Clear axis
        self.ax.clear()
        self.robot_parts = []

        # Lấy bounds từ vùng làm việc
        bounds = self.robot.workspace_bounds

        # Thiết lập giới hạn trục
        self.ax.set_xlim(bounds['left'] - 10, bounds['right'] + 10)
        self.ax.set_ylim(bounds['bottom'] - 10, bounds['top'] + 10)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_title("SCARA Robot Visualization")
        self.ax.grid(True)

        # Robot parameters
        x1 = self.robot.robot_params['x1']
        y1 = self.robot.robot_params['y1']
        x5 = self.robot.robot_params['x5']
        y5 = self.robot.robot_params['y5']
        L1 = self.robot.robot_params['L1']
        L4 = self.robot.robot_params['L4']

        # Vẽ giới hạn tầm với cho mỗi động cơ (tương tự song_song.py)
        reach_radius1 = self.robot.robot_params.get('reach_radius1', self.robot.r1_max)
        reach_radius2 = self.robot.robot_params.get('reach_radius2', self.robot.r2_max)

        reach_circle1 = Circle((x1, y1), reach_radius1, fill=False, color='blue',
                               alpha=0.2, linestyle='--')
        reach_circle2 = Circle((x5, y5), reach_radius2, fill=False, color='red',
                               alpha=0.2, linestyle='--')
        self.ax.add_patch(reach_circle1)
        self.ax.add_patch(reach_circle2)

        # Vùng làm việc cố định
        rect_width = bounds['right'] - bounds['left']
        rect_height = bounds['top'] - bounds['bottom']
        workspace_rect = Rectangle((bounds['left'], bounds['bottom']),
                                   rect_width, rect_height,
                                   linewidth=2, edgecolor='g', facecolor='none', alpha=0.7)
        self.ax.add_patch(workspace_rect)

        # Động cơ servo
        self.ax.plot(x1, y1, 'bo', markersize=8)
        self.ax.plot(x5, y5, 'ro', markersize=8)
        self.ax.text(x1 + 1, y1 + 1, "Motor Y", fontsize=8)
        self.ax.text(x5 + 1, y5 + 1, "Motor X", fontsize=8)

        # Cánh tay robot
        self.arm1 = FancyArrowPatch((0, 0), (L1, 0), lw=3, color='red', alpha=0.7, arrowstyle='-')
        self.arm2 = FancyArrowPatch((0, 0), (L1, 0), lw=3, color='blue', alpha=0.7, arrowstyle='-')
        self.arm3 = FancyArrowPatch((0, 0), (L1, 0), lw=3, color='blue', alpha=0.7, arrowstyle='-')
        self.arm4 = FancyArrowPatch((0, 0), (L1, 0), lw=3, color='green', alpha=0.7, arrowstyle='-')

        self.ax.add_patch(self.arm1)
        self.ax.add_patch(self.arm2)
        self.ax.add_patch(self.arm3)
        self.ax.add_patch(self.arm4)

        # Các khớp
        self.joint1 = Circle((0, 0), 1.2, color='darkblue', alpha=0.8)
        self.joint2 = Circle((0, 0), 1.2, color='darkblue', alpha=0.8)
        self.joint3 = Circle((0, 0), 1.2, color='darkblue', alpha=0.8)

        self.ax.add_patch(self.joint1)
        self.ax.add_patch(self.joint2)
        self.ax.add_patch(self.joint3)

        # Đầu bút vẽ
        self.pen_holder = Circle((0, 0), 1.5, color='black', alpha=0.6)
        self.ax.add_patch(self.pen_holder)

        # Vết vẽ
        self.trace, = self.ax.plot([], [], "r-", lw=1.5)

        # Text hiển thị
        self.text_theta1 = self.ax.text(bounds['left'] + 2, bounds['top'] - 2, "", fontsize=9)
        self.text_theta2 = self.ax.text(bounds['left'] + 2, bounds['top'] - 4, "", fontsize=9)
        self.pen_status_text = self.ax.text(bounds['left'] + 2, bounds['top'] - 6, "", fontsize=9)

        # Cập nhật hiển thị ban đầu
        self.fig.tight_layout()
        self.canvas.draw()

    def update_animation(self):
        """Cập nhật hiển thị robot"""
        # Lấy góc hiện tại
        theta1 = self.current_angles['theta1']
        theta2 = self.current_angles['theta2']

        try:
            # Tính vị trí các khớp sử dụng forward kinematics
            (x2, y2), (x3, y3), (x4, y4) = self.robot.forward_kinematics(theta1, theta2)

            # Thông số cơ bản
            x1 = self.robot.robot_params['x1']
            y1 = self.robot.robot_params['y1']
            x5 = self.robot.robot_params['x5']
            y5 = self.robot.robot_params['y5']

            # Cập nhật vị trí các cánh tay
            self.arm1.set_positions((x1, y1), (x2, y2))
            self.arm2.set_positions((x2, y2), (x3, y3))
            self.arm3.set_positions((x3, y3), (x4, y4))
            self.arm4.set_positions((x4, y4), (x5, y5))

            # Cập nhật vị trí các khớp
            self.joint1.center = (x2, y2)
            self.joint2.center = (x3, y3)
            self.joint3.center = (x4, y4)

            # Cập nhật vị trí pen
            self.pen_holder.center = (x3, y3)

            # Cập nhật vết vẽ nếu đang hạ bút
            if self.pen_is_down:
                if not self.current_segment:
                    self.current_segment = {'x': [x3], 'y': [y3]}
                else:
                    self.current_segment['x'].append(x3)
                    self.current_segment['y'].append(y3)

            # Cập nhật hiển thị vết
            all_x, all_y = [], []
            for segment in self.trace_segments:
                if segment and 'x' in segment:
                    all_x.extend(segment['x'])
                    all_y.extend(segment['y'])
                    all_x.append(None)  # Ngắt giữa các segment
                    all_y.append(None)

            if self.current_segment and 'x' in self.current_segment:
                all_x.extend(self.current_segment['x'])
                all_y.extend(self.current_segment['y'])

            self.trace.set_data(all_x, all_y)

            # Cập nhật text hiển thị
            self.text_theta1.set_text(f"θ1: {theta1:.2f}°")
            self.text_theta2.set_text(f"θ2: {theta2:.2f}°")
            self.pen_status_text.set_text(f"Bút: {'Hạ' if self.pen_is_down else 'Nâng'}")

            # Cập nhật canvas
            self.canvas.draw_idle()

        except Exception as e:
            self.log(f"Lỗi cập nhật animation: {str(e)}")

    def apply_delay(self):
        """Áp dụng thời gian chờ giữa các lệnh G-code"""
        try:
            delay = float(self.delay_var.get())
            if delay >= 0:
                self.gcode_delay = delay
                self.log(f"Đã thiết lập thời gian chờ giữa các lệnh: {delay} giây")
            else:
                messagebox.showwarning("Cảnh báo", "Thời gian chờ không thể âm!")
        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị thời gian không hợp lệ!")

    def update_ports(self):
        """Cập nhật danh sách cổng COM"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        """Kết nối hoặc ngắt kết nối với Arduino"""
        if self.is_connected:
            # Ngắt kết nối
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.is_connected = False
            self.connect_btn.configure(text="Kết nối")
            self.log("Đã ngắt kết nối")
        else:
            # Kết nối
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            if not port:
                messagebox.showerror("Lỗi", "Vui lòng chọn cổng COM!")
                return

            try:
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    time.sleep(0.5)

                self.log(f"Đang kết nối đến {port} với baud {baud}...")
                self.serial = serial.Serial(port, baud, timeout=2)
                time.sleep(2.0)  # Chờ Arduino khởi động

                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # Kiểm tra kết nối
                with self.serial_lock:
                    self.serial.write(b"status\n")

                time.sleep(1.0)
                responses = []
                while self.serial.in_waiting:
                    response = self.serial.readline().decode('utf-8', errors='replace').strip()
                    if response:
                        self.log(f"Nhận được: '{response}'")
                        responses.append(response)

                # Chấp nhận bất kỳ phản hồi nào
                if responses:
                    self.is_connected = True
                    self.connect_btn.configure(text="Ngắt kết nối")
                    self.log(f"✓ Kết nối thành công với {port}")
                    self.set_speed()
                    self.set_acceleration()
                    return

                self.log(f"❌ Không nhận được phản hồi từ Arduino!")
                self.serial.close()
                messagebox.showwarning("Cảnh báo", "Không nhận được phản hồi từ Arduino.")

            except Exception as e:
                self.log(f"❌ Lỗi kết nối: {str(e)}")
                messagebox.showerror("Lỗi", f"Không thể kết nối: {str(e)}")
                if self.serial and self.serial.is_open:
                    self.serial.close()

    def send_command(self, command, wait_for_response=True):
        """Gửi lệnh đến Arduino thông qua queue"""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.log("Chưa kết nối với Arduino!")
            return False

        self.command_queue.put((command, wait_for_response))
        return True

    def _execute_command(self, command, wait_for_response):
        """Thực thi lệnh với đồng bộ hóa"""
        try:
            is_gcode_running = self.gcode_running and not command.startswith("g")

            self.log(f"Gửi: {command}")

            # Gửi lệnh với lock
            cmd = command.strip() + '\n'
            with self.serial_lock:
                self.serial.write(cmd.encode('utf-8'))

            if wait_for_response:
                timeout = 1.0 if is_gcode_running else 3.0
                move_command = any(x in command for x in [',', 'G0', 'G1'])
                if move_command:
                    timeout *= 2

                responses = []
                start_time = time.time()
                while (time.time() - start_time < timeout):
                    with self.serial_lock:
                        if self.serial.in_waiting > 0:
                            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                            if response:
                                self.log(f"Nhận: {response}")
                                responses.append(response)

                                # Kiểm tra hoàn thành
                                if any(x in response for x in ["MOVE_COMPLETE", "READY", "OK"]):
                                    break
                    time.sleep(0.05)

                # Xử lý phản hồi
                for response in responses:
                    if "PEN_UP" in response:
                        self._update_pen_status(False)
                    elif "PEN_DOWN" in response:
                        self._update_pen_status(True)

                    # Tìm phản hồi về góc
                    match = re.search(r"MOVING_TO:(-?\d+\.?\d*),(-?\d+\.?\d*)", response)
                    if match:
                        theta1 = float(match.group(1))
                        theta2 = float(match.group(2))
                        self.update_position(theta1, theta2)

        except Exception as e:
            self.log(f"Lỗi: {str(e)}")

            # Xử lý lỗi kết nối
            if "PermissionError" in str(e):
                self.reconnect()

    def _update_pen_status(self, is_down):
        """Cập nhật trạng thái bút"""
        self.pen_is_down = is_down
        self.pen_label.configure(text="Hạ" if is_down else "Nâng")

        # Cập nhật animation
        self.update_animation()

    def move_to_angle(self):
        """Di chuyển đến góc"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino!")
            return

        try:
            theta1 = float(self.angle1_entry.get())
            theta2 = float(self.angle2_entry.get())

            # Kiểm tra giới hạn góc
            THETA1_MIN, THETA1_MAX = -120, 150
            THETA2_MIN, THETA2_MAX = 30, 300

            if not (THETA1_MIN <= theta1 <= THETA1_MAX):
                messagebox.showerror("Lỗi", f"Góc 1 nằm ngoài giới hạn: {THETA1_MIN}° đến {THETA1_MAX}°!")
                return False

            if not (THETA2_MIN <= theta2 <= THETA2_MAX):
                messagebox.showerror("Lỗi", f"Góc 2 nằm ngoài giới hạn: {THETA2_MIN}° đến {THETA2_MAX}°!")
                return False

            # Gửi lệnh và cập nhật vị trí hiện tại
            command = f"{theta1:.2f},{theta2:.2f}"
            self.log(f"Di chuyển đến góc: θ1={theta1:.2f}, θ2={theta2:.2f}")
            if self.send_command(command):
                self.update_position(theta1, theta2)
                return True

        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị góc không hợp lệ!")
            return False

    def move_to_xy(self):
        """Di chuyển đến tọa độ XY"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino!")
            return

        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())

            # Kiểm tra vùng làm việc
            if not self.robot.check_point_in_workspace(x, y):
                messagebox.showwarning("Cảnh báo", f"Vị trí ({x}, {y}) nằm ngoài vùng làm việc!")
                return

            # Tính góc với cấu hình hiện tại
            theta1, theta2, new_config = self.robot.inverse_kinematics(x, y, self.current_config)

            if theta1 is None or theta2 is None:
                messagebox.showwarning("Cảnh báo", f"Không thể tính góc cho vị trí ({x}, {y})!")
                return

            # Cập nhật cấu hình và góc hiển thị
            self.current_config = new_config
            self.log(f"Sử dụng cấu hình: {new_config}")
            self.angle1_entry.delete(0, tk.END)
            self.angle1_entry.insert(0, f"{theta1:.2f}")
            self.angle2_entry.delete(0, tk.END)
            self.angle2_entry.insert(0, f"{theta2:.2f}")

            # Gửi lệnh di chuyển
            command = f"{theta1:.2f},{theta2:.2f}"
            self.send_command(command)

        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị tọa độ không hợp lệ!")

    def update_position(self, theta1, theta2):
        """Cập nhật vị trí khi góc thay đổi"""
        self.current_angles['theta1'] = theta1
        self.current_angles['theta2'] = theta2

        # Tính vị trí XY với động học thuận
        try:
            joint1, joint3, joint2 = self.robot.forward_kinematics(theta1, theta2)
            x, y = joint3

            # Cập nhật vị trí hiện tại
            self.current_position['x'] = x
            self.current_position['y'] = y

            # Cập nhật hiển thị
            self.pos_label.configure(text=f"X={x:.2f}, Y={y:.2f}")
            self.angle_label.configure(text=f"θ1={theta1:.2f}°, θ2={theta2:.2f}°")

            # Cập nhật giá trị trường nhập liệu
            self.x_entry.delete(0, tk.END)
            self.x_entry.insert(0, f"{x:.2f}")
            self.y_entry.delete(0, tk.END)
            self.y_entry.insert(0, f"{y:.2f}")

            # Cập nhật visualization
            self.update_animation()

        except Exception as e:
            self.log(f"Lỗi cập nhật vị trí: {str(e)}")

    def pen_down(self):
        """Hạ bút"""
        if self.is_connected:
            self.send_command("d")
            # Bắt đầu segment vẽ mới
            x3 = self.current_position['x']
            y3 = self.current_position['y']
            self.current_segment = {'x': [x3], 'y': [y3]}

    def pen_up(self):
        """Nâng bút"""
        if self.is_connected:
            self.send_command("u")
            # Kết thúc segment vẽ hiện tại
            if self.current_segment and 'x' in self.current_segment and self.current_segment['x']:
                self.trace_segments.append(self.current_segment.copy())
                self.current_segment = {}

    def home(self):
        """Về home"""
        if self.is_connected:
            self.send_command("h", wait_for_response=True)
            # Cập nhật góc về 90 độ cho cả hai động cơ sau khi hoàn thành
            self.update_position(90, 90)

    def test_motors(self):
        """Kiểm tra động cơ"""
        if self.is_connected:
            self.send_command("test")

    def emergency_stop(self):
        """Dừng khẩn cấp"""
        if self.is_connected:
            self.send_command("disable")
            self.log("DỪNG KHẨN CẤP!")

            # Dừng G-code nếu đang chạy
            if self.gcode_running:
                self.stop_gcode()

    def set_speed(self):
        """Thiết lập tốc độ"""
        if self.is_connected:
            try:
                speed = int(self.speed_var.get())
                if speed > 0:
                    self.send_command(f"f{speed}")
            except ValueError:
                messagebox.showerror("Lỗi", "Giá trị tốc độ không hợp lệ!")

    def set_acceleration(self):
        """Thiết lập gia tốc"""
        if self.is_connected:
            try:
                accel = int(self.accel_var.get())
                if accel > 0:
                    self.send_command(f"a{accel}")
            except ValueError:
                messagebox.showerror("Lỗi", "Giá trị gia tốc không hợp lệ!")

    def log(self, message):
        """Ghi log"""
        timestamp = time.strftime("[%H:%M:%S]")
        self.log_text.insert(tk.END, f"{timestamp} {message}\n")
        self.log_text.see(tk.END)

        # Giới hạn số dòng
        if float(self.log_text.index('end-1c').split('.')[0]) > 500:
            self.log_text.delete(1.0, 2.0)

    def show_workspace(self):
        """Hiển thị vùng làm việc trên canvas"""
        self.log("Đang tạo bản đồ vùng làm việc...")

        # Lấy các điểm trong vùng làm việc
        points = self.robot.create_workspace_visualization()

        # Xóa canvas hiện tại và vẽ lại khung
        self.setup_plot()

        # Vẽ từng điểm trong vùng làm việc
        for x, y in points:
            self.ax.plot(x, y, 'b.', markersize=1)

        self.log(f"Hiển thị {len(points)} điểm trong vùng làm việc")
        self.canvas.draw()

    def load_gcode(self):
        """Tải và tối ưu file G-code"""
        file_path = filedialog.askopenfilename(
            title="Chọn file G-Code",
            filetypes=(("G-Code files", "*.gcode *.ngc *.nc"), ("All files", "*.*"))
        )

        if not file_path:
            return

        try:
            with open(file_path, "r") as f:
                content = f.read()

            # Tối ưu G-code
            optimized_gcode = []
            for line in content.split('\n'):
                line = line.strip()

                # Bỏ qua comment và dòng trống
                if not line or line.startswith(';'):
                    continue

                # Chỉ lấy phần lệnh trước comment
                if ';' in line:
                    line = line.split(';')[0].strip()

                # Thêm vào list tối ưu
                if line:
                    optimized_gcode.append(line)

            # Hiển thị nội dung
            self.gcode_text.delete(1.0, tk.END)
            self.gcode_text.insert(tk.END, content)

            # Lưu tên file và G-code đã tối ưu
            file_name = os.path.basename(file_path)
            self.file_label.config(text=file_name)
            self.gcode_lines = optimized_gcode
            self.gcode_total_lines = len(self.gcode_lines)

            # Hiển thị thông tin
            self.log(f"Đã tải và tối ưu file G-code: {file_name} ({self.gcode_total_lines} dòng)")

            # Kích hoạt nút chạy
            self.run_btn.config(state=tk.NORMAL)

        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể đọc file: {str(e)}")

    def run_gcode(self):
        """Bắt đầu chạy G-code"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino trước!")
            return

        if not self.gcode_lines:
            messagebox.showwarning("Cảnh báo", "Chưa tải file G-code!")
            return

        if self.gcode_running and self.gcode_paused:
            # Tiếp tục từ trạng thái tạm dừng
            self.gcode_paused = False
            self.pause_btn.config(text="Tạm dừng")
            self.log("Tiếp tục xử lý G-code")
            return

        # Nếu đang chạy thì không làm gì
        if self.gcode_running and not self.gcode_paused:
            return

        # Bắt đầu xử lý từ đầu
        self.gcode_running = True
        self.gcode_paused = False
        self.gcode_line_num = 0
        self.progress_var.set(0)
        self.progress_label.config(text="0% hoàn thành")

        # Gửi lệnh bắt đầu đến Arduino
        self.send_command("gstart", wait_for_response=True)

        # Kích hoạt các nút điều khiển
        self.pause_btn.config(text="Tạm dừng", state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        self.run_btn.config(state=tk.DISABLED)

        # Tạo và bắt đầu thread xử lý G-code
        gcode_thread = threading.Thread(target=self.process_gcode, daemon=True)
        gcode_thread.start()

        self.log("Bắt đầu xử lý G-code")

    def process_gcode(self):
        """Thread xử lý G-code"""
        try:
            while self.gcode_line_num < self.gcode_total_lines and self.gcode_running:
                # Kiểm tra tạm dừng
                if self.gcode_paused:
                    time.sleep(0.1)
                    continue

                # Lấy dòng tiếp theo
                line = self.gcode_lines[self.gcode_line_num].strip()

                # Bỏ qua comment và dòng trống
                if line and not line.startswith(";"):
                    # Highlight dòng hiện tại
                    self.highlight_current_line(self.gcode_line_num)

                    # Xử lý dòng G-code
                    if self.process_gcode_line(line):
                        # Chờ hoàn thành lệnh
                        self.wait_for_command_completion()

                # Tăng dòng và cập nhật tiến trình
                self.gcode_line_num += 1
                progress = int(100 * self.gcode_line_num / self.gcode_total_lines)
                self.root.after(0, lambda p=progress: self.update_progress(p))

                # Đợi một chút giữa các lệnh
                time.sleep(self.gcode_delay)

            # Hoàn thành
            if self.gcode_running:
                self.send_command("gend")
                self.root.after(0, self.complete_gcode)

        except Exception as e:
            self.log(f"Lỗi xử lý G-code: {str(e)}")
            self.send_command("gend")
            self.root.after(0, self.stop_gcode)

    def process_gcode_line(self, line):
        """Xử lý một dòng G-code"""
        # G0/G1: Di chuyển
        if line.upper().startswith(("G0", "G1")):
            # Phân tích tọa độ X, Y từ G-code
            x_cart = None
            y_cart = None
            z_value = None

            # Tách tham số
            parts = line.upper().split()
            gcode_type = parts[0]  # G0 hoặc G1
            other_params = []

            for part in parts[1:]:
                if part.startswith('X'):
                    try:
                        x_cart = float(part[1:])
                    except:
                        pass
                elif part.startswith('Y'):
                    try:
                        y_cart = float(part[1:])
                    except:
                        pass
                elif part.startswith('Z'):
                    try:
                        z_value = float(part[1:])
                        # Xử lý Z để điều khiển bút
                        if z_value < 0 and not self.pen_is_down:
                            self.pen_down()
                        elif z_value > 0 and self.pen_is_down:
                            self.pen_up()
                    except:
                        pass
                else:
                    other_params.append(part)

            # Nếu có tọa độ X, Y
            if x_cart is not None and y_cart is not None:
                self.log(f"Tọa độ Cartesian: X={x_cart}, Y={y_cart}")

                # Kiểm tra điểm có nằm trong vùng làm việc không
                if not self.robot.check_point_in_workspace(x_cart, y_cart):
                    self.log(f"⚠️ Điểm ({x_cart}, {y_cart}) nằm ngoài vùng làm việc")
                    return False

                # Tính góc với inverse kinematics
                theta1, theta2, config = self.robot.inverse_kinematics(x_cart, y_cart, self.current_config)

                if theta1 is not None and theta2 is not None:
                    # Cập nhật cấu hình hiện tại
                    self.current_config = config

                    # Tạo lệnh góc và gửi đi
                    gcode_cmd = f"{gcode_type} X{theta1:.2f} Y{theta2:.2f}"
                    if other_params:
                        gcode_cmd += " " + " ".join(other_params)

                    self.send_command(gcode_cmd)
                    return True
                else:
                    self.log(f"❌ Không thể tính góc cho vị trí X={x_cart}, Y={y_cart}")
                    return False
            else:
                # Gửi lệnh nguyên mẫu
                self.send_command(line)
                return True

        # Xử lý các lệnh G-code khác
        elif line.upper().startswith("G28"):  # Home
            self.send_command("h")
            return True
        elif line.upper().startswith(("M3", "M03")):  # Hạ bút
            self.pen_down()
            return True
        elif line.upper().startswith(("M5", "M05")):  # Nâng bút
            self.pen_up()
            return True
        elif line.upper().startswith(("G90")):  # Chế độ tọa độ tuyệt đối
            self.send_command(line)
            return True
        elif line.upper().startswith(("G91")):  # Chế độ tọa độ tương đối
            self.send_command(line)
            return True
        else:
            # Gửi lệnh khác
            self.send_command(line)
            return True

        return False

    def wait_for_command_completion(self):
        """Chờ cho đến khi hàng đợi lệnh rỗng"""
        while (not self.command_queue.empty()) or self.command_processing:
            time.sleep(0.1)
            if not self.gcode_running or self.gcode_paused:
                break

    def highlight_current_line(self, line_num):
        """Highlight dòng G-code hiện tại"""
        self.root.after(0, lambda: self.gcode_text.tag_remove("current", "1.0", tk.END))

        # Tính toán vị trí dòng
        pos = f"{line_num + 1}.0"
        end_pos = f"{line_num + 1}.end"

        # Highlight dòng
        self.root.after(0, lambda: self.gcode_text.tag_add("current", pos, end_pos))
        self.root.after(0, lambda: self.gcode_text.tag_config("current", background="yellow"))

        # Scroll để hiển thị dòng hiện tại
        self.root.after(0, lambda: self.gcode_text.see(pos))

    def update_progress(self, progress):
        """Cập nhật thanh tiến trình"""
        self.progress_var.set(progress)
        self.progress_label.config(text=f"{progress}% hoàn thành")

        # Cập nhật log
        if progress % 10 == 0:
            self.log(f"Tiến trình G-code: {progress}%")

    def pause_gcode(self):
        """Tạm dừng/tiếp tục xử lý G-code"""
        if not self.gcode_running:
            return

        self.gcode_paused = not self.gcode_paused

        if self.gcode_paused:
            self.pause_btn.config(text="Tiếp tục")
            self.log("Tạm dừng xử lý G-code")
        else:
            self.pause_btn.config(text="Tạm dừng")
            self.log("Tiếp tục xử lý G-code")

    def stop_gcode(self):
        """Dừng xử lý G-code"""
        if not self.gcode_running:
            return

        self.gcode_running = False
        self.gcode_paused = False

        # Gửi lệnh kết thúc đến Arduino
        self.send_command("gend", wait_for_response=True)

        # Reset UI
        self.run_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)

        # Xóa highlight
        self.gcode_text.tag_remove("current", "1.0", tk.END)

        self.log("Đã dừng xử lý G-code")

    def complete_gcode(self):
        """Hoàn thành xử lý G-code"""
        self.gcode_running = False

        # Gửi lệnh kết thúc đến Arduino
        self.send_command("gend", wait_for_response=True)

        # Reset UI
        self.run_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)

        # Xóa highlight
        self.gcode_text.tag_remove("current", "1.0", tk.END)

        # Cập nhật tiến trình
        self.progress_var.set(100)
        self.progress_label.config(text="100% hoàn thành")

        self.log("Đã hoàn thành xử lý G-code")

    def reconnect(self):
        """Kết nối lại với Arduino"""
        try:
            self.log("Đang thử kết nối lại...")

            if self.serial and self.serial.is_open:
                self.serial.close()

            time.sleep(1.0)  # Đợi port được giải phóng

            port = self.port_var.get()
            baud = int(self.baud_var.get())
            self.serial = serial.Serial(port, baud, timeout=2)

            # Đợi Arduino khởi động lại
            time.sleep(2.0)

            # Kiểm tra kết nối
            with self.serial_lock:
                self.serial.write(b"status\n")
            time.sleep(0.5)

            response = ""
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='replace').strip()

            if response:
                self.is_connected = True
                self.log("✓ Kết nối lại thành công")
                return True

            self.log(f"❌ Kết nối lại thất bại. Phản hồi: {response}")
            self.is_connected = False
            return False

        except Exception as e:
            self.log(f"❌ Lỗi kết nối lại: {str(e)}")
            self.is_connected = False
            return False


def main():
    root = tk.Tk()
    app = ScaraGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()