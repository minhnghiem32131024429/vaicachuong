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

class ScaraRobot:
    def __init__(self):
        # Thông số robot SCARA
        self.robot_params = {
            'x1': 0.0,  # Tọa độ X motor 1 (motor Y)
            'y1': 0.0,  # Tọa độ Y motor 1
            'x5': -20.0,  # Tọa độ X motor 2 (motor X) - cách motor Y 20cm
            'y5': 0.0,  # Tọa độ Y motor 2
            'L1': 15.0,  # Chiều dài cánh tay 1 từ motor 1
            'L2': 25.0,  # Chiều dài cánh tay 2 từ joint 1 đến đầu vẽ
            'L3': 25.0,  # Chiều dài cánh tay 3 từ đầu vẽ đến joint 2
            'L4': 15.0,  # Chiều dài cánh tay 4 từ joint 2 đến motor 2
        }
        self.home_position = {'x': -10.0, 'y': 38.0}  # Vị trí home
        self.calculate_workspace()

    def calculate_workspace(self):
        """Tính vùng làm việc robot chính xác hơn"""
        # Thông số
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Tính bán kính vùng làm việc từ mỗi động cơ
        r1_min = max(0, abs(L1 - L2)) + 0.5  # Bán kính tối thiểu từ motor 1
        r1_max = (L1 + L2) - 0.5  # Bán kính tối đa từ motor 1

        r2_min = max(0, abs(L4 - L3)) + 0.5  # Bán kính tối thiểu từ motor 2
        r2_max = (L4 + L3) - 0.5  # Bán kính tối đa từ motor 2

        # Tính hình chữ nhật bao quanh khu vực làm việc
        self.workspace_bounds = {
            'left': min(x1 - r1_max, x5 - r2_max),
            'right': max(x1 + r1_max, x5 + r2_max),
            'bottom': min(y1 - r1_max, y5 - r2_max),
            'top': max(y1 + r1_max, y5 + r2_max)
        }

        # Lưu bán kính cho kiểm tra chi tiết
        self.r1_min, self.r1_max = r1_min, r1_max
        self.r2_min, self.r2_max = r2_min, r2_max

    def forward_kinematics(self, theta1, theta2):
        """Tính toán vị trí các khớp dựa trên góc quay"""
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

        three_to_h = np.sqrt(L2 ** 2 - two_to_h ** 2)

        x3 = xh + (three_to_h / two_to_four) * (y4 - y2)
        y3 = yh - (three_to_h / two_to_four) * (x4 - x2)

        return (x2, y2), (x3, y3), (x4, y4)

    def inverse_kinematics(self, x3, y3, current_config=None):
        """
        Tính toán góc quay của các khớp dựa trên vị trí đầu vẽ
        Phiên bản đã điều chỉnh dấu cho phù hợp với hệ thống thực tế

        Args:
            x3, y3: Tọa độ điểm đầu vẽ
            current_config: Cấu hình hiện tại ('elbow_up', 'elbow_down', hoặc None)

        Returns:
            (theta1, theta2, config) hoặc (None, None, None) nếu không tìm được giải pháp
        """
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
        THETA1_MIN, THETA1_MAX = -120, 150  # Giới hạn góc động cơ 1
        THETA2_MIN, THETA2_MAX = 30, 300  # Giới hạn góc động cơ 2

        # Tính khoảng cách từ điểm vẽ đến các động cơ
        L13_sq = (x3 - x1) ** 2 + (y3 - y1) ** 2
        L13 = np.sqrt(L13_sq)
        L53_sq = (x3 - x5) ** 2 + (y3 - y5) ** 2
        L53 = np.sqrt(L53_sq)

        # Kiểm tra điểm có nằm trong phạm vi không - thêm biên an toàn 0.1cm
        min_dist1 = abs(L1 - L2) + 0.1
        max_dist1 = L1 + L2 - 0.1
        min_dist2 = abs(L4 - L3) + 0.1
        max_dist2 = L4 + L3 - 0.1

        if (L13 < min_dist1 or L13 > max_dist1 or L53 < min_dist2 or L53 > max_dist2):
            return None, None, None  # Điểm nằm ngoài phạm vi

        # ===== Tính góc cho motor 1 (Motor Y) =====
        cos_alpha1 = (L1 ** 2 + L13_sq - L2 ** 2) / (2 * L1 * L13)

        # Xử lý lỗi số học - clip để tránh lỗi domain error
        if abs(cos_alpha1) > 1.0 + 1e-10:
            return None, None, None

        cos_alpha1 = np.clip(cos_alpha1, -1.0, 1.0)
        alpha1 = np.arccos(cos_alpha1)
        beta1 = np.arctan2(y3 - y1, x3 - x1)

        # ĐẢO DẤU: Đổi dấu cho phù hợp với hệ thống thực tế
        # Khuỷu hướng xuống và hướng lên được đổi để phù hợp với thực tế
        theta1_elbow_down = np.degrees(beta1 + alpha1)  # Khuỷu hướng xuống (đảo dấu)
        theta1_elbow_up = np.degrees(beta1 - alpha1)  # Khuỷu hướng lên (đảo dấu)

        # ===== Tính góc cho motor 2 (Motor X) =====
        cos_alpha5 = (L4 ** 2 + L53_sq - L3 ** 2) / (2 * L4 * L53)

        if abs(cos_alpha5) > 1.0 + 1e-10:
            return None, None, None

        cos_alpha5 = np.clip(cos_alpha5, -1.0, 1.0)
        alpha5 = np.arccos(cos_alpha5)
        beta5 = np.arctan2(y3 - y5, x3 - x5)

        # ĐẢO DẤU: Đổi dấu cho motor X
        theta2_elbow_down = np.degrees(beta5 - alpha5)  # Khuỷu hướng xuống (đảo dấu)
        theta2_elbow_up = np.degrees(beta5 + alpha5)  # Khuỷu hướng lên (đảo dấu)

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
            return None, None, None  # Không có giải pháp thỏa mãn giới hạn góc

        # Chọn giải pháp phù hợp
        if current_config is not None and len(solutions) > 1:
            # Nếu có cấu hình hiện tại, ưu tiên giữ nguyên cấu hình
            for config, theta1, theta2 in solutions:
                if config == current_config:
                    return theta1, theta2, config

        # Nếu không có cấu hình hiện tại hoặc không thể giữ nguyên cấu hình
        # Chọn giải pháp đầu tiên
        config, theta1, theta2 = solutions[0]
        return theta1, theta2, config

    def check_point_in_workspace(self, x, y):
        """Kiểm tra nhanh xem điểm có nằm trong vùng làm việc không"""
        # Thông số
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']

        # Kiểm tra khoảng cách đến động cơ - tính toán nhanh
        d1_sq = (x - x1) ** 2 + (y - y1) ** 2  # Bình phương khoảng cách đến motor 1
        d2_sq = (x - x5) ** 2 + (y - y5) ** 2  # Bình phương khoảng cách đến motor 2

        # Kiểm tra điều kiện bán kính - tránh tính căn bậc 2
        r1_min_sq = self.r1_min ** 2
        r1_max_sq = self.r1_max ** 2
        r2_min_sq = self.r2_min ** 2
        r2_max_sq = self.r2_max ** 2

        # Kiểm tra điều kiện bán kính
        if (r1_min_sq <= d1_sq <= r1_max_sq and
                r2_min_sq <= d2_sq <= r2_max_sq):
            # Chỉ tính động học ngược nếu cần
            theta1, theta2, _ = self.inverse_kinematics(x, y)
            return theta1 is not None and theta2 is not None

        return False

class ScaraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Controller")
        self.root.geometry("1000x700")
        self.current_config = None  # Track the current elbow configuration

        # Khởi tạo robot và Serial
        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 0.0, 'theta2': 0.0}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.pen_is_down = False

        # Biến cho G-code
        self.gcode_lines = []
        self.gcode_running = False
        self.gcode_paused = False
        self.gcode_thread = None
        self.gcode_line_num = 0
        self.gcode_total_lines = 0
        self.gcode_queue = queue.Queue()

        # Thêm queue cho lệnh
        self.command_queue = queue.Queue()
        self.command_processing = False

        # Thêm xử lý lệnh
        self.process_command_thread = threading.Thread(target=self._process_command_queue, daemon=True)
        self.process_command_thread.start()

        # Tạo giao diện
        self.create_widgets()
        self.update_canvas()

        # Cập nhật danh sách cổng COM
        self.update_ports()

    # Thêm phương thức xử lý hàng đợi lệnh
    def _process_command_queue(self):
        """Xử lý hàng đợi lệnh trong luồng riêng"""
        while True:
            try:
                if not self.command_processing and not self.command_queue.empty():
                    command, wait_response = self.command_queue.get()
                    self.command_processing = True
                    self._execute_command(command, wait_response)
                    self.command_processing = False
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Error in command queue: {str(e)}")
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

        # --- Panel Điều Khiển ---
        # Panel kết nối
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

        # Panel điều khiển thủ công
        manual_frame = ttk.LabelFrame(control_frame, text="Điều khiển thủ công", padding=5)
        manual_frame.pack(fill=tk.X, pady=5)

        # Thêm sau Panel điều khiển thủ công:
        # Panel GCode
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

        # Panel thông báo
        log_frame = ttk.LabelFrame(control_frame, text="Nhật ký", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = tk.Text(log_frame, height=10, wrap=tk.WORD)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        log_scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scroll.set)

        # --- Panel Hiển thị ---
        # Vẽ robot
        self.canvas = tk.Canvas(canvas_frame, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

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

    def update_ports(self):
        """Cập nhật danh sách cổng COM"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        """Kết nối hoặc ngắt kết nối với Arduino - phiên bản cải tiến"""
        if self.is_connected:
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.is_connected = False
            self.connect_btn.configure(text="Kết nối")
            self.log("Đã ngắt kết nối")
        else:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            if not port:
                messagebox.showerror("Lỗi", "Vui lòng chọn cổng COM!")
                return

            try:
                # Đóng kết nối cũ nếu có
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    time.sleep(0.5)

                # Tạo kết nối mới
                self.log(f"Đang kết nối đến {port} với baud {baud}...")
                self.serial = serial.Serial(port, baud, timeout=2)

                # Chờ Arduino khởi động
                time.sleep(2.0)

                # Xóa bộ đệm
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # Kiểm tra kết nối - CẢI TIẾN: Chấp nhận nhiều loại phản hồi
                self.log("Gửi lệnh kiểm tra...")
                self.serial.write(b"status\n")

                # Đọc hết tất cả phản hồi hiện có
                time.sleep(1.0)
                responses = []
                while self.serial.in_waiting:
                    response = self.serial.readline().decode('utf-8', errors='replace').strip()
                    if response:
                        self.log(f"Nhận được: '{response}'")
                        responses.append(response)

                # THAY ĐỔI QUAN TRỌNG: Chấp nhận bất kỳ phản hồi nào coi là kết nối thành công
                if len(responses) > 0:
                    self.is_connected = True
                    self.connect_btn.configure(text="Ngắt kết nối")
                    self.log(f"✓ Kết nối thành công với {port}")

                    # Thiết lập tốc độ và gia tốc
                    self.set_speed()
                    self.set_acceleration()
                    return

                # Không nhận được phản hồi nào
                self.log(f"❌ Không nhận được phản hồi từ Arduino!")
                self.serial.close()
                messagebox.showwarning("Cảnh báo",
                                       "Không nhận được phản hồi từ Arduino.\nKiểm tra lại cổng COM và firmware.")

            except Exception as e:
                self.log(f"❌ Lỗi kết nối: {str(e)}")
                messagebox.showerror("Lỗi", f"Không thể kết nối: {str(e)}")
                if self.serial and self.serial.is_open:
                    self.serial.close()

    def _connect_serial(self):
        """Thực hiện kết nối trong luồng riêng"""
        port = self.port_var.get()
        baud = int(self.baud_var.get())

        if not port:
            self.root.after(0, lambda: messagebox.showerror("Lỗi", "Vui lòng chọn cổng COM!"))
            return

        try:
            # Đóng kết nối cũ nếu có
            if self.serial and self.serial.is_open:
                self.serial.close()
                time.sleep(0.5)

            # Tạo kết nối mới
            self.root.after(0, lambda: self.log(f"Đang kết nối đến {port} với baud {baud}..."))
            self.serial = serial.Serial(port, baud, timeout=1)  # Giảm timeout xuống 1 giây

            # Chờ Arduino khởi động
            time.sleep(1.5)  # Giảm thời gian chờ

            # Xóa bộ đệm
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Kiểm tra kết nối
            self.root.after(0, lambda: self.log("Gửi lệnh kiểm tra..."))
            self.serial.write(b"status\n")

            # Đọc phản hồi
            response_text = ""
            attempts = 0
            while attempts < 2:  # Giảm số lần thử
                if self.serial.in_waiting:
                    response = self.serial.readline().decode('utf-8', errors='replace').strip()
                    self.root.after(0, lambda r=response: self.log(f"Nhận được: '{r}'"))
                    response_text += response

                    # Kiểm tra phản hồi hợp lệ
                    if any(x in response for x in ["READY", "SCARA_READY", "OK"]):
                        self.is_connected = True
                        self.root.after(0, lambda: self.connect_btn.configure(text="Ngắt kết nối"))
                        self.root.after(0, lambda: self.log(f"✓ Kết nối thành công với {port}"))

                        # Thiết lập tốc độ và gia tốc
                        time.sleep(0.1)
                        self.serial.write(f"f{self.speed_var.get()}\n".encode('utf-8'))
                        time.sleep(0.1)
                        self.serial.write(f"a{self.accel_var.get()}\n".encode('utf-8'))
                        return
                else:
                    attempts += 1
                    self.root.after(0, lambda a=attempts: self.log(f"Không nhận được phản hồi, thử lại... ({a}/2)"))
                    self.serial.write(b"status\n")
                    time.sleep(0.5)  # Giảm thời gian chờ

            # Xử lý khi kết nối thất bại
            self.root.after(0, lambda: self.log(f"❌ Không nhận được phản hồi hợp lệ từ Arduino!"))
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.root.after(0, lambda: messagebox.showwarning("Cảnh báo",
                                                              "Không nhận được phản hồi đúng từ Arduino.\nKiểm tra lại cổng COM và firmware."))

        except Exception as e:
            self.root.after(0, lambda: self.log(f"❌ Lỗi kết nối: {str(e)}"))
            self.root.after(0, lambda e=str(e): messagebox.showerror("Lỗi", f"Không thể kết nối: {e}"))
            if self.serial and self.serial.is_open:
                self.serial.close()

    def convert_simulation_to_real_angle(self, sim_angle):
        """Chuyển đổi từ góc trong mô phỏng sang góc thực tế
        - Mô phỏng: 0° là hướng âm trục X, góc tăng ngược chiều kim đồng hồ
        - Thực tế: 0° là hướng dương trục X, góc tăng theo chiều kim đồng hồ
        """
        # Xoay 180° và đảo chiều
        real_angle = (-sim_angle + 180) % 360

        return real_angle

    def send_move_command(self, theta1, theta2):
        """Gửi lệnh di chuyển đến Arduino với quy ước góc đã chỉnh sửa"""
        if not self.is_connected:
            self.log("Chưa kết nối!")
            return False

        try:
            # Chuyển đổi góc từ mô phỏng sang góc thực tế
            # Đảo vị trí theta1/theta2 vì góc 1 điều khiển động cơ Y, góc 2 điều khiển động cơ X
            real_theta1 = self.convert_simulation_to_real_angle(theta2)  # theta2 (mô phỏng) -> góc motor X (thực tế)
            real_theta2 = self.convert_simulation_to_real_angle(theta1)  # theta1 (mô phỏng) -> góc motor Y (thực tế)

            # Đảm bảo động cơ không quay vào vùng dưới trục X (nửa dưới)
            if real_theta1 > 180 and real_theta1 < 360:
                self.log(f"❌ Góc motor X ({real_theta1:.2f}°) nằm trong vùng cấm (>180° và <360°)")
                messagebox.showerror("Lỗi", "Góc motor X nằm trong vùng không cho phép!")
                return False

            if real_theta2 > 180 and real_theta2 < 360:
                self.log(f"❌ Góc motor Y ({real_theta2:.2f}°) nằm trong vùng cấm (>180° và <360°)")
                messagebox.showerror("Lỗi", "Góc motor Y nằm trong vùng không cho phép!")
                return False

            # Gửi lệnh với góc đã điều chỉnh và hoán đổi vị trí
            command = f"{real_theta2:.2f},{real_theta1:.2f}"
            self.log(f"Gửi lệnh: {command} (Từ góc mô phỏng: θ1={theta1:.2f}, θ2={theta2:.2f})")
            self.send_command(command)
            return True
        except Exception as e:
            self.log(f"❌ Lỗi gửi lệnh: {str(e)}")
            return False

    def send_command(self, command, wait_for_response=True):
        """Gửi lệnh đến Arduino - phiên bản không chặn UI"""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.log("Chưa kết nối với Arduino!")
            return False

        # Tạo và bắt đầu luồng cho giao tiếp serial
        command_thread = threading.Thread(
            target=self._execute_command,
            args=(command, wait_for_response),
            daemon=True
        )
        command_thread.start()
        return True

    def _execute_command(self, command, wait_for_response):
        """Thực thi lệnh trong luồng riêng"""
        try:
            is_gcode_running = self.gcode_running and not command.startswith("g")
            verbose = not is_gcode_running

            if verbose:
                self.root.after(0, lambda: self.log(f"Gửi: {command}"))

            # Gửi lệnh
            cmd = command.strip() + '\n'
            self.serial.write(cmd.encode('utf-8'))

            if wait_for_response:
                # Giảm thời gian chờ xuống
                timeout = 0.5 if is_gcode_running else 2.0

                responses = []
                start_time = time.time()
                while (time.time() - start_time < timeout):
                    if self.serial.in_waiting > 0:
                        response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            if verbose:
                                self.root.after(0, lambda r=response: self.log(f"Nhận: {r}"))
                            responses.append(response)

                            # Kiểm tra hoàn thành
                            if "MOVE_COMPLETE" in response or "READY" in response or "OK" in response:
                                break
                    else:
                        time.sleep(0.01)  # Chờ ngắn hơn

                # Cập nhật trạng thái trong luồng UI
                for response in responses:
                    if "PEN_UP" in response:
                        self.root.after(0, lambda: self._update_pen_status(False))
                    elif "PEN_DOWN" in response:
                        self.root.after(0, lambda: self._update_pen_status(True))

                    # Tìm phản hồi về góc
                    match = re.search(r"MOVING_TO:(-?\d+\.?\d*),(-?\d+\.?\d*)", response)
                    if match:
                        theta1 = float(match.group(1))
                        theta2 = float(match.group(2))
                        self.root.after(0, lambda t1=theta1, t2=theta2: self.update_position(t1, t2))

        except Exception as e:
            self.root.after(0, lambda: self.log(f"Lỗi: {str(e)}"))

            # Xử lý lỗi kết nối
            if "PermissionError" in str(e):
                self.root.after(0, self.reconnect)

    def _update_pen_status(self, is_down):
        """Cập nhật trạng thái bút an toàn trong luồng UI"""
        self.pen_is_down = is_down
        self.pen_label.configure(text="Hạ" if is_down else "Nâng")

    def move_to_angle(self):
        """Di chuyển đến góc"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino!")
            return

        try:
            theta1 = float(self.angle1_entry.get())  # Góc 1 - mong muốn điều khiển X
            theta2 = float(self.angle2_entry.get())  # Góc 2 - mong muốn điều khiển Y

            # ===== CHÚ Ý: ĐIỂM QUAN TRỌNG =====
            # Arduino nhận lệnh theo định dạng: "X_angle,Y_angle"
            # Do đó gửi: theta1 (góc X), theta2 (góc Y)
            command = f"{theta1:.2f},{theta2:.2f}"

            # Log chi tiết để debug
            self.log(f"Góc vào: θ1(X)={theta1:.2f}, θ2(Y)={theta2:.2f}")
            self.log(f"Gửi lệnh: {command}")

            # Gửi lệnh
            self.send_command(command)

            # Cập nhật vị trí
            self.update_position(theta1, theta2)

        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị góc không hợp lệ!")

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

            # Cập nhật cấu hình hiện tại
            self.current_config = new_config
            self.log(f"Sử dụng cấu hình: {new_config}")

            # Cập nhật giá trị cho góc
            self.angle1_entry.delete(0, tk.END)
            self.angle1_entry.insert(0, f"{theta1:.2f}")
            self.angle2_entry.delete(0, tk.END)
            self.angle2_entry.insert(0, f"{theta2:.2f}")

            # Di chuyển
            self.send_command(f"{theta1:.2f},{theta2:.2f}")

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

            # Vẽ lại robot
            self.update_canvas()

        except Exception as e:
            self.log(f"Lỗi cập nhật vị trí: {str(e)}")

    def pen_up(self):
        """Nâng bút"""
        if self.is_connected:
            self.send_command("u")

    def pen_down(self):
        """Hạ bút"""
        if self.is_connected:
            self.send_command("d")

    def home(self):
        """Về home"""
        if self.is_connected:
            # Thay vì về góc 0,0 thì về góc 90,90 - cánh tay thẳng đứng
            self.send_command("h")

    def test_motors(self):
        """Kiểm tra động cơ"""
        if self.is_connected:
            self.send_command("test")

    def emergency_stop(self):
        """Dừng khẩn cấp"""
        if self.is_connected:
            self.send_command("disable")
            self.log("DỪNG KHẨN CẤP!")

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

        # Giới hạn số dòng (giữ 500 dòng gần nhất)
        if float(self.log_text.index('end-1c').split('.')[0]) > 500:
            self.log_text.delete(1.0, 2.0)

    def update_canvas(self):
        """Vẽ lại robot trên canvas với sự đảo ngược hoàn toàn để khớp thực tế"""
        # Xóa canvas
        self.canvas.delete("all")

        canvas_width = self.canvas.winfo_width() or 500
        canvas_height = self.canvas.winfo_height() or 400

        # Tỉ lệ chuyển đổi từ cm sang pixel
        scale = min(canvas_width, canvas_height) / 100

        # Tâm canvas
        cx = canvas_width / 2
        cy = canvas_height / 2

        # --- Vẽ Grid ---
        grid_spacing = max(5 * scale, 1)
        for x in range(int(-60 * scale), int(60 * scale), max(int(grid_spacing), 1)):
            self.canvas.create_line(cx + x, 0, cx + x, canvas_height, fill="#EEEEEE", width=1)
            if x % max(int(10 * scale), 1) == 0 and x != 0:
                cm_value = int(x / scale)
                self.canvas.create_text(cx + x, cy + 10, text=f"{cm_value}", fill="gray", font=("Arial", 8))

        for y in range(int(-60 * scale), int(60 * scale), max(int(grid_spacing), 1)):
            self.canvas.create_line(0, cy - y, canvas_width, cy - y, fill="#EEEEEE",
                                    width=1)  # FLIP: cy - y thay vì cy + y
            if y % max(int(10 * scale), 1) == 0 and y != 0:
                cm_value = int(y / scale)
                self.canvas.create_text(cx - 10, cy - y, text=f"{cm_value}", fill="gray",
                                        font=("Arial", 8))  # FLIP: cy - y

        # Vẽ trục tọa độ
        self.canvas.create_line(0, cy, canvas_width, cy, fill="gray", width=1)
        self.canvas.create_line(cx, 0, cx, canvas_height, fill="gray", width=1)
        self.canvas.create_text(cx + 55, cy + 10, text="X", fill="gray")
        self.canvas.create_text(cx - 10, cy - 55, text="Y", fill="gray")  # FLIP: cy - 55

        # Vẽ vùng làm việc - Flip Y coordinate
        bounds = self.robot.workspace_bounds
        x1 = cx + bounds['left'] * scale
        y1 = cy - bounds['top'] * scale  # FLIP: dùng top thay vì bottom
        x2 = cx + bounds['right'] * scale
        y2 = cy - bounds['bottom'] * scale  # FLIP: dùng bottom thay vì top
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="lightgray", dash=(2, 2))

        # Thông số robot
        robot = self.robot.robot_params

        # --- THAY ĐỔI QUAN TRỌNG: Hoán đổi vai trò động cơ ---
        # Vị trí motor - đổi vị trí để phản ánh đúng chức năng
        motor_x_x = cx + robot['x5'] * scale  # Motor X (cũ là motor 2/X5)
        motor_x_y = cy - robot['y5'] * scale
        motor_y_x = cx + robot['x1'] * scale  # Motor Y (cũ là motor 1/X1)
        motor_y_y = cy - robot['y1'] * scale

        # Vẽ motors
        motor_radius = 6
        self.canvas.create_oval(motor_x_x - motor_radius, motor_x_y - motor_radius,
                                motor_x_x + motor_radius, motor_x_y + motor_radius,
                                fill="blue", outline="black")
        self.canvas.create_text(motor_x_x, motor_x_y - 15, text="Motor X")

        self.canvas.create_oval(motor_y_x - motor_radius, motor_y_y - motor_radius,
                                motor_y_x + motor_radius, motor_y_y + motor_radius,
                                fill="blue", outline="black")
        self.canvas.create_text(motor_y_x, motor_y_y - 15, text="Motor Y")

        # Tính vị trí các khớp từ góc hiện tại
        try:
            theta1 = self.current_angles['theta1']
            theta2 = self.current_angles['theta2']
            (x2, y2), (x3, y3), (x4, y4) = self.robot.forward_kinematics(theta1, theta2)

            # THAY ĐỔI QUAN TRỌNG: Đảo ngược vai trò của các cánh tay
            # Trong mô phỏng mới:
            # - Cánh tay 1 đi từ Motor Y đến khớp 1
            # - Cánh tay 2 đi từ khớp 1 đến đầu bút
            # - Cánh tay 3 đi từ đầu bút đến khớp 2
            # - Cánh tay 4 đi từ khớp 2 đến Motor X

            # Chuyển đổi tọa độ
            joint1_x = cx + x2 * scale
            joint1_y = cy - y2 * scale  # Vẫn giữ hệ tọa độ Y hướng lên
            pen_x = cx + x3 * scale
            pen_y = cy - y3 * scale
            joint2_x = cx + x4 * scale
            joint2_y = cy - y4 * scale

            # Vẽ cánh tay (1 đến 4)
            self.canvas.create_line(motor_y_x, motor_y_y, joint1_x, joint1_y, fill="red",
                                    width=3)  # Cánh tay 1: Motor Y -> khớp 1
            self.canvas.create_line(joint1_x, joint1_y, pen_x, pen_y, fill="green",
                                    width=3)  # Cánh tay 2: khớp 1 -> đầu bút
            self.canvas.create_line(pen_x, pen_y, joint2_x, joint2_y, fill="green",
                                    width=3)  # Cánh tay 3: đầu bút -> khớp 2
            self.canvas.create_line(joint2_x, joint2_y, motor_x_x, motor_x_y, fill="red",
                                    width=3)  # Cánh tay 4: khớp 2 -> Motor X

            # Vẽ các khớp
            joint_radius = 4
            self.canvas.create_oval(joint1_x - joint_radius, joint1_y - joint_radius,
                                    joint1_x + joint_radius, joint1_y + joint_radius,
                                    fill="orange")
            self.canvas.create_oval(joint2_x - joint_radius, joint2_y - joint_radius,
                                    joint2_x + joint_radius, joint2_y + joint_radius,
                                    fill="orange")

            # Vẽ đầu bút
            pen_color = "green" if not self.pen_is_down else "red"
            self.canvas.create_oval(pen_x - 6, pen_y - 6, pen_x + 6, pen_y + 6, fill=pen_color)
            self.canvas.create_text(pen_x, pen_y - 15, text="Pen")

            # Vẽ home position
            home_x = self.robot.home_position['x']
            home_y = self.robot.home_position['y']
            home_px = cx + home_x * scale
            home_py = cy - home_y * scale  # Giữ hệ tọa độ Y hướng lên
            self.canvas.create_text(home_px, home_py, text="H", fill="blue")
            self.canvas.create_oval(home_px - 3, home_py - 3, home_px + 3, home_py + 3,
                                    fill="blue", outline="blue")

        except Exception as e:
            self.log(f"Lỗi vẽ robot: {str(e)}")

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

        # Kích hoạt nút tạm dừng và dừng
        self.pause_btn.config(text="Tạm dừng", state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        self.run_btn.config(state=tk.DISABLED)

        # Tạo và bắt đầu thread xử lý G-code
        self.gcode_thread = threading.Thread(target=self.process_gcode, daemon=True)
        self.gcode_thread.start()

        self.log("Bắt đầu xử lý G-code")

    def process_gcode(self):
        """Thread xử lý G-code - phiên bản cải tiến"""
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

                    # === XỬ LÝ G-CODE TRÊN PYTHON TRƯỚC KHI GỬI ĐẾN ARDUINO ===
                    try:
                        # Phân tích G-code
                        if any(line.upper().startswith(cmd) for cmd in ["G0", "G1", "G00", "G01"]):
                            # Phân tích tọa độ X, Y từ G-code
                            x, y = None, None
                            parts = line.upper().split()
                            for part in parts:
                                if part.startswith('X'):
                                    try:
                                        x = float(part[1:])
                                    except ValueError:
                                        pass
                                elif part.startswith('Y'):
                                    try:
                                        y = float(part[1:])
                                    except ValueError:
                                        pass

                            # Nếu có tọa độ X, Y hợp lệ
                            if x is not None and y is not None:
                                self.log(f"G-code: Đang di chuyển đến X={x}, Y={y}")

                                # Tính góc bằng inverse kinematics và di chuyển
                                theta1, theta2, new_config = self.robot.inverse_kinematics(x, y, self.current_config)

                                if theta1 is not None and theta2 is not None:
                                    self.current_config = new_config
                                    # Gửi góc trực tiếp đến Arduino
                                    self.send_command(f"{theta1:.2f},{theta2:.2f}")
                                else:
                                    self.log(f"Lỗi: Không thể tính góc cho vị trí X={x}, Y={y}")
                                    # Tạm dừng nếu gặp lỗi
                                    self.gcode_paused = True
                                    self.root.after(0, lambda: self.pause_btn.config(text="Tiếp tục"))
                                    return
                            else:
                                # Gửi lệnh G-code nguyên dạng nếu không phải lệnh di chuyển XY
                                self.send_command(line)
                        else:
                            # Gửi lệnh G-code nguyên dạng cho các lệnh khác
                            self.send_command(line)

                    except Exception as e:
                        self.log(f"Lỗi xử lý G-code: {str(e)}")
                        self.gcode_paused = True
                        self.root.after(0, lambda: self.pause_btn.config(text="Tiếp tục"))
                        return

                # Tăng dòng và cập nhật tiến trình
                self.gcode_line_num += 1
                progress = int(100 * self.gcode_line_num / self.gcode_total_lines)
                self.root.after(0, lambda p=progress: self.update_progress(p))

            # Hoàn thành
            if self.gcode_running:
                self.root.after(0, self.complete_gcode)

        except Exception as e:
            self.root.after(0, lambda: self.log(f"Lỗi xử lý G-code: {str(e)}"))
            self.root.after(0, self.stop_gcode)

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

    def create_workspace_visualization(self):
        """Tạo dữ liệu trực quan hóa vùng làm việc"""
        points = []
        resolution = 1.0  # cm

        bounds = self.workspace_bounds
        x_range = np.arange(bounds['left'], bounds['right'] + resolution, resolution)
        y_range = np.arange(bounds['bottom'], bounds['top'] + resolution, resolution)

        for x in x_range:
            for y in y_range:
                points.append((x, y))

        return points
    # Trong ScaraGUI, thêm nút hiển thị vùng làm việc:
    def show_workspace(self):
        """Hiển thị vùng làm việc trên canvas"""
        points = self.robot.create_workspace_visualization()

        # Xóa canvas hiện tại và vẽ khung
        self.update_canvas()

        # Vẽ các điểm trong vùng làm việc
        canvas_width = self.canvas.winfo_width() or 500
        canvas_height = self.canvas.winfo_height() or 400
        scale = min(canvas_width, canvas_height) / 100
        cx = canvas_width / 2
        cy = canvas_height / 2

        # Vẽ từng điểm
        for x, y in points:
            px = cx + x * scale
            py = cy - y * scale
            self.canvas.create_rectangle(px - 1, py - 1, px + 1, py + 1, fill="lightblue", outline="")

        self.log(f"Hiển thị {len(points)} điểm trong vùng làm việc")

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
            time.sleep(3.0)

            # Kiểm tra kết nối
            self.serial.write(b"status\n")
            time.sleep(0.5)

            response = ""
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='replace').strip()

            if "READY" in response or "SCARA_READY" in response:
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

    def split_and_run_gcode(self):
        """Chia nhỏ và chạy G-code theo đợt"""
        batch_size = 5  # Số lệnh mỗi đợt

        batches = []
        current_batch = []

        for line in self.gcode_lines:
            if line and not line.startswith(";"):
                current_batch.append(line)
                if len(current_batch) >= batch_size:
                    batches.append(current_batch)
                    current_batch = []

        # Thêm batch cuối nếu còn
        if current_batch:
            batches.append(current_batch)

        # Chạy từng batch
        self.log(f"Chia G-code thành {len(batches)} đợt")

        for i, batch in enumerate(batches):
            self.log(f"Đang chạy đợt {i + 1}/{len(batches)}")
            # Gửi từng lệnh trong batch
            for line in batch:
                self.send_command(line, wait_for_response=True)
                time.sleep(0.5)  # Đợi giữa các lệnh

            # Đợi hoàn thành đợt
            time.sleep(1.0)

def main():
    root = tk.Tk()
    app = ScaraGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()