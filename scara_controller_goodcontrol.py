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
import sys # Thêm dòng này ở đầu file
import logging
# Removed animation import as it wasn't used directly for drawing updates
from matplotlib.patches import Rectangle, Circle # Removed FancyArrowPatch
import logging

# --- Cấu hình Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    stream=sys.stdout # Chỉ định gửi ra stdout
)


class ScaraRobot:
    def __init__(self):
        self.robot_params = {
            'x1': 0.0,  # Tọa độ X motor 1 (motor Y) - Gốc tọa độ
            'y1': 0.0,  # Tọa độ Y motor 1
            'x5': -20.0, # Tọa độ X motor 2 (motor X)
            'y5': 0.0,  # Tọa độ Y motor 2
            'L1': 15.0,  # Chiều dài cánh tay 1 từ motor 1
            'L2': 25.0,  # Chiều dài cánh tay 2 từ joint 1 đến đầu vẽ
            'L3': 25.0,  # Chiều dài cánh tay 3 từ đầu vẽ đến joint 2
            'L4': 15.0,  # Chiều dài cánh tay 4 từ joint 2 đến motor 2
        }
        # Giới hạn góc vật lý (Quan trọng cho IK và kiểm tra)
        self.angle_limits = {
            'theta1_min': -120.0,
            'theta1_max': 150.0,
            'theta2_min': 30.0,
            'theta2_max': 300.0
        }
        self.home_angles = {'theta1': 90.0, 'theta2': 90.0} # Góc home
        # Tính toán trước vùng làm việc và các thông số khác nếu cần
        self.calculate_workspace()
        logging.info("Khởi tạo ScaraRobot với thông số: %s", self.robot_params)
        logging.info("Giới hạn góc: %s", self.angle_limits)

    def calculate_workspace(self):
        # Thông số
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Tính bán kính vùng làm việc từ mỗi động cơ (có thể thêm khoảng đệm nhỏ)
        self.r1_min = max(0, abs(L1 - L2)) + 0.5 # Thêm 0.5cm buffer
        self.r1_max = (L1 + L2) - 0.5 # Thêm 0.5cm buffer
        self.r2_min = max(0, abs(L4 - L3)) + 0.5 # Thêm 0.5cm buffer
        self.r2_max = (L4 + L3) - 0.5 # Thêm 0.5cm buffer

        # Tính hình chữ nhật bao quanh sơ bộ
        self.workspace_bounds = {
            'left': min(x1 - self.r1_max, x5 - self.r2_max),
            'right': max(x1 + self.r1_max, x5 + self.r2_max),
            'bottom': min(y1 - self.r1_max, y5 - self.r2_max),
            'top': max(y1 + self.r1_max, y5 + self.r2_max)
        }
        logging.info("Tính toán vùng làm việc: R1=[%.2f, %.2f], R2=[%.2f, %.2f], Bounds=%s",
                     self.r1_min, self.r1_max, self.r2_min, self.r2_max, self.workspace_bounds)

    def forward_kinematics(self, theta1_deg, theta2_deg):
        # Chuyển góc sang radian
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)

        # Lấy thông số robot
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Tính tọa độ các điểm khớp
        x2 = x1 + L1 * np.cos(theta1)
        y2 = y1 + L1 * np.sin(theta1)

        x4 = x5 + L4 * np.cos(theta2)
        y4 = y5 + L4 * np.sin(theta2)

        # Tính toán tọa độ điểm cuối (x3, y3)
        d_sq = (x4 - x2)**2 + (y4 - y2)**2
        d = np.sqrt(d_sq)

        # Kiểm tra xem có thể tạo thành tam giác không (thêm dung sai nhỏ epsilon)
        epsilon = 1e-6
        if d > L2 + L3 + epsilon or d < abs(L2 - L3) - epsilon:
             # logging.warning(f"FK: Không thể tạo tam giác với d={d:.2f}, L2={L2}, L3={L3}")
             return None, None, None # Khoảng cách không hợp lệ

        try:
            # Sử dụng định lý cos để tìm góc tại khớp 2 (khớp giữa L2 và d)
            # Góc đối diện với L3
            cos_angle_at_j1 = (L2**2 + d_sq - L3**2) / (2 * L2 * d)
            # Đảm bảo giá trị nằm trong [-1, 1] do lỗi làm tròn số
            cos_angle_at_j1 = np.clip(cos_angle_at_j1, -1.0, 1.0)
            angle_at_j1 = np.arccos(cos_angle_at_j1)

            # Góc của vector từ khớp 1 (x2,y2) đến khớp 3 (x4,y4)
            angle_j1_to_j2 = np.arctan2(y4 - y2, x4 - x2)

            # Tính góc của cánh tay L2 (có hai giải pháp, chọn một dựa trên cấu hình)
            # Giả sử cấu hình "elbow down" là ưu tiên (góc L2 < góc vector j1_to_j2)
            angle_L2 = angle_j1_to_j2 - angle_at_j1 # Elbow down configuration

            # Tính tọa độ điểm cuối (x3, y3)
            x3 = x2 + L2 * np.cos(angle_L2)
            y3 = y2 + L2 * np.sin(angle_L2)

            # Trả về tọa độ các khớp quan trọng
            # logging.debug(f"FK Result: J1({x2:.2f},{y2:.2f}), J3({x4:.2f},{y4:.2f}), Effector({x3:.2f},{y3:.2f}) for Angles({theta1_deg:.2f}, {theta2_deg:.2f})")
            return (x2, y2), (x3, y3), (x4, y4)

        except (ValueError, ZeroDivisionError) as e: # Bắt cả lỗi chia cho 0 nếu d quá nhỏ
             logging.error(f"Lỗi tính toán FK: {e} với d={d}, L2={L2}, L3={L3}, cos_angle={cos_angle_at_j1 if 'cos_angle_at_j1' in locals() else 'N/A'}")
             return None, None, None

    def inverse_kinematics(self, x3, y3, current_config="elbow_down_down"): # Ưu tiên elbow_down_down
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1, L2 = self.robot_params['L1'], self.robot_params['L2']
        L3, L4 = self.robot_params['L3'], self.robot_params['L4']
        th1_min, th1_max = self.angle_limits['theta1_min'], self.angle_limits['theta1_max']
        th2_min, th2_max = self.angle_limits['theta2_min'], self.angle_limits['theta2_max']
        epsilon = 1e-6 # Dung sai nhỏ

        solutions = []

        try:
            # --- Tính cho Motor 1 (theta1) ---
            dx13, dy13 = x3 - x1, y3 - y1
            L13_sq = dx13**2 + dy13**2
            L13 = np.sqrt(L13_sq)

            # Kiểm tra tầm với của motor 1
            if L13 > L1 + L2 + epsilon or L13 < abs(L1 - L2) - epsilon:
                # logging.warning(f"IK: Điểm ({x3:.2f}, {y3:.2f}) ngoài tầm với motor 1 (L13={L13:.2f})")
                return None, None, None

            # Góc giữa trục X và đường nối (x1,y1) -> (x3,y3)
            beta1 = np.arctan2(dy13, dx13)
            # Góc trong tam giác (x1,y1), (x3,y3), (x2,y2) tại đỉnh (x1,y1) - đối diện L2
            # Thêm kiểm tra mẫu số khác 0
            if 2 * L1 * L13 < epsilon: return None, None, None # Tránh chia cho 0
            cos_alpha1 = (L1**2 + L13_sq - L2**2) / (2 * L1 * L13)
            cos_alpha1 = np.clip(cos_alpha1, -1.0, 1.0) # Đảm bảo trong [-1, 1]
            alpha1 = np.arccos(cos_alpha1)

            theta1_sol1 = np.degrees(beta1 - alpha1) # Elbow Up (alpha1 trừ đi)
            theta1_sol2 = np.degrees(beta1 + alpha1) # Elbow Down (alpha1 cộng vào)

            # --- Tính cho Motor 2 (theta2) ---
            dx53, dy53 = x3 - x5, y3 - y5
            L53_sq = dx53**2 + dy53**2
            L53 = np.sqrt(L53_sq)

            # Kiểm tra tầm với của motor 2
            if L53 > L4 + L3 + epsilon or L53 < abs(L4 - L3) - epsilon:
                # logging.warning(f"IK: Điểm ({x3:.2f}, {y3:.2f}) ngoài tầm với motor 2 (L53={L53:.2f})")
                return None, None, None

            # Góc giữa trục X và đường nối (x5,y5) -> (x3,y3)
            beta5 = np.arctan2(dy53, dx53)
            # Góc trong tam giác (x5,y5), (x3,y3), (x4,y4) tại đỉnh (x5,y5) - đối diện L3
            if 2 * L4 * L53 < epsilon: return None, None, None # Tránh chia cho 0
            cos_alpha5 = (L4**2 + L53_sq - L3**2) / (2 * L4 * L53)
            cos_alpha5 = np.clip(cos_alpha5, -1.0, 1.0) # Đảm bảo trong [-1, 1]
            alpha5 = np.arccos(cos_alpha5)

            theta2_sol1 = np.degrees(beta5 + alpha5) # Elbow Up (alpha5 cộng vào)
            theta2_sol2 = np.degrees(beta5 - alpha5) # Elbow Down (alpha5 trừ đi)

            # --- Chuẩn hóa góc về [-180, 180] hoặc [0, 360] nếu cần ---
            # Hàm normalize_angle (tùy chọn, tùy thuộc vào giới hạn của Arduino)
            # def normalize_angle(angle):
            #     while angle > 180: angle -= 360
            #     while angle <= -180: angle += 360
            #     return angle
            # theta1_sol1 = normalize_angle(theta1_sol1)
            # ... (áp dụng cho tất cả 4 giải pháp)

            # --- Kết hợp và kiểm tra giải pháp với giới hạn góc ---
            # Cấu hình 1: Elbow Up (theta1) - Elbow Up (theta2)
            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol1 <= th2_max:
                solutions.append(("elbow_up_up", theta1_sol1, theta2_sol1))
            # Cấu hình 2: Elbow Up (theta1) - Elbow Down (theta2)
            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol2 <= th2_max:
                solutions.append(("elbow_up_down", theta1_sol1, theta2_sol2))
            # Cấu hình 3: Elbow Down (theta1) - Elbow Up (theta2)
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol1 <= th2_max:
                solutions.append(("elbow_down_up", theta1_sol2, theta2_sol1))
            # Cấu hình 4: Elbow Down (theta1) - Elbow Down (theta2)
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol2 <= th2_max:
                solutions.append(("elbow_down_down", theta1_sol2, theta2_sol2))

            if not solutions:
                # logging.warning(f"IK: Không tìm thấy giải pháp hợp lệ cho ({x3:.2f}, {y3:.2f}) sau khi kiểm tra giới hạn góc.")
                return None, None, None

            # --- Chọn giải pháp tốt nhất ---
            # Ưu tiên giữ nguyên cấu hình hiện tại nếu có thể
            preferred_solutions = [s for s in solutions if s[0] == current_config] # So sánh chính xác cấu hình
            if preferred_solutions:
                # logging.debug(f"IK: Giữ nguyên config {current_config} cho ({x3:.2f}, {y3:.2f}) -> {preferred_solutions[0]}")
                return preferred_solutions[0][1], preferred_solutions[0][2], preferred_solutions[0][0]

            # Nếu không giữ được, ưu tiên cấu hình elbow_down_down nếu có
            down_down_solution = [s for s in solutions if s[0] == "elbow_down_down"]
            if down_down_solution:
                 # logging.debug(f"IK: Chọn giải pháp elbow_down_down cho ({x3:.2f}, {y3:.2f}) -> {down_down_solution[0]}")
                 return down_down_solution[0][1], down_down_solution[0][2], down_down_solution[0][0]

            # Nếu không, chọn giải pháp đầu tiên tìm được
            best_solution = solutions[0]
            # logging.debug(f"IK: Chọn giải pháp đầu tiên {best_solution[0]} cho ({x3:.2f}, {y3:.2f}) -> {best_solution}")
            return best_solution[1], best_solution[2], best_solution[0]

        except (ValueError, ZeroDivisionError) as e:
            logging.error(f"Lỗi tính toán IK: {e}")
            return None, None, None

    def check_point_in_workspace(self, x, y):
        # --- THÊM DÒNG KIỂM TRA Y > 0 ---
        if y <= 0:
            # logging.debug(f"Point ({x:.2f}, {y:.2f}) rejected: Y <= 0.")
            return False
        # --- KẾT THÚC THÊM ---

        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        epsilon = 1e-6  # Dung sai

        # Kiểm tra khoảng cách đến các động cơ
        d1_sq = (x - x1) ** 2 + (y - y1) ** 2
        d2_sq = (x - x5) ** 2 + (y - y5) ** 2

        # Kiểm tra điều kiện bán kính (với dung sai)
        r1_min_sq = self.r1_min ** 2
        r1_max_sq = self.r1_max ** 2
        r2_min_sq = self.r2_min ** 2
        r2_max_sq = self.r2_max ** 2

        if not (r1_min_sq - epsilon <= d1_sq <= r1_max_sq + epsilon):
            # logging.debug(f"Point ({x:.2f}, {y:.2f}) rejected: Out of range for motor 1 (d1^2={d1_sq:.2f})")
            return False
        if not (r2_min_sq - epsilon <= d2_sq <= r2_max_sq + epsilon):
            # logging.debug(f"Point ({x:.2f}, {y:.2f}) rejected: Out of range for motor 2 (d2^2={d2_sq:.2f})")
            return False

        # Kiểm tra động học ngược có giải pháp hợp lệ không
        theta1, theta2, _ = self.inverse_kinematics(x, y)
        if theta1 is None or theta2 is None:
            # logging.debug(f"Point ({x:.2f}, {y:.2f}) rejected: No valid IK solution.")
            return False

        # logging.debug(f"Point ({x:.2f}, {y:.2f}) is within workspace.")
        return True

    def create_workspace_visualization_points(self, resolution=0.5):
        """Tạo danh sách các điểm (x, y) trong vùng làm việc."""
        points = []
        bounds = self.workspace_bounds
        # Tạo lưới điểm với resolution
        x_coords = np.arange(bounds['left'], bounds['right'] + resolution, resolution)
        y_coords = np.arange(bounds['bottom'], bounds['top'] + resolution, resolution)
        grid_x, grid_y = np.meshgrid(x_coords, y_coords)
        test_points = np.vstack([grid_x.ravel(), grid_y.ravel()]).T

        logging.info(f"Tạo visualization workspace với resolution={resolution} ({len(test_points)} điểm kiểm tra)...")
        start_time = time.time()

        valid_points = [p for p in test_points if self.check_point_in_workspace(p[0], p[1])]

        elapsed = time.time() - start_time
        logging.info(f"Hoàn thành tạo visualization workspace sau {elapsed:.2f}s, tìm thấy {len(valid_points)} điểm hợp lệ.")
        return valid_points


class ScaraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Controller")
        # self.root.geometry("1100x750") # Tăng kích thước cửa sổ
        try:
            self.root.state('zoomed') # Mở cửa sổ lớn nhất có thể (Windows)
        except tk.TclError:
            try:
                # Thử cách khác cho Linux/macOS
                m = root.maxsize()
                root.geometry('{}x{}+0+0'.format(*m))
            except:
                root.geometry("1200x800") # Fallback

        # Khởi tạo robot và Serial
        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 90.0, 'theta2': 90.0} # Bắt đầu ở home
        self.current_position = {'x': 0.0, 'y': 0.0} # Sẽ được tính từ góc home
        self.pen_is_down = False
        # Giả định cấu hình ban đầu (quan trọng cho lần IK đầu tiên)
        self.current_config = "elbow_down_down"

        # Tính vị trí XY ban đầu từ góc home
        _, effector_pos, _ = self.robot.forward_kinematics(
            self.robot.home_angles['theta1'], self.robot.home_angles['theta2']
        )
        if effector_pos:
            self.current_position['x'], self.current_position['y'] = effector_pos
            logging.info(f"Vị trí XY ban đầu (từ home): ({effector_pos[0]:.2f}, {effector_pos[1]:.2f})")
        else: # Xử lý trường hợp home không hợp lệ (ít khả năng)
             logging.warning("Không thể tính vị trí XY ban đầu từ góc home.")
             # Đặt giá trị an toàn gần vùng làm việc
             self.current_position['x'], self.current_position['y'] = -10, 30

        # Visualization variables
        self.trace_segments = [] # Lưu các đoạn vẽ (list các dict {'x': [], 'y': []})
        self.current_segment = {} # Đoạn vẽ hiện tại {'x': [], 'y': []}
        self.workspace_points = [] # Lưu các điểm vùng làm việc để vẽ lại nhanh
        self._workspace_visualization_active = False # Cờ trạng thái hiển thị workspace

        # Biến cho G-code
        self.gcode_lines = []
        self.gcode_running = False
        self.gcode_paused = False
        self.gcode_line_num = 0
        self.gcode_total_lines = 0

        # Hệ thống lệnh
        self.command_queue = queue.Queue()
        self.command_processing = False # Cờ báo lệnh đang được xử lý (_execute_command)
        self.serial_lock = threading.Lock() # Lock để truy cập serial
        self.last_response_time = time.time()
        self.command_timeout = 10.0 # Timeout chờ phản hồi lệnh (giây)

        # Tạo giao diện
        self.create_widgets()

        # Cập nhật danh sách cổng COM
        self.update_ports()

        # Bắt đầu thread xử lý hàng đợi lệnh
        self.process_command_thread = threading.Thread(target=self._process_command_queue, daemon=True)
        self.process_command_thread.start()

        # Cập nhật visualization ban đầu
        self.root.after(100, self.update_animation) # Delay nhỏ để UI kịp vẽ

        logging.info("Khởi tạo ScaraGUI hoàn tất.")

    def _process_command_queue(self):
        """Vòng lặp xử lý hàng đợi lệnh gửi đến Arduino."""
        while True:
            try:
                # Nếu không có lệnh nào đang xử lý và queue không rỗng
                if not self.command_processing and not self.command_queue.empty():
                    command, wait_response, is_move_cmd = self.command_queue.get()
                    self.command_processing = True # Đặt cờ đang xử lý
                    # Thực thi lệnh và chờ (nếu cần)
                    self._execute_command(command, wait_response, is_move_cmd)
                    self.command_processing = False # Xóa cờ
                    self.command_queue.task_done() # Báo hiệu lệnh đã xử lý xong
                    time.sleep(0.01) # Nghỉ ngắn
                else:
                    time.sleep(0.02) # Chờ nếu không có lệnh hoặc đang xử lý
            except Exception as e:
                logging.exception("Lỗi nghiêm trọng trong _process_command_queue:")
                self.command_processing = False # Đảm bảo reset cờ
                time.sleep(0.1)

    def create_widgets(self):
        # Frame chính - Sử dụng PanedWindow để có thể thay đổi kích thước panel
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- Panel điều khiển (bên trái) ---
        control_panel = ttk.Frame(main_frame, padding=10)
        main_frame.add(control_panel, weight=35) # Tỷ lệ nhỏ hơn cho control

        # --- Panel hiển thị (bên phải) ---
        display_panel = ttk.Frame(main_frame, padding=10)
        main_frame.add(display_panel, weight=65) # Tỷ lệ lớn hơn cho display

        # === Bố cục trong Control Panel ===
        control_panel.columnconfigure(0, weight=1) # Cho phép cột 0 co giãn

        # --- Khung Kết nối ---
        conn_frame = ttk.LabelFrame(control_panel, text="Kết nối", padding=10)
        conn_frame.grid(row=0, column=0, sticky=tk.EW, pady=(0, 10))
        conn_frame.columnconfigure(1, weight=1) # Cho combobox co giãn

        ttk.Label(conn_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.EW)

        refresh_btn = ttk.Button(conn_frame, text="⟳", width=3, command=self.update_ports)
        refresh_btn.grid(row=0, column=2, padx=(0, 5), pady=5)

        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200") # Tăng baudrate mặc định
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=15, state="readonly",
                                  values=["9600", "19200", "38400", "57600", "115200", "250000"])
        baud_combo.grid(row=1, column=1, padx=5, pady=5, sticky=tk.EW)

        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(0, 5), pady=5)

        # --- Khung Điều khiển thủ công ---
        manual_frame = ttk.LabelFrame(control_panel, text="Điều khiển thủ công", padding=10)
        manual_frame.grid(row=1, column=0, sticky=tk.EW, pady=(0, 10))
        manual_frame.columnconfigure(0, weight=1) # Chia đều cột

        # Điều khiển bút
        pen_frame = ttk.Frame(manual_frame)
        pen_frame.grid(row=0, column=0, columnspan=2, sticky=tk.EW, pady=5)
        pen_frame.columnconfigure(0, weight=1)
        pen_frame.columnconfigure(1, weight=1)

        pen_up_btn = ttk.Button(pen_frame, text="Nâng bút (U)", command=self.pen_up)
        pen_up_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.root.bind('<u>', lambda event=None: self.pen_up()) # Phím tắt
        self.root.bind('<U>', lambda event=None: self.pen_up()) # Phím tắt (Shift+u)


        pen_down_btn = ttk.Button(pen_frame, text="Hạ bút (D)", command=self.pen_down)
        pen_down_btn.grid(row=0, column=1, padx=(5, 0), sticky=tk.EW)
        self.root.bind('<d>', lambda event=None: self.pen_down()) # Phím tắt
        self.root.bind('<D>', lambda event=None: self.pen_down()) # Phím tắt (Shift+d)

        # Tọa độ XY
        xy_frame = ttk.Frame(manual_frame)
        xy_frame.grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=5)
        xy_frame.columnconfigure(1, weight=1) # Cho Entry X giãn
        xy_frame.columnconfigure(3, weight=1) # Cho Entry Y giãn
        xy_frame.columnconfigure(4, weight=2) # Cho Button giãn nhiều hơn

        ttk.Label(xy_frame, text="X:").grid(row=0, column=0, padx=(0, 2))
        self.x_entry = ttk.Entry(xy_frame, width=7)
        self.x_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        self.x_entry.insert(0, f"{self.current_position['x']:.2f}")

        ttk.Label(xy_frame, text="Y:").grid(row=0, column=2, padx=(5, 2))
        self.y_entry = ttk.Entry(xy_frame, width=7)
        self.y_entry.grid(row=0, column=3, padx=(0, 5), sticky=tk.EW)
        self.y_entry.insert(0, f"{self.current_position['y']:.2f}")

        go_xy_btn = ttk.Button(xy_frame, text="Di chuyển XY", command=self.move_to_xy)
        go_xy_btn.grid(row=0, column=4, sticky=tk.EW)

        # Góc
        angle_frame = ttk.Frame(manual_frame)
        angle_frame.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=5)
        angle_frame.columnconfigure(1, weight=1) # Cho Entry Theta1 giãn
        angle_frame.columnconfigure(3, weight=1) # Cho Entry Theta2 giãn
        angle_frame.columnconfigure(4, weight=2) # Cho Button giãn nhiều hơn

        ttk.Label(angle_frame, text="θ1:").grid(row=0, column=0, padx=(0, 2))
        self.angle1_entry = ttk.Entry(angle_frame, width=7)
        self.angle1_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        self.angle1_entry.insert(0, f"{self.current_angles['theta1']:.2f}")

        ttk.Label(angle_frame, text="θ2:").grid(row=0, column=2, padx=(5, 2))
        self.angle2_entry = ttk.Entry(angle_frame, width=7)
        self.angle2_entry.grid(row=0, column=3, padx=(0, 5), sticky=tk.EW)
        self.angle2_entry.insert(0, f"{self.current_angles['theta2']:.2f}")

        go_angle_btn = ttk.Button(angle_frame, text="Di chuyển Góc", command=self.move_to_angle)
        go_angle_btn.grid(row=0, column=4, sticky=tk.EW)

        # Tốc độ và gia tốc
        speed_accel_frame = ttk.Frame(manual_frame)
        speed_accel_frame.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=5)
        speed_accel_frame.columnconfigure(1, weight=1)
        speed_accel_frame.columnconfigure(4, weight=1) # Index cột của accel entry là 4

        ttk.Label(speed_accel_frame, text="Tốc độ:").grid(row=0, column=0, padx=(0, 2), sticky=tk.W)
        self.speed_var = tk.StringVar(value="4200") # Tăng tốc độ mặc định
        speed_entry = ttk.Entry(speed_accel_frame, textvariable=self.speed_var, width=6)
        speed_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_speed).grid(row=0, column=2, padx=(0, 10))

        ttk.Label(speed_accel_frame, text="Gia tốc:").grid(row=0, column=3, padx=(10, 2), sticky=tk.W)
        self.accel_var = tk.StringVar(value="1800") # Tăng gia tốc mặc định
        accel_entry = ttk.Entry(speed_accel_frame, textvariable=self.accel_var, width=6)
        accel_entry.grid(row=0, column=4, padx=(0, 5), sticky=tk.EW)
        ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_acceleration).grid(row=0, column=5, padx=(0, 0))

        # Chức năng bổ sung (Home, Stop)
        func_frame = ttk.Frame(manual_frame)
        func_frame.grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=5)
        func_frame.columnconfigure(0, weight=1)
        func_frame.columnconfigure(1, weight=1)

        home_btn = ttk.Button(func_frame, text="HOME (H)", command=self.home)
        home_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.root.bind('<h>', lambda event=None: self.home()) # Phím tắt
        self.root.bind('<H>', lambda event=None: self.home()) # Phím tắt (Shift+h)


        stop_btn = ttk.Button(func_frame, text="DỪNG KHẨN CẤP", command=self.emergency_stop, style="Emergency.TButton")
        stop_btn.grid(row=0, column=1, padx=(5, 0), sticky=tk.EW)

        # --- Khung G-code ---
        gcode_frame = ttk.LabelFrame(control_panel, text="Điều khiển G-Code", padding=10)
        gcode_frame.grid(row=2, column=0, sticky=tk.NSEW, pady=(0, 10))
        gcode_frame.columnconfigure(0, weight=1) # Cho text và progress bar giãn

        # Tải file
        load_frame = ttk.Frame(gcode_frame)
        load_frame.grid(row=0, column=0, sticky=tk.EW, pady=(0, 5))
        load_frame.columnconfigure(1, weight=1) # Cho label giãn

        load_btn = ttk.Button(load_frame, text="Tải G-Code", command=self.load_gcode)
        load_btn.grid(row=0, column=0, padx=(0, 10))

        self.file_label = ttk.Label(load_frame, text="Chưa tải file", anchor=tk.W, relief="sunken", padding=(2, 1)) # Thêm hiệu ứng
        self.file_label.grid(row=0, column=1, sticky=tk.EW)

        # Khung hiển thị G-code
        code_view_frame = ttk.Frame(gcode_frame)
        code_view_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(0, 5))
        code_view_frame.rowconfigure(0, weight=1)    # Cho text giãn theo chiều dọc
        code_view_frame.columnconfigure(0, weight=1) # Cho text giãn theo chiều ngang

        self.gcode_text = tk.Text(code_view_frame, height=8, wrap=tk.NONE, borderwidth=1, relief="sunken",
                                   font=("Courier New", 9)) # Đổi font dễ đọc code
        self.gcode_text.grid(row=0, column=0, sticky=tk.NSEW)

        gcode_yscroll = ttk.Scrollbar(code_view_frame, orient=tk.VERTICAL, command=self.gcode_text.yview)
        gcode_yscroll.grid(row=0, column=1, sticky=tk.NS)
        self.gcode_text.config(yscrollcommand=gcode_yscroll.set)

        gcode_xscroll = ttk.Scrollbar(gcode_frame, orient=tk.HORIZONTAL, command=self.gcode_text.xview)
        gcode_xscroll.grid(row=2, column=0, sticky=tk.EW)
        self.gcode_text.config(xscrollcommand=gcode_xscroll.set)

        # Điều khiển G-code (Run, Pause, Stop)
        gcode_ctrl_frame = ttk.Frame(gcode_frame)
        gcode_ctrl_frame.grid(row=3, column=0, sticky=tk.EW, pady=(5, 5))
        gcode_ctrl_frame.columnconfigure(0, weight=1)
        gcode_ctrl_frame.columnconfigure(1, weight=1)
        gcode_ctrl_frame.columnconfigure(2, weight=1)

        self.run_btn = ttk.Button(gcode_ctrl_frame, text="Chạy", command=self.run_gcode, state=tk.DISABLED)
        self.run_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)

        self.pause_btn = ttk.Button(gcode_ctrl_frame, text="Tạm dừng", command=self.pause_gcode, state=tk.DISABLED)
        self.pause_btn.grid(row=0, column=1, padx=5, sticky=tk.EW)

        self.stop_gcode_btn = ttk.Button(gcode_ctrl_frame, text="Dừng", command=self.stop_gcode, state=tk.DISABLED)
        self.stop_gcode_btn.grid(row=0, column=2, padx=(5, 0), sticky=tk.EW)

        # Tiến trình G-code
        self.progress_var = tk.DoubleVar() # Dùng DoubleVar cho mượt hơn
        progress_bar = ttk.Progressbar(gcode_frame, variable=self.progress_var, maximum=100, mode='determinate')
        progress_bar.grid(row=4, column=0, sticky=tk.EW, pady=(0, 2))

        self.progress_label = ttk.Label(gcode_frame, text="0.0% hoàn thành")
        self.progress_label.grid(row=5, column=0, sticky=tk.EW)

        # Delay G-code
        delay_frame = ttk.Frame(gcode_frame)
        delay_frame.grid(row=6, column=0, sticky=tk.EW, pady=(5, 0))
        ttk.Label(delay_frame, text="Chờ giữa lệnh (s):").pack(side=tk.LEFT, padx=(0, 5))
        self.gcode_delay_var = tk.DoubleVar(value=0.05) # Giảm delay mặc định
        delay_spinbox = ttk.Spinbox(delay_frame, from_=0.00, to=2.0, increment=0.01, # Cho phép delay 0
                                    textvariable=self.gcode_delay_var, width=5)
        delay_spinbox.pack(side=tk.LEFT)

        # --- Khung Log ---
        log_frame = ttk.LabelFrame(control_panel, text="Nhật ký", padding=10)
        log_frame.grid(row=3, column=0, sticky=tk.NSEW, pady=(0, 0))
        log_frame.rowconfigure(0, weight=1)    # Cho text giãn theo chiều dọc
        log_frame.columnconfigure(0, weight=1) # Cho text giãn theo chiều ngang
        # Cho phép khung log co giãn
        control_panel.rowconfigure(3, weight=1)

        self.log_text = tk.Text(log_frame, height=5, wrap=tk.WORD, borderwidth=1, relief="sunken",
                                 font=("Segoe UI", 9)) # Đổi font
        self.log_text.grid(row=0, column=0, sticky=tk.NSEW)
        # Configure tags for different log levels (optional)
        self.log_text.tag_configure("INFO", foreground="black")
        self.log_text.tag_configure("WARNING", foreground="orange")
        self.log_text.tag_configure("ERROR", foreground="red", font=("Segoe UI", 9, "bold"))
        self.log_text.tag_configure("DEBUG", foreground="gray")
        self.log_text.tag_configure("SENT", foreground="blue")
        self.log_text.tag_configure("RECEIVED", foreground="green")


        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky=tk.NS)
        self.log_text.config(yscrollcommand=log_scroll.set)

        # === Bố cục trong Display Panel ===
        display_panel.rowconfigure(0, weight=1) # Cho canvas giãn
        display_panel.columnconfigure(0, weight=1) # Cho canvas giãn

        # --- Khung Visualization ---
        canvas_ui_frame = ttk.Frame(display_panel) # Frame chứa canvas và các nút liên quan
        canvas_ui_frame.grid(row=0, column=0, sticky=tk.NSEW)
        canvas_ui_frame.rowconfigure(0, weight=1)
        canvas_ui_frame.columnconfigure(0, weight=1)

        # Canvas Matplotlib
        self.fig = plt.figure(figsize=(7, 6)) # Điều chỉnh kích thước nếu cần
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_ui_frame)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=0, sticky=tk.NSEW)

        # Frame chứa các nút điều khiển visualization
        vis_ctrl_frame = ttk.Frame(canvas_ui_frame)
        vis_ctrl_frame.grid(row=1, column=0, sticky=tk.EW, pady=(5, 0))

        workspace_btn = ttk.Button(vis_ctrl_frame, text="Hiện/Ẩn Vùng làm việc", command=self.toggle_workspace_visualization)
        workspace_btn.pack(side=tk.LEFT, padx=5)

        clear_drawing_btn = ttk.Button(vis_ctrl_frame, text="Xóa Hình Vẽ", command=self.clear_drawing)
        clear_drawing_btn.pack(side=tk.LEFT, padx=5)

        # --- Khung Trạng thái ---
        status_frame = ttk.LabelFrame(display_panel, text="Trạng thái Robot", padding=10)
        status_frame.grid(row=1, column=0, sticky=tk.EW, pady=(10, 0))
        status_frame.columnconfigure(1, weight=1)
        status_frame.columnconfigure(3, weight=1)

        ttk.Label(status_frame, text="Vị trí (X,Y):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.pos_label = ttk.Label(status_frame, text="X=???, Y=???", width=20, font=("Segoe UI", 10, "bold"))
        self.pos_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(status_frame, text="Góc (θ1,θ2):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.angle_label = ttk.Label(status_frame, text="θ1=???, θ2=???", width=20, font=("Segoe UI", 10, "bold"))
        self.angle_label.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)

        ttk.Label(status_frame, text="Bút:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.pen_label = ttk.Label(status_frame, text="Nâng", width=10, font=("Segoe UI", 10))
        self.pen_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(status_frame, text="Cấu hình IK:").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.config_label = ttk.Label(status_frame, text=self.current_config, width=15, font=("Segoe UI", 10))
        self.config_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)

        # Style cho nút dừng khẩn cấp
        style = ttk.Style()
        style.configure("Emergency.TButton", foreground="white", background="red", font=("Segoe UI", 10, "bold"))

        # Khởi tạo visualization plot
        self.setup_plot()
        self.update_position_display() # Cập nhật hiển thị trạng thái ban đầu

    def setup_plot(self):
        """Khởi tạo hoặc vẽ lại nền cho visualization plot."""
        self.ax.clear() # Xóa hoàn toàn trục

        # Lấy bounds từ vùng làm việc đã tính
        bounds = self.robot.workspace_bounds
        padding = 5 # Thêm khoảng đệm xung quanh

        # Thiết lập giới hạn trục chặt chẽ hơn dựa trên bounds
        self.ax.set_xlim(bounds['left'] - padding, bounds['right'] + padding)
        self.ax.set_ylim(bounds['bottom'] - padding, bounds['top'] + padding)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_title("SCARA Robot Visualization")
        self.ax.grid(True, linestyle='--', alpha=0.6) # Grid mờ hơn
        self.ax.set_aspect('equal', adjustable='box') # Đảm bảo tỷ lệ trục 1:1

        # --- Vẽ các thành phần tĩnh ---
        x1, y1 = self.robot.robot_params['x1'], self.robot.robot_params['y1']
        x5, y5 = self.robot.robot_params['x5'], self.robot.robot_params['y5']

        # Đánh dấu vị trí động cơ
        self.ax.plot(x1, y1, 'ks', markersize=8, label='Motor 1 (Y)') # Hình vuông đen
        self.ax.plot(x5, y5, 'ks', markersize=8, label='Motor 2 (X)') # Hình vuông đen
        self.ax.text(x1 + 1, y1 + 1, "M1(Y)", fontsize=8, color='black', ha='left', va='bottom')
        self.ax.text(x5 + 1, y5 + 1, "M2(X)", fontsize=8, color='black', ha='left', va='bottom')

        # Vẽ vùng làm việc tùy chỉnh nếu có
        # self.custom_workspace_patch = Rectangle((-20, 20), 20, 14,
        #                                         linewidth=1, edgecolor='darkred', facecolor='yellow',
        #                                         alpha=0.2, hatch='//', label='Custom Area')
        # self.ax.add_patch(self.custom_workspace_patch)
        # self.ax.text(-10, 27, "Custom", fontsize=8, ha='center', color='darkred')

        # --- Khởi tạo các thành phần động (sẽ được cập nhật trong update_animation) ---
        # Cánh tay robot (dùng plot thay vì FancyArrowPatch để dễ cập nhật hơn)
        self.arm1_line, = self.ax.plot([], [], 'b-', lw=5, alpha=0.7, label='Arm 1-2', solid_capstyle='round') # Dày hơn, bo tròn
        self.arm2_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, label='Arm 2-3', solid_capstyle='round') # Dày hơn, bo tròn
        self.arm3_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, solid_capstyle='round') # Cùng màu arm 2
        self.arm4_line, = self.ax.plot([], [], 'c-', lw=5, alpha=0.7, label='Arm 4-5', solid_capstyle='round') # Dày hơn, bo tròn

        # Các khớp (dùng plot với marker 'o') - To hơn
        self.joints, = self.ax.plot([], [], 'o', color='darkred', markersize=8, label='Joints')

        # Đầu bút vẽ (effector) - Marker to hơn, màu khác
        self.effector, = self.ax.plot([], [], 'o', color='magenta', markersize=10, label='Effector')

        # Vết vẽ (trace)
        self.trace_line, = self.ax.plot([], [], '-', color='red', lw=1.5, label='Drawing Trace')

        # Text hiển thị góc và trạng thái bút (vị trí cố định hơn)
        text_x_pos = bounds['left'] # Đặt ở góc trên bên trái
        text_y_pos = bounds['top'] + padding * 0.8
        self.angle_text = self.ax.text(text_x_pos, text_y_pos, "", fontsize=9, ha='left', va='top',
                                       bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none')) # Nền trắng mờ
        self.pen_status_text = self.ax.text(text_x_pos, text_y_pos - 3.0, "", fontsize=9, ha='left', va='top',
                                            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none')) # Nền trắng mờ


        # Hiển thị legend (tùy chọn)
        # self.ax.legend(loc='upper right', fontsize='small')

        # Vẽ lại canvas
        self.fig.tight_layout() # Tự động điều chỉnh layout
        self.canvas.draw()

        # Reset cờ hiển thị workspace
        self._workspace_visualization_active = False
        # Xóa đối tượng scatter cũ nếu có
        if hasattr(self, 'workspace_scatter'):
            try:
                self.workspace_scatter.remove()
            except ValueError: # Có thể đã bị remove trước đó
                pass
            del self.workspace_scatter


    def toggle_workspace_visualization(self):
        """Hiện hoặc ẩn các điểm biểu diễn vùng làm việc."""
        if self._workspace_visualization_active:
            # Nếu đang hiện -> Xóa đi
            if hasattr(self, 'workspace_scatter'):
                try:
                    self.workspace_scatter.remove()
                except ValueError:
                     pass # Đã bị xóa
                del self.workspace_scatter
                self._workspace_visualization_active = False
                self.log("Đã ẩn visualization vùng làm việc.")
                self.canvas.draw_idle() # Yêu cầu vẽ lại
                logging.info("Đã ẩn workspace visualization.")
            else:
                 # Trường hợp lạ: cờ bật nhưng scatter không tồn tại
                 self._workspace_visualization_active = False
                 logging.warning("Trạng thái workspace không nhất quán, đã reset.")
        else:
            # Nếu đang ẩn -> Vẽ lên
            if not self.workspace_points: # Nếu chưa có dữ liệu thì tạo
                self.log("Đang tính toán các điểm vùng làm việc...")
                logging.info("Bắt đầu tạo điểm workspace visualization...")
                # Chạy trong thread để không block UI
                threading.Thread(target=self._generate_and_draw_workspace, daemon=True).start()
            else:
                # Nếu đã có dữ liệu thì vẽ luôn
                self._draw_workspace_points()

    def _generate_and_draw_workspace(self):
        """Hàm chạy trong thread để tạo dữ liệu và vẽ vùng làm việc."""
        try:
            generated_points = self.robot.create_workspace_visualization_points(resolution=0.5) # Giảm resolution
            if generated_points:
                 self.workspace_points = generated_points # Lưu lại
                 # Gọi hàm vẽ trên main thread thông qua after()
                 self.root.after(0, self._draw_workspace_points)
            else:
                 self.log("Không tạo được điểm nào cho vùng làm việc.")
                 logging.warning("Không có điểm hợp lệ nào được tạo cho workspace.")
        except Exception as e:
            logging.exception("Lỗi khi tạo visualization vùng làm việc:")
            self.log(f"Lỗi tạo vùng làm việc: {e}")

    def _draw_workspace_points(self):
        """Vẽ các điểm vùng làm việc lên plot (chạy trên main thread)."""
        if not self.workspace_points:
            logging.warning("Không có điểm workspace để vẽ.")
            return
        # Xóa scatter cũ trước khi vẽ mới (nếu có)
        if hasattr(self, 'workspace_scatter'):
             try:
                 self.workspace_scatter.remove()
             except ValueError:
                 pass
             del self.workspace_scatter

        try:
            # Giải nén list các tuple (x,y) thành hai list riêng biệt
            x_coords, y_coords = zip(*self.workspace_points)
            # Vẽ các điểm bằng scatter plot cho hiệu năng tốt hơn plot nhiều điểm riêng lẻ
            self.workspace_scatter = self.ax.scatter(x_coords, y_coords, s=1, c='lightblue', alpha=0.5, label='Workspace', zorder=0) # zorder thấp để nằm dưới
            self._workspace_visualization_active = True
            self.log(f"Đã hiển thị {len(self.workspace_points)} điểm vùng làm việc.")
            self.canvas.draw_idle() # Yêu cầu vẽ lại
            logging.info("Đã vẽ workspace visualization.")
        except Exception as e:
            logging.exception("Lỗi khi vẽ điểm vùng làm việc:")
            self.log(f"Lỗi vẽ vùng làm việc: {e}")

    def update_animation(self):
        """Cập nhật vị trí các thành phần động trên visualization plot."""
        theta1_deg = self.current_angles['theta1']
        theta2_deg = self.current_angles['theta2']

        try:
            # Tính toán vị trí các khớp bằng FK
            joint1_pos, effector_pos, joint3_pos = self.robot.forward_kinematics(theta1_deg, theta2_deg)

            if joint1_pos is None or effector_pos is None or joint3_pos is None:
                # logging.warning("FK không trả về vị trí hợp lệ, không cập nhật animation.")
                # Giữ nguyên trạng thái cũ của plot nếu FK lỗi
                return

            x1, y1 = self.robot.robot_params['x1'], self.robot.robot_params['y1']
            x5, y5 = self.robot.robot_params['x5'], self.robot.robot_params['y5']
            x2, y2 = joint1_pos
            x3, y3 = effector_pos
            x4, y4 = joint3_pos

            # Cập nhật đường nối các cánh tay
            self.arm1_line.set_data([x1, x2], [y1, y2])
            self.arm2_line.set_data([x2, x3], [y2, y3])
            self.arm3_line.set_data([x3, x4], [y3, y4])
            self.arm4_line.set_data([x4, x5], [y4, y5])

            # Cập nhật vị trí các khớp (trừ motor và effector)
            self.joints.set_data([x2, x4], [y2, y4])

            # Cập nhật vị trí effector (đầu bút)
            self.effector.set_data([x3], [y3])

            # --- Cập nhật vết vẽ ---
            # Thêm điểm mới vào segment hiện tại nếu đang hạ bút
            if self.pen_is_down:
                if not self.current_segment or not self.current_segment['x']: # Bắt đầu segment mới
                    self.current_segment = {'x': [x3], 'y': [y3]}
                    # logging.debug(f"Bắt đầu segment vẽ mới tại ({x3:.2f}, {y3:.2f})")
                else: # Thêm vào segment đang vẽ
                    # Chỉ thêm nếu khác điểm cuối cùng để tránh trùng lặp (dung sai nhỏ)
                    dx = x3 - self.current_segment['x'][-1]
                    dy = y3 - self.current_segment['y'][-1]
                    if dx*dx + dy*dy > 1e-4: # Kiểm tra bình phương khoảng cách > 0.01^2
                         self.current_segment['x'].append(x3)
                         self.current_segment['y'].append(y3)
                         # logging.debug(f"Thêm điểm vẽ ({x3:.2f}, {y3:.2f})")

            # Chuẩn bị dữ liệu để vẽ tất cả các segment
            all_trace_x, all_trace_y = [], []
            for segment in self.trace_segments:
                if segment and segment.get('x'): # Kiểm tra segment hợp lệ
                    all_trace_x.extend(segment['x'])
                    all_trace_y.extend(segment['y'])
                    all_trace_x.append(None) # Thêm None để ngắt đoạn trên plot
                    all_trace_y.append(None)

            # Thêm segment hiện tại (nếu có)
            if self.current_segment and self.current_segment.get('x'):
                all_trace_x.extend(self.current_segment['x'])
                all_trace_y.extend(self.current_segment['y'])

            # Cập nhật dữ liệu cho line vết vẽ
            self.trace_line.set_data(all_trace_x, all_trace_y)

            # --- Cập nhật Text ---
            self.angle_text.set_text(f"θ1={theta1_deg:.2f}° | θ2={theta2_deg:.2f}°")
            pen_text = f"Bút: {'Hạ' if self.pen_is_down else 'Nâng'}"
            self.pen_status_text.set_text(pen_text)
            # Đổi màu text bút
            self.pen_status_text.set_color('red' if self.pen_is_down else 'black')

            # Yêu cầu vẽ lại canvas một cách hiệu quả
            self.canvas.draw_idle()

        except Exception as e:
            logging.exception("Lỗi trong update_animation:")

    def clear_drawing(self):
        """Xóa các vết vẽ trên visualization."""
        if not self.trace_segments and not self.current_segment.get('x'):
            self.log("Không có hình vẽ nào để xóa.")
            return

        self.trace_segments = []
        self.current_segment = {}
        # Cập nhật animation để xóa vết vẽ khỏi plot
        self.trace_line.set_data([], []) # Xóa dữ liệu của trace_line
        self.log("Đã xóa hình vẽ trên mô phỏng.")
        logging.info("Đã xóa hình vẽ.")
        self.canvas.draw_idle() # Yêu cầu vẽ lại

    def update_ports(self):
        """Cập nhật danh sách cổng COM có sẵn."""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                # Cố gắng giữ lại port đang chọn nếu còn tồn tại
                current_port = self.port_var.get()
                if current_port not in ports:
                    self.port_var.set(ports[0])
            else:
                self.port_var.set("") # Xóa lựa chọn nếu không có port
            logging.info(f"Cập nhật cổng COM: {ports}")
        except Exception as e:
            logging.error(f"Lỗi khi cập nhật cổng COM: {e}")
            self.port_combo['values'] = []
            self.port_var.set("")

    def toggle_connection(self):
        """Kết nối hoặc ngắt kết nối với Arduino."""
        if self.is_connected:
            # --- Ngắt kết nối ---
            try:
                # Gửi lệnh disable trước khi đóng port (nếu có thể)
                if self.serial and self.serial.is_open:
                     # Không gửi qua queue, gửi trực tiếp nếu có thể
                     try:
                         with self.serial_lock:
                              disable_cmd = b"disable\n"
                              self.serial.write(disable_cmd)
                              logging.info("Đã gửi lệnh disable trước khi ngắt kết nối.")
                     except Exception as send_err:
                          logging.warning(f"Không thể gửi lệnh disable khi ngắt kết nối: {send_err}")
                     time.sleep(0.1)
                     self.serial.close()
                self.is_connected = False
                self.connect_btn.configure(text="Kết nối")
                self.log("Đã ngắt kết nối khỏi Arduino.")
                logging.info("Đã ngắt kết nối.")
            except Exception as e:
                logging.exception("Lỗi khi ngắt kết nối:")
                self.log(f"Lỗi ngắt kết nối: {e}")
                self.is_connected = False # Đảm bảo trạng thái đúng
                self.connect_btn.configure(text="Kết nối")
        else:
            # --- Kết nối ---
            port = self.port_var.get()
            baud = self.baud_var.get()

            if not port:
                messagebox.showerror("Lỗi Kết Nối", "Vui lòng chọn một cổng COM hợp lệ!")
                return
            if not baud.isdigit():
                 messagebox.showerror("Lỗi Kết Nối", "Baud rate không hợp lệ!")
                 return
            baud = int(baud)

            try:
                # Đảm bảo đóng port cũ nếu còn mở
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    time.sleep(0.5) # Đợi một chút để OS giải phóng port

                self.log(f"Đang kết nối đến {port} @ {baud} baud...")
                logging.info(f"Đang kết nối đến {port} @ {baud} baud...")
                # Tăng timeout một chút
                self.serial = serial.Serial(port, baud, timeout=2.5, write_timeout=2.5)

                # Chờ Arduino khởi động lại sau khi mở Serial (quan trọng!)
                self.log("Chờ Arduino khởi động (2s)...")
                time.sleep(2.0)

                # Xóa buffer và gửi lệnh kiểm tra trạng thái
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # Gửi lệnh 'status' và chờ phản hồi 'OK' hoặc 'READY'
                test_command = "status"
                test_cmd_bytes = (test_command.strip() + '\n').encode('utf-8')
                logging.debug(f"Gửi lệnh kiểm tra: {test_command}")
                self.serial.write(test_cmd_bytes)

                # Chờ phản hồi trong một khoảng thời gian ngắn
                start_wait = time.time()
                response_received = ""
                ok_received = False
                while time.time() - start_wait < 3.0: # Chờ tối đa 3 giây
                    if self.serial.in_waiting > 0:
                        try:
                            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                logging.debug(f"Nhận phản hồi kết nối: '{line}'")
                                self.log(f"<- {line}") # Log cả phản hồi kết nối
                                response_received += line + "\n"
                                # Kiểm tra các phản hồi mong đợi
                                line_upper = line.upper()
                                if "OK" in line_upper or "READY" in line_upper or "SCARA_READY" in line_upper:
                                    ok_received = True
                                    break # Thoát vòng lặp khi nhận được OK/READY
                        except serial.SerialException as ser_e:
                             logging.warning(f"Lỗi đọc serial khi kiểm tra kết nối: {ser_e}")
                             self.log(f"⚠️ Lỗi đọc Serial: {ser_e}")
                             break # Thoát nếu có lỗi đọc
                    time.sleep(0.05)

                if ok_received:
                    self.is_connected = True
                    self.connect_btn.configure(text="Ngắt kết nối")
                    self.log(f"✓ Kết nối thành công với {port}")
                    logging.info(f"Kết nối thành công đến {port}")
                    # Tự động cài đặt tốc độ/gia tốc khi kết nối thành công
                    self.root.after(100, self.set_speed)
                    self.root.after(200, self.set_acceleration)
                    # Tự động về home sau khi kết nối? (Tùy chọn)
                    # self.root.after(500, self.home)
                else:
                    self.log(f"❌ Không nhận được phản hồi 'OK/READY' từ Arduino.")
                    logging.warning(f"Kết nối thất bại, không nhận được OK/READY. Phản hồi nhận được:\n{response_received}")
                    try:
                        self.serial.close()
                    except: pass # Đã đóng hoặc lỗi
                    messagebox.showwarning("Kết Nối Thất Bại", "Không nhận được phản hồi hợp lệ từ Arduino.\nKiểm tra firmware, baud rate và đảm bảo Arduino đã khởi động xong.")

            except serial.SerialException as e:
                logging.exception("Lỗi Serial khi kết nối:")
                self.log(f"❌ Lỗi Serial: {e}")
                messagebox.showerror("Lỗi Kết Nối", f"Lỗi Serial:\n{e}\n\nKiểm tra lại cổng COM và đảm bảo không có ứng dụng khác đang sử dụng.")
                if self.serial and self.serial.is_open:
                    try: self.serial.close()
                    except: pass
            except Exception as e:
                logging.exception("Lỗi không xác định khi kết nối:")
                self.log(f"❌ Lỗi không xác định: {e}")
                messagebox.showerror("Lỗi Kết Nối", f"Lỗi không xác định:\n{e}")
                if self.serial and self.serial.is_open:
                    try: self.serial.close()
                    except: pass

    def send_command(self, command, wait_for_response=True):
        """Thêm lệnh vào hàng đợi để gửi đến Arduino."""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.log("⚠️ Lỗi: Chưa kết nối với Arduino!")
            logging.warning("send_command: Chưa kết nối.")
            return False

        # Xác định xem có phải lệnh di chuyển không (quan trọng cho timeout và xử lý phản hồi)
        cmd_upper = command.strip().upper()
        is_move_cmd = (',' in command) or cmd_upper.startswith(("G0", "G1", "H")) or cmd_upper == "HOME"

        # Thêm lệnh, cờ chờ và cờ di chuyển vào queue
        self.command_queue.put((command, wait_for_response, is_move_cmd))
        logging.debug(f"Đã thêm vào queue: '{command}', Wait={wait_for_response}, Move={is_move_cmd}")
        return True

    def _execute_command(self, command, wait_for_response, is_move_cmd):
        """Thực thi lệnh gửi đến Arduino và xử lý phản hồi (chạy trong thread riêng)."""
        command_start_time = time.time()
        try:
            self.log(f"-> {command}", tag="SENT") # Log lệnh gửi đi
            cmd_bytes = (command.strip() + '\n').encode('utf-8')

            # --- Gửi lệnh (có khóa) ---
            with self.serial_lock:
                # self.serial.reset_input_buffer() # Cân nhắc xóa buffer trước khi gửi? Có thể làm mất phản hồi lệnh trước đó
                self.serial.write(cmd_bytes)
                # logging.debug(f"Đã gửi: {cmd_bytes}")

            # --- Chờ và xử lý phản hồi (nếu cần) ---
            if wait_for_response:
                # Xác định timeout dựa trên loại lệnh
                timeout = self.command_timeout * 2.5 if is_move_cmd else self.command_timeout

                responses = []
                move_completed_flag = False # Cờ báo nhận được MOVE_COMPLETE
                ok_ready_flag = False # Cờ báo nhận được OK/READY/COMPLETE chung
                final_theta1, final_theta2 = None, None # Lưu góc cuối từ phản hồi (nếu có)

                while time.time() - command_start_time < timeout:
                    line = ""
                    try:
                        # Đọc non-blocking với lock
                        with self.serial_lock:
                            if self.serial.in_waiting > 0:
                                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    except serial.SerialException as read_err:
                        logging.error(f"Lỗi đọc Serial khi chờ phản hồi cho '{command}': {read_err}")
                        self.log(f"⚠️ Lỗi đọc Serial: {read_err}", tag="ERROR")
                        # Cố gắng kết nối lại nếu lỗi nghiêm trọng
                        if "ClearCommError" in str(read_err) or "device disconnected" in str(read_err):
                            self.root.after(0, self.reconnect)
                        break # Thoát vòng chờ nếu có lỗi đọc

                    if line:
                        self.log(f"<- {line}", tag="RECEIVED") # Log phản hồi nhận được
                        responses.append(line)
                        self.last_response_time = time.time() # Cập nhật thời gian phản hồi cuối

                        # --- Phân tích các phản hồi quan trọng ---
                        line_upper = line.upper()

                        # 1. Kiểm tra hoàn thành di chuyển và lấy góc cuối
                        # Ưu tiên phản hồi này cho lệnh di chuyển
                        if "MOVE_COMPLETE AT:" in line_upper:
                            match = re.search(r"(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)", line) # Cho phép khoảng trắng quanh dấu phẩy
                            if match:
                                try:
                                    # Đây là góc chứ không phải XY
                                    final_theta1 = float(match.group(1))
                                    final_theta2 = float(match.group(2))
                                    logging.info(f"Xác nhận hoàn thành di chuyển tại Góc=({final_theta1:.2f}, {final_theta2:.2f})")
                                    move_completed_flag = True # Đánh dấu hoàn thành di chuyển
                                    # Không break ngay, có thể còn OK/READY sau đó
                                except ValueError:
                                    logging.warning(f"Không thể parse góc từ MOVE_COMPLETE: '{line}'")
                            else:
                                 logging.warning(f"Nhận MOVE_COMPLETE nhưng không tìm thấy góc: '{line}'")
                                 # Vẫn coi là xong di chuyển nếu không parse được góc? (tùy chọn)
                                 # move_completed_flag = True

                        # 2. Kiểm tra hoàn thành lệnh chung (OK, READY, COMPLETE)
                        # Dùng làm điều kiện thoát chính nếu không phải lệnh di chuyển
                        # hoặc nếu lệnh di chuyển đã báo MOVE_COMPLETE
                        if "OK" in line_upper or "READY" in line_upper or "COMPLETE" in line_upper:
                             ok_ready_flag = True
                             if not is_move_cmd or move_completed_flag: # Nếu không phải move cmd HOẶC move cmd đã báo xong
                                 break # Thoát vòng chờ

                        # 3. Cập nhật trạng thái bút từ phản hồi (dùng after để chạy trên main thread)
                        if "PEN_UP" in line_upper:
                            self.root.after(0, lambda: self._update_pen_status(False))
                        elif "PEN_DOWN" in line_upper:
                            self.root.after(0, lambda: self._update_pen_status(True))

                        # 4. Cập nhật cấu hình từ phản hồi (nếu có)
                        if "CONFIG:" in line_upper:
                             config_part = line.split(":")[-1].strip()
                             if config_part:
                                 # Cập nhật trên main thread
                                 self.root.after(0, lambda cfg=config_part: self._update_ik_config(cfg))

                        # 5. Xử lý lỗi từ Arduino
                        if "ERROR" in line_upper:
                             logging.error(f"Nhận lỗi từ Arduino cho lệnh '{command}': {line}")
                             self.log(f"⚠️ Arduino Error: {line}", tag="ERROR")
                             # Có thể dừng G-code nếu đang chạy?
                             if self.gcode_running:
                                 self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
                             break # Thoát vòng chờ khi có lỗi

                    else:
                        # Nếu không có dữ liệu, nghỉ ngắn để tránh CPU load cao
                        time.sleep(0.02)

                # --- Xử lý sau khi vòng chờ kết thúc ---
                command_duration = time.time() - command_start_time

                # Kiểm tra Timeout
                if not ok_ready_flag and wait_for_response:
                    # Nếu là lệnh di chuyển mà chưa nhận MOVE_COMPLETE cũng coi là timeout
                    if is_move_cmd and not move_completed_flag:
                         self.log(f"⚠️ Timeout ({timeout:.1f}s) chờ MOVE_COMPLETE cho: '{command}'", tag="ERROR")
                         logging.warning(f"Timeout ({timeout:.1f}s) chờ MOVE_COMPLETE cho: '{command}'")
                    elif not is_move_cmd:
                         self.log(f"⚠️ Timeout ({timeout:.1f}s) chờ OK/READY cho: '{command}'", tag="ERROR")
                         logging.warning(f"Timeout ({timeout:.1f}s) chờ OK/READY cho: '{command}'")
                    # Cân nhắc dừng G-code hoặc thử kết nối lại nếu timeout liên tục

                # --- Cập nhật vị trí và visualization SAU KHI lệnh di chuyển hoàn tất ---
                if is_move_cmd and move_completed_flag and final_theta1 is not None and final_theta2 is not None:
                     # Cập nhật trạng thái và UI trên main thread bằng góc cuối nhận được
                     self.root.after(0, lambda t1=final_theta1, t2=final_theta2, cfg=self.current_config: self.update_position_and_config(t1, t2, cfg))
                     logging.debug(f"Lệnh '{command}' hoàn thành sau {command_duration:.3f}s. Cập nhật vị trí cuối.")
                elif is_move_cmd and not move_completed_flag and ok_ready_flag:
                     # Trường hợp lạ: Nhận OK nhưng không có MOVE_COMPLETE cho lệnh di chuyển
                     logging.warning(f"Lệnh di chuyển '{command}' nhận OK nhưng không có MOVE_COMPLETE. Vị trí có thể không chính xác.")
                     # Cập nhật dựa trên góc target cuối cùng (kém chính xác hơn)
                     # self.root.after(0, self.update_position_display)
                     # self.root.after(0, self.update_animation)
                elif not is_move_cmd and ok_ready_flag:
                     # Lệnh không di chuyển hoàn thành
                     logging.debug(f"Lệnh '{command}' hoàn thành sau {command_duration:.3f}s.")

        except serial.SerialTimeoutException:
            self.log(f"⚠️ Lỗi Serial Timeout khi gửi: '{command}'", tag="ERROR")
            logging.error(f"Serial Timeout khi gửi: '{command}'")
            # Cân nhắc việc thử kết nối lại
            # self.root.after(0, self.reconnect)
        except serial.SerialException as e:
            self.log(f"❌ Lỗi Serial khi thực thi '{command}': {e}", tag="ERROR")
            logging.exception(f"Lỗi Serial khi thực thi '{command}':")
            # Cố gắng kết nối lại nếu lỗi nghiêm trọng
            if "ClearCommError" in str(e) or "device disconnected" in str(e):
                self.root.after(0, self.reconnect)
        except Exception as e:
            self.log(f"❌ Lỗi không xác định khi thực thi '{command}': {e}", tag="ERROR")
            logging.exception(f"Lỗi không xác định khi thực thi '{command}':")

    def _update_ik_config(self, config_str):
        """Cập nhật cấu hình IK và hiển thị trên UI."""
        if config_str != self.current_config:
            self.current_config = config_str
            self.config_label.config(text=config_str)
            logging.info(f"Cập nhật cấu hình IK thành: {config_str}")

    def _update_pen_status(self, is_down):
        """Cập nhật trạng thái bút và UI (chạy trên main thread)."""
        if self.pen_is_down == is_down: # Không thay đổi nếu trạng thái đã đúng
            return

        self.pen_is_down = is_down
        self.pen_label.configure(text="Hạ" if is_down else "Nâng")
        logging.debug(f"Cập nhật trạng thái bút: {'Hạ' if is_down else 'Nâng'}")

        if is_down:
            # Bắt đầu segment vẽ mới, lấy vị trí hiện tại chính xác
            # Đảm bảo lấy vị trí từ self.current_position đã được cập nhật
            x_now, y_now = self.current_position['x'], self.current_position['y']
            self.current_segment = {'x': [x_now], 'y': [y_now]}
            logging.debug(f"Bắt đầu segment vẽ mới tại ({x_now:.2f}, {y_now:.2f})")
        else:
            # Kết thúc segment vẽ hiện tại và lưu lại
            if self.current_segment and self.current_segment.get('x'):
                # Chỉ lưu nếu segment có nhiều hơn 1 điểm (tránh lưu các điểm đơn lẻ)
                if len(self.current_segment['x']) > 1:
                    self.trace_segments.append(self.current_segment.copy())
                    logging.debug(f"Kết thúc và lưu segment vẽ ({len(self.current_segment['x'])} điểm). Tổng cộng {len(self.trace_segments)} segments.")
                else:
                     logging.debug("Bỏ qua segment vẽ chỉ có 1 điểm.")
                self.current_segment = {} # Reset segment hiện tại

        # Cập nhật animation để hiển thị thay đổi (vết vẽ, trạng thái text)
        self.update_animation()

    def move_to_angle(self):
        """Di chuyển robot đến góc chỉ định."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return

        try:
            target_theta1 = float(self.angle1_entry.get())
            target_theta2 = float(self.angle2_entry.get())

            # --- SỬA LỖI F-STRING ---
            lim = self.robot.angle_limits
            th1_min, th1_max = lim['theta1_min'], lim['theta1_max'] # Lấy giá trị ra biến
            th2_min, th2_max = lim['theta2_min'], lim['theta2_max']

            # Kiểm tra giới hạn góc vật lý
            if not (th1_min <= target_theta1 <= th1_max):
                # Sử dụng biến trong f-string
                messagebox.showerror("Góc Không Hợp Lệ",
                                     f"Góc θ1 ({target_theta1:.2f}°) nằm ngoài giới hạn cho phép [{th1_min}°, {th1_max}°].")
                return
            if not (th2_min <= target_theta2 <= th2_max):
                 # Sử dụng biến trong f-string
                 messagebox.showerror("Góc Không Hợp Lệ",
                                      f"Góc θ2 ({target_theta2:.2f}°) nằm ngoài giới hạn cho phép [{th2_min}°, {th2_max}°].")
                 return
            # --- KẾT THÚC SỬA LỖI ---

            # --- Gửi lệnh di chuyển ---
            # Cập nhật UI và animation đến vị trí target NGAY LẬP TỨC để phản hồi người dùng
            self.current_angles['theta1'] = target_theta1 # Cập nhật góc target
            self.current_angles['theta2'] = target_theta2
            # Tính XY dự kiến từ góc target để cập nhật UI
            _, effector_pos_target, _ = self.robot.forward_kinematics(target_theta1, target_theta2)
            if effector_pos_target:
                 self.current_position['x'], self.current_position['y'] = effector_pos_target
            self.update_position_display() # Cập nhật hiển thị góc và XY dự kiến
            self.update_animation() # Cập nhật plot đến vị trí target

            command = f"{target_theta1:.2f},{target_theta2:.2f}"
            self.log(f"Yêu cầu di chuyển đến góc: θ1={target_theta1:.2f}, θ2={target_theta2:.2f}")
            self.send_command(command, wait_for_response=True) # Gửi lệnh và chờ hoàn thành

            # Vị trí cuối cùng sẽ được cập nhật lại trong _execute_command khi nhận MOVE_COMPLETE

        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị góc không hợp lệ! Vui lòng nhập số.")
        except Exception as e:
             logging.exception("Lỗi trong move_to_angle:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def move_to_xy(self):
        """Di chuyển robot đến tọa độ XY chỉ định."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return

        try:
            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())

            # 1. Kiểm tra điểm có trong vùng làm việc không
            if not self.robot.check_point_in_workspace(target_x, target_y):
                messagebox.showwarning("Ngoài Vùng Làm Việc",
                                     f"Vị trí ({target_x:.2f}, {target_y:.2f}) nằm ngoài vùng làm việc "
                                     "hoặc không thể tiếp cận được.")
                return

            # 2. Tính toán góc bằng Inverse Kinematics
            # Ưu tiên giữ nguyên cấu hình hiện tại
            target_theta1, target_theta2, selected_config = self.robot.inverse_kinematics(
                target_x, target_y, self.current_config
            )

            if target_theta1 is None or target_theta2 is None:
                # Trường hợp này ít xảy ra nếu check_point_in_workspace đã đúng
                messagebox.showerror("Lỗi Tính Toán",
                                     f"Không thể tính toán góc hợp lệ cho vị trí ({target_x:.2f}, {target_y:.2f}).")
                return

            # --- Gửi lệnh di chuyển ---
            # Cập nhật UI và animation đến vị trí target NGAY LẬP TỨC
            self.current_angles['theta1'] = target_theta1 # Cập nhật góc target
            self.current_angles['theta2'] = target_theta2
            self.current_position['x'] = target_x # Cập nhật XY target
            self.current_position['y'] = target_y
            self._update_ik_config(selected_config) # Cập nhật cấu hình đã chọn và UI
            self.update_position_display() # Cập nhật hiển thị góc, XY target, config
            self.update_animation() # Cập nhật plot đến vị trí target

            command = f"{target_theta1:.2f},{target_theta2:.2f}"
            self.log(f"Yêu cầu di chuyển đến XY: ({target_x:.2f}, {target_y:.2f}) -> Góc ({target_theta1:.2f}, {target_theta2:.2f}), Config: {selected_config}")
            self.send_command(command, wait_for_response=True) # Gửi lệnh và chờ hoàn thành

            # Vị trí góc/XY cuối cùng sẽ được cập nhật lại trong _execute_command

        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị tọa độ không hợp lệ! Vui lòng nhập số.")
        except Exception as e:
             logging.exception("Lỗi trong move_to_xy:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def update_position_and_config(self, final_theta1, final_theta2, final_config):
        """Cập nhật trạng thái góc, cấu hình và tính toán lại vị trí XY cuối cùng."""
        self.current_angles['theta1'] = final_theta1
        self.current_angles['theta2'] = final_theta2
        self._update_ik_config(final_config) # Cập nhật cấu hình và UI

        # Tính lại vị trí XY từ góc cuối cùng bằng FK
        _, effector_pos, _ = self.robot.forward_kinematics(final_theta1, final_theta2)
        if effector_pos:
            self.current_position['x'], self.current_position['y'] = effector_pos
            logging.info(f"Cập nhật trạng thái cuối: Angles=({final_theta1:.2f}, {final_theta2:.2f}), Pos=({effector_pos[0]:.2f}, {effector_pos[1]:.2f}), Config={final_config}")
        else:
            logging.warning("Không thể tính FK cho góc cuối cùng, vị trí XY có thể không chính xác.")
            # Giữ nguyên vị trí XY cũ hoặc đặt là None?

        # Cập nhật toàn bộ hiển thị UI và animation
        self.update_position_display()
        self.update_animation()

    def update_position_display(self):
        """Cập nhật các nhãn và entry hiển thị vị trí/góc/config trên UI."""
        # Cập nhật nhãn trạng thái
        x, y = self.current_position['x'], self.current_position['y']
        th1, th2 = self.current_angles['theta1'], self.current_angles['theta2']
        self.pos_label.config(text=f"X={x:.2f}, Y={y:.2f}")
        self.angle_label.config(text=f"θ1={th1:.2f}°, θ2={th2:.2f}°")
        # self.config_label đã được cập nhật trong _update_ik_config

        # Cập nhật các trường nhập liệu (chỉ khi không focus)
        try:
            focused_widget = self.root.focus_get()
            if focused_widget != self.x_entry:
                self.x_entry.delete(0, tk.END)
                self.x_entry.insert(0, f"{x:.2f}")
            if focused_widget != self.y_entry:
                self.y_entry.delete(0, tk.END)
                self.y_entry.insert(0, f"{y:.2f}")
            if focused_widget != self.angle1_entry:
                self.angle1_entry.delete(0, tk.END)
                self.angle1_entry.insert(0, f"{th1:.2f}")
            if focused_widget != self.angle2_entry:
                self.angle2_entry.delete(0, tk.END)
                self.angle2_entry.insert(0, f"{th2:.2f}")
        except tk.TclError:
            # Lỗi có thể xảy ra nếu widget không tồn tại hoặc bị hủy
            logging.warning("Lỗi cập nhật entry, widget có thể không tồn tại.")

    def pen_down(self):
        """Hạ bút xuống."""
        if self.is_connected:
            self.log("Yêu cầu hạ bút (d)")
            self.send_command("d", wait_for_response=True) # Chờ phản hồi để cập nhật trạng thái chính xác
            # Trạng thái isPenDown và segment vẽ sẽ được cập nhật trong _execute_command -> _update_pen_status

    def pen_up(self):
        """Nâng bút lên."""
        if self.is_connected:
            self.log("Yêu cầu nâng bút (u)")
            self.send_command("u", wait_for_response=True) # Chờ phản hồi để cập nhật trạng thái chính xác

    def home(self):
        """Gửi lệnh về vị trí Home và cập nhật UI/Animation."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return

        self.log("Yêu cầu về HOME (h)")
        # Cập nhật UI và animation đến vị trí home *trước* khi gửi lệnh
        home_th1 = self.robot.home_angles['theta1']
        home_th2 = self.robot.home_angles['theta2']
        # Tính XY home
        _, home_effector_pos, _ = self.robot.forward_kinematics(home_th1, home_th2)
        if home_effector_pos:
             self.current_position['x'], self.current_position['y'] = home_effector_pos
        self.current_angles['theta1'] = home_th1
        self.current_angles['theta2'] = home_th2
        # Giả sử home luôn là cấu hình elbow_down_down
        self._update_ik_config("elbow_down_down")
        self.update_position_display() # Cập nhật hiển thị góc và XY home
        self.update_animation() # Cập nhật plot đến vị trí home

        # Gửi lệnh và chờ hoàn thành
        self.send_command("h", wait_for_response=True)
        # Vị trí góc/XY cuối cùng sẽ được xác nhận lại trong _execute_command

    def emergency_stop(self):
        """Dừng khẩn cấp: Dừng G-code và gửi lệnh disable motors."""
        self.log("!!! DỪNG KHẨN CẤP !!!", tag="ERROR")
        logging.warning("Kích hoạt dừng khẩn cấp!")

        # 1. Dừng xử lý G-code nếu đang chạy
        if self.gcode_running:
            self.stop_gcode(is_emergency=True) # Gọi hàm stop với cờ khẩn cấp

        # 2. Xóa hàng đợi lệnh
        cleared_count = 0
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
                cleared_count += 1
            except queue.Empty:
                break
        if cleared_count > 0:
            self.log(f"Đã xóa {cleared_count} lệnh khỏi hàng đợi.")
            logging.info(f"Đã xóa {cleared_count} lệnh khỏi hàng đợi do dừng khẩn cấp.")

        # 3. Gửi lệnh disable motors ngay lập tức (không qua queue)
        if self.is_connected and self.serial and self.serial.is_open:
            try:
                self.log("Gửi lệnh disable trực tiếp...", tag="WARNING")
                disable_cmd = b"disable\n"
                with self.serial_lock:
                    # self.serial.reset_output_buffer() # Có thể không cần thiết
                    self.serial.write(disable_cmd)
                logging.info("Đã gửi lệnh disable khẩn cấp.")
                # Có thể chờ phản hồi ngắn gọn cho disable nếu cần
                # time.sleep(0.1)
                # self.serial.reset_input_buffer()
            except Exception as e:
                self.log(f"Lỗi gửi lệnh disable khẩn cấp: {e}", tag="ERROR")
                logging.exception("Lỗi gửi lệnh disable khẩn cấp:")
        else:
            self.log("Không thể gửi lệnh disable (chưa kết nối).", tag="WARNING")

        # 4. Đặt lại cờ command_processing nếu đang bị treo
        self.command_processing = False


    def set_speed(self):
        """Gửi lệnh cài đặt tốc độ tối đa cho Arduino."""
        if not self.is_connected:
            # messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối để cài đặt tốc độ!")
            return # Không cần thông báo nếu chỉ gọi tự động

        try:
            speed = int(self.speed_var.get())
            if speed > 0:
                command = f"f{speed}"
                self.log(f"Cài đặt tốc độ: {speed} steps/sec")
                self.send_command(command, wait_for_response=True) # Chờ xác nhận
            else:
                messagebox.showerror("Giá Trị Không Hợp Lệ", "Tốc độ phải là số dương.")
        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị tốc độ không hợp lệ! Vui lòng nhập số nguyên.")
        except Exception as e:
             logging.exception("Lỗi trong set_speed:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def set_acceleration(self):
        """Gửi lệnh cài đặt gia tốc cho Arduino."""
        if not self.is_connected:
            # messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối để cài đặt gia tốc!")
            return

        try:
            accel = int(self.accel_var.get())
            if accel > 0:
                command = f"a{accel}"
                self.log(f"Cài đặt gia tốc: {accel} steps/sec^2")
                self.send_command(command, wait_for_response=True) # Chờ xác nhận
            else:
                messagebox.showerror("Giá Trị Không Hợp Lệ", "Gia tốc phải là số dương.")
        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị gia tốc không hợp lệ! Vui lòng nhập số nguyên.")
        except Exception as e:
             logging.exception("Lỗi trong set_acceleration:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def log(self, message, tag="INFO"):
        """Hiển thị thông báo trong ô log của giao diện với tag màu."""

        # Đảm bảo hàm này được gọi từ main thread
        def _log_update():
            try:
                # --- THÊM KIỂM TRA ---
                # Kiểm tra xem root window và log_text còn tồn tại không
                if not self.root.winfo_exists() or not self.log_text.winfo_exists():
                    # print(f"Log widget destroyed, skipping log: {message}") # Log ra console nếu cần debug
                    return  # Không làm gì nếu widget đã bị hủy
                # --- KẾT THÚC KIỂM TRA ---

                # Giới hạn số dòng trong log để tránh quá tải
                num_lines = int(self.log_text.index('end - 1 line').split('.')[0])
                max_lines = 500
                if num_lines > max_lines:
                    self.log_text.delete('1.0', f'{num_lines - max_lines + 1}.0')

                timestamp = time.strftime("[%H:%M:%S]")
                # Insert message with timestamp and apply tag
                start_index = self.log_text.index(tk.END + "-1c")  # Vị trí bắt đầu của dòng mới
                self.log_text.insert(tk.END, f"{timestamp} {message}\n")
                end_index = self.log_text.index(tk.END + "-1c")  # Vị trí kết thúc của dòng mới
                # Apply tag if it exists
                if tag in self.log_text.tag_names():
                    self.log_text.tag_add(tag, start_index, end_index)

                self.log_text.see(tk.END)  # Tự động cuộn xuống dòng cuối
            except tk.TclError as e:
                # Lỗi có thể xảy ra nếu widget bị hủy giữa chừng kiểm tra và cập nhật
                logging.warning(f"TclError updating log_text (widget might be destroyed): {e}")
            except Exception as e:
                logging.exception("Lỗi không xác định khi cập nhật log:")

        # Sử dụng after(0, ...) để đảm bảo chạy trên main thread
        # Kiểm tra root tồn tại trước khi gọi after
        try:
            if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists():
                self.root.after(0, _log_update)
            else:
                print(f"LOG (root not available): {message}")  # Log ra console nếu root mất
        except Exception:  # Bắt lỗi chung nếu root không hợp lệ
            print(f"LOG (root error): {message}")

    # --- Các hàm xử lý G-code ---

    def load_gcode(self):
        """Mở hộp thoại để chọn và tải file G-code."""
        # Dừng G-code hiện tại nếu đang chạy
        if self.gcode_running:
            response = messagebox.askyesno("G-Code Đang Chạy",
                                           "G-code đang chạy. Bạn có muốn dừng và tải file mới không?")
            if response:
                self.stop_gcode()
                time.sleep(0.2) # Đợi một chút để dừng hẳn
            else:
                return # Hủy tải file mới

        file_path = filedialog.askopenfilename(
            title="Chọn file G-Code",
            filetypes=(("G-Code files", "*.gcode *.ngc *.nc *.tap"), ("All files", "*.*"))
        )

        if not file_path:
            return # Người dùng không chọn file

        try:
            with open(file_path, "r", encoding='utf-8', errors='ignore') as f:
                raw_content = f.readlines() # Đọc từng dòng

            # --- Tiền xử lý và tối ưu G-code ---
            processed_gcode = []
            cleaned_display_content = [] # Nội dung sạch để hiển thị

            for line_num, line in enumerate(raw_content):
                original_line = line.strip()
                cleaned_line = original_line

                # Bỏ comment (phần sau dấu ;)
                comment_index = cleaned_line.find(';')
                if comment_index != -1:
                    cleaned_line = cleaned_line[:comment_index].strip()

                # Bỏ comment dạng ngoặc tròn
                cleaned_line = re.sub(r'\([^)]*\)', '', cleaned_line).strip()

                # Bỏ các dòng trống hoặc chỉ chứa comment
                if not cleaned_line:
                    cleaned_display_content.append(original_line) # Giữ comment trong hiển thị
                    continue

                # Chuyển lệnh thành chữ hoa để dễ xử lý
                processed_line = cleaned_line.upper()

                # --- Các bước tối ưu/kiểm tra khác có thể thêm ở đây ---
                # Ví dụ: kiểm tra lệnh hợp lệ, chuẩn hóa định dạng,...

                processed_gcode.append(processed_line)
                cleaned_display_content.append(original_line) # Hiển thị dòng gốc

            # Hiển thị nội dung đã làm sạch (bao gồm comment)
            self.gcode_text.delete(1.0, tk.END)
            self.gcode_text.insert(tk.END, "\n".join(cleaned_display_content))

            # Lưu G-code đã xử lý và thông tin file
            self.gcode_lines = processed_gcode
            self.gcode_total_lines = len(self.gcode_lines)
            file_name = os.path.basename(file_path)
            self.file_label.config(text=file_name)

            self.log(f"Đã tải và xử lý file: {file_name} ({self.gcode_total_lines} lệnh thực thi)")
            logging.info(f"Đã tải G-code: {file_name}, {self.gcode_total_lines} lệnh.")

            # Reset tiến trình và kích hoạt nút chạy
            self.progress_var.set(0)
            self.progress_label.config(text="0.0% hoàn thành")
            self.run_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED)
            self.stop_gcode_btn.config(state=tk.DISABLED)
            # Xóa hình vẽ cũ khi tải file mới
            self.clear_drawing()

        except FileNotFoundError:
             messagebox.showerror("Lỗi Tải File", f"Không tìm thấy file:\n{file_path}")
             self.log(f"Lỗi: Không tìm thấy file G-code {file_path}", tag="ERROR")
        except Exception as e:
            logging.exception("Lỗi khi tải hoặc xử lý file G-code:")
            messagebox.showerror("Lỗi Tải File", f"Không thể đọc hoặc xử lý file:\n{e}")
            self.log(f"Lỗi tải file G-code: {e}", tag="ERROR")
            self.file_label.config(text="Lỗi tải file")
            self.gcode_lines = []
            self.gcode_total_lines = 0
            self.run_btn.config(state=tk.DISABLED)

    def run_gcode(self):
        """Bắt đầu hoặc tiếp tục chạy G-code."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return

        if not self.gcode_lines:
            messagebox.showwarning("Chưa Tải File", "Vui lòng tải một file G-code trước!")
            return

        # Nếu đang tạm dừng -> Tiếp tục
        if self.gcode_running and self.gcode_paused:
            self.gcode_paused = False
            self.pause_btn.config(text="Tạm dừng")
            self.log("Tiếp tục xử lý G-code...")
            logging.info("Tiếp tục G-code.")
            # Không cần làm gì thêm, thread xử lý sẽ tự chạy tiếp
            return

        # Nếu đang chạy (và không tạm dừng) -> Không làm gì
        if self.gcode_running and not self.gcode_paused:
            self.log("G-code đang chạy.")
            return

        # --- Bắt đầu chạy mới ---
        self.gcode_running = True
        self.gcode_paused = False
        self.gcode_line_num = 0 # Bắt đầu từ dòng đầu tiên
        self.update_progress(0.0) # Reset progress bar

        # Ghi log thời gian chờ
        delay = self.gcode_delay_var.get()
        self.log(f"Bắt đầu chạy G-code với delay {delay:.3f}s giữa các lệnh.")
        logging.info(f"Bắt đầu chạy G-code (delay={delay:.3f}s).")

        # Gửi lệnh 'gstart' đến Arduino (nếu firmware hỗ trợ)
        # self.send_command("gstart", wait_for_response=False) # Không cần chờ lệnh này

        # Cập nhật trạng thái các nút
        self.run_btn.config(state=tk.DISABLED)
        self.pause_btn.config(text="Tạm dừng", state=tk.NORMAL)
        self.stop_gcode_btn.config(state=tk.NORMAL)

        # Bắt đầu thread xử lý G-code
        # Đảm bảo không có thread cũ đang chạy (dù không nên xảy ra)
        if hasattr(self, '_gcode_thread') and self._gcode_thread.is_alive():
             logging.warning("Thread G-code cũ vẫn đang chạy? Bỏ qua bắt đầu mới.")
             return
        self._gcode_thread = threading.Thread(target=self._process_gcode_thread, daemon=True)
        self._gcode_thread.start()

    def _process_gcode_thread(self):
        """Thread chạy nền để xử lý từng dòng G-code."""
        logging.info("Thread xử lý G-code bắt đầu.")
        try:
            while self.gcode_line_num < self.gcode_total_lines and self.gcode_running:
                # --- Kiểm tra trạng thái tạm dừng ---
                while self.gcode_paused and self.gcode_running:
                    time.sleep(0.1) # Nghỉ khi đang tạm dừng
                if not self.gcode_running:
                    logging.info("Thread G-code: Nhận tín hiệu dừng.")
                    break # Thoát nếu bị dừng hẳn

                # --- Lấy và xử lý dòng G-code tiếp theo ---
                line_index = self.gcode_line_num # Lưu index hiện tại
                line = self.gcode_lines[line_index].strip()
                logging.debug(f"G-code Line {line_index + 1}/{self.gcode_total_lines}: {line}")

                # Highlight dòng hiện tại trên UI (gọi qua `after`)
                self.root.after(0, lambda ln=line_index: self.highlight_current_line(ln))

                # Gửi lệnh đã xử lý (nếu cần)
                # Hàm process_gcode_line sẽ đảm nhiệm việc tính toán IK và gửi lệnh phù hợp
                command_sent = self.process_gcode_line(line)

                # --- Chờ lệnh hoàn thành ---
                if command_sent:
                    wait_ok = self.wait_for_command_completion() # Chờ queue rỗng và command_processing=False
                    if not wait_ok: # Nếu chờ bị timeout hoặc G-code bị dừng
                         logging.warning(f"Chờ lệnh '{line}' không thành công hoặc bị ngắt.")
                         if not self.gcode_running: break # Thoát nếu G-code bị dừng trong lúc chờ

                # --- Cập nhật tiến trình và delay ---
                # Chỉ tăng nếu dòng hiện tại đã xử lý xong (tránh nhảy dòng khi đang chờ)
                if self.gcode_line_num == line_index:
                    self.gcode_line_num += 1
                    progress = (self.gcode_line_num / self.gcode_total_lines) * 100.0
                    self.root.after(0, lambda p=progress: self.update_progress(p))

                # Delay giữa các lệnh (sau khi lệnh đã hoàn thành)
                delay = self.gcode_delay_var.get()
                if delay > 0:
                    time.sleep(delay)

            # --- Kết thúc vòng lặp ---
            if self.gcode_running: # Nếu chạy hết mà không bị dừng
                logging.info("Hoàn thành xử lý G-code (hết file).")
                self.root.after(0, self.complete_gcode) # Gọi hàm hoàn thành trên main thread

        except Exception as e:
            logging.exception("Lỗi nghiêm trọng trong thread xử lý G-code:")
            self.log(f"❌ Lỗi G-code: {e}", tag="ERROR")
            # Đảm bảo dừng hẳn nếu có lỗi
            if self.gcode_running:
                 self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
        finally:
            logging.info("Thread xử lý G-code kết thúc.")


    def process_gcode_line(self, line):
        """Phân tích một dòng G-code, tính toán IK nếu cần và gửi lệnh đến Arduino."""
        # Bỏ qua nếu dòng trống (đã được lọc trước đó nhưng kiểm tra lại cho chắc)
        if not line:
            return False # Không có lệnh nào được gửi

        command_to_send = line # Mặc định gửi nguyên dòng lệnh
        needs_ik = False # Cờ báo có cần tính IK không

        # --- Xử lý G0/G1: Di chuyển tuyến tính/nhanh ---
        if line.startswith(("G0", "G1")):
            x_cart, y_cart, z_val = None, None, None
            feed_rate = None
            has_xy = False # Cờ báo có tọa độ XY trong lệnh không

            # Phân tích các tham số X, Y, Z, F
            parts = line.split()
            gcode_type = parts[0] # Lưu lại G0 hoặc G1
            other_params_str = "" # Lưu các tham số khác (vd: F)

            for part in parts[1:]:
                if not part: continue
                code = part[0]
                try:
                    value = float(part[1:])
                    if code == 'X':
                        x_cart = value
                        has_xy = True
                    elif code == 'Y':
                        y_cart = value
                        has_xy = True
                    elif code == 'Z': z_val = value
                    elif code == 'F':
                        feed_rate = value
                        other_params_str += f" F{value:.2f}" # Giữ lại F cho lệnh gửi đi (nếu cần)
                    # Bỏ qua các tham số khác không xử lý
                except ValueError:
                    logging.warning(f"Bỏ qua tham số không hợp lệ '{part}' trong dòng: {line}")
                    continue # Bỏ qua tham số lỗi

            # --- Xử lý tuần tự: Z -> F -> XY ---

            # 1. Xử lý Z (điều khiển bút)
            pen_action_done = False
            if z_val is not None:
                if z_val <= 0 and not self.pen_is_down: # Coi Z <= 0 là hạ bút
                    self.pen_down() # Gửi lệnh hạ bút
                    pen_action_done = True
                elif z_val > 0 and self.pen_is_down: # Coi Z > 0 là nâng bút
                    self.pen_up() # Gửi lệnh nâng bút
                    pen_action_done = True

                if pen_action_done:
                    wait_ok = self.wait_for_command_completion() # Chờ hạ/nâng bút xong
                    if not wait_ok or not self.gcode_running: return False # Thoát nếu chờ lỗi hoặc bị dừng

            # 2. Xử lý F (tốc độ)
            if feed_rate is not None and feed_rate > 0:
                 self.speed_var.set(str(int(feed_rate))) # Cập nhật UI (tùy chọn)
                 self.set_speed() # Gửi lệnh cài đặt tốc độ
                 wait_ok = self.wait_for_command_completion() # Chờ cài đặt xong
                 if not wait_ok or not self.gcode_running: return False

            # 3. Xử lý di chuyển XY (nếu có)
            if has_xy:
                # Nếu chỉ có X hoặc Y thì lỗi (cần cả hai)
                if x_cart is None or y_cart is None:
                     self.log(f"⚠️ Lỗi G-code: Lệnh {gcode_type} thiếu X hoặc Y: {line}", tag="WARNING")
                     logging.warning(f"Lệnh {gcode_type} thiếu X hoặc Y: {line}")
                     return False

                # Kiểm tra vùng làm việc
                if not self.robot.check_point_in_workspace(x_cart, y_cart):
                    self.log(f"⚠️ Điểm ({x_cart:.2f}, {y_cart:.2f}) ngoài vùng làm việc! Bỏ qua.", tag="WARNING")
                    logging.warning(f"G-code: Điểm ({x_cart:.2f}, {y_cart:.2f}) ngoài vùng làm việc.")
                    return False # Không gửi lệnh di chuyển

                # Tính toán IK
                target_theta1, target_theta2, selected_config = self.robot.inverse_kinematics(
                    x_cart, y_cart, self.current_config
                )

                if target_theta1 is not None and target_theta2 is not None:
                    # Cập nhật UI và Animation đến vị trí target *trước khi* gửi lệnh
                    # (Làm trong main thread)
                    def update_ui_before_move():
                        self.current_angles['theta1'] = target_theta1
                        self.current_angles['theta2'] = target_theta2
                        self.current_position['x'] = x_cart
                        self.current_position['y'] = y_cart
                        self._update_ik_config(selected_config)
                        self.update_position_display()
                        self.update_animation()
                    self.root.after(0, update_ui_before_move)

                    # Tạo lệnh góc để gửi đi (chỉ góc, không kèm G0/G1)
                    command_to_send = f"{target_theta1:.2f},{target_theta2:.2f}"
                    logging.debug(f"G-code XY ({x_cart:.2f},{y_cart:.2f}) -> Angle Cmd: {command_to_send}, Config: {selected_config}")
                    # Gửi lệnh góc
                    self.send_command(command_to_send, wait_for_response=True)
                    return True # Đã gửi lệnh di chuyển góc

                else: # Lỗi IK
                    self.log(f"❌ Lỗi IK cho ({x_cart:.2f}, {y_cart:.2f})! Bỏ qua.", tag="ERROR")
                    logging.error(f"G-code: Lỗi IK cho ({x_cart:.2f}, {y_cart:.2f}).")
                    return False # Không gửi lệnh nếu IK lỗi
            else:
                 # Nếu không có XY, chỉ có Z hoặc F đã xử lý ở trên
                 return False # Không có lệnh di chuyển XY nào được gửi trong bước này

        # --- Xử lý các lệnh G-code phổ biến khác ---
        elif line.startswith("G4"): # G4 P<ms>: Dừng (Dwell)
             try:
                 # Tìm giá trị P bằng regex để linh hoạt hơn
                 match = re.search(r"P(\d+\.?\d*)", line)
                 if match:
                     p_value_ms = float(match.group(1))
                     dwell_sec = p_value_ms / 1000.0
                     if dwell_sec > 0:
                         self.log(f"Tạm dừng G-code {dwell_sec:.3f} giây (G4)")
                         time.sleep(dwell_sec) # Dừng thread xử lý G-code
                 else:
                      logging.warning(f"Không tìm thấy tham số P hợp lệ trong G4: {line}")
                 return False # Không gửi lệnh gì đến Arduino
             except Exception as e:
                 logging.warning(f"Lỗi xử lý G4 dwell time từ '{line}': {e}")
                 return False
        elif line.startswith("G28"): # Về Home
            command_to_send = "h" # Gửi lệnh 'h' tương ứng
        elif line.startswith(("M3", "M03")): # Hạ bút
            command_to_send = "d" # Gửi lệnh 'd'
        elif line.startswith(("M5", "M05")): # Nâng bút
            command_to_send = "u" # Gửi lệnh 'u'
        elif line.startswith("M17"): # Bật động cơ
            command_to_send = "enable"
        elif line.startswith(("M18", "M84")): # Tắt động cơ
            command_to_send = "disable"
        # --- Các lệnh G/M khác ---
        # elif line.startswith("G90"): # Tọa độ tuyệt đối (thường là mặc định)
        #     # Có thể không cần gửi nếu firmware mặc định là G90
        #     # command_to_send = line
        #     pass # Bỏ qua
        # elif line.startswith("G91"): # Tọa độ tương đối (Cần xử lý phức tạp hơn nếu hỗ trợ)
        #     self.log("⚠️ Cảnh báo: Chế độ G91 (tọa độ tương đối) chưa được hỗ trợ đầy đủ.", tag="WARNING")
        #     logging.warning("G91 không được hỗ trợ đầy đủ.")
        #     return False # Bỏ qua lệnh này
        # elif line.startswith("G20"): # Đơn vị Inch
        #      self.log("⚠️ Lỗi: Đơn vị Inch (G20) không được hỗ trợ.", tag="ERROR")
        #      logging.error("G20 không được hỗ trợ.")
        #      return False
        # elif line.startswith("G21"): # Đơn vị mét (mm) - Mặc định
        #      pass # Bỏ qua
        else:
             # Nếu là lệnh không xác định, ghi log và có thể bỏ qua hoặc gửi đi
             logging.warning(f"Bỏ qua lệnh G-code không xác định hoặc chưa hỗ trợ: {line}")
             # self.send_command(command_to_send, wait_for_response=True) # Gửi nếu muốn thử
             return False # Bỏ qua

        # Gửi lệnh đã xác định (h, d, u, enable, disable,...)
        if command_to_send:
            self.send_command(command_to_send, wait_for_response=True)
            return True # Đã gửi lệnh

        return False # Mặc định là không gửi lệnh

    def wait_for_command_completion(self):
        """Chờ cho đến khi hàng đợi lệnh rỗng và lệnh hiện tại xử lý xong."""
        # logging.debug("Bắt đầu chờ hoàn thành lệnh...")
        start_wait = time.time()
        # Giảm max_wait để phản ứng nhanh hơn nếu có vấn đề
        max_wait = self.command_timeout * 2.0 if self.command_processing else self.command_timeout

        while time.time() - start_wait < max_wait:
            # Kiểm tra cả queue và cờ command_processing
            queue_empty = self.command_queue.empty()
            processing_done = not self.command_processing

            if queue_empty and processing_done:
                # logging.debug("Hàng đợi rỗng và không có lệnh nào đang xử lý.")
                return True # Hoàn thành

            # Kiểm tra nếu G-code bị dừng hoặc tạm dừng
            if not self.gcode_running or self.gcode_paused:
                logging.info("wait_for_command_completion: G-code bị dừng/tạm dừng.")
                return False # Thoát chờ

            time.sleep(0.03) # Nghỉ ngắn hơn một chút

        # Nếu vượt quá thời gian chờ
        logging.warning(f"Timeout ({max_wait:.1f}s) khi chờ lệnh hoàn thành!")
        self.log(f"⚠️ Timeout chờ lệnh hoàn thành!", tag="WARNING")
        # Dừng G-code nếu bị timeout khi chờ lệnh
        if self.gcode_running:
             self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
        return False

    def highlight_current_line(self, line_num):
        """Highlight dòng G-code đang thực thi trong Text widget."""
        try:
            # Xóa tag highlight cũ
            self.gcode_text.tag_remove("current_line", "1.0", tk.END)

            # Tính toán vị trí dòng (line_num bắt đầu từ 0, Text widget bắt đầu từ 1)
            start_index = f"{line_num + 1}.0"
            end_index = f"{line_num + 1}.end"

            # Thêm tag highlight mới
            self.gcode_text.tag_add("current_line", start_index, end_index)
            # Cấu hình tag trong hàm create_widgets hoặc một lần duy nhất
            if "current_line" not in self.gcode_text.tag_names():
                 self.gcode_text.tag_config("current_line", background="yellow", relief="raised")

            # Tự động cuộn để nhìn thấy dòng hiện tại (chỉ cuộn nếu cần)
            if not self.gcode_text.dlineinfo(start_index): # Kiểm tra xem dòng có hiển thị không
                 self.gcode_text.see(start_index)

        except tk.TclError:
            # Lỗi có thể xảy ra nếu line_num không hợp lệ hoặc widget bị hủy
            logging.warning(f"Lỗi highlight dòng {line_num + 1}, có thể nằm ngoài phạm vi.")
        except Exception as e:
             logging.exception(f"Lỗi không xác định khi highlight dòng {line_num + 1}:")

    def update_progress(self, progress):
        """Cập nhật thanh tiến trình và nhãn %."""
        try:
            clamped_progress = max(0.0, min(100.0, progress)) # Đảm bảo trong khoảng 0-100
            self.progress_var.set(clamped_progress)
            self.progress_label.config(text=f"{clamped_progress:.1f}% hoàn thành")
        except tk.TclError:
             logging.warning("Lỗi cập nhật progress bar, widget có thể đã bị hủy.")
        except Exception as e:
             logging.exception("Lỗi không xác định khi cập nhật progress:")

    def pause_gcode(self):
        """Tạm dừng hoặc tiếp tục thực thi G-code."""
        if not self.gcode_running:
            return

        self.gcode_paused = not self.gcode_paused

        if self.gcode_paused:
            self.pause_btn.config(text="Tiếp tục")
            self.log("Đã tạm dừng G-code.")
            logging.info("G-code tạm dừng.")
        else:
            self.pause_btn.config(text="Tạm dừng")
            self.log("Tiếp tục chạy G-code.")
            logging.info("G-code tiếp tục.")

    def stop_gcode(self, is_emergency=False):
        """Dừng hẳn việc thực thi G-code."""
        if not self.gcode_running:
            return # Đã dừng rồi

        self.gcode_running = False # Đặt cờ dừng cho thread xử lý
        self.gcode_paused = False # Đảm bảo không còn ở trạng thái paused

        if is_emergency:
            self.log("Đang dừng G-code (khẩn cấp)...", tag="WARNING")
            logging.warning("Dừng G-code khẩn cấp.")
        else:
            self.log("Đang dừng G-code...")
            logging.info("Dừng G-code.")

        # Gửi lệnh 'gend' đến Arduino (nếu firmware hỗ trợ và không phải dừng khẩn cấp?)
        # Có thể không cần thiết nếu đã gửi disable
        # if not is_emergency:
        #     self.send_command("gend", wait_for_response=False)

        # Reset giao diện G-code (chạy trên main thread)
        self.root.after(0, self._reset_gcode_ui)
        self.log("Đã dừng G-code.")

    def complete_gcode(self):
        """Được gọi khi G-code chạy xong một cách bình th��ờng."""
        if self.gcode_running: # Kiểm tra lại cờ (dù thường là false ở đây)
             self.gcode_running = False
        self.log("✓ Hoàn thành G-code.", tag="INFO")
        logging.info("G-code hoàn thành.")

        # Gửi lệnh 'gend' (nếu cần)
        # self.send_command("gend", wait_for_response=False)

        # Reset giao diện G-code
        self._reset_gcode_ui()
        # Đảm bảo thanh tiến trình hiển thị 100%
        self.update_progress(100.0)

    def _reset_gcode_ui(self):
        """Đặt lại các thành phần UI liên quan đến G-code về trạng thái ban đầu."""
        try:
            # Chỉ bật nút Run nếu có G-code đã tải
            run_state = tk.NORMAL if self.gcode_lines else tk.DISABLED
            self.run_btn.config(state=run_state)
            self.pause_btn.config(text="Tạm dừng", state=tk.DISABLED)
            self.stop_gcode_btn.config(state=tk.DISABLED)
            # Xóa highlight dòng hiện tại
            self.gcode_text.tag_remove("current_line", "1.0", tk.END)
        except tk.TclError:
            logging.warning("Lỗi reset G-code UI, widget có thể đã bị hủy.")
        except Exception as e:
             logging.exception("Lỗi không xác định khi reset G-code UI:")

    def reconnect(self):
        """Thử kết nối lại với Arduino nếu kết nối bị mất."""
        # Chỉ thử kết nối lại nếu đang không kết nối
        if not self.is_connected:
             self.log("⚠️ Mất kết nối? Đang thử kết nối lại...", tag="WARNING")
             logging.warning("Mất kết nối? Đang thử kết nối lại...")
             # Gọi lại hàm kết nối
             self.toggle_connection()
        else:
             # Nếu đang báo là kết nối nhưng có lỗi -> Ngắt kết nối trước rồi thử lại
             self.log("⚠️ Có lỗi xảy ra, đang thử ngắt và kết nối lại...", tag="WARNING")
             logging.warning("Có lỗi xảy ra, đang thử ngắt và kết nối lại...")
             self.toggle_connection() # Ngắt kết nối
             # Đợi một chút rồi thử kết nối lại
             self.root.after(1500, self.toggle_connection)


    def on_closing(self):
        """Xử lý sự kiện đóng cửa sổ."""
        logging.info("Đóng ứng dụng...")
        self.log("Đang đóng ứng dụng...")
        # Dừng G-code nếu đang chạy
        if self.gcode_running:
            self.stop_gcode(is_emergency=True)
            time.sleep(0.1) # Chờ một chút

        # Gửi lệnh disable motor trước khi thoát
        if self.is_connected:
            self.emergency_stop() # Gọi hàm dừng khẩn cấp để disable motor
            time.sleep(0.3) # Đợi lệnh disable được gửi đi
            if self.serial and self.serial.is_open:
                try:
                    self.serial.close()
                    logging.info("Đã đóng cổng Serial.")
                except Exception as e:
                     logging.error(f"Lỗi khi đóng cổng Serial: {e}")

        # Dừng các thread khác nếu cần (thread queue là daemon, sẽ tự thoát)

        self.root.destroy() # Đóng cửa sổ Tkinter
        logging.info("Ứng dụng đã đóng.")

# --- Hàm Main ---
def main():
    root = tk.Tk()
    # Sử dụng ttk theme để giao diện đẹp hơn (tùy chọn)
    try:
        style = ttk.Style(root)
        # Thử các theme khác nhau: 'clam', 'alt', 'default', 'classic', 'vista', 'xpnative'
        available_themes = style.theme_names()
        logging.info(f"Available themes: {available_themes}")
        if 'vista' in available_themes:
            style.theme_use('vista')
        logging.info(f"Using theme: {style.theme_use()}")
    except Exception as e:
        logging.warning(f"Không thể đặt theme ttk: {e}")

    app = ScaraGUI(root)
    # Xử lý sự kiện đóng cửa sổ
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        logging.info("Nhận KeyboardInterrupt, đang đóng ứng dụng...")
        app.on_closing()

if __name__ == "__main__":
    main()