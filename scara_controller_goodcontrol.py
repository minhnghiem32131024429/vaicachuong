import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext # Thêm scrolledtext nếu chưa có
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
matplotlib.use('TkAgg') # Đảm bảo backend TkAgg được sử dụng
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import sys # Thêm dòng này ở đầu file
import logging
# Removed animation import as it wasn't used directly for drawing updates
from matplotlib.patches import Rectangle, Circle # Removed FancyArrowPatch

# --- Cấu hình Logging ---
# Lấy đường dẫn thư mục chứa script
script_dir = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(script_dir, 'scara_controller.log')

logging.basicConfig(
    level=logging.INFO, # Mức log (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(log_file_path, mode='a'), # Ghi log vào file, 'a' để nối tiếp
        logging.StreamHandler(sys.stdout) # Gửi log INFO trở lên ra stdout
    ]
)
logging.info(f"Log file path: {log_file_path}")


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
        # Thêm kiểm tra d_sq > 0 trước khi sqrt
        if d_sq < 1e-12: # Nếu khoảng cách quá nhỏ, coi như không hợp lệ
             # logging.warning(f"FK: Khoảng cách giữa khớp 1 và 3 quá nhỏ (d^2={d_sq:.2f})")
             return None, None, None
        d = np.sqrt(d_sq)


        # Kiểm tra xem có thể tạo thành tam giác không (thêm dung sai nhỏ epsilon)
        epsilon = 1e-6
        # Thêm kiểm tra d > epsilon để tránh lỗi chia cho 0
        if d < epsilon or d > L2 + L3 + epsilon or d < abs(L2 - L3) - epsilon:
             # logging.warning(f"FK: Không thể tạo tam giác với d={d:.2f}, L2={L2}, L3={L3}")
             return None, None, None # Khoảng cách không hợp lệ

        try:
            # Sử dụng định lý cos để tìm góc tại khớp 1 (x2,y2) - góc đối diện L3
            cos_angle_at_j1 = (L2**2 + d_sq - L3**2) / (2 * L2 * d)
            cos_angle_at_j1 = np.clip(cos_angle_at_j1, -1.0, 1.0) # Đảm bảo trong [-1, 1]
            angle_at_j1 = np.arccos(cos_angle_at_j1)

            # Góc của vector từ khớp 1 (x2,y2) đến khớp 3 (x4,y4)
            angle_j1_to_j2 = np.arctan2(y4 - y2, x4 - x2)

            # Tính góc của cánh tay L2 (có hai giải pháp, chọn một dựa trên cấu hình)
            # Giả sử cấu hình "elbow down" là ưu tiên (khớp ở giữa hướng xuống tương đối)
            angle_L2 = angle_j1_to_j2 - angle_at_j1 # Elbow down configuration

            # Tính tọa độ điểm cuối (x3, y3)
            x3 = x2 + L2 * np.cos(angle_L2)
            y3 = y2 + L2 * np.sin(angle_L2)

            # Trả về tọa độ các khớp quan trọng
            return (x2, y2), (x3, y3), (x4, y4)

        except (ValueError, ZeroDivisionError) as e:
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
            # Thêm kiểm tra L13_sq > 0
            if L13_sq < epsilon**2: L13 = epsilon # Gán giá trị nhỏ nếu quá gần
            else: L13 = np.sqrt(L13_sq)


            # Kiểm tra tầm với của motor 1
            if L13 > L1 + L2 + epsilon or L13 < abs(L1 - L2) - epsilon:
                # logging.warning(f"IK: Điểm ({x3:.2f}, {y3:.2f}) ngoài tầm với motor 1 (L13={L13:.2f})")
                return None, None, None

            beta1 = np.arctan2(dy13, dx13)
            # Thêm kiểm tra mẫu số khác 0
            denominator1 = 2 * L1 * L13
            if denominator1 < epsilon: return None, None, None # Tránh chia cho 0
            cos_alpha1 = (L1**2 + L13_sq - L2**2) / denominator1
            cos_alpha1 = np.clip(cos_alpha1, -1.0, 1.0) # Đảm bảo trong [-1, 1]
            alpha1 = np.arccos(cos_alpha1)

            theta1_sol1 = np.degrees(beta1 - alpha1) # Elbow Up (alpha1 trừ đi)
            theta1_sol2 = np.degrees(beta1 + alpha1) # Elbow Down (alpha1 cộng vào)

            # --- Tính cho Motor 2 (theta2) ---
            dx53, dy53 = x3 - x5, y3 - y5
            L53_sq = dx53**2 + dy53**2
            # Thêm kiểm tra L53_sq > 0
            if L53_sq < epsilon**2: L53 = epsilon
            else: L53 = np.sqrt(L53_sq)

            # Kiểm tra tầm với của motor 2
            if L53 > L4 + L3 + epsilon or L53 < abs(L4 - L3) - epsilon:
                # logging.warning(f"IK: Điểm ({x3:.2f}, {y3:.2f}) ngoài tầm với motor 2 (L53={L53:.2f})")
                return None, None, None

            beta5 = np.arctan2(dy53, dx53)
            # Thêm kiểm tra mẫu số khác 0
            denominator5 = 2 * L4 * L53
            if denominator5 < epsilon: return None, None, None # Tránh chia cho 0
            cos_alpha5 = (L4**2 + L53_sq - L3**2) / denominator5
            cos_alpha5 = np.clip(cos_alpha5, -1.0, 1.0) # Đảm bảo trong [-1, 1]
            alpha5 = np.arccos(cos_alpha5)

            theta2_sol1 = np.degrees(beta5 + alpha5) # Elbow Up (alpha5 cộng vào)
            theta2_sol2 = np.degrees(beta5 - alpha5) # Elbow Down (alpha5 trừ đi)

            # --- Kết hợp và kiểm tra giải pháp với giới hạn góc ---
            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol1 <= th2_max:
                solutions.append(("elbow_up_up", theta1_sol1, theta2_sol1))
            if th1_min <= theta1_sol1 <= th1_max and th2_min <= theta2_sol2 <= th2_max:
                solutions.append(("elbow_up_down", theta1_sol1, theta2_sol2))
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol1 <= th2_max:
                solutions.append(("elbow_down_up", theta1_sol2, theta2_sol1))
            if th1_min <= theta1_sol2 <= th1_max and th2_min <= theta2_sol2 <= th2_max:
                solutions.append(("elbow_down_down", theta1_sol2, theta2_sol2))

            if not solutions:
                # logging.warning(f"IK: Không tìm thấy giải pháp hợp lệ cho ({x3:.2f}, {y3:.2f}) sau khi kiểm tra giới hạn góc.")
                return None, None, None

            # --- Chọn giải pháp tốt nhất ---
            preferred_solutions = [s for s in solutions if s[0] == current_config]
            if preferred_solutions:
                return preferred_solutions[0][1], preferred_solutions[0][2], preferred_solutions[0][0]

            down_down_solution = [s for s in solutions if s[0] == "elbow_down_down"]
            if down_down_solution:
                 return down_down_solution[0][1], down_down_solution[0][2], down_down_solution[0][0]

            best_solution = solutions[0]
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
            return False
        if not (r2_min_sq - epsilon <= d2_sq <= r2_max_sq + epsilon):
            return False

        # Kiểm tra động học ngược có giải pháp hợp lệ không
        theta1, theta2, _ = self.inverse_kinematics(x, y)
        if theta1 is None or theta2 is None:
            return False

        return True

    def create_workspace_visualization_points(self, resolution=0.5):
        """Tạo danh sách các điểm (x, y) trong vùng làm việc."""
        points = []
        bounds = self.workspace_bounds
        # Tạo lưới điểm với resolution
        y_start = max(bounds['bottom'], resolution) # Bắt đầu Y từ giá trị dương nhỏ nhất
        x_coords = np.arange(bounds['left'], bounds['right'] + resolution, resolution)
        y_coords = np.arange(y_start, bounds['top'] + resolution, resolution)
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
        self.root.title("SCARA Controller - Flipped X") # Thêm ghi chú vào title
        try:
            self.root.state('zoomed')
        except tk.TclError:
            try:
                m = root.maxsize()
                root.geometry('{}x{}+0+0'.format(*m))
            except:
                root.geometry("1200x800")

        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 90.0, 'theta2': 90.0}
        # Tọa độ nội bộ (X dương sang trái)
        fk_x_internal, fk_y = self.robot.forward_kinematics(90.0, 90.0)[1] if self.robot.forward_kinematics(90.0, 90.0)[1] else (float('nan'), float('nan'))
        self.current_position = {'x': fk_x_internal, 'y': fk_y}
        self.pen_is_down = False
        self.current_config = "elbow_down_down"

        # Tính vị trí XY ban đầu từ góc home (đã làm ở trên)
        if not math.isnan(self.current_position['x']):
             x_display = -self.current_position['x'] # Lật để log
             logging.info(f"Vị trí XY nội bộ ban đầu: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
             logging.info(f"Vị trí XY hiển thị ban đầu: ({x_display:.2f}, {self.current_position['y']:.2f})")
        else:
             logging.warning("Không thể tính vị trí XY ban đầu từ góc home.")
             # Đặt giá trị an toàn
             self.current_position['x'], self.current_position['y'] = 10, 30 # X nội bộ là 10 (display là -10)

        self.trace_segments = []
        self.current_segment = {}
        self.workspace_points = []
        self._workspace_visualization_active = False

        self.gcode_lines = []
        self.gcode_running = False
        self.gcode_paused = False
        self.gcode_line_num = 0
        self.gcode_total_lines = 0
        self.stop_gcode_flag = False # Thêm cờ dừng G-code

        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue() # Thêm queue cho phản hồi
        self.command_processing = False
        self.serial_lock = threading.Lock()
        self.last_response_time = time.time()
        self.command_timeout = 10.0
        self.move_timeout_factor = 2.5 # Thêm hệ số timeout cho lệnh di chuyển

        self.create_widgets()
        self.update_ports()

        # Bắt đầu thread đọc serial và xử lý command queue
        self.read_thread = None # Sẽ khởi tạo trong connect
        self.stop_read_thread = False # Cờ dừng thread đọc
        self.process_command_thread = threading.Thread(target=self._process_command_queue, daemon=True)
        self.process_command_thread.start()

        # Bắt đầu thread xử lý response queue
        self.process_response_thread = threading.Thread(target=self._process_response_queue, daemon=True)
        self.process_response_thread.start()


        self.root.after(100, self.update_animation)
        logging.info("Khởi tạo ScaraGUI hoàn tất.")

    # --- THÊM HÀM XỬ LÝ RESPONSE QUEUE ---
    def _process_response_queue(self):
        """Vòng lặp xử lý các phản hồi từ Arduino đã được đưa vào response_queue."""
        while True:
            try:
                response_line = self.response_queue.get(block=True, timeout=None) # Chờ vô hạn
                if response_line is None: # Tín hiệu dừng thread
                    break

                # Xử lý phản hồi trên main thread để cập nhật UI an toàn
                self.root.after(0, self._handle_arduino_response, response_line)

            except queue.Empty:
                # Sẽ không xảy ra với block=True, timeout=None
                continue
            except Exception as e:
                logging.exception("Lỗi trong thread xử lý response queue:")
                time.sleep(0.1) # Nghỉ ngắn trước khi thử lại

    # --- THÊM HÀM XỬ LÝ PHẢN HỒI TRÊN MAIN THREAD ---
    def _handle_arduino_response(self, line):
        """Xử lý một dòng phản hồi từ Arduino (chạy trên main thread)."""
        # Hàm này sẽ được gọi bởi _process_response_queue thông qua self.root.after
        line_upper = line.upper()

        # 1. Cập nhật trạng thái bút
        if "PEN_UP" in line_upper:
            self._update_pen_status(False)
        elif "PEN_DOWN" in line_upper:
            self._update_pen_status(True)

        # 2. Cập nhật cấu hình IK (nếu có)
        elif "CONFIG:" in line_upper:
             config_part = line.split(":")[-1].strip()
             if config_part:
                 self._update_ik_config(config_part)

        # 3. Xử lý hoàn thành di chuyển (MOVE_COMPLETE AT:)
        elif "MOVE_COMPLETE AT:" in line_upper:
            match = re.search(r"(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)", line)
            if match:
                try:
                    final_theta1 = float(match.group(1))
                    final_theta2 = float(match.group(2))
                    logging.info(f"Xác nhận hoàn thành di chuyển tại Góc=({final_theta1:.2f}, {final_theta2:.2f})")
                    # Gọi hàm cập nhật vị trí cuối cùng
                    self.update_position_and_config(final_theta1, final_theta2, self.current_config) # Truyền cả config hiện tại
                except ValueError:
                    logging.warning(f"Không thể parse góc từ MOVE_COMPLETE: '{line}'")
            else:
                 logging.warning(f"Nhận MOVE_COMPLETE nhưng không tìm thấy góc: '{line}'")

        # 4. Các phản hồi khác (STATUS, ENABLED, DISABLED, CALIBRATED, etc.)
        # Có thể cập nhật các label trạng thái khác ở đây nếu cần
        elif "MOTORS_ENABLED" in line_upper:
             # self.motors_enabled = True # Cập nhật biến trạng thái nếu có
             pass
        elif "MOTORS_DISABLED" in line_upper:
             # self.motors_enabled = False
             pass
        # elif "STATUS:" in line_upper:
             # Xử lý phản hồi status nếu cần lấy thông tin ban đầu

        # Không cần xử lý "OK", "READY", "ERROR" ở đây vì chúng được xử lý trong _execute_command

    # --- SỬA LẠI _process_command_queue ---
    def _process_command_queue(self):
        """Vòng lặp xử lý hàng đợi lệnh gửi đến Arduino."""
        while True:
            try:
                command_item = self.command_queue.get(block=True, timeout=None) # Chờ lệnh mới
                if command_item is None: # Tín hiệu dừng thread
                    logging.info("Thread xử lý command queue đang dừng.")
                    break

                command, wait_response, is_move_cmd = command_item
                self.command_processing = True # Đặt cờ đang xử lý

                # Thực thi lệnh và chờ (nếu cần)
                success = self._execute_command(command, wait_response, is_move_cmd)

                self.command_processing = False # Xóa cờ
                self.command_queue.task_done() # Báo hiệu lệnh đã xử lý xong

                # Log kết quả (tùy chọn)
                # logging.debug(f"Xử lý xong lệnh '{command}', Success={success}")

                # Nghỉ ngắn để tránh chiếm CPU hoàn toàn
                time.sleep(0.01)

            except queue.Empty:
                # Sẽ không xảy ra với block=True, timeout=None
                continue
            except Exception as e:
                logging.exception("Lỗi nghiêm trọng trong _process_command_queue:")
                self.command_processing = False # Đảm bảo reset cờ
                time.sleep(0.1) # Nghỉ dài hơn nếu có lỗi

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

        # Tọa độ XY (Hiển thị X đã lật)
        xy_frame = ttk.Frame(manual_frame)
        xy_frame.grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=5)
        xy_frame.columnconfigure(1, weight=1) # Cho Entry X giãn
        xy_frame.columnconfigure(3, weight=1) # Cho Entry Y giãn
        xy_frame.columnconfigure(4, weight=2) # Cho Button giãn nhiều hơn

        ttk.Label(xy_frame, text="X:").grid(row=0, column=0, padx=(0, 2))
        self.x_entry = ttk.Entry(xy_frame, width=7)
        self.x_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        # Hiển thị X display ban đầu
        x_display_init = -self.current_position.get('x', 0.0) if not math.isnan(self.current_position.get('x', float('nan'))) else 0.0
        self.x_entry.insert(0, f"{x_display_init:.2f}")


        ttk.Label(xy_frame, text="Y:").grid(row=0, column=2, padx=(5, 2))
        self.y_entry = ttk.Entry(xy_frame, width=7)
        self.y_entry.grid(row=0, column=3, padx=(0, 5), sticky=tk.EW)
        y_init = self.current_position.get('y', 0.0) if not math.isnan(self.current_position.get('y', float('nan'))) else 0.0
        self.y_entry.insert(0, f"{y_init:.2f}")


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
        self.speed_var = tk.StringVar(value="8000") # Tăng tốc độ mặc định firmware
        speed_entry = ttk.Entry(speed_accel_frame, textvariable=self.speed_var, width=6)
        speed_entry.grid(row=0, column=1, padx=(0, 5), sticky=tk.EW)
        ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_speed).grid(row=0, column=2, padx=(0, 10))

        ttk.Label(speed_accel_frame, text="Gia tốc:").grid(row=0, column=3, padx=(10, 2), sticky=tk.W)
        self.accel_var = tk.StringVar(value="3000") # Tăng gia tốc mặc định firmware
        accel_entry = ttk.Entry(speed_accel_frame, textvariable=self.accel_var, width=6)
        accel_entry.grid(row=0, column=4, padx=(0, 5), sticky=tk.EW)
        ttk.Button(speed_accel_frame, text="Set", width=4, command=self.set_acceleration).grid(row=0, column=5, padx=(0, 0))

        # Chức năng bổ sung (Home, Stop, Calibrate)
        func_frame = ttk.Frame(manual_frame)
        func_frame.grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=5)
        func_frame.columnconfigure(0, weight=1)
        func_frame.columnconfigure(1, weight=1)
        func_frame.columnconfigure(2, weight=1) # Thêm cột cho Calibrate

        home_btn = ttk.Button(func_frame, text="HOME (H)", command=self.home)
        home_btn.grid(row=0, column=0, padx=(0, 5), sticky=tk.EW)
        self.root.bind('<h>', lambda event=None: self.home()) # Phím tắt
        self.root.bind('<H>', lambda event=None: self.home()) # Phím tắt (Shift+h)

        # Thêm nút Calibrate
        calibrate_btn = ttk.Button(func_frame, text="Calib 90,90", command=self.calibrate_position)
        calibrate_btn.grid(row=0, column=1, padx=5, sticky=tk.EW)

        stop_btn = ttk.Button(func_frame, text="DỪNG KHẨN CẤP", command=self.emergency_stop, style="Emergency.TButton")
        stop_btn.grid(row=0, column=2, padx=(5, 0), sticky=tk.EW)


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

        # Sử dụng scrolledtext thay vì Text + Scrollbar thủ công
        self.gcode_text = scrolledtext.ScrolledText(code_view_frame, height=8, wrap=tk.NONE, borderwidth=1, relief="sunken",
                                                    font=("Courier New", 9)) # Đổi font dễ đọc code
        self.gcode_text.grid(row=0, column=0, sticky=tk.NSEW)
        # Không cần tạo và grid scrollbar riêng

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
        self.gcode_delay_var = tk.DoubleVar(value=0.01) # Giảm delay mặc định từ 0.05
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

        # Sử dụng scrolledtext cho Log
        self.log_text = scrolledtext.ScrolledText(log_frame, height=5, wrap=tk.WORD, borderwidth=1, relief="sunken",
                                                  font=("Segoe UI", 9), state=tk.DISABLED) # Bắt đầu với state DISABLED
        self.log_text.grid(row=0, column=0, sticky=tk.NSEW)
        # Configure tags for different log levels
        self.log_text.tag_configure("INFO", foreground="black")
        self.log_text.tag_configure("WARNING", foreground="orange")
        self.log_text.tag_configure("ERROR", foreground="red", font=("Segoe UI", 9, "bold"))
        self.log_text.tag_configure("DEBUG", foreground="gray")
        self.log_text.tag_configure("SENT", foreground="blue")
        self.log_text.tag_configure("RECEIVED", foreground="green")
        self.log_text.tag_configure("SUCCESS", foreground="dark green") # Thêm tag SUCCESS

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

        self.workspace_btn = ttk.Button(vis_ctrl_frame, text="Hiện/Ẩn Vùng làm việc", command=self.toggle_workspace_visualization) # Đổi tên biến
        self.workspace_btn.pack(side=tk.LEFT, padx=5)

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

        bounds = self.robot.workspace_bounds
        padding = 5

        # --- LẬT GIỚI HẠN TRỤC X VÀ ĐIỂM TĨNH ---
        # Đảo ngược giới hạn X để trục X dương hiển thị bên trái
        self.ax.set_xlim(-(bounds['right'] + padding), -(bounds['left'] - padding))
        # Giữ nguyên Y, có thể giới hạn Y > 0 nếu muốn
        # self.ax.set_ylim(bounds['bottom'] - padding, bounds['top'] + padding)
        self.ax.set_ylim(0 - padding, bounds['top'] + padding) # Chỉ hiển thị Y >= 0
        self.ax.set_xlabel("X (cm) - Flipped Display") # Ghi chú trục bị lật
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_title("SCARA Robot Visualization")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.set_aspect('equal', adjustable='box')

        # Vẽ vị trí các motor tĩnh với X đã lật
        x1_vis = -self.robot.robot_params['x1'] # Lật X của motor 1
        y1 = self.robot.robot_params['y1']
        x5_vis = -self.robot.robot_params['x5'] # Lật X của motor 2
        y5 = self.robot.robot_params['y5']
        self.ax.plot(x1_vis, y1, 'ks', markersize=8, label='Motor 1 (Y)')
        self.ax.plot(x5_vis, y5, 'ks', markersize=8, label='Motor 2 (X)')
        # Tùy chỉnh vị trí text nếu cần
        self.ax.text(x1_vis + 1, y1 + 1, "M1(Y)", fontsize=8, color='black', ha='left', va='bottom')
        self.ax.text(x5_vis + 1, y5 + 1, "M2(X)", fontsize=8, color='black', ha='left', va='bottom')
        # --- KẾT THÚC LẬT ---

        # --- Khởi tạo các thành phần động (sẽ được cập nhật trong update_animation) ---
        self.arm1_line, = self.ax.plot([], [], 'b-', lw=5, alpha=0.7, label='Arm 1-2', solid_capstyle='round')
        self.arm2_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, label='Arm 2-3', solid_capstyle='round')
        self.arm3_line, = self.ax.plot([], [], 'g-', lw=5, alpha=0.7, solid_capstyle='round')
        self.arm4_line, = self.ax.plot([], [], 'c-', lw=5, alpha=0.7, label='Arm 4-5', solid_capstyle='round')
        self.joints, = self.ax.plot([], [], 'o', color='darkred', markersize=8, label='Joints')
        self.effector, = self.ax.plot([], [], 'o', color='magenta', markersize=10, label='Effector')
        self.trace_line, = self.ax.plot([], [], '-', color='red', lw=1.5, label='Drawing Trace')

        # --- Đặt vị trí text dựa trên giới hạn đã lật ---
        # Đặt ở góc trên bên trái của plot (dựa trên giới hạn X đã lật)
        text_x_pos = -(bounds['right'] + padding * 0.8) # Gần cạnh trái của plot
        text_y_pos = bounds['top'] + padding * 0.8     # Gần cạnh trên của plot
        self.angle_text = self.ax.text(text_x_pos, text_y_pos, "", fontsize=9, ha='left', va='top',
                                       bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none'))
        self.pen_status_text = self.ax.text(text_x_pos, text_y_pos - 3.0, "", fontsize=9, ha='left', va='top',
                                            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7, ec='none'))
        # --- KẾT THÚC ĐẶT VỊ TRÍ TEXT ---

        # Hiển thị legend (tùy chọn)
        # self.ax.legend(loc='upper right', fontsize='small')

        self.fig.tight_layout() # Tự động điều chỉnh layout
        self.canvas.draw()

        # Reset cờ hiển thị workspace và xóa scatter cũ nếu có
        self._workspace_visualization_active = False
        if hasattr(self, 'workspace_scatter'):
            try:
                self.workspace_scatter.remove()
            except ValueError: pass
            del self.workspace_scatter

    def toggle_workspace_visualization(self):
        """Hiện hoặc ẩn các điểm biểu diễn vùng làm việc."""
        if self._workspace_visualization_active:
            # Nếu đang hiện -> Xóa đi
            if hasattr(self, 'workspace_scatter'):
                try:
                    self.workspace_scatter.remove()
                except ValueError: pass
                del self.workspace_scatter
                self._workspace_visualization_active = False
                self.log("Đã ẩn visualization vùng làm việc.")
                self.canvas.draw_idle()
                logging.info("Đã ẩn workspace visualization.")
            else:
                 self._workspace_visualization_active = False
                 logging.warning("Trạng thái workspace không nhất quán, đã reset.")
        else:
            # Nếu đang ẩn -> Vẽ lên
            if not self.workspace_points: # Nếu chưa có dữ liệu thì tạo
                self.log("Đang tính toán các điểm vùng làm việc...")
                logging.info("Bắt đầu tạo điểm workspace visualization...")
                self.workspace_btn.config(state=tk.DISABLED) # Vô hiệu hóa nút
                threading.Thread(target=self._generate_and_draw_workspace, daemon=True).start()
            else:
                # Nếu đã có dữ liệu thì vẽ luôn
                self._draw_workspace_points()

    def _generate_and_draw_workspace(self):
        """Hàm chạy trong thread để tạo dữ liệu và vẽ vùng làm việc."""
        generated_points = []
        try:
            generated_points = self.robot.create_workspace_visualization_points(resolution=0.5)
        except Exception as e:
            logging.exception("Lỗi khi tạo visualization vùng làm việc:")
            self.log(f"Lỗi tạo vùng làm việc: {e}", tag="ERROR")

        self.workspace_points = generated_points # Lưu lại dù có lỗi hay không

        # Gọi hàm vẽ trên main thread và kích hoạt lại nút
        def _task_after_generate():
            self.workspace_btn.config(state=tk.NORMAL) # Kích hoạt lại nút
            if self.workspace_points:
                 self._draw_workspace_points() # Yêu cầu vẽ
            else:
                 self.log("Không tạo được điểm nào cho vùng làm việc.", tag="WARNING")

        if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists():
            self.root.after(0, _task_after_generate)

    def _draw_workspace_points(self):
        """Vẽ các điểm vùng làm việc lên plot (chạy trên main thread)."""
        if not self.workspace_points:
            logging.warning("Không có điểm workspace để vẽ.")
            return

        if hasattr(self, 'workspace_scatter'):
             try: self.workspace_scatter.remove()
             except ValueError: pass
             del self.workspace_scatter

        try:
            # self.workspace_points chứa tọa độ internal (chưa lật)
            internal_x_coords, y_coords = zip(*self.workspace_points)

            # --- LẬT X ĐỂ VẼ WORKSPACE ---
            vis_x_coords = [-x for x in internal_x_coords]
            # --- KẾT THÚC LẬT ---

            # Vẽ các điểm đã lật X
            self.workspace_scatter = self.ax.scatter(vis_x_coords, y_coords, s=1, c='lightblue', alpha=0.3, label='Workspace', zorder=0)
            self._workspace_visualization_active = True
            self.log(f"Đã hiển thị {len(self.workspace_points)} điểm vùng làm việc.")
            self.canvas.draw_idle()
            logging.info("Đã vẽ workspace visualization.")
        except Exception as e:
            logging.exception("Lỗi khi vẽ điểm vùng làm việc:")
            self.log(f"Lỗi vẽ vùng làm việc: {e}", tag="ERROR")

    def update_animation(self):
        """Cập nhật vị trí các thành phần động trên visualization plot."""
        theta1_deg = self.current_angles.get('theta1', 90.0) # Dùng get để tránh lỗi nếu chưa có key
        theta2_deg = self.current_angles.get('theta2', 90.0)

        try:
            # Tính toán vị trí các khớp bằng FK (kết quả là tọa độ nội bộ)
            joint1_pos, effector_pos_internal, joint3_pos = self.robot.forward_kinematics(theta1_deg, theta2_deg)

            if joint1_pos is None or effector_pos_internal is None or joint3_pos is None:
                return

            # --- LẬT TẤT CẢ TỌA ĐỘ X ĐỂ VẼ ---
            x1_vis = -self.robot.robot_params['x1']
            y1 = self.robot.robot_params['y1']
            x5_vis = -self.robot.robot_params['x5']
            y5 = self.robot.robot_params['y5']
            x2_vis, y2 = -joint1_pos[0], joint1_pos[1]
            x3_vis, y3 = -effector_pos_internal[0], effector_pos_internal[1]
            x4_vis, y4 = -joint3_pos[0], joint3_pos[1]
            # --- KẾT THÚC LẬT ---

            # Cập nhật đường nối các cánh tay (dùng tọa độ _vis)
            self.arm1_line.set_data([x1_vis, x2_vis], [y1, y2])
            self.arm2_line.set_data([x2_vis, x3_vis], [y2, y3])
            self.arm3_line.set_data([x3_vis, x4_vis], [y3, y4])
            self.arm4_line.set_data([x4_vis, x5_vis], [y4, y5])

            # Cập nhật vị trí các khớp (dùng tọa độ _vis)
            self.joints.set_data([x2_vis, x4_vis], [y2, y4])

            # Cập nhật vị trí effector (dùng tọa độ _vis)
            self.effector.set_data([x3_vis], [y3])

            # --- Cập nhật vết vẽ (Sử dụng tọa độ _vis) ---
            if self.pen_is_down:
                if not self.current_segment or not self.current_segment.get('x'):
                    self.current_segment = {'x': [x3_vis], 'y': [y3]}
                else:
                    dx = x3_vis - self.current_segment['x'][-1]
                    dy = y3 - self.current_segment['y'][-1]
                    if dx*dx + dy*dy > 1e-6:
                         self.current_segment['x'].append(x3_vis)
                         self.current_segment['y'].append(y3)

            all_trace_x, all_trace_y = [], []
            for segment in self.trace_segments:
                 if segment and segment.get('x'):
                     all_trace_x.extend(segment['x'])
                     all_trace_y.extend(segment['y'])
                     all_trace_x.append(None)
                     all_trace_y.append(None)
            if self.current_segment and self.current_segment.get('x'):
                 all_trace_x.extend(self.current_segment['x'])
                 all_trace_y.extend(self.current_segment['y'])
            self.trace_line.set_data(all_trace_x, all_trace_y)
            # --- KẾT THÚC CẬP NHẬT VẾT VẼ ---

            # --- Cập nhật Text ---
            self.angle_text.set_text(f"θ1={theta1_deg:.2f}° | θ2={theta2_deg:.2f}°")
            pen_text = f"Bút: {'Hạ' if self.pen_is_down else 'Nâng'}"
            self.pen_status_text.set_text(pen_text)
            self.pen_status_text.set_color('red' if self.pen_is_down else 'black')

            # Yêu cầu vẽ lại canvas
            if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists():
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
        self.trace_line.set_data([], [])
        self.log("Đã xóa hình vẽ trên mô phỏng.")
        logging.info("Đã xóa hình vẽ.")
        self.canvas.draw_idle()

    def update_ports(self):
        """Cập nhật danh sách cổng COM có sẵn."""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                current_port = self.port_var.get()
                if current_port not in ports:
                    self.port_var.set(ports[0])
            else:
                self.port_var.set("")
            logging.info(f"Cập nhật cổng COM: {ports}")
        except Exception as e:
            logging.error(f"Lỗi khi cập nhật cổng COM: {e}")
            self.port_combo['values'] = []
            self.port_var.set("")

    # --- SỬA LẠI toggle_connection ---
    def toggle_connection(self):
        """Kết nối hoặc ngắt kết nối với Arduino."""
        if self.is_connected:
            # --- Ngắt kết nối ---
            self.disconnect() # Gọi hàm disconnect mới
        else:
            # --- Kết nối ---
            self.connect() # Gọi hàm connect mới

    # --- TÁCH HÀM connect VÀ disconnect ---
    # --- THAY THẾ HÀM connect CŨ BẰNG HÀM NÀY ---
    def connect(self):
        """Thực hiện kết nối với Arduino và xác nhận phản hồi ban đầu."""
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
            if self.serial and self.serial.is_open:
                self.serial.close()
                time.sleep(0.5)  # Đợi OS giải phóng port

            self.log(f"Đang kết nối đến {port} @ {baud} baud...")
            logging.info(f"Đang kết nối đến {port} @ {baud} baud...")
            # Mở cổng serial, timeout đọc ban đầu có thể ngắn hơn
            self.serial = serial.Serial(port, baud, timeout=0.5, write_timeout=1.0)

            self.log("Chờ Arduino khởi động và gửi tín hiệu sẵn sàng (tối đa 5s)...")
            time.sleep(1.0)  # Chờ ngắn để Arduino bắt đầu gửi

            # --- Logic mới: Lắng nghe tích cực phản hồi ban đầu ---
            start_wait = time.time()
            initial_ok_received = False
            initial_buffer = ""
            connection_success = False
            max_initial_wait = 5.0  # Tăng thời gian chờ ban đầu

            while time.time() - start_wait < max_initial_wait:
                try:
                    # Đọc dữ liệu có sẵn (non-blocking)
                    if self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                        initial_buffer += data
                        # Xử lý từng dòng trong buffer
                        while '\n' in initial_buffer:
                            line, initial_buffer = initial_buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                logging.debug(f"Nhận phản hồi khởi tạo: '{line}'")
                                self.log(f"<- {line}")  # Log ra UI
                                line_upper = line.upper()
                                # Kiểm tra các tín hiệu sẵn sàng quan trọng
                                if "OK" in line_upper or "READY" in line_upper or "SCARA_READY" in line_upper:
                                    initial_ok_received = True
                                    connection_success = True  # Đánh dấu thành công
                                    logging.info("Nhận được tín hiệu sẵn sàng từ Arduino.")
                                    break  # Thoát vòng đọc khi nhận được tín hiệu
                    if connection_success:
                        break  # Thoát vòng chờ chính

                except serial.SerialException as ser_e:
                    logging.warning(f"Lỗi đọc serial khi chờ tín hiệu sẵn sàng: {ser_e}")
                    self.log(f"⚠️ Lỗi đọc Serial khi chờ: {ser_e}", tag="ERROR")
                    break  # Thoát nếu có lỗi đọc
                except Exception as e:
                    logging.exception("Lỗi không xác định khi chờ tín hiệu sẵn sàng:")
                    break

                time.sleep(0.05)  # Nghỉ ngắn giữa các lần đọc
            # --- Kết thúc Logic mới ---

            if connection_success:
                # --- Kết nối thành công ---
                self.is_connected = True
                self.connect_btn.configure(text="Ngắt kết nối")
                self.log(f"✓ Kết nối thành công với {port}", tag="SUCCESS")
                logging.info(f"Kết nối thành công đến {port}")

                # Xóa buffer trước khi bắt đầu thread đọc chính thức
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # Bắt đầu thread đọc serial chính thức
                self.stop_read_thread = False
                if self.read_thread and self.read_thread.is_alive():
                    self.read_thread.join(timeout=0.5)  # Đảm bảo thread cũ đã dừng hẳn
                self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
                self.read_thread.start()

                # Tự động cài đặt tốc độ/gia tốc và về home
                self.root.after(100, self.set_speed)
                self.root.after(200, self.set_acceleration)
                self.root.after(500, self.home)
            else:
                # --- Kết nối thất bại ---
                self.log(f"❌ Không nhận được tín hiệu sẵn sàng từ Arduino sau {max_initial_wait}s.", tag="ERROR")
                logging.warning(f"Kết nối thất bại, không nhận được tín hiệu sẵn sàng.")
                self.disconnect(silent=True)  # Ngắt kết nối ngầm
                messagebox.showwarning("Kết Nối Thất Bại",
                                       "Không nhận được phản hồi hợp lệ từ Arduino khi khởi tạo.\nKiểm tra firmware, baud rate và đảm bảo Arduino đã khởi động.")

        except serial.SerialException as e:
            logging.exception("Lỗi Serial khi kết nối:")
            self.log(f"❌ Lỗi Serial: {e}", tag="ERROR")
            messagebox.showerror("Lỗi Kết Nối",
                                 f"Lỗi Serial:\n{e}\n\nKiểm tra lại cổng COM và đảm bảo không có ứng dụng khác đang sử dụng.")
            self.disconnect(silent=True)
        except Exception as e:
            logging.exception("Lỗi không xác định khi kết nối:")
            self.log(f"❌ Lỗi không xác định: {e}", tag="ERROR")
            messagebox.showerror("Lỗi Kết Nối", f"Lỗi không xác định:\n{e}")
            self.disconnect(silent=True)

    # --- CÁC HÀM KHÁC GIỮ NGUYÊN ---
    # ... (disconnect, _read_serial_loop, send_command, _execute_command, etc.) ...

    def disconnect(self, silent=False):
        """Thực hiện ngắt kết nối."""
        if not self.is_connected and not silent:
             logging.info("Đã ở trạng thái ngắt kết nối.")
             return

        is_was_connected = self.is_connected # Lưu trạng thái cũ
        self.is_connected = False # Đặt trạng thái ngay lập tức

        # Dừng G-code nếu đang chạy
        if self.gcode_running:
            self.stop_gcode(is_emergency=True)

        # Dừng thread đọc serial
        self.stop_read_thread = True
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=0.5) # Chờ ngắn

        # Xóa hàng đợi lệnh và phản hồi
        self._clear_queues()

        # Đóng cổng serial
        serial_closed = False
        if self.serial and self.serial.is_open:
            try:
                # Gửi lệnh disable trực tiếp
                with self.serial_lock:
                    disable_cmd = b"disable\n"
                    self.serial.write(disable_cmd)
                    logging.info("Đã gửi lệnh disable trước khi đóng cổng.")
                time.sleep(0.1)
                self.serial.close()
                serial_closed = True
                logging.info("Đã đóng cổng Serial.")
            except Exception as e:
                logging.error(f"Lỗi khi đóng cổng Serial: {e}")
                if not silent: self.log(f"Lỗi đóng cổng: {e}", tag="ERROR")

        self.serial = None # Xóa đối tượng serial

        # Cập nhật UI
        self.connect_btn.configure(text="Kết nối")
        if is_was_connected and not silent: # Chỉ log nếu thực sự đã ngắt kết nối
             self.log("Đã ngắt kết nối khỏi Arduino.")
             logging.info("Đã ngắt kết nối.")
        elif serial_closed and not silent: # Log nếu chỉ đóng cổng thành công
             self.log("Đã đóng cổng Serial.")

    def _clear_queues(self):
        """Xóa sạch command và response queue."""
        cleared_cmd = 0
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
                cleared_cmd += 1
            except queue.Empty: break
        if cleared_cmd > 0: logging.debug(f"Đã xóa {cleared_cmd} lệnh khỏi command queue.")

        cleared_resp = 0
        while not self.response_queue.empty():
            try:
                self.response_queue.get_nowait()
                cleared_resp +=1
            except queue.Empty: break
        if cleared_resp > 0: logging.debug(f"Đã xóa {cleared_resp} phản hồi khỏi response queue.")


    # --- SỬA LẠI _read_serial_loop ---
    def _read_serial_loop(self):
        """Vòng lặp đọc dữ liệu từ cổng Serial và đưa vào response_queue."""
        logging.info("Thread đọc Serial bắt đầu.")
        buffer = ""
        while not self.stop_read_thread:
            if self.serial and self.serial.is_open:
                try:
                    # Đọc non-blocking với lock
                    bytes_to_read = 0
                    with self.serial_lock:
                         bytes_to_read = self.serial.in_waiting

                    if bytes_to_read > 0:
                        with self.serial_lock:
                             data = self.serial.read(bytes_to_read).decode('utf-8', errors='ignore')
                        buffer += data
                        # Xử lý từng dòng hoàn chỉnh trong buffer
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                # Đưa dòng vào response queue để xử lý ở thread khác
                                self.response_queue.put(line)
                                # Log ra UI ngay lập tức (tùy chọn, có thể gây chậm)
                                # self.log(f"<- {line}", tag="RECEIVED")
                    else:
                        # Nếu không có dữ liệu, nghỉ ngắn
                        time.sleep(0.01)

                except serial.SerialException as e:
                    logging.error(f"Lỗi đọc Serial: {e}")
                    self.log(f"⚠️ Lỗi đọc Serial: {e}", tag="ERROR")
                    # Yêu cầu main thread ngắt kết nối an toàn
                    if self.is_connected: # Chỉ gọi nếu đang thực sự kết nối
                        self.root.after(0, self.disconnect)
                    break # Thoát vòng lặp đọc
                except Exception as e:
                     logging.exception("Lỗi không xác định trong thread đọc serial:")
                     # Cân nhắc dừng thread hoặc ngắt kết nối
                     if self.is_connected:
                          self.root.after(0, self.disconnect)
                     break
            else:
                # Nếu serial không mở hoặc không tồn tại, thoát thread
                logging.warning("Thread đọc Serial: Cổng Serial không mở hoặc không tồn tại.")
                break
        logging.info("Thread đọc Serial kết thúc.")

    def send_command(self, command, wait_for_response=True):
        """Thêm lệnh vào hàng đợi để gửi đến Arduino."""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.log("⚠️ Lỗi: Chưa kết nối với Arduino!", tag="ERROR") # Thêm tag ERROR
            logging.warning("send_command: Chưa kết nối.")
            return False

        cmd_upper = command.strip().upper()
        # Sửa logic xác định lệnh di chuyển
        is_move_cmd = (',' in command) or cmd_upper == "H" or cmd_upper == "HOME" or cmd_upper.startswith(("G0", "G1"))

        self.command_queue.put((command, wait_for_response, is_move_cmd))
        logging.debug(f"Đã thêm vào queue: '{command}', Wait={wait_for_response}, Move={is_move_cmd}")
        return True

    # --- SỬA LẠI _execute_command ---
    def _execute_command(self, command, wait_for_response, is_move_cmd):
        """Thực thi lệnh gửi đến Arduino và chờ phản hồi (chạy trong thread xử lý queue)."""
        command_start_time = time.time()
        ok_received = False
        move_complete_received = False
        error_received = False

        try:
            self.log(f"-> {command}", tag="SENT") # Log lệnh gửi đi
            cmd_bytes = (command.strip() + '\n').encode('utf-8')

            # Gửi lệnh (có khóa)
            with self.serial_lock:
                if not self.serial or not self.serial.is_open:
                     logging.warning(f"Bỏ qua gửi lệnh '{command}' do serial không mở.")
                     return False # Thất bại
                self.serial.write(cmd_bytes)

            # Chờ và xử lý phản hồi (nếu cần)
            if wait_for_response:
                timeout = self.command_timeout * self.move_timeout_factor if is_move_cmd else self.command_timeout
                # logging.debug(f"Chờ phản hồi cho '{command}' với timeout {timeout:.1f}s")

                while time.time() - command_start_time < timeout:
                    try:
                        # Lấy phản hồi từ response_queue (đã được đọc bởi thread khác)
                        response = self.response_queue.get(block=True, timeout=0.1) # Chờ tối đa 0.1s
                        # Không cần gọi _handle_arduino_response ở đây nữa vì nó chạy riêng
                        # logging.debug(f"Response Queue nhận: '{response}' cho lệnh '{command}'")

                        # --- Phân tích các phản hồi quan trọng để quyết định KẾT THÚC CHỜ ---
                        line_upper = response.upper()

                        # 1. Kiểm tra hoàn thành di chuyển
                        if is_move_cmd and "MOVE_COMPLETE AT:" in line_upper:
                            move_complete_received = True
                            # Không break ngay, chờ thêm OK

                        # 2. Kiểm tra hoàn thành lệnh chung (OK, READY, COMPLETE)
                        if "OK" in line_upper or "READY" in line_upper or "COMPLETE" in line_upper:
                             ok_received = True
                             # Điều kiện thoát vòng chờ:
                             # - Lệnh không phải di chuyển: Chỉ cần OK/READY.
                             # - Lệnh di chuyển: Cần cả MOVE_COMPLETE và OK/READY.
                             if not is_move_cmd or move_complete_received:
                                 # logging.debug(f"Thoát chờ cho '{command}'. MoveOK:{move_complete_received}, OK:{ok_received}")
                                 break # Thoát vòng chờ

                        # 3. Kiểm tra lỗi từ Arduino
                        if "ERROR" in line_upper:
                             logging.error(f"Nhận lỗi từ Arduino cho lệnh '{command}': {response}")
                             self.log(f"⚠️ Arduino Error: {response}", tag="ERROR")
                             error_received = True
                             if self.gcode_running: # Dừng G-code nếu có lỗi
                                 self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
                             break # Thoát vòng chờ khi có lỗi

                    except queue.Empty:
                        # Không có phản hồi trong timeout ngắn, tiếp tục chờ
                        # Kiểm tra nếu G-code bị dừng thì thoát chờ
                        if self.gcode_running == False and is_move_cmd: # Nếu G-code dừng trong khi chờ lệnh di chuyển
                            logging.info(f"G-code dừng, hủy chờ lệnh '{command}'")
                            return False # Coi như lệnh không hoàn thành
                        continue
                    except Exception as e:
                         logging.exception(f"Lỗi khi lấy phản hồi từ queue cho '{command}':")
                         error_received = True
                         break # Thoát nếu có lỗi queue

                # --- Xử lý sau khi vòng chờ kết thúc ---
                command_duration = time.time() - command_start_time

                if error_received:
                    logging.warning(f"Lệnh '{command}' kết thúc do lỗi sau {command_duration:.3f}s.")
                    return False # Thất bại

                # Kiểm tra Timeout
                if not ok_received and wait_for_response:
                    log_msg = f"⚠️ Timeout ({timeout:.1f}s) chờ OK/READY cho: '{command}'"
                    if is_move_cmd and not move_complete_received:
                        log_msg = f"⚠️ Timeout ({timeout:.1f}s) chờ MOVE_COMPLETE cho: '{command}'"
                    self.log(log_msg, tag="ERROR")
                    logging.warning(log_msg)
                    if self.gcode_running: # Dừng G-code nếu timeout
                         self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
                    return False # Thất bại

                # Kiểm tra trường hợp lạ: OK nhưng không có MOVE_COMPLETE cho lệnh di chuyển
                elif is_move_cmd and ok_received and not move_complete_received:
                     log_msg = f"⚠️ Lệnh di chuyển '{command}' nhận OK nhưng thiếu MOVE_COMPLETE!"
                     self.log(log_msg, tag="WARNING")
                     logging.warning(log_msg)
                     # Vẫn coi là thành công nhưng có cảnh báo
                     return True

                else: # Thành công
                     logging.debug(f"Lệnh '{command}' hoàn thành sau {command_duration:.3f}s.")
                     return True

            else: # Không cần chờ phản hồi
                # logging.debug(f"Đã gửi lệnh '{command}' không chờ phản hồi.")
                return True # Coi như thành công ngay

        except serial.SerialTimeoutException:
            self.log(f"⚠️ Lỗi Serial Timeout khi gửi: '{command}'", tag="ERROR")
            logging.error(f"Serial Timeout khi gửi: '{command}'")
            self.root.after(0, self.reconnect) # Thử kết nối lại
            return False
        except serial.SerialException as e:
            self.log(f"❌ Lỗi Serial khi thực thi '{command}': {e}", tag="ERROR")
            logging.exception(f"Lỗi Serial khi thực thi '{command}':")
            if "ClearCommError" in str(e) or "device disconnected" in str(e):
                self.root.after(0, self.reconnect)
            return False
        except Exception as e:
            self.log(f"❌ Lỗi không xác định khi thực thi '{command}': {e}", tag="ERROR")
            logging.exception(f"Lỗi không xác định khi thực thi '{command}':")
            return False

    def _update_ik_config(self, config_str):
        """Cập nhật cấu hình IK và hiển thị trên UI."""
        if config_str and config_str != self.current_config: # Chỉ cập nhật nếu khác
            self.current_config = config_str
            try:
                # Đảm bảo widget còn tồn tại
                if hasattr(self, 'config_label') and self.config_label.winfo_exists():
                    self.config_label.config(text=config_str)
            except tk.TclError:
                 logging.warning("Lỗi cập nhật config_label (widget có thể đã bị hủy).")
            logging.info(f"Cập nhật cấu hình IK thành: {config_str}")

    def _update_pen_status(self, is_down):
        """Cập nhật trạng thái bút và UI (chạy trên main thread)."""
        if self.pen_is_down == is_down:
            return

        self.pen_is_down = is_down
        try:
            # Đảm bảo widget còn tồn tại
            if hasattr(self, 'pen_label') and self.pen_label.winfo_exists():
                 self.pen_label.configure(text="Hạ" if is_down else "Nâng")
        except tk.TclError:
            logging.warning("Lỗi cập nhật pen_label (widget có thể đã bị hủy).")

        logging.debug(f"Cập nhật trạng thái bút: {'Hạ' if is_down else 'Nâng'}")

        if is_down:
            # Bắt đầu segment vẽ mới với tọa độ NỘI BỘ hiện tại
            x_internal_now = self.current_position.get('x', float('nan'))
            y_now = self.current_position.get('y', float('nan'))
            if not math.isnan(x_internal_now):
                 # Lật X để vẽ
                 x_vis_now = -x_internal_now
                 self.current_segment = {'x': [x_vis_now], 'y': [y_now]}
                 logging.debug(f"Bắt đầu segment vẽ mới tại Vis({x_vis_now:.2f}, {y_now:.2f}) / Int({x_internal_now:.2f}, {y_now:.2f})")
            else:
                 logging.warning("Không thể bắt đầu segment vẽ do tọa độ hiện tại không xác định.")
                 self.current_segment = {}
        else:
            # Kết thúc segment vẽ hiện tại và lưu lại (nếu hợp lệ)
            if self.current_segment and self.current_segment.get('x'):
                if len(self.current_segment['x']) > 1:
                    self.trace_segments.append(self.current_segment.copy())
                    logging.debug(f"Kết thúc và lưu segment vẽ ({len(self.current_segment['x'])} điểm). Tổng cộng {len(self.trace_segments)} segments.")
                else:
                     logging.debug("Bỏ qua segment vẽ chỉ có 1 điểm.")
                self.current_segment = {}

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

            lim = self.robot.angle_limits
            th1_min, th1_max = lim['theta1_min'], lim['theta1_max']
            th2_min, th2_max = lim['theta2_min'], lim['theta2_max']

            if not (th1_min <= target_theta1 <= th1_max):
                messagebox.showerror("Góc Không Hợp Lệ", f"Góc θ1 ({target_theta1:.2f}°) nằm ngoài giới hạn [{th1_min}°, {th1_max}°].")
                return
            if not (th2_min <= target_theta2 <= th2_max):
                 messagebox.showerror("Góc Không Hợp Lệ", f"Góc θ2 ({target_theta2:.2f}°) nằm ngoài giới hạn [{th2_min}°, {th2_max}°].")
                 return

            # Không cập nhật UI/Animation ngay, chờ phản hồi MOVE_COMPLETE
            command = f"{target_theta1:.2f},{target_theta2:.2f}"
            self.log(f"Yêu cầu di chuyển đến góc: θ1={target_theta1:.2f}, θ2={target_theta2:.2f}")
            self.send_command(command, wait_for_response=True)

        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị góc không hợp lệ! Vui lòng nhập số.")
        except Exception as e:
             logging.exception("Lỗi trong move_to_angle:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def move_to_xy(self):
        """Di chuyển robot đến tọa độ XY chỉ định (hiển thị đã lật)."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return

        try:
            # Lấy tọa độ X, Y người dùng nhập (X là giá trị hiển thị đã lật)
            target_x_display = float(self.x_entry.get())
            target_y = float(self.y_entry.get())

            # --- LẬT X ĐỂ TÍNH TOÁN NỘI BỘ ---
            target_x_internal = -target_x_display
            # --- KẾT THÚC LẬT ---

            # 1. Kiểm tra điểm nội bộ có trong vùng làm việc không
            if not self.robot.check_point_in_workspace(target_x_internal, target_y):
                # Hiển thị lỗi với tọa độ display
                messagebox.showwarning("Ngoài Vùng Làm Việc",
                                     f"Vị trí (X={target_x_display:.2f}, Y={target_y:.2f}) nằm ngoài vùng làm việc hoặc không hợp lệ (Y>0).")
                return

            # 2. Tính toán góc bằng Inverse Kinematics (dùng tọa độ internal)
            target_theta1, target_theta2, selected_config = self.robot.inverse_kinematics(
                target_x_internal, target_y, self.current_config
            )

            if target_theta1 is None or target_theta2 is None:
                # Hiển thị lỗi với tọa độ display
                messagebox.showerror("Lỗi Tính Toán",
                                     f"Không thể tính toán góc hợp lệ cho vị trí (X={target_x_display:.2f}, Y={target_y:.2f}). Có thể do giới hạn góc.")
                return

            # --- Gửi lệnh di chuyển ---
            # Không cập nhật UI/Animation ngay, chờ phản hồi MOVE_COMPLETE
            command = f"{target_theta1:.2f},{target_theta2:.2f}"
            # Log với tọa độ display và góc tính được
            self.log(f"Yêu cầu di chuyển đến XY (Display): ({target_x_display:.2f}, {target_y:.2f}) -> Góc ({target_theta1:.2f}, {target_theta2:.2f}), Config: {selected_config}")
            self.send_command(command, wait_for_response=True)

        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị tọa độ không hợp lệ! Vui lòng nhập số.")
        except Exception as e:
             logging.exception("Lỗi trong move_to_xy:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def update_position_and_config(self, final_theta1, final_theta2, final_config):
        """Cập nhật trạng thái góc, cấu hình và tính toán lại vị trí XY cuối cùng."""
        # Hàm này được gọi bởi _handle_arduino_response khi nhận MOVE_COMPLETE
        self.current_angles['theta1'] = final_theta1
        self.current_angles['theta2'] = final_theta2
        self._update_ik_config(final_config) # Cập nhật cấu hình và UI label

        # Tính lại vị trí XY NỘI BỘ từ góc cuối cùng bằng FK
        _, effector_pos_internal, _ = self.robot.forward_kinematics(final_theta1, final_theta2)
        if effector_pos_internal:
            self.current_position['x'] = effector_pos_internal[0] # Lưu X nội bộ
            self.current_position['y'] = effector_pos_internal[1]
            # Lật X để log tọa độ display
            x_display = -self.current_position['x']
            logging.info(f"Cập nhật trạng thái cuối: Angles=({final_theta1:.2f}, {final_theta2:.2f}), PosDisplay=({x_display:.2f}, {effector_pos_internal[1]:.2f}), Config={final_config}")
        else:
            logging.warning("Không thể tính FK cho góc cuối cùng, vị trí XY có thể không chính xác.")
            # Không thay đổi self.current_position

        # Cập nhật toàn bộ hiển thị UI và animation
        self.update_position_display()
        self.update_animation()

    def update_position_display(self):
        """Cập nhật các nhãn và entry hiển thị vị trí/góc/config trên UI."""
        x_internal = self.current_position.get('x', float('nan'))
        y = self.current_position.get('y', float('nan'))
        th1 = self.current_angles.get('theta1', float('nan'))
        th2 = self.current_angles.get('theta2', float('nan'))

        # --- LẬT X ĐỂ HIỂN THỊ ---
        x_display = -x_internal if not math.isnan(x_internal) else float('nan')
        # --- KẾT THÚC LẬT ---

        # Cập nhật nhãn trạng thái (hiển thị X đã lật)
        try:
            if hasattr(self, 'pos_label') and self.pos_label.winfo_exists():
                 self.pos_label.config(text=f"X={x_display:.2f}, Y={y:.2f}")
            if hasattr(self, 'angle_label') and self.angle_label.winfo_exists():
                 self.angle_label.config(text=f"θ1={th1:.2f}°, θ2={th2:.2f}°")
        except tk.TclError: pass # Bỏ qua lỗi nếu widget bị hủy

        # Cập nhật các trường nhập liệu (chỉ khi không focus)
        try:
            focused_widget = self.root.focus_get()
            # Cập nhật entry X với giá trị display (đã lật)
            if hasattr(self, 'x_entry') and self.x_entry.winfo_exists() and focused_widget != self.x_entry:
                self.x_entry.delete(0, tk.END)
                self.x_entry.insert(0, f"{x_display:.2f}")
            # Cập nhật các entry khác
            if hasattr(self, 'y_entry') and self.y_entry.winfo_exists() and focused_widget != self.y_entry:
                self.y_entry.delete(0, tk.END)
                self.y_entry.insert(0, f"{y:.2f}")
            if hasattr(self, 'angle1_entry') and self.angle1_entry.winfo_exists() and focused_widget != self.angle1_entry:
                self.angle1_entry.delete(0, tk.END)
                self.angle1_entry.insert(0, f"{th1:.2f}")
            if hasattr(self, 'angle2_entry') and self.angle2_entry.winfo_exists() and focused_widget != self.angle2_entry:
                self.angle2_entry.delete(0, tk.END)
                self.angle2_entry.insert(0, f"{th2:.2f}")
        except tk.TclError:
            logging.warning("Lỗi cập nhật entry, widget có thể không tồn tại.")
        except Exception as e:
             logging.exception("Lỗi không xác định khi cập nhật entry:")


    def pen_down(self):
        """Hạ bút xuống."""
        if self.is_connected:
            self.log("Yêu cầu hạ bút (d)")
            self.send_command("d", wait_for_response=True)

    def pen_up(self):
        """Nâng bút lên."""
        if self.is_connected:
            self.log("Yêu cầu nâng bút (u)")
            self.send_command("u", wait_for_response=True)

    def home(self):
        """Gửi lệnh về vị trí Home."""
        if not self.is_connected:
            messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối với Arduino trước!")
            return
        self.log("Yêu cầu về HOME (h)")
        # Không cập nhật UI/Animation ngay, chờ phản hồi MOVE_COMPLETE từ lệnh 'h'
        self.send_command("h", wait_for_response=True)

    # --- THÊM HÀM calibrate_position ---
    def calibrate_position(self):
         """Gửi lệnh hiệu chuẩn và cập nhật trạng thái Python."""
         if not self.is_connected:
             messagebox.showwarning("Chưa Kết Nối", "Vui lòng kết nối để hiệu chuẩn!")
             return

         if messagebox.askokcancel("Xác nhận Hiệu Chuẩn",
                                   "Thao tác này sẽ đặt lại vị trí góc của robot thành (90°, 90°) trong firmware.\n"
                                   "Robot vật lý SẼ KHÔNG DI CHUYỂN.\n"
                                   "Chỉ thực hiện khi bạn chắc chắn robot đang ở vị trí home vật lý.\n"
                                   "Bạn có muốn tiếp tục không?"):
             self.log("Yêu cầu hiệu chuẩn vị trí về (90, 90)...")
             # Gửi lệnh calibrate và chờ phản hồi OK
             success = self.send_command('calibrate', wait_for_response=True)
             if success:
                 # Cập nhật trạng thái Python ngay lập tức sau khi nhận OK
                 self.current_angles = {'theta1': 90.0, 'theta2': 90.0}
                 # Tính lại vị trí XY nội bộ từ góc home
                 _, home_effector_pos, _ = self.robot.forward_kinematics(90.0, 90.0)
                 if home_effector_pos:
                      self.current_position['x'] = home_effector_pos[0] # X nội bộ
                      self.current_position['y'] = home_effector_pos[1]
                 else: # Xử lý lỗi nếu FK home thất bại
                      self.current_position['x'] = 10.0 # Đặt giá trị an toàn (X nội bộ)
                      self.current_position['y'] = 30.0
                 self._update_ik_config("elbow_down_down") # Reset config về mặc định
                 self.update_position_display() # Cập nhật UI
                 self.update_animation() # Cập nhật plot
                 self.log("✓ Hiệu chuẩn thành công. Vị trí đặt lại thành (90, 90).", tag="SUCCESS")
             else:
                  self.log("❌ Hiệu chuẩn thất bại (không nhận được OK).", tag="ERROR")


    def emergency_stop(self):
        """Dừng khẩn cấp: Dừng G-code và gửi lệnh disable motors."""
        self.log("!!! DỪNG KHẨN CẤP !!!", tag="ERROR")
        logging.warning("Kích hoạt dừng khẩn cấp!")

        # 1. Dừng xử lý G-code nếu đang chạy
        if self.gcode_running:
            self.stop_gcode(is_emergency=True)

        # 2. Xóa hàng đợi lệnh
        self._clear_queues() # Gọi hàm xóa queue mới

        # 3. Gửi lệnh disable motors ngay lập tức (không qua queue)
        if self.is_connected and self.serial and self.serial.is_open:
            try:
                self.log("Gửi lệnh disable trực tiếp...", tag="WARNING")
                disable_cmd = b"disable\n"
                with self.serial_lock:
                    self.serial.write(disable_cmd)
                logging.info("Đã gửi lệnh disable khẩn cấp.")
            except Exception as e:
                self.log(f"Lỗi gửi lệnh disable khẩn cấp: {e}", tag="ERROR")
                logging.exception("Lỗi gửi lệnh disable khẩn cấp:")
        else:
            self.log("Không thể gửi lệnh disable (chưa kết nối).", tag="WARNING")

        # 4. Đặt lại cờ command_processing
        self.command_processing = False

    def set_speed(self):
        """Gửi lệnh cài đặt tốc độ tối đa cho Arduino."""
        if not self.is_connected:
            return
        try:
            speed = int(self.speed_var.get())
            if speed > 0:
                command = f"f{speed}"
                self.log(f"Cài đặt tốc độ: {speed} steps/sec")
                self.send_command(command, wait_for_response=True)
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
            return
        try:
            accel = int(self.accel_var.get())
            if accel > 0:
                command = f"a{accel}"
                self.log(f"Cài đặt gia tốc: {accel} steps/sec^2")
                self.send_command(command, wait_for_response=True)
            else:
                messagebox.showerror("Giá Trị Không Hợp Lệ", "Gia tốc phải là số dương.")
        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Giá trị gia tốc không hợp lệ! Vui lòng nhập số nguyên.")
        except Exception as e:
             logging.exception("Lỗi trong set_acceleration:")
             messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {e}")

    def log(self, message, tag="INFO"):
        """Hiển thị thông báo trong ô log của giao diện với tag màu."""
        def _log_update():
            try:
                if not self.root.winfo_exists() or not hasattr(self, 'log_text') or not self.log_text.winfo_exists():
                    return
                self.log_text.config(state=tk.NORMAL) # Bật để sửa
                num_lines = int(self.log_text.index('end - 1 line').split('.')[0])
                max_lines = 500
                if num_lines > max_lines:
                    self.log_text.delete('1.0', f'{num_lines - max_lines + 1}.0')
                timestamp = time.strftime("[%H:%M:%S]")
                start_index = self.log_text.index(tk.END + "-1c")
                self.log_text.insert(tk.END, f"{timestamp} {message}\n")
                end_index = self.log_text.index(tk.END + "-1c")
                if tag in self.log_text.tag_names():
                    self.log_text.tag_add(tag, start_index, end_index)
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED) # Tắt để chỉ đọc
            except tk.TclError as e:
                logging.warning(f"TclError updating log_text: {e}")
            except Exception as e:
                logging.exception("Lỗi không xác định khi cập nhật log:")
        try:
            if hasattr(self.root, 'winfo_exists') and self.root.winfo_exists():
                self.root.after(0, _log_update)
            else:
                print(f"LOG (root not available): {message}")
        except Exception:
            print(f"LOG (root error): {message}")

    # --- Các hàm xử lý G-code ---
    def load_gcode(self):
        """Mở hộp thoại để chọn và tải file G-code."""
        if self.gcode_running:
            response = messagebox.askyesno("G-Code Đang Chạy", "Dừng G-code hiện tại và tải file mới?")
            if response: self.stop_gcode()
            else: return

        file_path = filedialog.askopenfilename(
            title="Chọn file G-Code",
            filetypes=(("G-Code files", "*.gcode *.ngc *.nc *.tap"), ("All files", "*.*"))
        )
        if not file_path: return

        try:
            with open(file_path, "r", encoding='utf-8', errors='ignore') as f:
                raw_content = f.readlines()

            processed_gcode = []
            cleaned_display_content = []
            for line_num, line in enumerate(raw_content):
                original_line = line.strip()
                cleaned_line = original_line
                comment_index = cleaned_line.find(';')
                if comment_index != -1: cleaned_line = cleaned_line[:comment_index].strip()
                cleaned_line = re.sub(r'\([^)]*\)', '', cleaned_line).strip()
                if not cleaned_line:
                    cleaned_display_content.append(original_line)
                    continue
                processed_line = cleaned_line.upper()
                processed_gcode.append(processed_line)
                cleaned_display_content.append(original_line)

            self.gcode_text.config(state=tk.NORMAL) # Bật để sửa
            self.gcode_text.delete(1.0, tk.END)
            self.gcode_text.insert(tk.END, "\n".join(cleaned_display_content))
            self.gcode_text.config(state=tk.DISABLED) # Tắt để chỉ đọc

            self.gcode_lines = processed_gcode
            self.gcode_total_lines = len(self.gcode_lines)
            file_name = os.path.basename(file_path)
            self.file_label.config(text=file_name)
            self.log(f"Đã tải và xử lý: {file_name} ({self.gcode_total_lines} lệnh)")
            logging.info(f"Đã tải G-code: {file_name}, {self.gcode_total_lines} lệnh.")

            self.progress_var.set(0)
            self.progress_label.config(text="0.0% hoàn thành")
            self.run_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED)
            self.stop_gcode_btn.config(state=tk.DISABLED)
            self.clear_drawing()
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

        if self.gcode_running and self.gcode_paused:
            self.gcode_paused = False
            self.pause_btn.config(text="Tạm dừng")
            self.log("Tiếp tục xử lý G-code...")
            logging.info("Tiếp tục G-code.")
            return

        if self.gcode_running and not self.gcode_paused:
            self.log("G-code đang chạy.")
            return

        self.gcode_running = True
        self.gcode_paused = False
        self.stop_gcode_flag = False # Reset cờ dừng
        self.gcode_line_num = 0
        self.update_progress(0.0)

        delay = self.gcode_delay_var.get()
        self.log(f"Bắt đầu chạy G-code với delay {delay:.3f}s.")
        logging.info(f"Bắt đầu chạy G-code (delay={delay:.3f}s).")

        # self.send_command("gstart", wait_for_response=False) # Tùy chọn

        self.run_btn.config(state=tk.DISABLED)
        self.pause_btn.config(text="Tạm dừng", state=tk.NORMAL)
        self.stop_gcode_btn.config(state=tk.NORMAL)
        self.direct_controls_enable(False) # Vô hiệu hóa điều khiển thủ công

        if hasattr(self, '_gcode_thread') and self._gcode_thread.is_alive():
             logging.warning("Thread G-code cũ vẫn đang chạy? Bỏ qua.")
             return
        self._gcode_thread = threading.Thread(target=self._process_gcode_thread, daemon=True)
        self._gcode_thread.start()

    def _process_gcode_thread(self):
        """Thread chạy nền để xử lý từng dòng G-code."""
        logging.info("Thread xử lý G-code bắt đầu.")
        try:
            while self.gcode_line_num < self.gcode_total_lines and not self.stop_gcode_flag:
                while self.gcode_paused and not self.stop_gcode_flag:
                    time.sleep(0.1)
                if self.stop_gcode_flag:
                    logging.info("Thread G-code: Nhận tín hiệu dừng.")
                    break

                line_index = self.gcode_line_num
                line = self.gcode_lines[line_index].strip()
                logging.debug(f"G-code Line {line_index + 1}/{self.gcode_total_lines}: {line}")

                self.root.after(0, lambda ln=line_index: self.highlight_current_line(ln))

                # Xử lý dòng G-code và chờ hoàn thành
                success = self.process_gcode_line_and_wait(line) # Hàm mới để chờ

                if not success or self.stop_gcode_flag: # Dừng nếu lỗi hoặc có cờ dừng
                     logging.warning(f"Dừng G-code tại dòng {line_index + 1} do lỗi hoặc yêu cầu dừng.")
                     self.root.after(0, lambda: self.stop_gcode(is_emergency=not success)) # Dừng khẩn cấp nếu lỗi
                     break

                # Chỉ tăng nếu dòng hiện tại đã xử lý xong
                if self.gcode_line_num == line_index:
                    self.gcode_line_num += 1
                    progress = (self.gcode_line_num / self.gcode_total_lines) * 100.0
                    self.root.after(0, lambda p=progress: self.update_progress(p))

                delay = self.gcode_delay_var.get()
                if delay > 0: time.sleep(delay)

            if not self.stop_gcode_flag: # Nếu chạy hết mà không bị dừng
                logging.info("Hoàn thành xử lý G-code (hết file).")
                self.root.after(0, self.complete_gcode)

        except Exception as e:
            logging.exception("Lỗi nghiêm trọng trong thread xử lý G-code:")
            self.log(f"❌ Lỗi G-code: {e}", tag="ERROR")
            if self.gcode_running: # Đảm bảo dừng hẳn
                 self.root.after(0, lambda: self.stop_gcode(is_emergency=True))
        finally:
            logging.info("Thread xử lý G-code kết thúc.")

    # --- THÊM HÀM MỚI process_gcode_line_and_wait ---
    def process_gcode_line_and_wait(self, line):
        """Phân tích dòng G-code, gửi lệnh và chờ hoàn thành."""
        if not line: # Bỏ qua dòng trống
             return True # Coi như thành công

        command_to_send = None
        is_move_cmd_gcode = False
        action_in_python = False # Cho G4

        # --- Phân tích G-code (tương tự process_gcode_line cũ) ---
        command_code = line.split()[0]
        x_gcode, y_gcode, z_gcode, feed, p_value = None, None, None, None, None
        has_xy = False

        parts = line.split()
        for part in parts[1:]:
            if not part or len(part) < 2: continue
            code = part[0]
            try:
                value = float(part[1:])
                if code == 'X': x_gcode = value; has_xy = True
                elif code == 'Y': y_gcode = value; has_xy = True
                elif code == 'Z': z_gcode = value
                elif code == 'F': feed = value
                elif code == 'P': p_value = value
            except ValueError: continue

        # --- Xử lý tuần tự: Z -> F -> XY ---
        # 1. Xử lý Z (Pen)
        pen_command = None
        if z_gcode is not None:
            if z_gcode <= 0 and not self.pen_is_down: pen_command = 'd'
            elif z_gcode > 0 and self.pen_is_down: pen_command = 'u'
        if pen_command:
             if not self.send_command(pen_command, wait_for_response=True): return False
             if not self.wait_for_command_completion(): return False # Chờ queue và processing
             if self.stop_gcode_flag: return False # Kiểm tra dừng

        # 2. Xử lý F (Speed)
        if feed is not None and feed > 0:
             if not self.send_command(f"f{int(feed)}", wait_for_response=True): return False
             if not self.wait_for_command_completion(): return False
             if self.stop_gcode_flag: return False

        # 3. Xử lý XY (Move)
        if has_xy:
            if x_gcode is None or y_gcode is None: return False # Lỗi thiếu tọa độ

            # --- LẬT X TỪ GCODE ĐỂ TÍNH TOÁN NỘI BỘ ---
            x_internal = -x_gcode
            y_internal = y_gcode
            # --- KẾT THÚC LẬT ---

            if not self.robot.check_point_in_workspace(x_internal, y_internal): return False # Lỗi ngoài vùng

            target_theta1, target_theta2, selected_config = self.robot.inverse_kinematics(
                x_internal, y_internal, self.current_config
            )
            if target_theta1 is None or target_theta2 is None: return False # Lỗi IK

            # Cập nhật UI/Animation trên main thread
            def update_ui_before_move_gcode():
                self.current_angles['theta1'] = target_theta1
                self.current_angles['theta2'] = target_theta2
                self.current_position['x'] = x_internal # Lưu X nội bộ
                self.current_position['y'] = y_internal
                self._update_ik_config(selected_config)
                self.update_position_display()
                self.update_animation()
            self.root.after(0, update_ui_before_move_gcode)

            command_to_send = f"{target_theta1:.2f},{target_theta2:.2f}"
            is_move_cmd_gcode = True
            # Gửi lệnh góc
            if not self.send_command(command_to_send, wait_for_response=True): return False
            # Chờ lệnh di chuyển hoàn thành
            if not self.wait_for_command_completion(): return False
            if self.stop_gcode_flag: return False
            return True # Di chuyển XY thành công

        # --- Xử lý các lệnh G/M khác ---
        elif command_code == 'G4': # Dwell
             if p_value is not None and p_value > 0:
                 dwell_sec = p_value / 1000.0
                 self.log(f"Tạm dừng G-code {dwell_sec:.3f} giây (G4)")
                 time.sleep(dwell_sec)
                 action_in_python = True
             else: return False # Lỗi thiếu P
        elif command_code == 'G28': command_to_send = "h"; is_move_cmd_gcode = True
        elif command_code.startswith(("M3", "M03")): command_to_send = ('d' if not self.pen_is_down else None)
        elif command_code.startswith(("M5", "M05")): command_to_send = ('u' if self.pen_is_down else None)
        elif command_code == 'M17': command_to_send = "enable"
        elif command_code.startswith(("M18", "M84")): command_to_send = "disable"
        elif command_code.startswith(("G90", "G91", "G20", "G21")): # Bỏ qua các lệnh này
             command_to_send = None
             action_in_python = True # Coi như đã xử lý (bằng cách bỏ qua)
        else:
             logging.warning(f"Bỏ qua lệnh G-code không xác định: {line}")
             return True # Coi như thành công khi bỏ qua lệnh không xác định

        # Gửi lệnh đã xác định (nếu có)
        if command_to_send:
            if not self.send_command(command_to_send, wait_for_response=True): return False
            if not self.wait_for_command_completion(): return False
            if self.stop_gcode_flag: return False
            return True # Gửi và chờ thành công

        elif action_in_python:
            return True # Đã xử lý trong Python (G4) hoặc bỏ qua (G90, etc)

        else: # Không có lệnh nào được gửi (vd: M3 khi bút đã down)
             return True # Coi như thành công

    def wait_for_command_completion(self):
        """Chờ cho đến khi hàng đợi lệnh rỗng và lệnh hiện tại xử lý xong."""
        start_wait = time.time()
        # Timeout chờ có thể cần dài hơn nếu lệnh phức tạp
        max_wait = self.command_timeout * 3.0 # Tăng timeout chờ

        while time.time() - start_wait < max_wait:
            queue_empty = self.command_queue.empty()
            processing_done = not self.command_processing

            if queue_empty and processing_done:
                return True # Hoàn thành

            if self.stop_gcode_flag: # Kiểm tra cờ dừng G-code
                logging.info("wait_for_command_completion: Nhận tín hiệu dừng G-code.")
                return False # Thoát chờ nếu G-code bị dừng

            time.sleep(0.02) # Nghỉ ngắn hơn

        logging.warning(f"Timeout ({max_wait:.1f}s) khi chờ lệnh hoàn thành!")
        self.log(f"⚠️ Timeout chờ lệnh hoàn thành!", tag="WARNING")
        # Không tự động dừng G-code ở đây, để thread xử lý quyết định
        return False

    def highlight_current_line(self, line_num):
        """Highlight dòng G-code đang thực thi trong Text widget."""
        try:
            if not hasattr(self, 'gcode_text') or not self.gcode_text.winfo_exists(): return

            self.gcode_text.tag_remove("current_line", "1.0", tk.END)
            start_index = f"{line_num + 1}.0"
            end_index = f"{line_num + 1}.end"
            self.gcode_text.tag_add("current_line", start_index, end_index)
            if "current_line" not in self.gcode_text.tag_names():
                 self.gcode_text.tag_config("current_line", background="yellow", relief="raised")
            # Tự động cuộn để nhìn thấy dòng hiện tại
            self.gcode_text.see(start_index)
        except tk.TclError:
            logging.warning(f"Lỗi highlight dòng {line_num + 1}.")
        except Exception as e:
             logging.exception(f"Lỗi highlight dòng {line_num + 1}:")

    def update_progress(self, progress):
        """Cập nhật thanh tiến trình và nhãn %."""
        try:
            if not hasattr(self, 'progress_var') or not self.root.winfo_exists(): return
            clamped_progress = max(0.0, min(100.0, progress))
            self.progress_var.set(clamped_progress)
            self.progress_label.config(text=f"{clamped_progress:.1f}% hoàn thành")
        except tk.TclError:
             logging.warning("Lỗi cập nhật progress bar.")
        except Exception as e:
             logging.exception("Lỗi cập nhật progress:")

    def pause_gcode(self):
        """Tạm dừng hoặc tiếp tục thực thi G-code."""
        if not self.gcode_running: return
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
        if not self.gcode_running: return

        self.stop_gcode_flag = True # Đặt cờ dừng cho thread
        self.gcode_paused = False # Đảm bảo không bị kẹt ở pause
        self.gcode_running = False # Đặt cờ chính

        if is_emergency:
            self.log("Đang dừng G-code (khẩn cấp)...", tag="WARNING")
            logging.warning("Dừng G-code khẩn cấp.")
        else:
            self.log("Đang dừng G-code...")
            logging.info("Dừng G-code.")

        # Không cần chờ thread G-code join ở đây, nó sẽ tự thoát khi kiểm tra cờ
        # self.send_command("gend", wait_for_response=False) # Tùy chọn

        self.root.after(0, self._reset_gcode_ui) # Reset UI trên main thread
        self.log("Đã dừng G-code.")

    def complete_gcode(self):
        """Được gọi khi G-code chạy xong bình thường."""
        if self.gcode_running: self.gcode_running = False
        self.log("✓ Hoàn thành G-code.", tag="SUCCESS") # Dùng tag SUCCESS
        logging.info("G-code hoàn thành.")
        # self.send_command("gend", wait_for_response=False) # Tùy chọn
        self._reset_gcode_ui()
        self.update_progress(100.0)

    def _reset_gcode_ui(self):
        """Đặt lại các thành phần UI liên quan đến G-code."""
        try:
            if not self.root.winfo_exists(): return # Kiểm tra root window
            run_state = tk.NORMAL if self.gcode_lines else tk.DISABLED
            if hasattr(self, 'run_btn') and self.run_btn.winfo_exists(): self.run_btn.config(state=run_state)
            if hasattr(self, 'pause_btn') and self.pause_btn.winfo_exists(): self.pause_btn.config(text="Tạm dừng", state=tk.DISABLED)
            if hasattr(self, 'stop_gcode_btn') and self.stop_gcode_btn.winfo_exists(): self.stop_gcode_btn.config(state=tk.DISABLED)
            if hasattr(self, 'gcode_text') and self.gcode_text.winfo_exists(): self.gcode_text.tag_remove("current_line", "1.0", tk.END)
            self.direct_controls_enable(True) # Bật lại điều khiển thủ công
        except tk.TclError:
            logging.warning("Lỗi reset G-code UI.")
        except Exception as e:
             logging.exception("Lỗi reset G-code UI:")

    # --- THÊM HÀM direct_controls_enable ---
    def direct_controls_enable(self, enabled=True):
         """Bật/tắt các nút điều khiển trực tiếp."""
         state = tk.NORMAL if enabled else tk.DISABLED
         # Liệt kê các widget cần thay đổi trạng thái
         widgets_to_toggle = [
             getattr(self, 'go_xy_btn', None), getattr(self, 'go_angle_btn', None),
             getattr(self, 'home_btn', None), getattr(self, 'pen_up_btn', None),
             getattr(self, 'pen_down_btn', None), getattr(self, 'calibrate_btn', None), # Thêm nút calibrate nếu có
             # Các nút motor enable/disable (nếu có riêng)
             getattr(self, 'set_speed_button', None), getattr(self, 'set_accel_button', None) # Các nút set tốc độ/gia tốc
         ]
         for widget in widgets_to_toggle:
             try:
                 # Kiểm tra xem widget có tồn tại và có thuộc tính 'config' không
                 if widget and hasattr(widget, 'config') and widget.winfo_exists():
                     widget.config(state=state)
             except tk.TclError:
                  logging.warning(f"Lỗi khi thay đổi trạng thái widget (có thể đã bị hủy).")
             except Exception as e:
                  logging.exception(f"Lỗi không xác định khi thay đổi trạng thái widget:")


    def reconnect(self):
        """Thử kết nối lại với Arduino nếu kết nối bị mất."""
        if not self.is_connected:
             self.log("⚠️ Mất kết nối? Đang thử kết nối lại...", tag="WARNING")
             logging.warning("Mất kết nối? Đang thử kết nối lại...")
             self.connect() # Gọi hàm connect mới
        else:
             self.log("⚠️ Có lỗi xảy ra, đang thử ngắt và kết nối lại...", tag="WARNING")
             logging.warning("Có lỗi xảy ra, đang thử ngắt và kết nối lại...")
             self.disconnect()
             self.root.after(1500, self.connect)

    def on_closing(self):
        """Xử lý sự kiện đóng cửa sổ."""
        logging.info("Đóng ứng dụng...")
        self.log("Đang đóng ứng dụng...")
        # Dừng các thread và đóng kết nối
        self.stop_gcode_flag = True # Dừng G-code
        self.stop_read_thread = True # Dừng đọc serial
        self.command_queue.put(None) # Dừng xử lý command queue
        self.response_queue.put(None) # Dừng xử lý response queue

        # Chờ các thread dừng (ngắn)
        if hasattr(self, '_gcode_thread') and self._gcode_thread.is_alive():
             self._gcode_thread.join(timeout=0.5)
        if self.read_thread and self.read_thread.is_alive():
             self.read_thread.join(timeout=0.5)
        if self.process_command_thread and self.process_command_thread.is_alive():
             self.process_command_thread.join(timeout=0.5)
        if self.process_response_thread and self.process_response_thread.is_alive():
             self.process_response_thread.join(timeout=0.5)


        # Gửi lệnh disable motor trước khi thoát
        if self.is_connected:
             self.emergency_stop() # Gọi hàm dừng khẩn cấp để disable motor
             time.sleep(0.1) # Đợi một chút
             self.disconnect(silent=True) # Đóng cổng serial

        self.root.destroy()
        logging.info("Ứng dụng đã đóng.")

# --- Hàm Main ---
def main():
    root = tk.Tk()
    # Giữ nguyên phần theme gốc của bạn
    try:
        style = ttk.Style(root)
        available_themes = style.theme_names()
        logging.info(f"Available themes: {available_themes}")
        if 'vista' in available_themes:
            style.theme_use('vista')
        logging.info(f"Using theme: {style.theme_use()}")
    except Exception as e:
        logging.warning(f"Không thể đặt theme ttk: {e}")

    app = ScaraGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        logging.info("Nhận KeyboardInterrupt, đang đóng ứng dụng...")
        app.on_closing()

if __name__ == "__main__":
    main()