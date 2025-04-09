import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import re
import os


class ScaraRobot:
    def __init__(self):
        # Thông số robot SCARA
        self.robot_params = {
            'x1': 0.0,  # Tọa độ X motor 1 (motor Y)
            'y1': 0.0,  # Tọa độ Y motor 1
            'x5': -20.0,  # Tọa độ X motor 2 (motor X) - cách motor Y 20cm
            'y5': 0.0,  # Tọa độ Y motor 2
            'L1': 15.0,  # Chiều dài cánh tay 1 từ motor 1
            'L2': 12.0,  # Chiều dài cánh tay 2 từ joint 1 đến đầu vẽ
            'L3': 12.0,  # Chiều dài cánh tay 3 từ đầu vẽ đến joint 2
            'L4': 15.0,  # Chiều dài cánh tay 4 từ joint 2 đến motor 2
        }
        self.home_position = {'x': -10.0, 'y': 38.0}  # Vị trí home
        self.calculate_workspace()

    def calculate_workspace(self):
        """Tính vùng làm việc robot"""
        # Thông số
        x1, y1 = self.robot_params['x1'], self.robot_params['y1']
        x5, y5 = self.robot_params['x5'], self.robot_params['y5']
        L1 = self.robot_params['L1']
        L4 = self.robot_params['L4']
        L_mid = self.robot_params['L2'] + self.robot_params['L3']

        # Vùng làm việc (hình chữ nhật bảo thủ)
        margin = 2.0  # Biên an toàn 2cm
        r1_min = max(0, L1 - L_mid) + margin
        r1_max = L1 + L_mid - margin
        r2_min = max(0, L4 - L_mid) + margin
        r2_max = L4 + L_mid - margin

        # Giới hạn vùng làm việc
        self.workspace_bounds = {
            'left': x5 + r2_min,
            'right': x1 + r1_max * 0.75,
            'bottom': y1 + r1_min,
            'top': max(y1, y5) + max(r1_max, r2_max) * 0.9
        }

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

    def inverse_kinematics(self, x3, y3):
        """Tính toán góc quay của các khớp dựa trên vị trí đầu vẽ"""
        x1 = self.robot_params['x1']
        y1 = self.robot_params['y1']
        x5 = self.robot_params['x5']
        y5 = self.robot_params['y5']
        L1 = self.robot_params['L1']
        L2 = self.robot_params['L2']
        L3 = self.robot_params['L3']
        L4 = self.robot_params['L4']

        # Giới hạn góc
        THETA1_MIN, THETA1_MAX = -90, 135
        THETA2_MIN, THETA2_MAX = 45, 270
        THETA_DIFF_MAX = 95

        # Tính khoảng cách từ điểm vẽ đến motor
        L13 = np.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
        L53 = np.sqrt((x3 - x5) ** 2 + (y3 - y5) ** 2)

        # Kiểm tra điểm có nằm trong phạm vi không
        if (L13 < abs(L1 - L2) or L13 > L1 + L2 or
                L53 < abs(L4 - L3) or L53 > L4 + L3):
            return None, None  # Điểm nằm ngoài phạm vi

        # Tính góc cho motor 1
        alpha_one = np.arccos(np.clip((L1 ** 2 + L13 ** 2 - L2 ** 2) / (2 * L1 * L13), -1, 1))
        beta_one = np.arctan2(y3 - y1, x3 - x1)
        theta1 = np.degrees(beta_one - alpha_one)

        # Tính góc cho motor 2
        alpha_five = np.arccos(np.clip((L4 ** 2 + L53 ** 2 - L3 ** 2) / (2 * L4 * L53), -1, 1))
        beta_five = np.arctan2(y3 - y5, x3 - x5)
        theta2 = np.degrees(beta_five + alpha_five)

        # Chuẩn hóa góc về khoảng [-180, 180]
        while theta1 > 180: theta1 -= 360
        while theta1 < -180: theta1 += 360
        while theta2 > 180: theta2 -= 360
        while theta2 < -180: theta2 += 360

        # Áp dụng giới hạn góc
        if (theta1 < THETA1_MIN or theta1 > THETA1_MAX or
                theta2 < THETA2_MIN or theta2 > THETA2_MAX or
                abs(theta1 - theta2) > THETA_DIFF_MAX):
            return None, None  # Góc nằm ngoài giới hạn

        return theta1, theta2

    def check_point_in_workspace(self, x, y):
        """Kiểm tra xem điểm có nằm trong vùng làm việc không"""
        bounds = self.workspace_bounds

        # Kiểm tra theo hình chữ nhật
        if (bounds['left'] <= x <= bounds['right'] and
                bounds['bottom'] <= y <= bounds['top']):
            # Kiểm tra với động học ngược
            theta1, theta2 = self.inverse_kinematics(x, y)
            return theta1 is not None and theta2 is not None

        return False


class ScaraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Controller")
        self.root.geometry("1000x700")

        # Khởi tạo robot và Serial
        self.robot = ScaraRobot()
        self.serial = None
        self.is_connected = False
        self.current_angles = {'theta1': 0.0, 'theta2': 0.0}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.pen_is_down = False

        # Tạo giao diện
        self.create_widgets()
        self.update_canvas()

        # Cập nhật danh sách cổng COM
        self.update_ports()

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
        self.speed_var = tk.StringVar(value="800")
        speed_entry = ttk.Entry(speed_frame, textvariable=self.speed_var, width=8)
        speed_entry.grid(row=0, column=1, padx=5)

        ttk.Button(speed_frame, text="Set", command=self.set_speed).grid(row=0, column=2, padx=5)

        ttk.Label(speed_frame, text="Gia tốc:").grid(row=1, column=0, padx=5, pady=5)
        self.accel_var = tk.StringVar(value="300")
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
        """Kết nối hoặc ngắt kết nối với Arduino"""
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
                time.sleep(2.5)

                # Xóa bộ đệm
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # Kiểm tra kết nối
                self.log("Gửi lệnh kiểm tra...")
                self.serial.write(b"status\n")
                time.sleep(0.5)

                # Đọc và hiển thị toàn bộ phản hồi để debug
                response_text = ""
                attempts = 0
                while attempts < 3:  # Thử tối đa 3 lần
                    if self.serial.in_waiting:
                        response = self.serial.readline().decode('utf-8', errors='replace').strip()
                        self.log(f"Nhận được: '{response}'")
                        response_text += response

                        # Kiểm tra nếu có phản hồi hợp lệ
                        if any(x in response for x in ["READY", "SCARA_READY", "OK"]):
                            self.is_connected = True
                            self.connect_btn.configure(text="Ngắt kết nối")
                            self.log(f"✓ Kết nối thành công với {port}")

                            # Thiết lập tốc độ và gia tốc
                            self.set_speed()
                            self.set_acceleration()
                            return
                    else:
                        attempts += 1
                        self.log(f"Không nhận được phản hồi, thử lại... ({attempts}/3)")
                        self.serial.write(b"status\n")
                        time.sleep(1.0)

                # Nếu không nhận được phản hồi đúng sau 3 lần
                self.log(f"❌ Không nhận được phản hồi hợp lệ từ Arduino!")
                self.log("Phản hồi nhận được: " + (response_text if response_text else "Không có"))
                self.serial.close()
                messagebox.showwarning("Cảnh báo",
                                       "Không nhận được phản hồi đúng từ Arduino.\nKiểm tra lại cổng COM và đảm bảo Arduino đã được nạp firmware đúng.")
                self.is_connected = False

            except Exception as e:
                self.log(f"❌ Lỗi kết nối: {str(e)}")
                messagebox.showerror("Lỗi", f"Không thể kết nối: {str(e)}")
                if self.serial and self.serial.is_open:
                    self.serial.close()

    def send_command(self, command, wait_for_response=True):
        """Gửi lệnh đến Arduino"""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.log("Chưa kết nối với Arduino!")
            return False

        try:
            # Xóa buffer
            self.serial.reset_input_buffer()

            # Gửi lệnh
            cmd = command.strip() + '\n'
            self.serial.write(cmd.encode('utf-8'))
            self.log(f"Gửi: {command}")

            # Đợi phản hồi
            if wait_for_response:
                time.sleep(0.1)  # Đợi Arduino xử lý

                # Đọc phản hồi
                responses = []
                start_time = time.time()
                while (time.time() - start_time < 5.0):  # Timeout 5 giây
                    if self.serial.in_waiting > 0:
                        response = self.serial.readline().decode('utf-8').strip()
                        if response:
                            self.log(f"Nhận: {response}")
                            responses.append(response)

                            # Kiểm tra hoàn thành
                            if "MOVE_COMPLETE" in response or "READY" in response:
                                break
                    else:
                        time.sleep(0.1)

                # Cập nhật trạng thái
                for response in responses:
                    if "PEN_UP" in response:
                        self.pen_is_down = False
                        self.pen_label.configure(text="Nâng")
                    elif "PEN_DOWN" in response:
                        self.pen_is_down = True
                        self.pen_label.configure(text="Hạ")

                    # Tìm phản hồi về góc
                    match = re.search(r"MOVING_TO:(-?\d+\.?\d*),(-?\d+\.?\d*)", response)
                    if match:
                        theta1 = float(match.group(1))
                        theta2 = float(match.group(2))
                        self.update_position(theta1, theta2)

                return len(responses) > 0

        except Exception as e:
            self.log(f"Lỗi: {str(e)}")
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

            # Tính góc
            theta1, theta2 = self.robot.inverse_kinematics(x, y)
            if theta1 is None or theta2 is None:
                messagebox.showwarning("Cảnh báo", f"Không thể tính góc cho vị trí ({x}, {y})!")
                return

            # Cập nhật giá trị cho góc
            self.angle1_entry.delete(0, tk.END)
            self.angle1_entry.insert(0, f"{theta1:.2f}")
            self.angle2_entry.delete(0, tk.END)
            self.angle2_entry.insert(0, f"{theta2:.2f}")

            # Di chuyển
            self.send_command(f"{theta1:.2f},{theta2:.2f}")

        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị tọa độ không hợp lệ!")

    def move_to_angle(self):
        """Di chuyển đến góc"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino!")
            return

        try:
            theta1 = float(self.angle1_entry.get())
            theta2 = float(self.angle2_entry.get())

            # Di chuyển
            self.send_command(f"{theta1:.2f},{theta2:.2f}")

            # Cập nhật vị trí
            self.update_position(theta1, theta2)

        except ValueError:
            messagebox.showerror("Lỗi", "Giá trị góc không hợp lệ!")

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
            self.send_command("90,90")

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
        """Vẽ lại robot trên canvas"""
        # Xóa canvas
        self.canvas.delete("all")

        canvas_width = self.canvas.winfo_width() or 500
        canvas_height = self.canvas.winfo_height() or 400

        # Tỉ lệ chuyển đổi từ cm sang pixel
        scale = min(canvas_width, canvas_height) / 100

        # Tâm canvas
        cx = canvas_width / 2
        cy = canvas_height / 2

        # Vẽ vùng làm việc
        bounds = self.robot.workspace_bounds
        x1 = cx + bounds['left'] * scale
        y1 = cy - bounds['top'] * scale
        x2 = cx + bounds['right'] * scale
        y2 = cy - bounds['bottom'] * scale
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="lightgray", dash=(2, 2))

        # Vẽ trục tọa độ
        self.canvas.create_line(cx - 50, cy, cx + 50, cy, fill="lightgray", arrow=tk.LAST)
        self.canvas.create_line(cx, cy + 50, cx, cy - 50, fill="lightgray", arrow=tk.LAST)
        self.canvas.create_text(cx + 55, cy, text="X", fill="gray")
        self.canvas.create_text(cx, cy - 55, text="Y", fill="gray")

        # Thông số robot
        robot = self.robot.robot_params

        # Vị trí motor
        motor1_x = cx + robot['x1'] * scale
        motor1_y = cy - robot['y1'] * scale
        motor2_x = cx + robot['x5'] * scale
        motor2_y = cy - robot['y5'] * scale

        # Vẽ motors
        motor_radius = 6
        self.canvas.create_oval(motor1_x - motor_radius, motor1_y - motor_radius,
                                motor1_x + motor_radius, motor1_y + motor_radius,
                                fill="blue", outline="black")
        self.canvas.create_text(motor1_x, motor1_y - 15, text="Motor Y")

        self.canvas.create_oval(motor2_x - motor_radius, motor2_y - motor_radius,
                                motor2_x + motor_radius, motor2_y + motor_radius,
                                fill="blue", outline="black")
        self.canvas.create_text(motor2_x, motor2_y - 15, text="Motor X")

        # Tính vị trí các khớp từ góc hiện tại
        try:
            theta1 = self.current_angles['theta1']
            theta2 = self.current_angles['theta2']
            (x2, y2), (x3, y3), (x4, y4) = self.robot.forward_kinematics(theta1, theta2)

            # Chuyển đổi tọa độ
            joint1_x = cx + x2 * scale
            joint1_y = cy - y2 * scale
            pen_x = cx + x3 * scale
            pen_y = cy - y3 * scale
            joint2_x = cx + x4 * scale
            joint2_y = cy - y4 * scale

            # Vẽ cánh tay 1
            self.canvas.create_line(motor1_x, motor1_y, joint1_x, joint1_y, fill="red", width=3)

            # Vẽ cánh tay 2
            self.canvas.create_line(joint1_x, joint1_y, pen_x, pen_y, fill="green", width=3)

            # Vẽ cánh tay 3
            self.canvas.create_line(pen_x, pen_y, joint2_x, joint2_y, fill="green", width=3)

            # Vẽ cánh tay 4
            self.canvas.create_line(joint2_x, joint2_y, motor2_x, motor2_y, fill="red", width=3)

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
            home_py = cy - home_y * scale
            self.canvas.create_text(home_px, home_py, text="H", fill="blue")
            self.canvas.create_oval(home_px - 3, home_py - 3, home_px + 3, home_py + 3,
                                    fill="blue", outline="blue")

        except Exception as e:
            self.log(f"Lỗi vẽ robot: {str(e)}")


def main():
    root = tk.Tk()
    app = ScaraGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()