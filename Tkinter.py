import serial
import time
import os.path
import tkinter as tk
from tkinter import filedialog, scrolledtext, ttk
import serial.tools.list_ports
import threading


class GCodeSender:
    def __init__(self, port, baud=9600, timeout=0.1):
        self.serial = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Đợi Arduino khởi động

    def send_command(self, cmd):
        """Gửi lệnh đến Arduino và đọc phản hồi."""
        cmd = cmd.strip()
        if not cmd:
            return ""

        self.serial.write((cmd + '\n').encode())
        time.sleep(0.1)  # Đợi một chút để Arduino xử lý

        # Đọc phản hồi từ Arduino
        response = ""
        time_start = time.time()
        while time.time() - time_start < 0.5:  # Đợi tối đa 0.5 giây
            if self.serial.in_waiting:
                response += self.serial.readline().decode('utf-8', errors='ignore')
            else:
                time.sleep(0.01)
        return response

    def send_gcode_file(self, filename):
        """Đọc và gửi file G-code đến Arduino - phiên bản cải tiến."""
        if not os.path.exists(filename):
            print(f"File {filename} không tồn tại!")
            return False

        print(f"Đang mở file {filename}...")

        # Đọc toàn bộ file G-code với xử lý nhiều loại mã hóa
        try:
            with open(filename, 'r', encoding='utf-8', errors='ignore') as file:
                gcode_lines = [line.strip() for line in file.readlines()]
        except UnicodeDecodeError:
            try:
                with open(filename, 'r', encoding='latin-1') as file:
                    gcode_lines = [line.strip() for line in file.readlines()]
            except Exception as e:
                print(f"Không thể đọc file: {e}")
                return False

        # Xử lý và lọc các dòng G-code
        valid_lines = []
        for line in gcode_lines:
            # Bỏ qua dòng trống và comment hoàn toàn
            if not line or line.startswith(';'):
                continue

            # Xử lý comment ở cuối dòng
            if ';' in line:
                line = line[:line.index(';')].strip()

            if line:
                valid_lines.append(line)

        total_lines = len(valid_lines)
        print(f"Tổng số lệnh G-code: {total_lines}")

        # Bắt đầu chế độ G-code
        print(self.send_command("gstart"))
        time.sleep(1)

        # Gửi từng dòng với xử lý phản hồi
        for i, line in enumerate(valid_lines):
            try:
                print(f"Gửi ({i + 1}/{total_lines}): {line}")
                response = self.send_command(line)
                print(f"  Phản hồi: {response}")

                # Đợi cho đến khi Arduino báo đã hoàn thành
                self.wait_for_completion()

            except Exception as e:
                print(f"Lỗi khi gửi lệnh: {e}")
                break

        # Kết thúc chế độ G-code
        print(self.send_command("gend"))
        return True

    def wait_for_completion(self, max_attempts=60):
        """Đợi cho đến khi Arduino hoàn thành lệnh."""
        attempts = 0

        while attempts < max_attempts:
            response = self.send_command("status")
            if "READY" in response:
                return True

            attempts += 1
            time.sleep(0.5)  # Đợi 0.5 giây giữa các lần kiểm tra

        print("Hết thời gian đợi Arduino!")
        return False

    def close(self):
        """Đóng kết nối serial."""
        self.serial.close()


class GCodeSenderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("G-Code Sender - Máy Vẽ CNC")
        self.root.geometry("800x600")
        self.sender = None
        self.is_connected = False

        self.setup_ui()
        self.update_ports()

    def setup_ui(self):
        # Frame kết nối
        conn_frame = ttk.LabelFrame(self.root, text="Kết nối Arduino")
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(conn_frame, text="Làm mới", command=self.update_ports).grid(row=0, column=2, padx=5, pady=5)
        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)

        # Frame chọn và gửi file
        file_frame = ttk.LabelFrame(self.root, text="File G-Code")
        file_frame.pack(fill="x", padx=10, pady=5)

        self.file_path = tk.StringVar()
        ttk.Entry(file_frame, textvariable=self.file_path, width=60).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(file_frame, text="Chọn file", command=self.select_file).grid(row=0, column=1, padx=5, pady=5)
        self.send_file_btn = ttk.Button(file_frame, text="Gửi file", command=self.send_file, state=tk.DISABLED)
        self.send_file_btn.grid(row=0, column=2, padx=5, pady=5)

        # Frame điều khiển nhanh
        control_frame = ttk.LabelFrame(self.root, text="Điều khiển nhanh")
        control_frame.pack(fill="x", padx=10, pady=5)

        self.home_btn = ttk.Button(control_frame, text="Home (0,0)", command=lambda: self.send_command("a"),
                                   state=tk.DISABLED)
        self.home_btn.grid(row=0, column=0, padx=5, pady=5)

        self.pen_up_btn = ttk.Button(control_frame, text="Bút lên", command=lambda: self.send_command("u"),
                                     state=tk.DISABLED)
        self.pen_up_btn.grid(row=0, column=1, padx=5, pady=5)

        self.pen_down_btn = ttk.Button(control_frame, text="Bút xuống", command=lambda: self.send_command("d"),
                                       state=tk.DISABLED)
        self.pen_down_btn.grid(row=0, column=2, padx=5, pady=5)

        # Frame gửi lệnh đơn
        cmd_frame = ttk.LabelFrame(self.root, text="Gửi lệnh")
        cmd_frame.pack(fill="x", padx=10, pady=5)

        self.cmd_entry = ttk.Entry(cmd_frame, width=60)
        self.cmd_entry.grid(row=0, column=0, padx=5, pady=5)
        self.cmd_entry.bind('<Return>', self.send_single_command)

        self.send_cmd_btn = ttk.Button(cmd_frame, text="Gửi", command=self.send_single_command, state=tk.DISABLED)
        self.send_cmd_btn.grid(row=0, column=1, padx=5, pady=5)

        # Log area
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, width=80, height=15)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Chưa kết nối")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def update_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.get()
            if not port:
                self.log("Vui lòng chọn cổng COM!")
                return

            try:
                self.sender = GCodeSender(port)
                self.is_connected = True
                self.connect_btn.configure(text="Ngắt kết nối")
                self.enable_buttons()
                self.log(f"Đã kết nối đến {port}")
                self.status_var.set(f"Đã kết nối - {port}")
            except Exception as e:
                self.log(f"Lỗi kết nối: {e}")
                self.status_var.set(f"Lỗi kết nối")
        else:
            if self.sender:
                self.sender.close()
            self.is_connected = False
            self.connect_btn.configure(text="Kết nối")
            self.disable_buttons()
            self.log("Đã ngắt kết nối")
            self.status_var.set("Chưa kết nối")

    def enable_buttons(self):
        self.send_file_btn.configure(state=tk.NORMAL)
        self.home_btn.configure(state=tk.NORMAL)
        self.pen_up_btn.configure(state=tk.NORMAL)
        self.pen_down_btn.configure(state=tk.NORMAL)
        self.send_cmd_btn.configure(state=tk.NORMAL)

    def disable_buttons(self):
        self.send_file_btn.configure(state=tk.DISABLED)
        self.home_btn.configure(state=tk.DISABLED)
        self.pen_up_btn.configure(state=tk.DISABLED)
        self.pen_down_btn.configure(state=tk.DISABLED)
        self.send_cmd_btn.configure(state=tk.DISABLED)

    def select_file(self):
        filename = filedialog.askopenfilename(
            title="Chọn file G-code",
            filetypes=(("G-code files", "*.gcode *.nc *.txt"), ("All files", "*.*"))
        )
        if filename:
            self.file_path.set(filename)

    def send_file(self):
        filename = self.file_path.get()
        if not filename:
            self.log("Vui lòng chọn file G-code!")
            return

        self.disable_buttons()
        self.status_var.set("Đang gửi G-code...")
        threading.Thread(target=self._send_file_thread, args=(filename,), daemon=True).start()

    def _send_file_thread(self, filename):
        self.log(f"Đang gửi file {filename}...")
        try:
            self.sender.send_gcode_file(filename)
            self.log("Gửi file hoàn tất!")
            self.root.after(0, lambda: self.status_var.set("Hoàn thành"))
        except Exception as e:
            self.log(f"Lỗi gửi file: {e}")
            self.root.after(0, lambda: self.status_var.set("Lỗi gửi file"))
        finally:
            self.root.after(0, self.enable_buttons)

    def send_single_command(self, event=None):
        cmd = self.cmd_entry.get()
        if not cmd:
            return

        response = self.send_command(cmd)
        self.cmd_entry.delete(0, tk.END)

    def send_command(self, cmd):
        if not self.is_connected or not self.sender:
            self.log("Chưa kết nối đến Arduino!")
            return None

        try:
            self.log(f">> {cmd}")
            response = self.sender.send_command(cmd)
            if response:
                self.log(response)
            return response
        except Exception as e:
            self.log(f"Lỗi: {e}")
            return None

    def log(self, message):
        self.log_text.insert(tk.END, str(message) + "\n")
        self.log_text.see(tk.END)


if __name__ == "__main__":
    root = tk.Tk()
    app = GCodeSenderGUI(root)
    root.mainloop()