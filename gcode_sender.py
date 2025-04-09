import serial
import time
import os.path


class GCodeSender:
    def __init__(self, port, baud=9600, timeout=0.1):
        self.serial = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Đợi Arduino khởi động

    def send_command(self, cmd):
        """Gửi lệnh đến Arduino và đọc phản hồi."""
        cmd = cmd.strip()
        if not cmd:
            return

        self.serial.write((cmd + '\n').encode())
        time.sleep(0.1)  # Đợi một chút để Arduino xử lý

        # Đọc phản hồi từ Arduino
        response = ""
        while self.serial.in_waiting:
            response += self.serial.readline().decode('utf-8')
        return response

    # Thay đổi hàm send_gcode_file trong class GCodeSender
    def send_gcode_file(self, filename):
        """Đọc và gửi file G-code đến Arduino - phiên bản cải tiến."""
        if not os.path.exists(filename):
            print(f"File {filename} không tồn tại!")
            return False

        print(f"Đang mở file {filename}...")

        # Bắt đầu chế độ G-code
        print(self.send_command("gstart"))
        time.sleep(1)

        # Đọc toàn bộ file G-code
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

        # Gửi từng dòng với xử lý phản hồi
        for i, line in enumerate(valid_lines):
            try:
                print(f"Gửi ({i + 1}/{total_lines}): {line}")
                response = self.send_command(line)
                print(f"  Phản hồi: {response}")

                # Chờ cho đến khi Arduino hoàn thành lệnh
                self.wait_for_completion()

            except Exception as e:
                print(f"Lỗi khi gửi lệnh: {e}")
                break

        # Kết thúc chế độ G-code
        print(self.send_command("gend"))
        return True

    def wait_for_completion(self, timeout=30):
        """Đợi cho đến khi Arduino hoàn thành lệnh hoặc hết thời gian."""
        start_time = time.time()

        # Gửi lệnh kiểm tra
        self.serial.write(b"status\n")
        time.sleep(0.1)

        # Đợi phản hồi "sẵn sàng"
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if "hoàn thành" in response.lower() or "ready" in response.lower():
                    return True

            # Gửi lệnh kiểm tra mỗi giây
            time.sleep(0.5)
            self.serial.write(b"status\n")
            time.sleep(0.1)

        print("Hết thời gian đợi Arduino!")
        return False

    def close(self):
        """Đóng kết nối serial."""
        self.serial.close()

    # Thêm phương thức hỗ trợ để xử lý từng dòng G-code
    def _process_gcode_lines(self, file):
        for line in file:
            line = line.strip()
            # Bỏ qua dòng trống và comment hoàn toàn
            if not line or line[0] == ';':
                continue

            # Xử lý comment ở cuối dòng
            if ';' in line:
                line = line[:line.index(';')].strip()

            if line:
                print(f"Gửi: {line}")
                response = self.send_command(line)
                print(f"Phản hồi: {response}")

                # Đợi hoàn thành lệnh trước khi gửi lệnh tiếp theo
                while "đến" in response or "Di chuyển" in response:
                    time.sleep(0.1)



if __name__ == "__main__":
    # Thay đổi COM port phù hợp với máy tính của bạn
    PORT = "COM3"  # Windows: "COM3", Linux/Mac: "/dev/ttyUSB0" hoặc "/dev/ttyACM0"

    try:
        sender = GCodeSender(PORT)
        print("Kết nối thành công với Arduino!")

        # Menu đơn giản
        print("\n=== MENU G-CODE SENDER ===")
        print("1. Gửi lệnh đơn")
        print("2. Gửi file G-code")
        print("3. Về home")
        print("4. Nhấc bút lên")
        print("5. Hạ bút xuống")
        print("6. Thoát")

        choice = input("Lựa chọn: ")

        if choice == '1':
            while True:
                cmd = input("Nhập lệnh (hoặc 'exit' để thoát): ")
                if cmd.lower() == 'exit':
                    break
                print(sender.send_command(cmd))

        elif choice == '2':
            filename = input("Nhập đường dẫn đến file G-code: ")
            sender.send_gcode_file(filename)

        elif choice == '3':
            print(sender.send_command("a"))  # Lệnh home

        elif choice == '4':
            print(sender.send_command("u"))  # Nhấc bút

        elif choice == '5':
            print(sender.send_command("d"))  # Hạ bút

        sender.close()

    except serial.SerialException as e:
        print(f"Lỗi kết nối: {e}")
        print(f"Kiểm tra lại cổng {PORT} và đảm bảo Arduino đã được kết nối.")