import socket
import struct
import logging
import time
from donghoc import calc_crc16

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

logger = logging.getLogger("RobotComm")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

HEADER_COMMAND  = 0xAA55 # Magic byte cho gói tin gửi từ Laptop -> ESP32-S3
HEADER_FEEDBACK = 0x55AA # Magic byte cho gói tin phản hồi từ ESP32-S3 -> Laptop

CMD_STOP      = 0
CMD_MOVE_PC   = 1
CMD_REC_WP    = 2
CMD_PLAY_AUTO = 3
CMD_TOOL_ONLY = 4
CMD_SET_ZERO  = 5

class RobotCommunicator:
    """
    Hệ thống giao tiếp Dual-Mode hỗ trợ song song:
    1. USB Type-C Serial (COM Port tốc độ cao 115200 - 921600 baud)
    2. Ethernet W5500 Mạng LAN (UDP Socket Port 8888)
    Cả 2 chế độ đều dùng chung chuẩn nhị phân 30-bytes + CRC16 bảo mật chống nhiễu tuyệt đối.
    """
    def __init__(self, ip="192.168.1.177", port=8888, serial_port=None, baudrate=115200):
        self.ip = ip
        self.port = port
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.mode = "UDP" # "UDP" hoặc "SERIAL"
        self.sock = None
        self.ser = None
        self.is_connected = False
        self.last_tx_time = 0
        self.serial_rx_buffer = bytearray()
        # Theo dõi đường truyền: phát hiện cắm nhầm cổng USB gốc ESP32-S3 / thiết bị không đọc dữ liệu
        self.native_usb = False
        self.connect_time = 0.0
        self.last_rx_time = 0.0
        self.tx_timeouts = 0
        self._last_timeout_log = 0.0

    @staticmethod
    def list_serial_ports():
        """Liệt kê danh sách các cổng COM khả dụng trên máy tính."""
        if not HAS_SERIAL:
            return []
        try:
            ports = serial.tools.list_ports.comports()
            return [p.device for p in ports]
        except Exception:
            return []

    def connect(self, target="192.168.1.177", port_or_baud=None):
        """
        Kết nối thông minh tới Robot:
        - Nếu target bắt đầu bằng 'COM' hoặc '/dev/' -> Chế độ USB Serial (Type-C)
        - Nếu target là IP (ví dụ '192.168.1.177') -> Chế độ Ethernet W5500 UDP
        """
        self.disconnect()
        target_str = str(target).strip()

        # -------------------------------------------------------------
        # 1. KẾT NỐI QUA CỔNG USB SERIAL (TYPE-C)
        # -------------------------------------------------------------
        if target_str.upper().startswith("COM") or target_str.startswith("/dev/"):
            if not HAS_SERIAL:
                return False, "Chưa cài đặt pyserial! Vui lòng cài đặt: pip install pyserial"
            
            baud = int(port_or_baud) if (port_or_baud and str(port_or_baud).isdigit() and int(port_or_baud) > 1000) else self.baudrate
            self.serial_port = target_str.upper() if target_str.upper().startswith("COM") else target_str
            self.baudrate = baud
            try:
                # write_timeout: nếu thiết bị không đọc (sai cổng, ESP32 treo) thì write() trả lỗi sau 50 ms
                # thay vì chặn luồng GUI vô thời hạn (biểu hiện "Not Responding" khi kéo slider).
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.02, write_timeout=0.05)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.serial_rx_buffer.clear()
                self.mode = "SERIAL"
                self.is_connected = True
                self.connect_time = time.time()
                self.last_rx_time = 0.0
                self.tx_timeouts = 0
                # VID 0x303A = cổng USB gốc (USB-Serial/JTAG) của ESP32-S3. Firmware robotarmver2 xuất Serial
                # qua UART0 → chip CH343 (cổng COM khác), nên cổng USB gốc không có telemetry, không nhận lệnh.
                info = next((p for p in serial.tools.list_ports.comports() if p.device.upper() == self.serial_port.upper()), None)
                self.native_usb = bool(info and info.vid == 0x303A)
                logger.info(f"Đã mở cổng USB Type-C Serial {self.serial_port} baud {self.baudrate}")

                # Gửi gói khởi tạo STOP ban đầu để đánh thức luồng telemetry ESP32
                self._send_packet(CMD_STOP, 0, [0.0]*6)
                if self.native_usb:
                    return True, (f"CẢNH BÁO: {self.serial_port} là cổng USB GỐC của ESP32-S3 (303A:1001). Firmware giao tiếp qua "
                                  "cổng UART (chip CH343, cổng COM khác) → cắm cáp sang cổng USB-C còn lại của board rồi kết nối lại.")
                return True, f"Kết nối USB Serial thành công! ({self.serial_port} @ {self.baudrate} baud)"
            except Exception as e:
                self.ser = None
                self.is_connected = False
                return False, f"Lỗi mở cổng USB Serial {self.serial_port}: {str(e)}"

        # -------------------------------------------------------------
        # 2. KẾT NỐI QUA MẠNG LAN ETHERNET W5500 (UDP)
        # -------------------------------------------------------------
        else:
            self.ip = target_str
            if port_or_baud and str(port_or_baud).isdigit():
                self.port = int(port_or_baud)
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.settimeout(0.02) # Giới hạn chờ 20ms
                self.mode = "UDP"
                self.is_connected = True
                logger.info(f"Đã mở cổng UDP Mạng LAN tới Robot: {self.ip}:{self.port}")
                return True, f"Kết nối UDP LAN W5500 thành công! ({self.ip}:{self.port})"
            except Exception as e:
                self.sock = None
                self.is_connected = False
                return False, f"Lỗi tạo Socket UDP LAN: {str(e)}"

    def disconnect(self):
        """Đóng kết nối an toàn cho cả Socket UDP và Serial."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.is_connected = False
        self.serial_rx_buffer.clear()

    def _send_packet(self, cmd_type: int, tool_state: int, angles: list, force: bool = False) -> bool:
        """Đóng gói Binary Struct 30-bytes + CRC16 và gửi qua USB Serial hoặc Ethernet UDP."""
        if not self.is_connected:
            return False

        if len(angles) != 6:
            angles = [0.0] * 6

        now = time.time()
        # Rate-limiting chỉ áp dụng cho stream lệnh di chuyển thông thường (CMD_MOVE_PC).
        # Các lệnh đặc biệt (STOP, SET_ZERO, TOOL, REC_WP) hoặc khi có cờ force=True luôn được gửi tức thì!
        if not force and cmd_type == CMD_MOVE_PC:
            if now - self.last_tx_time < 0.015:
                return True
        self.last_tx_time = now

        try:
            # 1. Đóng gói dữ liệu chưa có CRC (28 bytes)
            # Format: <HBB6f (Header: 2B, CMD: 1B, Tool: 1B, 6 Floats: 24B)
            raw_payload = struct.pack('<HBB6f', HEADER_COMMAND, cmd_type, tool_state, *angles)
            
            # 2. Tính mã kiểm tra CRC16 chuẩn CCITT/MODBUS
            crc = calc_crc16(raw_payload)
            
            # 3. Gắn mã CRC16 vào cuối gói tin (Tổng 30 bytes)
            full_packet = raw_payload + struct.pack('<H', crc)
            
            # 4. Gửi dữ liệu theo chế độ đang kết nối
            if self.mode == "SERIAL" and self.ser:
                self.ser.write(full_packet)
                self._consecutive_timeouts = 0
                return True
            elif self.mode == "UDP" and self.sock:
                self.sock.sendto(full_packet, (self.ip, self.port))
                return True
            return False
        except serial.SerialTimeoutException if HAS_SERIAL else ():
            # Thiết bị không đọc dữ liệu: bỏ gói này, KHÔNG chặn GUI; nhắc lỗi tối đa 1 lần / 3 s
            self.tx_timeouts += 1
            self._consecutive_timeouts = getattr(self, "_consecutive_timeouts", 0) + 1
            if self._consecutive_timeouts >= 5:  # ~0.25 s không ghi được → ngắt hẳn, tránh GUI giật liên tục
                logger.error(f"{self.serial_port} không nhận dữ liệu (5 lần liên tiếp) → tự ngắt kết nối. "
                             "Kiểm tra đúng cổng COM của chip CH343 (không phải cổng USB gốc ESP32-S3).")
                self._consecutive_timeouts = 0
                self.disconnect()
                return False
            if now - self._last_timeout_log > 3.0:
                self._last_timeout_log = now
                logger.warning(f"Gửi lệnh quá thời gian ({self.tx_timeouts} gói bị bỏ): thiết bị trên {self.serial_port} "
                               "không đọc dữ liệu (sai cổng USB, firmware treo hoặc board đang reset).")
            return False
        except (PermissionError, OSError) as e:
            logger.error(f"Mất kết nối cổng COM khi gửi lệnh ({e}). Đang tự động ngắt kết nối an toàn.")
            self.disconnect()
            return False
        except Exception as e:
            logger.error(f"Lỗi gửi packet ({self.mode}): {e}")
            return False

    # ==========================================
    # CÁC HÀM GỬI LỆNH ĐIỀU KHIỂN ROBOT
    # ==========================================

    def send_pc_command(self, angles: list, tool_state: int = 0, force: bool = False):
        """Gửi tọa độ 6 khớp điều khiển trực tiếp từ Slider / IK."""
        return self._send_packet(CMD_MOVE_PC, tool_state, angles, force=force)

    def send_set_zero(self) -> bool:
        """Cài đặt vị trí vật lý hiện tại của Robot làm gốc 0.0° (Zero Reference Calibration)."""
        return self._send_packet(CMD_SET_ZERO, 0, [0.0]*6, force=True)

    def send_rec_command(self, angles: list, tool_state: int = 0) -> bool:
        """Lưu điểm Waypoint vào bộ nhớ Flash của ESP32-S3."""
        return self._send_packet(CMD_REC_WP, tool_state, angles, force=True)

    def send_tool_action(self, action: int):
        """Gửi lệnh điều khiển bơm/van tức thời."""
        return self._send_packet(CMD_TOOL_ONLY, action, [0.0]*6, force=True)

    def play_sequence(self) -> bool:
        """Bắt đầu chạy chu trình tự động."""
        return self._send_packet(CMD_PLAY_AUTO, 0, [0.0]*6, force=True)

    def stop_robot(self) -> bool:
        """Dừng khẩn cấp mọi chuyển động và tắt bơm."""
        return self._send_packet(CMD_STOP, 0, [0.0]*6, force=True)

    def clear_sequence(self) -> bool:
        """Xóa toàn bộ Waypoint khỏi bộ nhớ."""
        return self._send_packet(CMD_STOP, 255, [0.0]*6, force=True)

    # ==========================================
    # LẮNG NGHE PHẢN HỒI TELEMETRY TỪ ESP32-S3
    # ==========================================

    def read_telemetry(self):
        """Đọc và giải mã gói tin phản hồi 30-bytes nhị phân từ ESP32-S3."""
        if not self.is_connected:
            return None

        # --- CHẾ ĐỘ 1: UDP QUA ETHERNET W5500 ---
        if self.mode == "UDP":
            if not self.sock:
                return None
            try:
                data, addr = self.sock.recvfrom(64)
                if len(data) == 30:
                    payload = data[:28]
                    recv_crc = struct.unpack('<H', data[28:30])[0]
                    if calc_crc16(payload) == recv_crc:
                        header, robot_mode, tool_state, a1, a2, a3, a4, a5, a6 = struct.unpack('<HBB6f', payload)
                        if header == HEADER_FEEDBACK:
                            return {
                                "robot_mode": robot_mode & 0x7F,
                                "is_calibrated": bool(robot_mode & 0x80),
                                "tool_state": tool_state,
                                "angles": [a1, a2, a3, a4, a5, a6]
                            }
                    else:
                        logger.warning("Gói tin UDP phản hồi bị lỗi CRC!")
            except socket.timeout:
                pass
            except Exception as e:
                logger.error(f"Lỗi đọc telemetry UDP: {e}")
            return None

        # --- CHẾ ĐỘ 2: SERIAL STREAM QUA USB TYPE-C ---
        elif self.mode == "SERIAL":
            if not self.ser:
                return None
            try:
                in_waiting = self.ser.in_waiting
                if in_waiting > 0:
                    chunk = self.ser.read(in_waiting)
                    self.serial_rx_buffer.extend(chunk)

                # Giới hạn kích thước buffer tối đa 256 bytes để tránh ngập bộ nhớ
                if len(self.serial_rx_buffer) > 256:
                    self.serial_rx_buffer = self.serial_rx_buffer[-128:]

                # Tìm kiếm magic header phản hồi: HEADER_FEEDBACK = 0x55AA (Little Endian: b'\xaa\x55')
                while len(self.serial_rx_buffer) >= 30:
                    idx = self.serial_rx_buffer.find(b'\xaa\x55')
                    if idx == -1:
                        if len(self.serial_rx_buffer) > 0 and self.serial_rx_buffer[-1] == 0xAA:
                            self.serial_rx_buffer = self.serial_rx_buffer[-1:]
                        else:
                            self.serial_rx_buffer.clear()
                        break

                    if idx > 0:
                        del self.serial_rx_buffer[:idx]

                    if len(self.serial_rx_buffer) < 30:
                        break

                    packet_candidate = bytes(self.serial_rx_buffer[:30])
                    payload = packet_candidate[:28]
                    recv_crc = struct.unpack('<H', packet_candidate[28:30])[0]

                    if calc_crc16(payload) == recv_crc:
                        del self.serial_rx_buffer[:30]
                        self.last_rx_time = time.time()
                        header, robot_mode, tool_state, a1, a2, a3, a4, a5, a6 = struct.unpack('<HBB6f', payload)
                        return {
                            "robot_mode": robot_mode & 0x7F,
                            "is_calibrated": bool(robot_mode & 0x80),
                            "tool_state": tool_state,
                            "angles": [a1, a2, a3, a4, a5, a6]
                        }
                    else:
                        del self.serial_rx_buffer[0]
            except (PermissionError, OSError) as e:
                logger.error(f"Mất kết nối cổng COM ({e}). Đang tự động ngắt kết nối an toàn.")
                self.disconnect()
                return None
            except Exception as e:
                logger.error(f"Lỗi đọc telemetry Serial: {e}")
                return None
