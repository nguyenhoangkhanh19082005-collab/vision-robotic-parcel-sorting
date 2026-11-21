import sys
if sys.platform == "win32" and hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from communication import RobotCommunicator

def test_serial_disconnect_recovery():
    print("[TEST] Kiểm tra khả năng tự phục hồi khi cổng COM bị PermissionError...")
    comm = RobotCommunicator(serial_port="COM_DEAD")
    comm.mode = "SERIAL"
    comm.is_connected = True

    # Giả lập đối tượng Serial bị hỏng handle (ném PermissionError như trên Windows thực tế)
    class BrokenSerial:
        @property
        def in_waiting(self):
            raise PermissionError(13, 'ClearCommError failed (PermissionError: Access is denied.)', None, 5)
        def write(self, data):
            raise PermissionError(13, 'WriteFile failed (PermissionError: Access is denied.)', None, 5)
        def close(self):
            pass

    comm.ser = BrokenSerial()

    # 1. Gọi read_telemetry khi handle bị hỏng
    result = comm.read_telemetry()
    assert result is None, "Kết quả read_telemetry phải là None khi cổng hỏng"
    assert comm.is_connected is False, "comm.is_connected phải tự động chuyển thành False"
    assert comm.ser is None, "comm.ser phải được giải phóng (None)"
    print("  -> Tự động ngắt kết nối an toàn khi đọc: PASS!")

    # 2. Kiểm tra _send_packet khi handle bị hỏng
    comm.is_connected = True
    comm.ser = BrokenSerial()
    send_ok = comm._send_packet(1, 0, [0.0]*6)
    assert send_ok is False, "Gửi packet phải trả về False khi cổng hỏng"
    assert comm.is_connected is False, "comm.is_connected phải tự động chuyển thành False khi gửi hỏng"
    assert comm.ser is None, "comm.ser phải được đóng hoàn toàn"
    print("  -> Tự động ngắt kết nối an toàn khi gửi: PASS!")

if __name__ == "__main__":
    test_serial_disconnect_recovery()
    print("🎉 TẤT CẢ CÁC BÀI TEST TỰ ĐỘNG PHỤC HỒI CỔNG COM ĐÃ PASS 100%!")
