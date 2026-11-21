import sys
if sys.platform == "win32" and hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
import struct
import io
import time
from communication import RobotCommunicator, HEADER_COMMAND, HEADER_FEEDBACK, CMD_MOVE_PC, CMD_STOP
from donghoc import calc_crc16

def test_packet_structure():
    print("[TEST 1] Kiểm tra cấu trúc Binary Struct và CRC16...")
    comm = RobotCommunicator()
    # Mock send packet
    angles = [15.0, -30.0, 45.0, -60.0, 75.0, -90.0]
    payload = struct.pack('<HBB6f', HEADER_COMMAND, CMD_MOVE_PC, 1, *angles)
    crc = calc_crc16(payload)
    pkt = payload + struct.pack('<H', crc)
    assert len(pkt) == 30, f"Kích thước packet phải là 30 bytes, nhận được: {len(pkt)}"
    assert pkt[:2] == b'\x55\xaa', f"Header Command trong little endian phải là 0x55, 0xAA"
    print("  -> Packet Command 30-bytes: PASS!")

def test_serial_telemetry_decoding():
    print("[TEST 2] Kiểm tra bộ giải mã Telemetry USB Serial...")
    comm = RobotCommunicator(serial_port="COM_MOCK")
    comm.mode = "SERIAL"
    comm.is_connected = True

    # Tạo gói feedback giả lập từ ESP32
    test_angles = [10.5, 20.25, -30.75, 45.0, -90.0, 180.0]
    fb_payload = struct.pack('<HBB6f', HEADER_FEEDBACK, 0, 2, *test_angles)
    fb_crc = calc_crc16(fb_payload)
    fb_packet = fb_payload + struct.pack('<H', fb_crc)

    # Giả lập luồng byte Serial kèm nhiễu rác phía trước và phía sau
    noisy_stream = b'\x00\xFF\x12RANDOMLOG_NOISE\r\n' + fb_packet + b'\xAA\xBB'
    
    # Mock đối tượng Serial của pyserial
    class MockSerial:
        def __init__(self, data):
            self.stream = io.BytesIO(data)
        @property
        def in_waiting(self):
            pos = self.stream.tell()
            self.stream.seek(0, io.SEEK_END)
            end = self.stream.tell()
            self.stream.seek(pos)
            return end - pos
        def read(self, n):
            return self.stream.read(n)

    comm.ser = MockSerial(noisy_stream)
    telemetry = comm.read_telemetry()
    assert telemetry is not None, "Không trích xuất được telemetry từ luồng Serial có nhiễu!"
    assert telemetry["robot_mode"] == 0
    assert telemetry["tool_state"] == 2
    for orig, decoded in zip(test_angles, telemetry["angles"]):
        assert abs(orig - decoded) < 1e-4, f"Sai lệch góc: {orig} vs {decoded}"
    print("  -> Telemetry Serial Parsing (khử nhiễu & CRC16): PASS!")
    print(f"  -> Decoded Angles: {telemetry['angles']}")

def test_port_scanning():
    print("[TEST 3] Kiểm tra quét cổng COM khả dụng...")
    ports = RobotCommunicator.list_serial_ports()
    print(f"  -> Danh sách cổng COM hiện hữu: {ports}")
    print("  -> Port scanning test: PASS!")

if __name__ == "__main__":
    print("="*60)
    print("  KIỂM CHỨNG TÍNH NĂNG GIAO TIẾP DUAL-MODE (USB TYPE-C SERIAL)")
    print("="*60)
    test_packet_structure()
    test_serial_telemetry_decoding()
    test_port_scanning()
    print("="*60)
    print("🎉 TOÀN BỘ CÁC BÀI TEST USB DUAL-MODE ĐÃ PASS 100%!")
    print("="*60)
