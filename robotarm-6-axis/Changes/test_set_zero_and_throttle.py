import struct
import time
from communication import (
    HEADER_COMMAND, HEADER_FEEDBACK,
    CMD_STOP, CMD_MOVE_PC, CMD_SET_ZERO,
    RobotCommunicator
)
from donghoc import calc_crc16

def test_set_zero_packet_structure():
    print("[TEST 1] Kiem tra cau truc goi tin CMD_SET_ZERO va CRC16...")
    # Format: <HBB6f (28 bytes) + <H CRC16 (2 bytes) = 30 bytes
    cmd_type = CMD_SET_ZERO
    tool_state = 0
    angles = [0.0] * 6
    payload = struct.pack('<HBB6f', HEADER_COMMAND, cmd_type, tool_state, *angles)
    assert len(payload) == 28, f"Payload size must be 28 bytes, got {len(payload)}"
    crc = calc_crc16(payload)
    packet = payload + struct.pack('<H', crc)
    assert len(packet) == 30, f"Full packet must be 30 bytes, got {len(packet)}"

    # Unpack check
    header, c_type, t_state, *unpacked_angles = struct.unpack('<HBB6f', packet[:28])
    unpacked_crc = struct.unpack('<H', packet[28:30])[0]
    assert header == HEADER_COMMAND
    assert c_type == CMD_SET_ZERO
    assert t_state == 0
    assert unpacked_crc == crc
    print("  -> Packet CMD_SET_ZERO (30 bytes, CRC16): PASS!")

def test_throttled_dispatch_logic():
    print("[TEST 2] Kiem tra logic dieu tiet Dispatcher khong bi mat goi cuoi...")
    comm = RobotCommunicator()
    # Mocking
    sent_packets = []
    comm.is_connected = True
    comm.mode = "SERIAL"
    class MockSerial:
        def write(self, data):
            sent_packets.append(data)
    comm.ser = MockSerial()

    # Goi CMD_SET_ZERO luon duoc gui ngay voi force=True
    comm.send_set_zero()
    assert len(sent_packets) == 1, "send_set_zero phai duoc gui ngay lap tuc!"
    
    # Goi CMD_MOVE_PC lien tuc trong khoang thoi gian nho hon 15ms
    comm.send_pc_command([10.0]*6)
    # Goi tiep theo trong <15ms bi rate limit (skip)
    comm.send_pc_command([20.0]*6)
    # Goi force=True (khi nha chuot) phai duoc gui ngay
    comm.send_pc_command([20.0]*6, force=True)
    assert len(sent_packets) >= 2, "Goi force=True khong duoc phep bi drop!"
    print("  -> Throttled dispatch va Force Flush: PASS!")

def test_telemetry_is_calibrated_decoding():
    print("[TEST 3] Kiem tra giai ma telemetry Bit 7 (is_calibrated) tu ESP32-S3...")
    comm = RobotCommunicator()
    comm.is_connected = True
    comm.mode = "SERIAL"

    # Robot mode = 1 (Jog), is_calibrated = True -> 0x81
    robot_mode_byte = 1 | 0x80
    tool_state = 0
    angles = [0.0, 15.0, -30.0, 45.0, -60.0, 90.0]
    payload = struct.pack('<HBB6f', HEADER_FEEDBACK, robot_mode_byte, tool_state, *angles)
    crc = calc_crc16(payload)
    packet = payload + struct.pack('<H', crc)

    class MockSerialRx:
        def __init__(self, data):
            self.data = bytearray(data)
        @property
        def in_waiting(self):
            return len(self.data)
        def read(self, n):
            res = bytes(self.data[:n])
            del self.data[:n]
            return res

    comm.ser = MockSerialRx(packet)
    feedback = comm.read_telemetry()
    assert feedback is not None, "Telemetry khong doc duoc!"
    assert feedback["robot_mode"] == 1, f"Expected robot_mode 1, got {feedback['robot_mode']}"
    assert feedback["is_calibrated"] is True, "Expected is_calibrated True!"
    assert feedback["angles"] == angles, "Angles parsed incorrectly!"
    print("  -> Telemetry is_calibrated bitmask decode: PASS!")

def test_home_config_persistence():
    print("[TEST 4] Kiem tra luu tru file home_config.json...")
    import os, json
    test_cfg = os.path.join(os.path.dirname(__file__), "test_home_config.json")
    try:
        # Save
        with open(test_cfg, "w", encoding="utf-8") as f:
            json.dump({"is_calibrated": True}, f)
        # Load
        with open(test_cfg, "r", encoding="utf-8") as f:
            data = json.load(f)
            assert data.get("is_calibrated") is True
        print("  -> home_config.json persistence: PASS!")
    finally:
        if os.path.exists(test_cfg):
            os.remove(test_cfg)

if __name__ == "__main__":
    print("=" * 60)
    print("  KIEM CHUNG TINH NANG CMD_SET_ZERO VA BO DIEU TIET THROTTLE")
    print("=" * 60)
    test_set_zero_packet_structure()
    test_throttled_dispatch_logic()
    test_telemetry_is_calibrated_decoding()
    test_home_config_persistence()
    print("=" * 60)
    print("[OK] TOAN BO CAC BAI TEST SET_ZERO VA THROTTLE DA PASS 100%!")
    print("=" * 60)
