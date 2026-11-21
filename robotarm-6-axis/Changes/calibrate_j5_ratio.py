import sys

if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'reconfigure'):
            sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

def main():
    print("=" * 65)
    print("  CÔNG CỤ HIỆU CHUẨN TỈ SỐ TRUYỀN KHỚP 5 (J5 - WRIST PITCH)")
    print("=" * 65)
    print("\n[HƯỚNG DẪN ĐO]")
    print("1. Đặt Khớp 5 về vị trí Home (0 độ - trục cổ tay thẳng hàng).")
    print("2. Trên giao diện, kéo thanh trượt Wrist Pitch (J5) đến góc +90 độ (hoặc -90 độ).")
    print("3. Dùng thước đo góc hoặc ứng dụng đo góc trên điện thoại đặt dọc theo đầu bích cổ tay.")
    print("4. Đọc góc quay THỰC TẾ của cánh tay ngoài đời (đơn vị: Độ).\n")

    cmd_angle = 90.0
    print(f"-> Góc ra lệnh trên phần mềm (Commanded Angle): {cmd_angle}°")
    
    val_str = input("-> Nhập góc thực tế đo được trên cánh tay ngoài đời (mặc định 90): ").strip()
    if not val_str:
        measured_angle = 90.0
    else:
        try:
            measured_angle = abs(float(val_str))
        except ValueError:
            print("Lỗi: Giá trị nhập không hợp lệ!")
            return

    if measured_angle < 1.0:
        print("Lỗi: Góc đo quá nhỏ, vui lòng kiểm tra lại chuyển động!")
        return

    # Tỉ số truyền hiện tại trong code
    current_ratio = 1.0
    # Tỉ số truyền thực tế = current_ratio * (cmd_angle / measured_angle)
    actual_ratio = current_ratio * (cmd_angle / measured_angle)
    
    print("\n" + "=" * 65)
    print("  KẾT QUẢ HIỆU CHUẨN (CALIBRATION RESULT)")
    print("=" * 65)
    print(f"  * Góc ra lệnh: {cmd_angle:.1f}°")
    print(f"  * Góc thực tế: {measured_angle:.1f}°")
    print(f"  * Tỉ số truyền thực tế tính toán: {actual_ratio:.4f}")
    
    # Dự đoán loại cơ cấu
    if abs(actual_ratio - 1.0) < 0.15:
        print("  -> Kết luận: Trục J5 là truyền động TRỰC TIẾP (Direct Drive 1:1).")
        ratio_code = "1.0"
    elif abs(actual_ratio - 2.5) < 0.25:
        print("  -> Kết luận: Trục J5 dùng bộ truyền đai puly 16T / 40T (Tỉ số truyền chuẩn 2.5:1).")
        ratio_code = "2.5"
    elif abs(actual_ratio - 3.44) < 0.3:
        print("  -> Kết luận: Trục J5 dùng bộ truyền đai puly 16T / 55T (Tỉ số truyền chuẩn 3.4375:1).")
        ratio_code = "3.4375"
    else:
        ratio_code = f"{actual_ratio:.4f}"
        print(f"  -> Kết luận: Bộ truyền có tỉ số phi tiêu chuẩn: {actual_ratio:.4f}")

    print("\n[HƯỚNG DẪN CẬP NHẬT FIRMWARE]")
    print("Mở file 'robotarmver2.ino', tìm dòng GEAR_RATIOS và thay đổi giá trị thứ 5 (J5):")
    print(f"  const float GEAR_RATIOS[6] = {{1.0, 5.181818, 5.181818, 5.181818, {ratio_code}, 1.0}};\n")
    print("Sau đó nhấn Upload nạp lại vào ESP32-S3 là góc ngoài đời sẽ trùng khớp 100% với giao diện!")
    print("=" * 65)

if __name__ == "__main__":
    main()
