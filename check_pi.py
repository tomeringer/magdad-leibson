import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def decode_imu(val):
    if val == 0b01: return "HIGH"
    if val == 0b10: return "LOW"
    return "CENTER"

def main():
    print(f"Connecting to Arduino on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.dtr = False
        time.sleep(1)
        ser.reset_input_buffer()
        ser.dtr = True
        
        print("Ready! Listening for Array Data...\n" + "="*60)
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line.startswith("DATA:"):
                    try:
                        parts = line.split(":")[1].split(",")
                        b1, b2, b3, b4 = [int(x) for x in parts]
                        
                        # פירוק 5 האצבעות (10 ביטים מ-b1 ו-b2)
                        flex_data = (b2 << 8) | b1
                        fingers = [(flex_data >> (i * 2)) & 0x03 for i in range(5)]
                        
                        # פירוק סטטוס IMU (4 ביטים מ-b2)
                        roll_state = decode_imu((b2 >> 2) & 0x03)
                        pitch_state = decode_imu((b2 >> 4) & 0x03)

                        # שחזור זוויות גולמיות (החסרת 128 שהוספנו ב-ESP)
                        raw_roll = b3 - 128
                        raw_pitch = b4 - 128
                        
                        print(f"Flex: {fingers} | IMU Roll: {raw_roll:4d}° ({roll_state:6}) | Pitch: {raw_pitch:4d}° ({pitch_state:6})")
                        
                    except Exception as e:
                        pass
                else:
                    print(f"[Arduino] {line}")

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == '__main__':
    main()
