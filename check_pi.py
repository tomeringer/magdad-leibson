import serial
import time

# הגדרת הפורט - ודא שזה תואם למה שמופיע אצלך (ACM0 או USB0)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# פונקציית עזר להפיכת הקידוד למילים מובנות
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
        
        print("Ready! Listening for Glove Array Data...\n" + "="*70)
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # אם השורה מתחילה במילה DATA
                if line.startswith("DATA:"):
                    try:
                        # חילוץ 4 הבתים מהמחרוזת
                        parts = line.split(":")[1].split(",")
                        b1, b2, b3, b4 = [int(x) for x in parts]
                        
                        # 1. שחזור 5 האצבעות (10 ביטים שנמצאים ב-b1 וחלק מ-b2)
                        flex_data = (b2 << 8) | b1
                        fingers = [(flex_data >> (i * 2)) & 0x03 for i in range(5)]
                        
                        # 2. שחזור מצב תנועת היד (4 ביטים מ-b2)
                        roll_state = decode_imu((b2 >> 2) & 0x03)
                        pitch_state = decode_imu((b2 >> 4) & 0x03)

                        # 3. שחזור הזוויות האמיתיות במעלות (החסרת ה-128)
                        raw_roll = b3 - 128
                        raw_pitch = b4 - 128
                        
                        # הדפסה יפה ומסודרת למסך
                        print(f"Fingers: {fingers} | Roll: {raw_roll:4d}° ({roll_state:6}) | Pitch: {raw_pitch:4d}° ({pitch_state:6})")
                        
                    except Exception as e:
                        # התעלמות משגיאות פירוק אקראיות
                        pass
                else:
                    # הדפסת הודעות מערכת של הארדואינו
                    print(f"[Arduino Info] {line}")

    except serial.SerialException as e:
        print(f"Serial Error: {e}. Is the USB plugged in?")
    except KeyboardInterrupt:
        print("\nStopping Script...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
