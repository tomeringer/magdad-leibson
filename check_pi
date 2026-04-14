import serial
import time

# הגדר את הפורט שבו הארדואינו מחובר לפאי
SERIAL_PORT = '/dev/ttyACM0'  # יכול להיות גם /dev/ttyUSB0
BAUD_RATE = 115200

def main():
    print(f"Connecting to Arduino on {SERIAL_PORT}...")
    try:
        # פתיחת צינור התקשורת
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        # איפוס קצר כדי שהארדואינו יתאפס בצורה נקייה
        ser.dtr = False
        time.sleep(1)
        ser.reset_input_buffer()
        ser.dtr = True
        
        print("Ready! Listening for RF packets...\n" + "-"*30)
        
        expected_counter = None

        while True:
            # אם יש נתונים שמחכים בצינור ה-USB
            if ser.in_waiting > 0:
                # קרא שורה, נקה רווחים ופענח לטקסט
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # אנחנו מחפשים רק שורות שמתחילות ב-"DATA:" (הפורמט שהגדרנו בארדואינו)
                if line.startswith("DATA:"):
                    try:
                        # חילוץ המספר
                        counter_val = int(line.split(":")[1])
                        
                        # בדיקה אם דילגנו על חבילות
                        status = "Perfect"
                        if expected_counter is not None:
                            if counter_val != expected_counter:
                                status = f"MISSED PACKET! Expected {expected_counter}"
                        
                        print(f"Received Counter: {counter_val:3d} | Status: {status}")
                        
                        # חישוב המספר הבא שאנחנו אמורים לקבל (עם חזרה ל-0 אחרי 255)
                        expected_counter = (counter_val + 1) % 256
                        
                    except ValueError:
                        pass # אם הגיע זבל נתעלם
                else:
                    # הדפסת הודעות מערכת של הארדואינו (כמו "Ready")
                    print(f"[Arduino Info] {line}")

    except serial.SerialException as e:
        print(f"Serial Error: {e}. Check the USB cable and Port name.")
    except KeyboardInterrupt:
        print("\nStopping script...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
