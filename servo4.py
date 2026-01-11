import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# --- הגדרות חומרה ---
SERVO_PIN = 8
factory = PiGPIOFactory()

# הפתרון כאן: initial_value=None גורם למנוע לא לזוז בכלל כשהקוד עולה
servo = Servo(
    SERVO_PIN,
    initial_value=None,
    min_pulse_width=0.5 / 1000,
    max_pulse_width=2.5 / 1000,
    pin_factory=factory
)

# --- הגדרות תנועה ---
MOVE_STEP = 0.39  # 70 מעלות עבור מנוע 360

# אנחנו מתחילים ב-0, אבל המנוע לא ידע מזה עד הלחיצה הראשונה
current_pos = 0.0


def execute_move(direction):
    global current_pos

    # במידה וזו הפעם הראשונה, המנוע יתחיל מהאמצע (0) או מכל ערך שתבחר כאן
    if direction == "open":
        new_val = current_pos - MOVE_STEP
    else:
        new_val = current_pos + MOVE_STEP

    # הגנה על גבולות
    if new_val > 1.0: new_val = 1.0
    if new_val < -1.0: new_val = -1.0

    current_pos = new_val
    servo.value = current_pos
    print(f">> בתנועה לערך: {current_pos:.2f}")


try:
    print("\n" + "=" * 45)
    print("המערכת רצה. המנוע לא אמור לזוז עכשיו!")
    print("כוון אותו עם היד לאן שתרצה.")
    print("ברגע שתלחץ o או c, הוא יתחיל לפעול מהנקודה הנוכחית.")
    print("=" * 45)

    while True:
        cmd = input("\nהכנס פקודה (o/c/q): ").lower().strip()

        if cmd == 'o':
            execute_move("open")
        elif cmd == 'c':
            execute_move("close")
        elif cmd == 'q':
            break

except KeyboardInterrupt:
    pass
finally:
    servo.detach()
    print("המערכת נעצרה.")