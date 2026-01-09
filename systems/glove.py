import socket
import time
from typing import Optional

# ============================================================
# WIFI / UDP GLOVE CONFIG
#   ESP sends 3 bytes per datagram: 0xAA <payload> 0x55
#   Pi listens on UDP_PORT on all interfaces
# ============================================================
UDP_LISTEN_IP = "0.0.0.0"
UDP_PORT = 4210

FRAME_START = 0xAA
FRAME_END = 0x55

CONTROL_PERIOD_SEC = 0.01
SILENCE_STOP_SEC = 0.7


# ============================================================
# UDP HELPERS (reliable like your working receiver)
# ============================================================
def open_udp_socket_forever(bind_ip: str, bind_port: int):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Intentionally NOT using SO_REUSEADDR (helps catch port-conflicts loudly)
            s.bind((bind_ip, bind_port))
            s.settimeout(0.05)  # 50ms blocking wait
            print(f"[UDP] Listening on {bind_ip}:{bind_port}")
            return s
        except Exception as e:
            print("[UDP] Failed to bind, retrying:", e)
            time.sleep(1.0)


def read_one_udp_payload(sock) -> Optional[int]:
    """
    Receives one UDP datagram (or times out).
    Expects: 0xAA <payload> 0x55
    Returns payload (0..255) or None.
    """
    try:
        data, _addr = sock.recvfrom(1024)
    except socket.timeout:
        return None
    except Exception as e:
        print("[UDP] recv error:", e)
        return None

    if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
        return int(data[1])
    return None


# ============================================================
# PAYLOAD HANDLER
# ============================================================
last_ultra_block_print = 0.0


def handle_payload(payload: int):
    global last_ultra_block_print

    flex = (payload >> 4) & 0x0F
    roll_code = (payload >> 2) & 0x03
    pitch_code = payload & 0x03

    print(f"[GLOVE] payload=0x{payload:02X}")

    # FLEX bits
    f0 = (flex >> 0) & 1
    f1 = (flex >> 1) & 1
    f2 = (flex >> 2) & 1
    f3 = (flex >> 3) & 1

    # Servo (independent)
    servo_spin(f1) if f0 else servo_stop()

    # Stepper (independent)
    if f2 and not f3:
        stepper_move(STEPPER_STEP_CHUNK, 1)
    elif f3 and not f2:
        stepper_move(STEPPER_STEP_CHUNK, 0)

    # Decide desired DC drive action
    # (same logic as before)
    desired = "stop"
    if pitch_code == 0b01:
        desired = "forward"
    elif pitch_code == 0b10:
        desired = "reverse"
    else:
        if roll_code == 0b01:
            desired = "turn_right"
        elif roll_code == 0b10:
            desired = "turn_left"
        else:
            desired = "stop"

    # Ultrasonic safety:
    # If too close (<40cm), BLOCK forward/turn, but allow reverse to back away.
    if ultrasonic_too_close() and desired in ("forward", "turn_left", "turn_right"):
        motor_stop()
        now = time.time()
        if now - last_ultra_block_print > 0.5:
            last_ultra_block_print = now
            print(f"[ULTRA] BLOCK DRIVE: {_last_distance_cm:.1f} cm < {ULTRA_STOP_CM:.1f} cm")
        return

    # Execute DC drive
    if desired == "forward":
        motor1_forward()
        motor2_forward()
    elif desired == "reverse":
        motor1_reverse()
        motor2_reverse()
    elif desired == "turn_right":
        motor1_forward()
        motor2_reverse()
    elif desired == "turn_left":
        motor1_reverse()
        motor2_forward()
    else:
        motor_stop()


# ============================================================
# MAIN GLOVE LOOP (UDP PUSH-BASED)
# ============================================================
def run_glove_loop():
    print("[GLOVE] WiFi UDP mode (listen-only)")
    sock = open_udp_socket_forever(UDP_LISTEN_IP, UDP_PORT)

    last_rx = time.time()
    last_stop_action = 0.0
    last_silence_report = 0.0
    SILENCE_REPORT_EVERY = 0.5

    try:
        while True:
            # ultrasonic runs continuously + prints periodically
            ultrasonic_tick()

            payload = read_one_udp_payload(sock)

            if payload is not None:
                now = time.time()
                gap = now - last_rx
                if gap > 0.2:
                    print(f"[GLOVE] gap {gap:.3f}s (time since last payload)")
                last_rx = now

                t0 = time.time()
                handle_payload(payload)
                t1 = time.time()
                handle_ms = (t1 - t0) * 1000.0
                if handle_ms > 50.0:
                    print(f"[DBG][CPU] handle_payload took {handle_ms:.1f}ms")

            else:
                now = time.time()
                silent_for = now - last_rx

                if silent_for > 0.3 and (now - last_silence_report) >= SILENCE_REPORT_EVERY:
                    print(f"[DBG][SILENCE] silent_for={silent_for:.2f}s")
                    last_silence_report = now

                if silent_for > SILENCE_STOP_SEC and (now - last_stop_action) > 0.5:
                    motor_stop()
                    servo_stop()
                    _stepper.stop()
                    last_stop_action = now

            time.sleep(CONTROL_PERIOD_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass
