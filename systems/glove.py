import socket
import time
from typing import Optional

from systems.superstructure import empty_command
from systems.chassis import States
from systems.command import Command


class GloveController:
    def __init__(self):
        # ============================================================
        # WIFI / UDP GLOVE CONFIG
        # ============================================================
        self.UDP_LISTEN_IP = "0.0.0.0"
        self.UDP_PORT = 4210

        self.FRAME_START = 0xAA
        self.FRAME_END = 0x55

        self.SILENCE_STOP_SEC = 0.7

        # ============================================================
        # STATE
        # ============================================================
        self.sock = None

        self.last_rx = time.time()
        self.last_stop_action = 0.0
        self.last_silence_report = 0.0
        self.SILENCE_REPORT_EVERY = 0.5

        self.last_ultra_block_print = 0.0

        self.command = Command(None, None, States.STOP)

    # ============================================================
    # UDP HELPERS
    # ============================================================
    def open_udp_socket_forever(self):
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind((self.UDP_LISTEN_IP, self.UDP_PORT))
                s.settimeout(0.05)
                print(f"[UDP] Listening on {self.UDP_LISTEN_IP}:{self.UDP_PORT}")
                return s
            except Exception as e:
                print("[UDP] Failed to bind, retrying:", e)
                time.sleep(1.0)

    def read_one_udp_payload(self) -> Optional[int]:
        try:
            data, _addr = self.sock.recvfrom(1024)
        except socket.timeout:
            return None
        except Exception as e:
            print("[UDP] recv error:", e)
            return None

        if (
                len(data) >= 3
                and data[0] == self.FRAME_START
                and data[2] == self.FRAME_END
        ):
            return int(data[1])

        return None

    # ============================================================
    # PAYLOAD HANDLER
    # ============================================================
    def handle_payload(self, payload: int):
        self.command = empty_command()

        flex = (payload >> 4) & 0x0F
        roll_code = (payload >> 2) & 0x03
        pitch_code = payload & 0x03

        print(f"[GLOVE] payload=0x{payload:02X}")

        # FLEX bits
        f0 = (flex >> 0) & 1
        f1 = (flex >> 1) & 1
        f2 = (flex >> 2) & 1
        f3 = (flex >> 3) & 1

        # Servo
        if f0:
            self.command.gripper = f1

        # Stepper
        if f2 and not f3:
            self.command.arm = 1
        elif f3 and not f2:
            self.command.arm = 0

        # DC drive
        self.command.chassis = States.STOP
        if pitch_code == 0b01:
            self.command.chassis = States.FORWARD
        elif pitch_code == 0b10:
            self.command.chassis = States.REVERSE
        else:
            if roll_code == 0b01:
                self.command.chassis = States.TURN_RIGHT
            elif roll_code == 0b10:
                self.command.chassis = States.TURN_LEFT
            else:
                self.command.chassis = States.STOP

        self.command.stop = False

    # ============================================================
    # LIFECYCLE
    # ============================================================
    def start(self):
        print("[GLOVE] WiFi UDP mode (listen-only)")
        self.sock = self.open_udp_socket_forever()

    def stop(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    # ============================================================
    # SINGLE LOOP ITERATION (was the while body)
    # ============================================================
    def tick(self):
        payload = self.read_one_udp_payload()

        if payload is not None:
            now = time.time()
            gap = now - self.last_rx
            if gap > 0.2:
                print(f"[GLOVE] gap {gap:.3f}s (time since last payload)")
            self.last_rx = now

            t0 = time.time()
            self.handle_payload(payload)
            t1 = time.time()
            handle_ms = (t1 - t0) * 1000.0
            if handle_ms > 50.0:
                print(f"[DBG][CPU] handle_payload took {handle_ms:.1f}ms")

        else:
            now = time.time()
            silent_for = now - self.last_rx

            if (
                    silent_for > 0.3
                    and (now - self.last_silence_report) >= self.SILENCE_REPORT_EVERY
            ):
                print(f"[DBG][SILENCE] silent_for={silent_for:.2f}s")
                self.last_silence_report = now

            if (
                    silent_for > self.SILENCE_STOP_SEC
                    and (now - self.last_stop_action) > 0.5
            ):
                self.command = empty_command()
                self.last_stop_action = now
