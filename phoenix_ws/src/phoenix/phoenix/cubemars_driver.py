import can
import struct
import threading
import time


CAN_PACKET_SET_RPM = 3
CAN_DISABLE        = 15


class CubeMarsAK:
    """Single CubeMars AK motor CAN driver. Shares a Bus instance with sibling motors."""

    ERPM_HARD_MAX = 100000

    def __init__(self, bus: can.Bus, motor_id: int):
        self.motor_id = motor_id
        self._bus     = bus

    def _send_ext(self, mode_id: int, data: bytes = b""):
        # Extended CAN ID: [28:8] = mode_id, [7:0] = motor_id
        can_id = (mode_id << 8) | self.motor_id
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        self._bus.send(msg)

    def set_velocity_erpm(self, erpm: int):
        erpm = max(min(int(erpm), self.ERPM_HARD_MAX), -self.ERPM_HARD_MAX)
        self._send_ext(CAN_PACKET_SET_RPM, struct.pack(">i", erpm))

    def disable(self):
        self._send_ext(CAN_DISABLE)


class MotorManager:
    """
    Manages multiple CubeMars motors on one CAN bus.

    - Single Bus instance shared across all motors.
    - Threading lock prevents concurrent writes.
    - Background thread drains the receive buffer (extend _handle_feedback for telemetry).
    """

    def __init__(self, channel: str, motor_ids: tuple):
        self._bus    = can.interface.Bus(channel=channel, interface="socketcan")
        self.motors: dict[int, CubeMarsAK] = {
            mid: CubeMarsAK(self._bus, mid) for mid in motor_ids
        }
        self._lock    = threading.Lock()
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        print(f"[MotorManager] Bus '{channel}' opened, motors: {list(motor_ids)}")

    # ── Command interface ──────────────────────────────────────────────────────

    def set_velocity(self, motor_id: int, erpm: int):
        with self._lock:
            self.motors[motor_id].set_velocity_erpm(erpm)

    def set_all_velocity(self, erpm_map: dict):
        """Send velocity commands to multiple motors atomically.
        erpm_map: {motor_id: erpm}, e.g. {1: 5000, 2: -5000}
        """
        with self._lock:
            for mid, erpm in erpm_map.items():
                self.motors[mid].set_velocity_erpm(erpm)

    def stop_all(self):
        with self._lock:
            for motor in self.motors.values():
                motor.set_velocity_erpm(0)

    def disable_all(self):
        with self._lock:
            for motor in self.motors.values():
                motor.disable()

    def close(self):
        self._running = False
        self._recv_thread.join(timeout=1.0)
        self.stop_all()
        time.sleep(0.2)
        self.disable_all()
        time.sleep(0.1)
        self._bus.shutdown()
        print("[MotorManager] Shutdown complete.")

    # ── Receive loop ───────────────────────────────────────────────────────────

    def _recv_loop(self):
        while self._running:
            msg = self._bus.recv(timeout=0.1)
            if msg is None or not msg.is_extended_id:
                continue
            motor_id = msg.arbitration_id & 0xFF
            mode_id  = (msg.arbitration_id >> 8) & 0x1FFFFF
            if motor_id in self.motors:
                self._handle_feedback(motor_id, mode_id, msg.data)

    def _handle_feedback(self, motor_id: int, mode_id: int, data: bytes):
        # Reserved for future telemetry parsing (current, temperature, fault codes).
        pass
