from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from machine import FPIOA, Pin, UART
import gc
import os
import sys
import time

try:
    import ujson as json
except ImportError:
    import json


# ==================== Hardware and model ====================
MODEL_PATH = "/data/steelball_yolo11n_320_v6.kmodel"
CALIBRATION_PATH = "/sdcard/h3_calibration.json"
CALIBRATION_TEMP_PATH = "/sdcard/h3_calibration.tmp"
CALIBRATION_VERSION = 1

LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]
FRAME_SIZE = [640, 360]
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None
SEND_TO_IDE = False
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 3

# Low-latency display:
# project the detected center forward by the measured inference time. The
# maximum projection is limited to avoid a noisy detection causing a large jump.
PREDICTION_MAX_MS = 80
PREDICTION_MAX_PIXELS = 20
VELOCITY_FILTER_NEW_WEIGHT = 0.40
GC_INTERVAL_FRAMES = 30
PRINT_INTERVAL_MS = 1000

# K230 40Pin:
# pin 8 = GPIO3/UART1_TXD, pin 10 = GPIO4/UART1_RXD.
UART_TX_GPIO = 3
UART_RX_GPIO = 4

# Three active-low keys:
# physical pin 11 = GPIO5, pin 13 = GPIO6, pin 15 = GPIO26.
KEY_START_GPIO = 5
KEY_PAUSE_GPIO = 6
KEY_MODE_GPIO = 26
KEY_DEBOUNCE_MS = 40
KEY_LONG_PRESS_MS = 1000


# ==================== Modes and states ====================
MODE_DETECT = 1
MODE_TUBE_CAL = 2
MODE_Q3_CAL = 3
MODE_ARBITRARY_CAL = 4
MODE_HOLD_CENTER = 5
MODE_Q3_SEQUENCE = 6
MODE_HOLD_ARBITRARY = 7
MODE_MIN = MODE_DETECT
MODE_MAX = MODE_HOLD_ARBITRARY

RUN_IDLE = 0
RUN_RUNNING = 1
RUN_PAUSED = 2

MODE_NAMES = (
    "",
    "1 LEVEL ZERO",
    "2 TUBE CAL",
    "3 Q3 CAL",
    "4 ANY CAL",
    "5 HOLD CENTER",
    "6 Q3 SEQUENCE",
    "7 HOLD ANY",
)
RUN_NAMES = ("IDLE", "RUN", "PAUSE")

TUBE_MARKS = (
    ("LEFT", -120),
    ("CENTER", 0),
    ("RIGHT", 120),
)
Q3_MARKS = (
    ("O", 0),
    ("+5CM", 50),
    ("-5CM", -50),
)

START_CENTER_TOLERANCE_MM = 15
TARGET_TOLERANCE_MM = 8
POSITIVE_HOLD_MS = 150
POSITIVE_TIMEOUT_MS = 2200
NEGATIVE_HOLD_MS = 300

SEQ_WAIT_CENTER = 0
SEQ_TO_POSITIVE = 1
SEQ_TO_NEGATIVE = 2
SEQ_HOLD_NEGATIVE = 3


# ==================== UART protocol ====================
FRAME_HEAD_0 = 0xAA
FRAME_HEAD_1 = 0x55
MSG_BALL_POSITION = 0x01
MSG_CONTROL_STATE = 0x02
MSG_PIXEL_POSITION = 0x03
MSG_MCU_STATUS = 0x81
CONTROL_SEND_INTERVAL_MS = 100
# Always confirm the physical O point after the camera and tube are mounted.
# A valid saved three-point calibration is still used for the mm scale.
AUTO_HOLD_CENTER_ON_BOOT = False
# Allow for occasional long AI/display frames without falsely declaring the
# 100 ms MCU heartbeat offline.
MCU_STATUS_TIMEOUT_MS = 1500


def ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.time() * 1000)


def ticks_diff(now, before):
    try:
        return time.ticks_diff(now, before)
    except AttributeError:
        return now - before


def ticks_add(base, delta):
    try:
        return time.ticks_add(base, delta)
    except AttributeError:
        return base + delta


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def median(values):
    if not values:
        return None
    ordered = list(values)
    ordered.sort()
    return ordered[len(ordered) // 2]


class MotionPredictor:
    def __init__(self):
        self.last_x = None
        self.last_y = None
        self.last_ms = 0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.missed_frames = 0

    def reset(self):
        self.last_x = None
        self.last_y = None
        self.last_ms = 0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.missed_frames = 0

    def miss(self):
        self.missed_frames += 1
        if self.missed_frames >= 3:
            self.reset()

    def update(self, measured_x, measured_y, now, inference_ms):
        self.missed_frames = 0
        if self.last_x is not None:
            dt_ms = ticks_diff(now, self.last_ms)
            if 5 <= dt_ms <= 500:
                instant_vx = (
                    (measured_x - self.last_x) * 1000.0 / dt_ms
                )
                instant_vy = (
                    (measured_y - self.last_y) * 1000.0 / dt_ms
                )
                old_weight = 1.0 - VELOCITY_FILTER_NEW_WEIGHT
                self.velocity_x = (
                    old_weight * self.velocity_x
                    + VELOCITY_FILTER_NEW_WEIGHT * instant_vx
                )
                self.velocity_y = (
                    old_weight * self.velocity_y
                    + VELOCITY_FILTER_NEW_WEIGHT * instant_vy
                )

                if abs(measured_x - self.last_x) <= 1:
                    self.velocity_x *= 0.5
                if abs(measured_y - self.last_y) <= 1:
                    self.velocity_y *= 0.5

        self.last_x = measured_x
        self.last_y = measured_y
        self.last_ms = now

        lookahead_ms = clamp(
            int(inference_ms), 0, PREDICTION_MAX_MS
        )
        shift_x = clamp(
            int(round(self.velocity_x * lookahead_ms / 1000.0)),
            -PREDICTION_MAX_PIXELS,
            PREDICTION_MAX_PIXELS,
        )
        shift_y = clamp(
            int(round(self.velocity_y * lookahead_ms / 1000.0)),
            -PREDICTION_MAX_PIXELS,
            PREDICTION_MAX_PIXELS,
        )
        return measured_x + shift_x, measured_y + shift_y, shift_x, shift_y


def make_frame(message_type, payload):
    checksum = message_type ^ len(payload)
    for value in payload:
        checksum ^= value
    return (
        bytes((FRAME_HEAD_0, FRAME_HEAD_1, message_type, len(payload)))
        + payload
        + bytes((checksum,))
    )


def int16_bytes(value):
    value = clamp(int(value), -32768, 32767)
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def bytes_to_int16(low, high):
    value = low | (high << 8)
    return value - 65536 if value & 0x8000 else value


class MCUStatus:
    def __init__(self):
        self.phase = 0
        self.target_mm = 0
        self.angle_cdeg = 0
        self.velocity_mm_s = 0
        self.flags = 0
        self.last_rx_ms = 0

    def online(self, now):
        return (
            self.last_rx_ms != 0
            and ticks_diff(now, self.last_rx_ms) <= MCU_STATUS_TIMEOUT_MS
        )


class MCUStatusParser:
    """Parse Tianmengxing status frames without blocking the camera loop."""

    WAIT_HEAD_0 = 0
    WAIT_HEAD_1 = 1
    WAIT_TYPE = 2
    WAIT_LENGTH = 3
    WAIT_PAYLOAD = 4
    WAIT_CHECKSUM = 5
    MAX_PAYLOAD_LENGTH = 32

    def __init__(self):
        self.status = MCUStatus()
        self.reset()

    def reset(self):
        self.state = self.WAIT_HEAD_0
        self.message_type = 0
        self.payload_length = 0
        self.payload = bytearray()
        self.checksum = 0

    def accept_status(self, now):
        if (
            self.message_type != MSG_MCU_STATUS
            or self.payload_length != 8
        ):
            return
        payload = self.payload
        self.status.phase = payload[0]
        self.status.target_mm = bytes_to_int16(payload[1], payload[2])
        self.status.angle_cdeg = bytes_to_int16(payload[3], payload[4])
        self.status.velocity_mm_s = bytes_to_int16(payload[5], payload[6])
        self.status.flags = payload[7]
        self.status.last_rx_ms = now

    def feed(self, data, now):
        if not data:
            return
        for value in data:
            if self.state == self.WAIT_HEAD_0:
                if value == FRAME_HEAD_0:
                    self.state = self.WAIT_HEAD_1

            elif self.state == self.WAIT_HEAD_1:
                if value == FRAME_HEAD_1:
                    self.state = self.WAIT_TYPE
                elif value != FRAME_HEAD_0:
                    self.state = self.WAIT_HEAD_0

            elif self.state == self.WAIT_TYPE:
                self.message_type = value
                self.checksum = value
                self.state = self.WAIT_LENGTH

            elif self.state == self.WAIT_LENGTH:
                self.payload_length = value
                self.checksum ^= value
                self.payload = bytearray()
                if value > self.MAX_PAYLOAD_LENGTH:
                    self.reset()
                elif value == 0:
                    self.state = self.WAIT_CHECKSUM
                else:
                    self.state = self.WAIT_PAYLOAD

            elif self.state == self.WAIT_PAYLOAD:
                self.payload.append(value)
                self.checksum ^= value
                if len(self.payload) >= self.payload_length:
                    self.state = self.WAIT_CHECKSUM

            elif self.state == self.WAIT_CHECKSUM:
                valid = value == self.checksum
                if valid:
                    self.accept_status(now)
                self.reset()
                # Recover faster if a damaged checksum is the next 0xAA.
                if not valid and value == FRAME_HEAD_0:
                    self.state = self.WAIT_HEAD_1

            else:
                self.reset()


def make_position_frame(position_mm, confidence_percent, frame_id,
                        frame_time_ms, inference_ms):
    """Send stable X plus frame timing so the MCU can predict once."""
    confidence_percent = clamp(int(confidence_percent), 0, 100)
    frame_id = int(frame_id) & 0xFFFF
    frame_time_ms = int(frame_time_ms) & 0xFFFF
    inference_ms = clamp(int(inference_ms), 0, 255)
    payload = (
        int16_bytes(position_mm)
        + bytes((confidence_percent,))
        + bytes((frame_id & 0xFF, (frame_id >> 8) & 0xFF))
        + bytes((frame_time_ms & 0xFF, (frame_time_ms >> 8) & 0xFF))
        + bytes((inference_ms,))
    )
    return make_frame(
        MSG_BALL_POSITION,
        payload,
    )


def make_pixel_frame(pixel_x, pixel_y, confidence_percent):
    pixel_x = clamp(int(pixel_x), 0, 65535)
    pixel_y = clamp(int(pixel_y), 0, 65535)
    confidence_percent = clamp(int(confidence_percent), 0, 100)
    payload = bytes(
        (
            pixel_x & 0xFF,
            (pixel_x >> 8) & 0xFF,
            pixel_y & 0xFF,
            (pixel_y >> 8) & 0xFF,
            confidence_percent,
        )
    )
    return make_frame(MSG_PIXEL_POSITION, payload)


def make_control_frame(mode, run_state, target_mm, sequence_phase):
    payload = (
        bytes((mode, run_state))
        + int16_bytes(target_mm)
        + bytes((sequence_phase,))
    )
    return make_frame(MSG_CONTROL_STATE, payload)


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART1_RXD)
    return UART(
        UART.UART1,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


class Button:
    EVENT_NONE = 0
    EVENT_SHORT = 1
    EVENT_LONG = 2

    def __init__(self, gpio):
        self.pin = Pin(gpio, Pin.IN, pull=Pin.PULL_UP, drive=7)
        self.raw = self.pin.value()
        self.stable = self.raw
        now = ticks_ms()
        self.raw_changed_ms = now
        self.pressed_ms = now if self.stable == 0 else 0
        self.long_sent = False

    def update(self, now):
        raw = self.pin.value()
        if raw != self.raw:
            self.raw = raw
            self.raw_changed_ms = now

        if (
            raw != self.stable
            and ticks_diff(now, self.raw_changed_ms) >= KEY_DEBOUNCE_MS
        ):
            self.stable = raw
            if self.stable == 0:
                self.pressed_ms = now
                self.long_sent = False
            else:
                if not self.long_sent:
                    return self.EVENT_SHORT

        if (
            self.stable == 0
            and not self.long_sent
            and ticks_diff(now, self.pressed_ms) >= KEY_LONG_PRESS_MS
        ):
            self.long_sent = True
            return self.EVENT_LONG

        return self.EVENT_NONE


def init_buttons():
    fpioa = FPIOA()
    fpioa.set_function(KEY_START_GPIO, FPIOA.GPIO5)
    fpioa.set_function(KEY_PAUSE_GPIO, FPIOA.GPIO6)
    fpioa.set_function(KEY_MODE_GPIO, FPIOA.GPIO26)
    return (
        Button(KEY_START_GPIO),
        Button(KEY_PAUSE_GPIO),
        Button(KEY_MODE_GPIO),
    )


class CalibrationStore:
    def __init__(self):
        self.display_size = None
        self.tube_pixels = []
        self.q3_pixels = []
        self.arbitrary_pixel = None
        self.arbitrary_mm = None
        self.loaded = False

    def set_display_size(self, display_size):
        self.display_size = [int(display_size[0]), int(display_size[1])]

    def _valid_pixel(self, value):
        return (
            isinstance(value, (int, float))
            and self.display_size is not None
            and 0 <= int(value) < self.display_size[0]
        )

    def tube_valid(self):
        if len(self.tube_pixels) != 3:
            return False
        if not all(self._valid_pixel(value) for value in self.tube_pixels):
            return False
        left, center, right = self.tube_pixels
        increasing = left < center < right
        decreasing = left > center > right
        return (increasing or decreasing) and abs(right - left) >= 100

    def q3_valid(self):
        if len(self.q3_pixels) != 3:
            return False
        if not all(self._valid_pixel(value) for value in self.q3_pixels):
            return False
        point_o, point_positive, point_negative = self.q3_pixels
        return (
            abs(point_positive - point_negative) >= 20
            and min(point_positive, point_negative)
            < point_o
            < max(point_positive, point_negative)
        )

    def arbitrary_valid(self):
        return (
            self._valid_pixel(self.arbitrary_pixel)
            and isinstance(self.arbitrary_mm, (int, float))
            and -120 <= int(self.arbitrary_mm) <= 120
        )

    def active_points(self):
        if not self.tube_valid():
            return []

        points = [
            (int(self.tube_pixels[0]), -120),
            (int(self.tube_pixels[1]), 0),
            (int(self.tube_pixels[2]), 120),
        ]

        if self.q3_valid():
            points = [
                (int(self.tube_pixels[0]), -120),
                (int(self.q3_pixels[2]), -50),
                (int(self.q3_pixels[0]), 0),
                (int(self.q3_pixels[1]), 50),
                (int(self.tube_pixels[2]), 120),
            ]

        points.sort()
        millimeters = [point[1] for point in points]
        increasing = all(
            millimeters[index] < millimeters[index + 1]
            for index in range(len(millimeters) - 1)
        )
        decreasing = all(
            millimeters[index] > millimeters[index + 1]
            for index in range(len(millimeters) - 1)
        )
        if not (increasing or decreasing):
            return []
        return points

    def pixel_to_mm(self, pixel_x):
        points = self.active_points()
        if len(points) < 2:
            return None

        if pixel_x <= points[0][0]:
            left, right = points[0], points[1]
        elif pixel_x >= points[-1][0]:
            left, right = points[-2], points[-1]
        else:
            left, right = points[0], points[1]
            for index in range(len(points) - 1):
                if points[index][0] <= pixel_x <= points[index + 1][0]:
                    left, right = points[index], points[index + 1]
                    break

        dx = right[0] - left[0]
        if dx == 0:
            return None
        value = left[1] + (
            (pixel_x - left[0]) * (right[1] - left[1]) / dx
        )
        return clamp(int(round(value)), -150, 150)

    def recenter(self, center_x):
        """Shift the saved pixel/mm map so the current ball position is 0 mm."""
        if not self.tube_valid() or not self._valid_pixel(center_x):
            return False

        old_tube = list(self.tube_pixels)
        old_q3 = list(self.q3_pixels)
        old_any_pixel = self.arbitrary_pixel
        delta = int(center_x) - int(self.tube_pixels[1])

        self.tube_pixels = [int(value) + delta for value in old_tube]
        self.q3_pixels = [int(value) + delta for value in old_q3]
        if old_any_pixel is not None:
            self.arbitrary_pixel = int(old_any_pixel) + delta

        valid = self.tube_valid()
        if old_q3 and not self.q3_valid():
            valid = False
        if old_any_pixel is not None and not self._valid_pixel(
            self.arbitrary_pixel
        ):
            valid = False

        if not valid:
            self.tube_pixels = old_tube
            self.q3_pixels = old_q3
            self.arbitrary_pixel = old_any_pixel
            return False

        self.loaded = True
        self.save()
        return True

    def save(self):
        data = {
            "version": CALIBRATION_VERSION,
            "display_size": self.display_size,
            "tube_pixels": self.tube_pixels,
            "q3_pixels": self.q3_pixels,
            "arbitrary_pixel": self.arbitrary_pixel,
            "arbitrary_mm": self.arbitrary_mm,
        }
        with open(CALIBRATION_TEMP_PATH, "w") as file:
            file.write(json.dumps(data))

        try:
            os.remove(CALIBRATION_PATH)
        except OSError:
            pass
        os.rename(CALIBRATION_TEMP_PATH, CALIBRATION_PATH)

    def load(self):
        try:
            with open(CALIBRATION_PATH, "r") as file:
                data = json.loads(file.read())
        except (OSError, ValueError):
            return False

        if data.get("version") != CALIBRATION_VERSION:
            return False
        if data.get("display_size") != self.display_size:
            return False

        self.tube_pixels = data.get("tube_pixels", [])
        self.q3_pixels = data.get("q3_pixels", [])
        self.arbitrary_pixel = data.get("arbitrary_pixel")
        self.arbitrary_mm = data.get("arbitrary_mm")
        self.loaded = self.tube_valid()
        return self.loaded


class H3StateMachine:
    def __init__(self, calibration):
        self.calibration = calibration
        self.mode = MODE_DETECT
        self.run_state = RUN_IDLE
        self.tube_step = 0
        self.q3_step = 0
        self.pending_tube_pixels = []
        self.pending_q3_pixels = []
        self.sequence_phase = SEQ_WAIT_CENTER
        self.target_mm = 0
        self.phase_start_ms = 0
        self.in_tolerance_ms = 0
        self.notice = "READY"
        self.notice_until_ms = 0
        self.mcu_status = None
        self.mcu_online = False
        self.task_completed = False

    def set_notice(self, text, now, duration_ms=1800):
        self.notice = text
        self.notice_until_ms = ticks_add(now, duration_ms)

    def visible_notice(self, now):
        if ticks_diff(self.notice_until_ms, now) > 0:
            return self.notice
        return ""

    def update_mcu_status(self, status, now):
        self.mcu_status = status
        self.mcu_online = status.online(now)
        if (
            self.mcu_online
            and self.mode in (
                MODE_HOLD_CENTER,
                MODE_Q3_SEQUENCE,
                MODE_HOLD_ARBITRARY,
            )
            and self.run_state == RUN_RUNNING
            and (status.flags & 0x04)
            and not self.task_completed
        ):
            self.task_completed = True
            self.set_notice("TASK COMPLETED", now)

    def reset_operation(self):
        self.run_state = RUN_IDLE
        self.sequence_phase = SEQ_WAIT_CENTER
        self.target_mm = 0
        self.phase_start_ms = 0
        self.in_tolerance_ms = 0
        self.task_completed = False

    def cycle_mode(self, now):
        if self.run_state == RUN_RUNNING:
            self.set_notice("PAUSE BEFORE MODE CHANGE", now)
            return
        self.reset_operation()
        self.mode += 1
        if self.mode > MODE_MAX:
            self.mode = MODE_MIN
        self.tube_step = 0
        self.q3_step = 0
        self.pending_tube_pixels = []
        self.pending_q3_pixels = []
        self.set_notice(MODE_NAMES[self.mode], now)

    def return_to_detect(self, now):
        self.reset_operation()
        self.mode = MODE_DETECT
        self.tube_step = 0
        self.q3_step = 0
        self.pending_tube_pixels = []
        self.pending_q3_pixels = []
        self.set_notice("CANCEL -> MODE 1", now)

    def toggle_pause(self, now):
        if self.run_state == RUN_RUNNING:
            self.run_state = RUN_PAUSED
            self.set_notice("PAUSED", now)
        elif self.run_state == RUN_PAUSED:
            self.run_state = RUN_RUNNING
            self.set_notice("RESUMED", now)
        else:
            self.set_notice("NOT RUNNING", now)

    def record_tube_point(self, center_x, now):
        if center_x is None:
            self.set_notice("BALL NOT FOUND", now)
            return
        if self.tube_step == 0:
            self.pending_tube_pixels = []

        self.pending_tube_pixels.append(int(center_x))
        self.tube_step += 1
        if self.tube_step >= len(TUBE_MARKS):
            previous_tube = self.calibration.tube_pixels
            previous_q3 = self.calibration.q3_pixels
            previous_any_pixel = self.calibration.arbitrary_pixel
            previous_any_mm = self.calibration.arbitrary_mm
            self.calibration.tube_pixels = list(self.pending_tube_pixels)
            self.calibration.q3_pixels = []
            self.calibration.arbitrary_pixel = None
            self.calibration.arbitrary_mm = None
            if not self.calibration.tube_valid():
                self.calibration.tube_pixels = previous_tube
                self.calibration.q3_pixels = previous_q3
                self.calibration.arbitrary_pixel = previous_any_pixel
                self.calibration.arbitrary_mm = previous_any_mm
                self.pending_tube_pixels = []
                self.tube_step = 0
                self.set_notice("TUBE CAL INVALID", now)
                return
            self.calibration.save()
            self.pending_tube_pixels = []
            self.tube_step = 0
            self.set_notice("TUBE CAL SAVED", now)
        else:
            self.set_notice(
                "SAVED, NEXT " + TUBE_MARKS[self.tube_step][0], now
            )

    def record_q3_point(self, center_x, now):
        if not self.calibration.tube_valid():
            self.set_notice("DO MODE 2 FIRST", now)
            return
        if center_x is None:
            self.set_notice("BALL NOT FOUND", now)
            return
        if self.q3_step == 0:
            self.pending_q3_pixels = []

        self.pending_q3_pixels.append(int(center_x))
        self.q3_step += 1
        if self.q3_step >= len(Q3_MARKS):
            previous_q3 = self.calibration.q3_pixels
            self.calibration.q3_pixels = list(self.pending_q3_pixels)
            if not self.calibration.q3_valid():
                self.calibration.q3_pixels = previous_q3
                self.pending_q3_pixels = []
                self.q3_step = 0
                self.set_notice("Q3 CAL INVALID", now)
                return
            if not self.calibration.active_points():
                self.calibration.q3_pixels = previous_q3
                self.pending_q3_pixels = []
                self.q3_step = 0
                self.set_notice("Q3 ORDER INVALID", now)
                return
            self.calibration.save()
            self.pending_q3_pixels = []
            self.q3_step = 0
            self.set_notice("Q3 CAL SAVED", now)
        else:
            self.set_notice(
                "SAVED, NEXT " + Q3_MARKS[self.q3_step][0], now
            )

    def record_arbitrary(self, center_x, position_mm, now):
        if not self.calibration.tube_valid():
            self.set_notice("DO MODE 2 FIRST", now)
            return
        if center_x is None or position_mm is None:
            self.set_notice("BALL NOT FOUND", now)
            return
        self.calibration.arbitrary_pixel = int(center_x)
        self.calibration.arbitrary_mm = int(position_mm)
        self.calibration.save()
        self.set_notice("ANY POINT SAVED %dMM" % position_mm, now)

    def press_start(self, center_x, position_mm, now):
        self.task_completed = False
        if self.mode == MODE_DETECT:
            if center_x is None:
                self.set_notice("BALL NOT FOUND", now)
                return
            if not self.calibration.tube_valid():
                self.set_notice("DO MODE 2 FIRST", now)
                return
            if not self.calibration.recenter(center_x):
                self.set_notice("CENTER CAL INVALID", now)
                return
            self.mode = MODE_HOLD_CENTER
            self.run_state = RUN_RUNNING
            self.target_mm = 0
            self.set_notice("CENTER SAVED -> HOLD", now)
            return
        if self.mode == MODE_TUBE_CAL:
            self.record_tube_point(center_x, now)
            return
        if self.mode == MODE_Q3_CAL:
            self.record_q3_point(center_x, now)
            return
        if self.mode == MODE_ARBITRARY_CAL:
            self.record_arbitrary(center_x, position_mm, now)
            return

        if self.mode == MODE_HOLD_ARBITRARY:
            if self.run_state == RUN_PAUSED:
                self.run_state = RUN_RUNNING
                self.set_notice("RESUMED", now)
                return
            if self.run_state == RUN_RUNNING:
                self.set_notice("ALREADY RUNNING", now)
                return
            if not self.calibration.tube_valid():
                self.set_notice("DO MODE 2 FIRST", now)
                return
            if center_x is None or position_mm is None:
                self.set_notice("BALL NOT FOUND", now)
                return
            self.calibration.arbitrary_pixel = int(center_x)
            self.calibration.arbitrary_mm = int(position_mm)
            self.calibration.save()
            self.target_mm = int(position_mm)
            self.run_state = RUN_RUNNING
            self.set_notice(
                "ANY %dMM SAVED -> HOLD" % self.target_mm, now
            )
            return

        if self.run_state == RUN_PAUSED:
            self.run_state = RUN_RUNNING
            self.set_notice("RESUMED", now)
            return
        if self.run_state == RUN_RUNNING:
            self.set_notice("ALREADY RUNNING", now)
            return

        if self.mode in (
            MODE_HOLD_CENTER,
            MODE_Q3_SEQUENCE,
            MODE_HOLD_ARBITRARY,
        ) and not self.calibration.tube_valid():
            self.set_notice("DO MODE 2 FIRST", now)
            return

        if self.mode == MODE_Q3_SEQUENCE:
            self.sequence_phase = SEQ_WAIT_CENTER
            self.target_mm = 0
        else:
            self.target_mm = 0

        self.run_state = RUN_RUNNING
        self.set_notice("STARTED", now)

    def update_sequence(self, position_mm, now):
        if (
            self.mode != MODE_Q3_SEQUENCE
            or self.run_state != RUN_RUNNING
            or position_mm is None
        ):
            return

        if self.sequence_phase == SEQ_WAIT_CENTER:
            self.target_mm = 0
            if abs(position_mm) <= START_CENTER_TOLERANCE_MM:
                self.sequence_phase = SEQ_TO_POSITIVE
                self.target_mm = 50
                self.phase_start_ms = now
                self.in_tolerance_ms = 0
                self.set_notice("TARGET +50MM", now)
            return

        error = self.target_mm - position_mm
        if abs(error) <= TARGET_TOLERANCE_MM:
            if self.in_tolerance_ms == 0:
                self.in_tolerance_ms = now
        else:
            self.in_tolerance_ms = 0

        if self.sequence_phase == SEQ_TO_POSITIVE:
            reached = (
                self.in_tolerance_ms != 0
                and ticks_diff(now, self.in_tolerance_ms)
                >= POSITIVE_HOLD_MS
            )
            timed_out = (
                ticks_diff(now, self.phase_start_ms) >= POSITIVE_TIMEOUT_MS
            )
            if reached or timed_out:
                self.sequence_phase = SEQ_TO_NEGATIVE
                self.target_mm = -50
                self.phase_start_ms = now
                self.in_tolerance_ms = 0
                self.set_notice("TARGET -50MM", now)
        elif self.sequence_phase == SEQ_TO_NEGATIVE:
            reached = (
                self.in_tolerance_ms != 0
                and ticks_diff(now, self.in_tolerance_ms)
                >= NEGATIVE_HOLD_MS
            )
            if reached:
                self.sequence_phase = SEQ_HOLD_NEGATIVE
                self.target_mm = -50
                self.set_notice("HOLD -50MM", now)

    def prompt(self):
        if self.task_completed:
            return "TASK COMPLETED"
        if self.mode == MODE_DETECT:
            return "BALL AT O; KEY1 SAVE CENTER"
        if self.mode == MODE_TUBE_CAL:
            return "KEY1 MARK " + TUBE_MARKS[self.tube_step][0]
        if self.mode == MODE_Q3_CAL:
            return "KEY1 MARK " + Q3_MARKS[self.q3_step][0]
        if self.mode == MODE_ARBITRARY_CAL:
            return "KEY1 SAVE CURRENT POINT"
        if self.mode == MODE_HOLD_CENTER:
            return "TARGET 0MM"
        if self.mode == MODE_Q3_SEQUENCE:
            if self.mcu_online:
                return "MCU PHASE %d TARGET %dMM" % (
                    self.mcu_status.phase,
                    self.mcu_status.target_mm,
                )
            if self.run_state == RUN_RUNNING:
                return "WAITING FOR MCU"
            if self.sequence_phase == SEQ_WAIT_CENTER:
                return "PLACE BALL AT O"
            return "TARGET %dMM" % self.target_mm
        if self.mode == MODE_HOLD_ARBITRARY:
            if self.run_state == RUN_IDLE:
                return "PLACE BALL; KEY1 SET TARGET"
            return "TARGET %dMM" % self.target_mm
        return "POSITION OUTPUT"


def draw_status(osd, display_size, state, center_x, center_y, position_mm,
                confidence, inference_ms, fps, prediction_pixels, now):
    white = (255, 255, 255)
    yellow = (255, 255, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)

    osd.draw_string_advanced(
        8, 6, 24,
        "%s  %s" % (MODE_NAMES[state.mode], RUN_NAMES[state.run_state]),
        color=yellow,
    )
    osd.draw_string_advanced(8, 34, 22, state.prompt(), color=white)

    if center_x is None:
        position_text = "BALL NOT FOUND"
        position_color = red
    elif position_mm is None:
        position_text = "PIXEL (%d,%d)  UNCALIBRATED" % (center_x, center_y)
        position_color = red
    else:
        position_text = "BALL %dMM  PIXEL %d  CONF %d%%" % (
            position_mm,
            center_x,
            confidence,
        )
        position_color = green

    osd.draw_string_advanced(
        8, 62, 22, position_text, color=position_color
    )

    notice = state.visible_notice(now)
    if notice:
        osd.draw_string_advanced(
            8,
            min(display_size[1] - 34, 92),
            22,
            notice,
            color=yellow,
        )

    if state.mcu_online:
        status = state.mcu_status
        osd.draw_string_advanced(
            8,
            min(display_size[1] - 58, 120),
            18,
            "MCU A%+.2fDEG V%+dMM/S P%d"
            % (
                status.angle_cdeg / 100.0,
                status.velocity_mm_s,
                status.phase,
            ),
            color=green if not (status.flags & 0x08) else red,
        )
    elif state.mcu_status is None or state.mcu_status.last_rx_ms == 0:
        osd.draw_string_advanced(
            8,
            min(display_size[1] - 58, 120),
            18,
            "MCU NO STATUS RX",
            color=red,
        )
    else:
        status = state.mcu_status
        age_ms = ticks_diff(now, status.last_rx_ms)
        osd.draw_string_advanced(
            8,
            min(display_size[1] - 58, 120),
            18,
            "MCU LOST %dMS LAST A%+.2f P%d"
            % (
                age_ms,
                status.angle_cdeg / 100.0,
                status.phase,
            ),
            color=red,
        )

    osd.draw_string_advanced(
        8,
        display_size[1] - 26,
        18,
        "AI %dMS  FPS %.1f  PRED %+dPX"
        % (inference_ms, fps, prediction_pixels),
        color=white,
    )


def main():
    try:
        os.stat(MODEL_PATH)
    except OSError:
        raise RuntimeError("Model not found: " + MODEL_PATH)

    uart = init_uart()
    key_start, key_pause, key_mode = init_buttons()
    pipeline = None
    detector = None
    calibration = CalibrationStore()
    state = H3StateMachine(calibration)
    mcu_parser = MCUStatusParser()
    predictor = MotionPredictor()
    recent_centers_x = []
    recent_centers_y = []
    frame_counter = 0
    last_control_send_ms = 0
    last_print_ms = 0
    last_frame_done_ms = 0
    fps = 0.0

    try:
        pipeline = PipeLine(
            rgb888p_size=FRAME_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create(sensor_id=2, to_ide=SEND_TO_IDE)
        display_size = pipeline.get_display_size()
        calibration.set_display_size(display_size)
        loaded = calibration.load()
        if AUTO_HOLD_CENTER_ON_BOOT and loaded:
            # 第四问上电即需要连续发送球位置。已有三点标定时自动进入回中
            # 运行态，避免用户漏按 KEY3/KEY1 后天猛星只能做开环补偿。
            state.mode = MODE_HOLD_CENTER
            state.run_state = RUN_RUNNING
            state.target_mm = 0
            state.set_notice("AUTO HOLD CENTER", ticks_ms())

        detector = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=MODEL_PATH,
            labels=LABELS,
            rgb888p_size=FRAME_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        detector.config_preprocess()

        print("H3 K230 state machine started")
        print("Display:", display_size)
        print("Calibration loaded:", loaded)
        print("KEY1=start/mark KEY2=pause KEY3=mode")

        while True:
            frame_counter += 1
            image_array = pipeline.get_frame()
            inference_start_ms = ticks_ms()
            result = detector.run(image_array)
            now = ticks_ms()
            inference_ms = ticks_diff(now, inference_start_ms)

            received = uart.read()
            if received:
                mcu_parser.feed(received, now)
            state.update_mcu_status(mcu_parser.status, now)

            # OSD is a persistent overlay buffer. It must be cleared on every
            # frame, including frames where no ball is detected; otherwise old
            # boxes, crosses and text accumulate and eventually corrupt display.
            pipeline.osd_img.clear()

            if last_frame_done_ms != 0:
                frame_ms = ticks_diff(now, last_frame_done_ms)
                if frame_ms > 0:
                    instant_fps = 1000.0 / frame_ms
                    fps = (
                        instant_fps
                        if fps == 0.0
                        else 0.8 * fps + 0.2 * instant_fps
                    )
            last_frame_done_ms = now

            center_x = None
            center_y = None
            control_x = None
            prediction_x = 0
            confidence = 0

            if result and len(result[0]) > 0:
                boxes = result[0]
                scores = result[2]
                best = 0
                for index in range(1, len(scores)):
                    if scores[index] > scores[best]:
                        best = index

                x, y, width, height = boxes[best]
                x = int(round(x))
                y = int(round(y))
                width = int(round(width))
                height = int(round(height))
                detected_x = x + width // 2
                detected_y = y + height // 2
                confidence = int(round(float(scores[best]) * 100))

                recent_centers_x.append(detected_x)
                recent_centers_y.append(detected_y)
                if len(recent_centers_x) > 3:
                    recent_centers_x.pop(0)
                    recent_centers_y.pop(0)

                # 三帧中值先去除检测框偶发跳点，再做延时预测；发送给 MCU
                # 的位置因此更稳，回中控制不会被单帧像素噪声反复推拉。
                stable_x = median(recent_centers_x)
                stable_y = median(recent_centers_y)
                # MCU 使用未外推的三帧中值位置统一计算速度和预测；这里的
                # center_x 预测值只用于屏幕显示，避免双重超前补偿。
                control_x = stable_x
                center_x, center_y, prediction_x, prediction_y = (
                    predictor.update(
                        stable_x,
                        stable_y,
                        now,
                        inference_ms,
                    )
                )
                center_x = clamp(center_x, 0, display_size[0] - 1)
                center_y = clamp(center_y, 0, display_size[1] - 1)

                predicted_box_x = clamp(
                    x + prediction_x,
                    0,
                    max(0, display_size[0] - width - 1),
                )
                predicted_box_y = clamp(
                    y + prediction_y,
                    0,
                    max(0, display_size[1] - height - 1),
                )
                pipeline.osd_img.draw_rectangle(
                    predicted_box_x,
                    predicted_box_y,
                    width,
                    height,
                    color=(255, 255, 0),
                    thickness=2,
                )
                pipeline.osd_img.draw_cross(
                    center_x,
                    center_y,
                    color=(255, 0, 0),
                    size=12,
                    thickness=3,
                )
            else:
                recent_centers_x = []
                recent_centers_y = []
                predictor.miss()

            position_mm = (
                calibration.pixel_to_mm(center_x)
                if center_x is not None
                else None
            )
            control_position_mm = (
                calibration.pixel_to_mm(control_x)
                if control_x is not None
                else None
            )

            start_event = key_start.update(now)
            pause_event = key_pause.update(now)
            mode_event = key_mode.update(now)

            if start_event == Button.EVENT_SHORT:
                marking_mode = state.mode in (
                    MODE_DETECT,
                    MODE_TUBE_CAL,
                    MODE_Q3_CAL,
                    MODE_ARBITRARY_CAL,
                ) or (
                    state.mode == MODE_HOLD_ARBITRARY
                    and state.run_state == RUN_IDLE
                )
                center_stable = (
                    len(recent_centers_x) >= 3
                    and max(recent_centers_x) - min(recent_centers_x) <= 4
                )
                if marking_mode and not center_stable:
                    state.set_notice("HOLD BALL STEADY", now)
                else:
                    if marking_mode:
                        mark_center_x = median(recent_centers_x)
                        mark_position_mm = calibration.pixel_to_mm(
                            mark_center_x
                        )
                        state.press_start(
                            mark_center_x,
                            mark_position_mm,
                            now,
                        )
                    else:
                        state.press_start(center_x, position_mm, now)
            if pause_event == Button.EVENT_SHORT:
                state.toggle_pause(now)
            if mode_event == Button.EVENT_SHORT:
                state.cycle_mode(now)
            elif mode_event == Button.EVENT_LONG:
                state.return_to_detect(now)

            if (
                control_x is not None
                and control_position_mm is not None
                and state.run_state != RUN_IDLE
            ):
                uart.write(
                    make_position_frame(
                        control_position_mm,
                        confidence,
                        frame_counter,
                        now,
                        inference_ms,
                    )
                )

            if (
                ticks_diff(now, last_control_send_ms)
                >= CONTROL_SEND_INTERVAL_MS
            ):
                uart.write(
                    make_control_frame(
                        state.mode,
                        state.run_state,
                        state.target_mm,
                        state.sequence_phase,
                    )
                )
                last_control_send_ms = now

            draw_status(
                pipeline.osd_img,
                display_size,
                state,
                center_x,
                center_y,
                position_mm,
                confidence,
                inference_ms,
                fps,
                prediction_x,
                now,
            )

            if ticks_diff(now, last_print_ms) >= PRINT_INTERVAL_MS:
                if center_x is None:
                    print(
                        "mode=%d state=%d BALL not found"
                        % (state.mode, state.run_state)
                    )
                else:
                    print(
                        "mode=%d state=%d pixel=(%d,%d) mm=%s target=%d"
                        % (
                            state.mode,
                            state.run_state,
                            center_x,
                            center_y,
                            str(position_mm),
                            state.target_mm,
                        )
                    )
                if state.mcu_online:
                    status = state.mcu_status
                    print(
                        "mcu phase=%d target=%d angle=%.2fdeg velocity=%d flags=0x%02X"
                        % (
                            status.phase,
                            status.target_mm,
                            status.angle_cdeg / 100.0,
                            status.velocity_mm_s,
                            status.flags,
                        )
                    )
                last_print_ms = now

            pipeline.show_image()
            if frame_counter % GC_INTERVAL_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("User stopped")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        uart.deinit()
        gc.collect()


main()
