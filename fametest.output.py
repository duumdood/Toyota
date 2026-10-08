"""fametest.output.py — FAKE BOARD: ทดสอบ simulator โดยไม่ต้องต่อบอร์ด / COM port

ไฟล์นี้จำลอง "บอร์ด STM32" ทั้งตัวใน Python แล้วป้อนเฟรม RD1 เข้า parser จริง
(road_board_io.LineDecoder / BoardView) และหน้าจอ 3D จริง (road_simulator_board)
เพื่อให้เห็น OUTPUT ทุกตัวอย่างละเอียด โดยคุณเป็นคนใส่ INPUT เองทั้งหมด

  * Logic ของบอร์ด (IDM, ปุ่ม, LDR, ระยะ, done/reset, accel_blocked) และ
    encoder RD1 + CRC32 แปลงมาจาก misra_fixed/road_app.c + road_protocol.c
    บรรทัดต่อบรรทัด และคำนวณแบบ float32 เหมือน Cortex-M4
  * ส่วน driver (debounce, ADC, TIM3 echo, LED PWM, 7-segment) จำลองระดับผลลัพธ์
  * ไม่เปิด serial port ใด ๆ ทั้งสิ้น

วางไฟล์นี้ไว้โฟลเดอร์เดียวกับ road_board_io.py และ road_simulator_board.py

วิธีรัน
  python fametest.output.py              หน้าจอ 3D + แผง OUTPUT ด้านซ้าย (กดคีย์บอร์ดแทนปุ่มบอร์ด)
  python fametest.output.py --console    โหมดพิมพ์คำสั่ง แสดง output ละเอียดใน terminal
  python fametest.output.py --demo       รันสถานการณ์ตัวอย่างอัตโนมัติ แล้วพิมพ์ตารางผลลัพธ์

โหมด 3D มี "FAKE INPUT PANEL" (มุมขวาล่าง กรอบส้ม) ให้คลิกแทนบอร์ดจริง:
  ปุ่ม SPEED- / SPEED+ / RESET (กดเมาส์ค้าง = กดปุ่มค้าง), slider LDR (ADC 0..4095),
  slider US-100 (1..120 cm), ปุ่ม NO ECHO / FAULT / USB
  ลากแถบส้มด้านบนเพื่อย้ายแผง, Tab ซ่อน/แสดงแผง output, H ซ่อนแผง fake ทั้งหมด (ดู UI จริงล้วน ๆ)
  แผงนี้เป็นของปลอมโดยเจตนา มีไว้ปรับความสวยของ UI และตรวจความถูกต้องของการแสดงผลเท่านั้น

คีย์ลัด (ใช้ร่วมกับ panel ได้)
  W / ↑   กดค้าง = SPEED+ (PA10)        S / ↓   กดค้าง = SPEED- (PB3)
  R       กดค้าง = RESET (PB4)
  A / ←   วัตถุเข้าใกล้ (กดค้าง)          D / →   วัตถุถอยออก (กดค้าง)
  1..9    ตั้งระยะ 10..90 cm               0       ไม่มีวัตถุ (no echo)
  F       สลับ sensor fault (status 2)
  N       สลับปิด/เปิด LDR (มืด/สว่าง)      [ / ]   ปรับค่า ADC ทีละ -100 / +100
  U       ถอด/เสียบ USB (หยุดส่ง telemetry)
"""
import argparse
import struct
import sys
import time
import zlib
from dataclasses import dataclass

from road_board_io import BoardView, LineDecoder, visual_gap


# =====================================================================
# float32 helper: Cortex-M4 FPU is single precision. Each + - * / done in
# double then rounded to float32 gives exactly the float32 result.
# =====================================================================
def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


U32 = 0xFFFFFFFF

# ------------------------------------------------------- road_config.h
TICK_MS = 10
TELEMETRY_MS = 50
MS_TO_S = f32(0.001)
SENSOR_STALE_MS = 250
TAP_MPS = f32(0.5)
HOLD_DELAY_MS = 350
HOLD_MPS_PER_SEC = f32(4.0)
ADC_MAX = 4095
LDR_DARK_IS_HIGH = True
NIGHT_ENTER_ADC = 3000
NIGHT_EXIT_ADC = 3000
NIGHT_ENTER_MS = 400
NIGHT_EXIT_MS = 200
METRES_PER_SENSOR_MM = f32(0.1)
MIN_GAP_M = f32(2.0)
MAX_GAP_M = f32(70.0)
ECHO_PERIOD_MS = 60
ECHO_MIN_US = 100
ECHO_MAX_US = 26000
RANGE_VALID, RANGE_NO_ECHO, RANGE_FAULT = 0, 1, 2
IDM_A_MAX = f32(3.0)
IDM_DECEL_LIMIT = f32(-7.0)
IDM_STOP_DECEL = f32(-3.0)
IDM_TWO_SQRT_AB = f32(6.480740698)
IDM_T_DAY_S, IDM_T_NIGHT_S = f32(1.0), f32(1.8)
IDM_S0_DAY_M, IDM_S0_NIGHT_M = f32(4.0), f32(7.0)
IDM_DYNAMIC_MAX_M = f32(100000.0)
KMH_PER_MPS = f32(3.6)
INITIAL_TARGET_KMH = f32(50.0)
VMAX_DAY_KMH, VMAX_NIGHT_KMH = f32(120.0), f32(100.0)
SPEED_LIMIT_MPS = f32(40.0)
ODOMETER_WRAP_M = f32(4000000.0)
BRAKING_ACCEL = f32(-0.35)
RATE_MAX_DT_S = f32(0.25)
RATE_MAX_JUMP_MPS = f32(80.0)
RATE_FILTER_TAU_S = f32(0.14)
LEAD_VALID_AGE_S = f32(0.12)
SPEED_TOL_MPS = f32(0.05)
ACCEL_TOL_MPS2 = f32(0.05)
STOP_SPEED_MPS = f32(0.3)
STABLE_RATE_MPS = f32(0.5)
STABLE_ACCEL_MPS2 = f32(0.4)
SETTLE_TIME_S = f32(1.2)
DONE_RUNNING, DONE_STOPPED, DONE_SETTLED = 0, 1, 2
RED_NEAR_DAY_MM, RED_NEAR_NIGHT_MM = f32(80.0), f32(140.0)
RED_FAR_MM = f32(600.0)
RED_PWM_PERIOD_US = 1000
RED_FADE_MS = 500
PERMILLE_SCALE = f32(1000.0)
ROUND_HALF = f32(0.5)
TRAPEZOID_HALF = f32(0.5)
PROTO_SCALE = f32(1000.0)
DONE_TEXT = {0: 'running', 1: 'STOP COMPLETE', 2: 'DECELERATION COMPLETE'}
STATUS_TEXT = {0: 'VALID ECHO', 1: 'NO RETURN', 2: 'SENSOR FAULT'}


# =====================================================================
# road_app.h — RoadInputs / RoadOutput / RoadApp
# =====================================================================
@dataclass
class RoadInputs:
    speed_up_pressed: bool = False
    speed_down_pressed: bool = False
    reset_pressed: bool = False
    ldr_adc: int = 0
    ldr_ms: int = 0
    ldr_ready: bool = False
    distance_mm: int = 0
    range_ms: int = 0
    range_sequence: int = 0
    range_status: int = 0
    range_ready: bool = False

    def copy(self):
        return RoadInputs(**self.__dict__)


@dataclass
class RoadOutput:
    speed_mps: float = 0.0
    target_mps: float = 0.0
    acceleration_mps2: float = 0.0
    odometer_m: float = 0.0
    gap_m: float = 0.0
    lead_speed_mps: float = 0.0
    desired_gap_m: float = 0.0
    red_near_mm: float = 0.0
    red_far_mm: float = 0.0
    red_target_permille: int = 0
    night: bool = False
    present: bool = False
    lead_valid: bool = False
    braking: bool = False
    done: bool = False
    fault: bool = False
    accel_blocked: bool = False
    done_reason: int = 0
    reset_id: int = 0


def limit(x, low, high):
    if x < low:
        return low
    if x > high:
        return high
    return x


def magnitude(x):
    return -x if x < 0.0 else x


def speed_cap_mps(night):
    return f32((VMAX_NIGHT_KMH if night else VMAX_DAY_KMH) / KMH_PER_MPS)


def road_desired_gap(speed, closing, night):
    headway = IDM_T_NIGHT_S if night else IDM_T_DAY_S
    jam_gap = IDM_S0_NIGHT_M if night else IDM_S0_DAY_M
    dynamic = f32(f32(speed * headway) + f32(f32(speed * closing) / IDM_TWO_SQRT_AB))
    return f32(jam_gap + limit(dynamic, 0.0, IDM_DYNAMIC_MAX_M))


def idm_with_desired(speed, target, gap, present, desired):
    if target <= 0.0:
        return IDM_STOP_DECEL if speed > 0.0 else 0.0
    interaction = 0.0
    if present:
        gap_ratio = f32(desired / limit(gap, MIN_GAP_M, MAX_GAP_M))
        interaction = f32(gap_ratio * gap_ratio)
    speed_ratio = f32(speed / target)
    square = f32(speed_ratio * speed_ratio)
    return limit(f32(IDM_A_MAX * f32(f32(1.0 - f32(square * square)) - interaction)),
                 IDM_DECEL_LIMIT, IDM_A_MAX)


class RoadApp:
    """Python mirror of road_app.c (same field names as the C struct)."""

    def __init__(self):
        self.out = RoadOutput()
        self.previous_up = self.previous_down = self.previous_reset = False
        self.did_brake = False
        self.hold_up_ms = self.hold_down_ms = self.light_ms = 0
        self.seen_range = self.previous_range_ms = 0
        self.have_previous_range = False
        self.previous_gap = self.range_rate = self.estimate_age = 0.0
        self.settled_s = self.accel_request_mps = 0.0
        self.out.target_mps = f32(INITIAL_TARGET_KMH / KMH_PER_MPS)
        self.out.gap_m = MAX_GAP_M
        self.out.fault = True

    # ------------------------------------------------------------ pieces
    def update_red_target(self, inp, valid):
        o = self.out
        o.red_near_mm = RED_NEAR_NIGHT_MM if o.night else RED_NEAR_DAY_MM
        o.red_far_mm = RED_FAR_MM
        o.red_target_permille = 0
        if valid and inp.range_status == RANGE_VALID:
            level = f32(f32(o.red_far_mm - f32(float(inp.distance_mm)))
                        / f32(o.red_far_mm - o.red_near_mm))
            o.red_target_permille = int(f32(f32(limit(level, 0.0, 1.0) * PERMILLE_SCALE) + ROUND_HALF))

    def reset_sequence(self):
        o = self.out
        o.speed_mps = o.acceleration_mps2 = o.odometer_m = 0.0
        o.done = o.braking = False
        self.did_brake = False
        o.done_reason = DONE_RUNNING
        o.reset_id = (o.reset_id + 1) & U32
        self.settled_s = 0.0
        self.hold_up_ms = self.hold_down_ms = 0

    def update_light(self, inp):
        dark = inp.ldr_adc if LDR_DARK_IS_HIGH else ADC_MAX - inp.ldr_adc
        if self.out.night:
            crossing, dwell = dark < NIGHT_EXIT_ADC, NIGHT_EXIT_MS
        else:
            crossing, dwell = dark > NIGHT_ENTER_ADC, NIGHT_ENTER_MS
        self.light_ms = (self.light_ms + TICK_MS) & U32 if crossing else 0
        if self.light_ms >= dwell:
            self.out.night = not self.out.night
            self.light_ms = 0

    def range_lost(self):
        self.out.present = self.out.lead_valid = False
        self.have_previous_range = False
        self.range_rate = self.estimate_age = self.settled_s = 0.0
        self.did_brake = False

    def range_accept(self, inp):
        gap = limit(f32(f32(float(inp.distance_mm)) * METRES_PER_SENSOR_MM), MIN_GAP_M, MAX_GAP_M)
        dt = f32(f32(float((inp.range_ms - self.previous_range_ms) & U32)) * MS_TO_S)
        rate = f32(f32(gap - self.previous_gap) / dt) if dt > 0.0 else 0.0
        if (not self.have_previous_range) or dt <= 0.0 or dt > RATE_MAX_DT_S \
                or magnitude(rate) > RATE_MAX_JUMP_MPS:
            self.out.lead_valid = False
            self.range_rate = self.estimate_age = self.settled_s = 0.0
        else:
            self.estimate_age = f32(self.estimate_age + dt)
            self.range_rate = f32(self.range_rate + f32(f32(f32(rate - self.range_rate) * dt)
                                                        / f32(RATE_FILTER_TAU_S + dt)))
            self.out.lead_valid = self.estimate_age >= LEAD_VALID_AGE_S
        self.out.present = True
        self.have_previous_range = True
        self.out.gap_m = gap
        self.previous_gap = gap
        self.previous_range_ms = inp.range_ms

    def update_range(self, inp):
        if inp.range_ready and inp.range_sequence != self.seen_range:
            self.seen_range = inp.range_sequence
            if inp.range_status != RANGE_VALID:
                self.range_lost()
            else:
                self.range_accept(inp)

    @staticmethod
    def button_delta(pressed, previous, held):
        if not pressed:
            return 0.0, 0
        if not previous:
            return TAP_MPS, 0
        if held < HOLD_DELAY_MS:
            held += TICK_MS
        delta = f32(HOLD_MPS_PER_SEC * f32(f32(float(TICK_MS)) * MS_TO_S)) if held >= HOLD_DELAY_MS else 0.0
        return delta, held

    def handle_speed_buttons(self, inp):
        if inp.speed_up_pressed and inp.speed_down_pressed:
            self.hold_up_ms = self.hold_down_ms = 0
        else:
            d, self.hold_up_ms = self.button_delta(inp.speed_up_pressed, self.previous_up, self.hold_up_ms)
            self.out.target_mps = f32(self.out.target_mps + d)
            d, self.hold_down_ms = self.button_delta(inp.speed_down_pressed, self.previous_down, self.hold_down_ms)
            self.out.target_mps = f32(self.out.target_mps - d)

    def freeze_outputs(self):
        o = self.out
        o.accel_blocked = False
        self.accel_request_mps = 0.0
        o.acceleration_mps2 = 0.0
        o.braking = False
        if o.fault:
            self.have_previous_range = False
            o.lead_valid = False
            self.hold_up_ms = self.hold_down_ms = 0

    def update_accel_warning(self, inp, old):
        o = self.out
        requested = o.target_mps
        if o.accel_blocked and self.accel_request_mps > requested:
            requested = self.accel_request_mps
        requested = limit(requested, 0.0, speed_cap_mps(o.night))
        requested_accel = idm_with_desired(old, requested, o.gap_m, o.present, o.desired_gap_m)
        requested_free = idm_with_desired(old, requested, o.gap_m, False, o.desired_gap_m)
        blocks = (o.present and requested > f32(old + SPEED_TOL_MPS)
                  and requested_accel <= ACCEL_TOL_MPS2 and requested_free > ACCEL_TOL_MPS2)
        if not blocks:
            o.accel_blocked = False
            self.accel_request_mps = 0.0
        elif inp.speed_up_pressed and not inp.speed_down_pressed:
            o.accel_blocked = True
            self.accel_request_mps = requested

    def integrate_motion(self, old, dt):
        o = self.out
        o.speed_mps = limit(f32(old + f32(o.acceleration_mps2 * dt)), 0.0, SPEED_LIMIT_MPS)
        o.odometer_m = f32(o.odometer_m + f32(f32(f32(old + o.speed_mps) * TRAPEZOID_HALF) * dt))
        if o.odometer_m >= ODOMETER_WRAP_M:
            o.odometer_m = 0.0
        o.lead_speed_mps = f32(o.speed_mps + self.range_rate)
        o.braking = o.acceleration_mps2 < BRAKING_ACCEL

    def follow_cruise(self, free_accel):
        o = self.out
        if o.present and o.braking and free_accel > f32(o.acceleration_mps2 + ACCEL_TOL_MPS2) \
                and o.target_mps > o.speed_mps:
            o.target_mps = o.speed_mps

    def check_sequence_end(self, dt):
        o = self.out
        if o.present and o.braking:
            self.did_brake = True
        stable = (o.present and self.did_brake and o.lead_valid
                  and magnitude(self.range_rate) < STABLE_RATE_MPS
                  and magnitude(o.acceleration_mps2) < STABLE_ACCEL_MPS2)
        self.settled_s = f32(self.settled_s + dt) if stable else 0.0
        if o.present and self.did_brake and o.speed_mps < STOP_SPEED_MPS:
            o.speed_mps = 0.0
            o.target_mps = 0.0
            o.done = True
            o.done_reason = DONE_STOPPED
        elif self.settled_s > SETTLE_TIME_S:
            o.done = True
            o.done_reason = DONE_SETTLED
        if o.done:
            o.accel_blocked = False

    def run_control_step(self, inp, old, dt):
        o = self.out
        o.acceleration_mps2 = idm_with_desired(old, o.target_mps, o.gap_m, o.present, o.desired_gap_m)
        free_accel = idm_with_desired(old, o.target_mps, o.gap_m, False, o.desired_gap_m)
        self.update_accel_warning(inp, old)
        self.integrate_motion(old, dt)
        self.follow_cruise(free_accel)
        self.check_sequence_end(dt)

    # -------------------------------------------------------------- tick
    def tick(self, inp, now_ms):
        o = self.out
        dt = f32(f32(float(TICK_MS)) * MS_TO_S)
        reset = inp.reset_pressed and not self.previous_reset
        adc_ok = (inp.ldr_ready and inp.ldr_adc <= ADC_MAX
                  and ((now_ms - inp.ldr_ms) & U32) <= SENSOR_STALE_MS)
        range_ok = (inp.range_ready and inp.range_status <= RANGE_NO_ECHO
                    and ((now_ms - inp.range_ms) & U32) <= SENSOR_STALE_MS)
        o.fault = (not adc_ok) or (not range_ok)
        if adc_ok:
            self.update_light(inp)
        else:
            self.light_ms = 0
        self.update_range(inp)
        if reset:
            self.reset_sequence()
        if (not o.done) and (not o.fault) and (not reset):
            self.handle_speed_buttons(inp)
        self.previous_up = inp.speed_up_pressed
        self.previous_down = inp.speed_down_pressed
        self.previous_reset = inp.reset_pressed
        o.target_mps = limit(o.target_mps, 0.0, speed_cap_mps(o.night))
        old = o.speed_mps
        closing = -self.range_rate if o.lead_valid else old
        o.desired_gap_m = road_desired_gap(old, closing, o.night)
        self.update_red_target(inp, adc_ok and range_ok)
        if o.fault or o.done or reset:
            self.freeze_outputs()
        else:
            self.run_control_step(inp, old, dt)


# =====================================================================
# road_protocol.c — RD1 frame + CRC32
# =====================================================================
def _scaled(value):
    return int(f32(value * PROTO_SCALE))  # C float->int32 cast truncates toward zero


def encode_frame(o, inp, seq, ms):
    flags = ((1 if o.night else 0) | (2 if o.present else 0) | (4 if o.lead_valid else 0)
             | (8 if o.braking else 0) | (16 if o.done else 0) | (32 if o.fault else 0)
             | (64 if o.accel_blocked else 0))
    fields = [
        seq & U32, ms & U32,
        _scaled(o.speed_mps), _scaled(o.target_mps), _scaled(o.acceleration_mps2),
        int(f32(o.odometer_m * PROTO_SCALE)) & U32,
        _scaled(o.gap_m) if o.present else -1,
        _scaled(o.lead_speed_mps) if o.lead_valid else 0,
        flags, inp.ldr_adc,
        inp.distance_mm if (inp.range_ready and inp.range_status == RANGE_VALID) else -1,
        o.reset_id, o.done_reason,
        inp.range_status if inp.range_ready else RANGE_FAULT,
    ]
    body = ('RD1,' + ','.join(str(v) for v in fields)).encode()
    return body + b'*' + f'{zlib.crc32(body):08X}'.encode() + b'\n'


# =====================================================================
# Fake board: user inputs -> driver layer -> RoadApp -> UART frames
# =====================================================================
class FakeBoard:
    def __init__(self):
        # ----- things YOU control (physical world)
        self.btn_up = self.btn_down = self.btn_reset = False
        self.ldr_value = 1200            # raw ADC the LDR would give
        self.object_cm = 70.0            # real distance in front of US-100
        self.no_echo = False             # nothing in front (open space)
        self.sensor_fault = False        # broken wiring -> status 2
        self.usb_connected = True
        # ----- board internals
        self.app = RoadApp()
        self.inputs = RoadInputs()
        self.ms = 0
        self.packet_sequence = 0
        self.red_duty_us = 0
        self.seg_digit = 0
        self.blue_led = False
        self.last_frame = b''
        self.last_snapshot = RoadInputs()
        self.last_width_us = None

    # TIM3 capture result for one 60 ms trigger period
    def _range_measurement(self):
        inp = self.inputs
        if self.sensor_fault:
            inp.distance_mm, status = 0, RANGE_FAULT
            self.last_width_us = None
        elif self.no_echo:
            inp.distance_mm, status = 0, RANGE_NO_ECHO
            self.last_width_us = None
        else:
            width = int(self.object_cm * 10.0 * 2000.0 / 343.0 + 0.5)  # echo pulse in us
            self.last_width_us = width
            if ECHO_MIN_US <= width <= ECHO_MAX_US:
                inp.distance_mm, status = (width * 343 + 1000) // 2000, RANGE_VALID
            else:
                inp.distance_mm, status = 0, RANGE_FAULT
        inp.range_status = status
        inp.range_ms = self.ms
        inp.range_sequence = (inp.range_sequence + 1) & U32
        inp.range_ready = True

    def _update_red_led(self):
        target = (self.app.out.red_target_permille * RED_PWM_PERIOD_US) // 1000
        step = (RED_PWM_PERIOD_US * TICK_MS) // RED_FADE_MS
        duty = self.red_duty_us
        if target > duty:
            duty += min(step, target - duty)
        else:
            duty -= min(step, duty - target)
        self.red_duty_us = duty

    def _update_seven_segment(self, snap):
        valid = (snap.range_ready and snap.range_status == RANGE_VALID
                 and ((self.ms - snap.range_ms) & U32) <= SENSOR_STALE_MS)
        if valid:
            self.seg_digit = min(9, int(f32(self.app.out.gap_m / f32(10.0))))
        else:
            self.seg_digit = 0 if (self.ms % 1000) < 500 else 9

    def step_10ms(self):
        """One TIM4 application tick (10 ms). Returns UART bytes sent, if any."""
        self.ms = (self.ms + TICK_MS) & U32
        # EXTI + debounce (keyboard is already clean, so state is copied directly)
        self.inputs.speed_up_pressed = self.btn_up
        self.inputs.speed_down_pressed = self.btn_down
        self.inputs.reset_pressed = self.btn_reset
        if self.ms % ECHO_PERIOD_MS == 0:
            self._range_measurement()          # TIM3 IRQ
        snap = self.inputs.copy()              # atomic snapshot
        self.app.tick(snap, self.ms)           # RoadApp_Tick
        self.blue_led = self.app.out.night     # PA5
        self._update_red_led()                 # PA6 via TIM2
        self._update_seven_segment(snap)       # PC7/PA8/PB10/PA9
        sent = b''
        if self.ms % TELEMETRY_MS == 0:        # USART2 DMA
            self.last_frame = encode_frame(self.app.out, snap, self.packet_sequence, self.ms)
            self.packet_sequence = (self.packet_sequence + 1) & U32
            if self.usb_connected:
                sent = self.last_frame
        # ADC1 EOC IRQ shortly after SWSTART
        self.inputs.ldr_adc = max(0, min(ADC_MAX, int(self.ldr_value)))
        self.inputs.ldr_ms = self.ms
        self.inputs.ldr_ready = True
        self.last_snapshot = snap
        return sent


class FakeReceiver:
    """Same interface as road_board_io.SerialReceiver, but the 'serial port'
    is the FakeBoard. Frames still pass through the real LineDecoder (CRC)."""

    def __init__(self, board, clock=time.monotonic, on_frame=None):
        self.board = board
        self.clock = clock
        self.decoder = LineDecoder()
        self.latest = None
        self.received_at = None
        self.message = 'WAITING FOR BOARD'
        self.on_frame = on_frame
        self.on_poll = None        # read inputs before the board runs
        self.after_poll = None     # draw panel after the board ran
        self._t0 = None
        self.frames_ok = 0

    def advance_to(self, now):
        if self._t0 is None:
            self._t0 = now
        target_ms = int((now - self._t0) * 1000)
        steps = 0
        while (target_ms - self.board.ms) >= TICK_MS and steps < 50:
            data = self.board.step_10ms()
            steps += 1
            # arrival time = board time of this frame mapped onto our clock
            # (never later than 'now', so BoardView sees it as fresh)
            arrived = self._t0 + self.board.ms / 1000.0 - 0.001
            for frame in self.decoder.feed(data):
                self.latest, self.received_at = frame, arrived
                self.message = 'BOARD CONNECTED'
                self.frames_ok += 1
                if self.on_frame:
                    self.on_frame(frame)
        if steps == 50:                      # window was frozen / dragged: resync
            self._t0 = now - self.board.ms / 1000.0

    def snapshot(self):
        now = self.clock()
        if self.on_poll:
            self.on_poll(now)
        self.advance_to(now)
        if self.after_poll:
            self.after_poll(now)
        return self.latest, self.received_at, self.message, self.decoder.invalid

    def start(self):
        pass

    def close(self):
        pass


# =====================================================================
# Detailed output report (shared by GUI panel and console)
# =====================================================================
def _onoff(b):
    return 'ON ' if b else 'off'


def report_lines(board, view, wide=False):
    o, a, snap = board.app.out, board.app, board.last_snapshot
    pkt = view.packet
    L = []
    L.append('== INPUT (you) ==')
    L.append(f'SPEED+ PA10 : {"HELD" if board.btn_up else "-"}   SPEED- PB3 : {"HELD" if board.btn_down else "-"}')
    L.append(f'RESET  PB4  : {"HELD" if board.btn_reset else "-"}')
    L.append(f'LDR    PA1  : ADC {board.ldr_value}  ({"dark" if board.ldr_value > NIGHT_ENTER_ADC else "light"})')
    if board.sensor_fault:
        obj = 'SENSOR FAULT'
    elif board.no_echo:
        obj = 'nothing (no echo)'
    else:
        obj = f'{board.object_cm:.1f} cm (echo {board.last_width_us} us)'
    L.append(f'US-100 PC6  : {obj}')
    L.append(f'USB UART    : {"connected" if board.usb_connected else "UNPLUGGED"}')
    L.append('')
    L.append('== MCU OUTPUT  (RD1 frame on UART) ==')
    L.append(f'board time  : {board.ms/1000:7.2f} s  seq {board.packet_sequence}')
    L.append(f'speed       : {o.speed_mps*3.6:6.1f} km/h ({o.speed_mps:6.2f} m/s)')
    L.append(f'cruise tgt  : {o.target_mps*3.6:6.1f} km/h (cap {speed_cap_mps(o.night)*3.6:.0f})')
    L.append(f'accel (IDM) : {o.acceleration_mps2:+6.2f} m/s2')
    L.append(f'odometer    : {o.odometer_m:8.1f} m')
    L.append(f'gap         : ' + (f'{o.gap_m:6.1f} m' if o.present else '  none'))
    L.append(f'lead speed  : ' + (f'{o.lead_speed_mps*3.6:6.1f} km/h (est)' if o.lead_valid else '  unknown'))
    L.append(f'ldr_adc     : {snap.ldr_adc}')
    L.append('distance_mm : '
             + (str(snap.distance_mm) if snap.range_ready and snap.range_status == 0 else '-1'))
    L.append(f'range_status: {snap.range_status} {STATUS_TEXT.get(snap.range_status, "")}')
    L.append(f'reset_id    : {o.reset_id}')
    L.append(f'done_reason : {o.done_reason} {DONE_TEXT[o.done_reason]}')
    L.append('flags       : ' + '  '.join(
        f'{n}={int(v)}' for n, v in (('NIGHT', o.night), ('PRESENT', o.present), ('LEAD', o.lead_valid))))
    L.append('              ' + '  '.join(
        f'{n}={int(v)}' for n, v in (('BRAKING', o.braking), ('DONE', o.done))))
    L.append('              ' + '  '.join(
        f'{n}={int(v)}' for n, v in (('FAULT', o.fault), ('ACCEL_BLOCKED', o.accel_blocked))))
    L.append('')
    L.append('== MCU INTERNAL (debugger only) ==')
    L.append(f'desired gap : {o.desired_gap_m:6.1f} m  (IDM s*)')
    L.append(f'range_rate  : {a.range_rate:+6.2f} m/s')
    L.append(f'est age {a.estimate_age:5.2f} s  settled {a.settled_s:4.2f} s')
    L.append(f'did_brake {int(a.did_brake)}  hold {a.hold_up_ms}/{a.hold_down_ms} ms  dwell {a.light_ms}')
    L.append('')
    L.append('== BOARD PINS ==')
    L.append(f'PA5 blue LED (night) : {_onoff(board.blue_led)}')
    L.append(f'PA6 red LED PWM      : {board.red_duty_us/10:5.1f} %')
    L.append(f'  (fade target       : {o.red_target_permille/10:5.1f} %)')
    L.append(f'7-segment            : {board.seg_digit}')
    L.append('')
    L.append('== PC SIDE (road_board_io) ==')
    L.append(f'link        : {view.status}')
    margin = view.stop_margin_m
    L.append('stop margin : ' + ('-' if margin is None else f'{margin:+.1f} m'))
    L.append(f'crash       : {"YES (simulated)" if view.crashed else "no"}')
    if pkt is not None:
        vg = visual_gap(pkt)
        L.append('scene gap   : ' + ('-' if vg is None else f'{vg:.1f} m'))
    frame = board.last_frame.decode(errors='replace').strip()
    if wide:
        L.append(f'raw frame   : {frame}')
    else:
        L.append('raw frame:')
        for i in range(0, len(frame), 40):
            L.append('  ' + frame[i:i + 40])
    return L


# =====================================================================
# GUI mode: real road_simulator_board.main() + keyboard + left panel
# =====================================================================
FAKE_ORANGE = (255, 170, 30)
TAP_KEYS = {'w': 'up', 'up arrow': 'up', 's': 'down', 'down arrow': 'down', 'r': 'reset'}


class FakeInputPanel:
    """Clickable stand-in for the PHYSICAL board inputs. Test/UI-design only.

    Everything drawn here is clearly marked FAKE: on the real system these
    values come from the STM32 buttons, LDR and US-100, never from the PC.
    Drag the orange title bar to move the panel; H hides all fake overlays.
    """

    def __init__(self, board):
        from ursina import Entity, Text, Button, Slider, camera, color
        self.board = board
        self.dragging = None
        o = color.rgb32(*FAKE_ORANGE)
        dark = color.rgb32(30, 32, 38)
        self.root = Entity(parent=camera.ui, position=(.565, -.35, -.30))
        # hazard frame: orange border + dark body
        Entity(parent=self.root, model='quad', scale=(.47, .27), color=o, z=.02)
        Entity(parent=self.root, model='quad', scale=(.458, .258), color=color.rgba(.10, .11, .14, .96), z=.01)
        self.title = Button(parent=self.root, model='quad', position=(0, .116), scale=(.458, .034),
                            color=o, highlight_color=color.rgb32(255, 195, 90), pressed_color=o)
        Text(parent=self.root, text='!! FAKE INPUT PANEL - TEST ONLY !!', position=(0, .122, -.02),
             origin=(0, 0), scale=.58, color=dark)
        Text(parent=self.root, text='NOT THE STM32 BOARD  -  drag this bar  -  H hide',
             position=(0, .106, -.02), origin=(0, 0), scale=.40, color=dark)

        def make_button(label, x, y, w=.135):
            b = Button(parent=self.root, text=label, position=(x, y, -.02), scale=(w, .036),
                       color=color.rgb32(62, 66, 78), highlight_color=color.rgb32(88, 94, 110),
                       pressed_color=o, text_size=.62)
            return b

        self.btn_down = make_button('SPEED -', -.15, .072)
        self.btn_up = make_button('SPEED +', 0, .072)
        self.btn_reset = make_button('RESET', .15, .072)

        self.ldr_label = Text(parent=self.root, text='', position=(-.215, .043, -.02), scale=.52,
                              color=color.rgb32(235, 238, 245))
        self.ldr = Slider(parent=self.root, min=0, max=ADC_MAX, default=board.ldr_value, step=1,
                          dynamic=True, position=(-.2, .020, -.02), scale=.76)
        self.ldr.on_value_changed = self._ldr_changed

        self.dist_label = Text(parent=self.root, text='', position=(-.215, -.009, -.02), scale=.52,
                               color=color.rgb32(235, 238, 245))
        self.dist = Slider(parent=self.root, min=1, max=120, default=board.object_cm, step=.5,
                           dynamic=True, position=(-.2, -.032, -.02), scale=.76)
        self.dist.on_value_changed = self._dist_changed
        for sl in (self.ldr, self.dist):
            sl.knob.text_entity.enabled = False  # value is shown in the label above

        self.tog_echo = make_button('', -.15, -.073)
        self.tog_fault = make_button('', 0, -.073)
        self.tog_usb = make_button('', .15, -.073)
        self.tog_echo.on_click = self._toggle_echo
        self.tog_fault.on_click = self._toggle_fault
        self.tog_usb.on_click = self._toggle_usb
        Text(parent=self.root, text='keyboard still works: W/S R A/D 1-9 0 N [ ] F U  Tab=output',
             position=(0, -.108, -.02), origin=(0, 0), scale=.38, color=o)
        self._held_color = o
        self._idle_color = color.rgb32(62, 66, 78)
        self._on_color = color.rgb32(200, 60, 50)

    # ---- slider callbacks (user moved the knob)
    def _ldr_changed(self):
        self.board.ldr_value = int(self.ldr.value)

    def _dist_changed(self):
        self.board.no_echo = False
        self.board.object_cm = float(self.dist.value)

    def _toggle_echo(self):
        self.board.no_echo = not self.board.no_echo

    def _toggle_fault(self):
        self.board.sensor_fault = not self.board.sensor_fault

    def _toggle_usb(self):
        self.board.usb_connected = not self.board.usb_connected

    def sync_from_board(self):
        """Keyboard changed a value: move the knobs without fighting a drag."""
        if not self.ldr.knob.dragging and int(self.ldr.value) != self.board.ldr_value:
            self.ldr.value = self.board.ldr_value
        if not self.dist.knob.dragging and abs(self.dist.value - self.board.object_cm) > .26:
            keep = self.board.no_echo
            self.dist.value = min(120.0, max(1.0, self.board.object_cm))
            self.board.no_echo = keep

    def held(self, button):
        from ursina import held_keys
        return bool(self.root.enabled and button.hovered and held_keys['left mouse'])

    def update(self):
        from ursina import held_keys, mouse
        b = self.board
        # drag the whole panel by its title bar
        if self.title.hovered and held_keys['left mouse'] and self.dragging is None:
            self.dragging = self.root.position - mouse.position
        if self.dragging is not None:
            if held_keys['left mouse']:
                p = mouse.position + self.dragging
                self.root.position = (p[0], p[1], self.root.z)
            else:
                self.dragging = None
        for btn, on in ((self.btn_up, b.btn_up), (self.btn_down, b.btn_down), (self.btn_reset, b.btn_reset)):
            btn.color = self._held_color if on else self._idle_color
        dark = b.ldr_value > NIGHT_ENTER_ADC
        self.ldr_label.text = f'LDR  ADC {b.ldr_value:4d}  ({"dark -> night" if dark else "light -> day"})'
        if b.sensor_fault:
            d = 'SENSOR FAULT'
        elif b.no_echo:
            d = 'NO ECHO (nothing ahead)'
        else:
            d = f'{b.object_cm:5.1f} cm  =  {b.object_cm:5.1f} m road'
        self.dist_label.text = f'US-100  {d}'
        for btn, on, label in ((self.tog_echo, b.no_echo, 'NO ECHO'),
                               (self.tog_fault, b.sensor_fault, 'FAULT'),
                               (self.tog_usb, not b.usb_connected, 'USB OFF' if not b.usb_connected else 'USB ON')):
            btn.text = label
            btn.color = self._on_color if on else self._idle_color


def run_gui(print_frames):
    import road_simulator_board

    board = FakeBoard()
    view_for_panel = BoardView()
    state = {'panel': None, 'out_root': None, 'inputs': None, 'catcher': None, 'last': None,
             'tap': {'up': 0.0, 'down': 0.0, 'reset': 0.0}}

    def on_frame(frame):
        if print_frames:
            print(board.last_frame.decode().strip())

    receiver = FakeReceiver(board, on_frame=on_frame)

    def on_key(key):
        """One-shot keys use input events, so even a very short tap is seen."""
        panel = state['inputs']
        if key in TAP_KEYS:      # a very short tap still lasts >= 0.1 s on the 'board'
            state['tap'][TAP_KEYS[key]] = time.monotonic() + 0.1
        if key == 'tab':
            state['out_root'].enabled = not state['out_root'].enabled
        elif key == 'h':
            show = not panel.root.enabled
            panel.root.enabled = show
            state['out_root'].enabled = show
        elif key in tuple('123456789'):
            board.no_echo = False
            board.object_cm = 10.0 * int(key)
        elif key == '0':
            board.no_echo = True
        elif key == 'f':
            board.sensor_fault = not board.sensor_fault
        elif key == 'n':
            board.ldr_value = 1200 if board.ldr_value > NIGHT_ENTER_ADC else 3900
        elif key == '[':
            board.ldr_value = max(0, board.ldr_value - 100)
        elif key == ']':
            board.ldr_value = min(ADC_MAX, board.ldr_value + 100)
        elif key == 'u':
            board.usb_connected = not board.usb_connected

    def build_overlays():
        from ursina import Text, Entity, camera, color
        o = color.rgb32(*FAKE_ORANGE)
        root = Entity(parent=camera.ui)
        Entity(parent=root, model='quad', position=(-.585, -.005, -.01),
               scale=(.43, .73), color=color.rgba(0.08, 0.09, 0.12, 0.85))
        Entity(parent=root, model='quad', position=(-.585, .352, -.012), scale=(.43, .016), color=o)
        Text(parent=root, text='FAKE BOARD DEBUG OUTPUT', position=(-.79, .330, -.02),
             scale=.62, color=o)
        Text(parent=root, text='Tab = hide / show this panel', position=(-.79, .303, -.02),
             scale=.42, font='VeraMono.ttf', color=o)
        state['panel'] = Text(parent=root, text='', position=(-.79, .272, -.02),
                              scale=.53, font='VeraMono.ttf', color=color.rgb32(235, 238, 245))
        state['out_root'] = root
        state['inputs'] = FakeInputPanel(board)
        for name, btn in (('up', state['inputs'].btn_up), ('down', state['inputs'].btn_down),
                          ('reset', state['inputs'].btn_reset)):
            btn.on_click = (lambda n=name: state['tap'].__setitem__(n, time.monotonic() + 0.1))
        catcher = Entity()
        catcher.input = on_key
        state['catcher'] = catcher

    def on_poll(now):
        from ursina import held_keys
        if state['panel'] is None:
            build_overlays()
        panel = state['inputs']
        dt = 0.0 if state['last'] is None else min(now - state['last'], .2)
        state['last'] = now
        tap = state['tap']
        board.btn_up = bool(held_keys['w'] or held_keys['up arrow'] or panel.held(panel.btn_up)
                            or time.monotonic() < tap['up'])
        board.btn_down = bool(held_keys['s'] or held_keys['down arrow'] or panel.held(panel.btn_down)
                              or time.monotonic() < tap['down'])
        board.btn_reset = bool(held_keys['r'] or panel.held(panel.btn_reset)
                               or time.monotonic() < tap['reset'])
        if held_keys['a'] or held_keys['left arrow']:
            board.no_echo = False
            board.object_cm = max(1.0, board.object_cm - 20.0 * dt)
        if held_keys['d'] or held_keys['right arrow']:
            board.no_echo = False
            board.object_cm = min(120.0, board.object_cm + 20.0 * dt)
        panel.sync_from_board()
        panel.update()

    def after_poll(now):
        view_for_panel.refresh(_PanelReceiver(receiver), now)
        if state['out_root'].enabled:
            state['panel'].text = '\n'.join(report_lines(board, view_for_panel))

    receiver.on_poll = on_poll
    receiver.after_poll = after_poll
    road_simulator_board.main(receiver)


class _PanelReceiver:
    """Read-only view of FakeReceiver without advancing the board again."""
    def __init__(self, r):
        self.r = r

    def snapshot(self):
        return self.r.latest, self.r.received_at, self.r.message, self.r.decoder.invalid


# =====================================================================
# Console mode
# =====================================================================
class ConsoleSim:
    def __init__(self):
        self.board = FakeBoard()
        self.t = 0.0
        self.receiver = FakeReceiver(self.board, clock=lambda: self.t)
        self.view = BoardView()
        self.receiver.snapshot()

    def run(self, seconds, every=None, table=False):
        end = self.t + seconds
        next_print = self.t
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.01, 6)
            self.receiver.snapshot()
            self.view.refresh(self.receiver, self.t)
            if table and every and self.t >= next_print - 1e-9:
                print(self.table_row())
                next_print += every

    def press(self, attr, seconds):
        setattr(self.board, attr, True)
        self.run(seconds)
        setattr(self.board, attr, False)

    @staticmethod
    def table_header():
        return (' time | speed | target | accel | object | gap  | s*   | flags     | redLED | 7seg | PC\n'
                '  (s) | km/h  |  km/h  | m/s2  |  cm    |  m   |  m   | N P L B D F X |   %    |      |')

    def table_row(self):
        b, o = self.board, self.board.app.out
        obj = 'fault' if b.sensor_fault else ('none' if b.no_echo else f'{b.object_cm:5.1f}')
        gap = f'{o.gap_m:4.1f}' if o.present else '  - '
        fl = ' '.join(str(int(x)) for x in (o.night, o.present, o.lead_valid, o.braking,
                                             o.done, o.fault, o.accel_blocked))
        pc = 'CRASH' if self.view.crashed else (self.view.reason or ('TIMEOUT' if not self.view.connected else ''))
        return (f'{self.t:5.2f} | {o.speed_mps*3.6:5.1f} | {o.target_mps*3.6:6.1f} | {o.acceleration_mps2:+5.2f} | '
                f'{obj:>6} | {gap} | {o.desired_gap_m:4.1f} | {fl} | {b.red_duty_us/10:5.1f}  |  {b.seg_digit}   | {pc}')

    def show(self):
        print('\n'.join(report_lines(self.board, self.view, wide=True)))
        print('-' * 78)


HELP = """คำสั่ง (เวลาเป็นวินาที):
  up [s]        กด SPEED+ ค้าง s วินาที (ไม่ใส่ = แตะ 0.1 s)
  down [s]      กด SPEED- ค้าง s วินาที
  both [s]      กดสองปุ่มพร้อมกัน
  reset         กด RESET 0.1 s
  dist <cm>     ตั้งระยะวัตถุ (cm จริงหน้า US-100)
  approach <cm> <s>   เลื่อนวัตถุไปถึง <cm> ภายใน <s> วินาที (พิมพ์ตารางระหว่างทาง)
  noecho        ไม่มีวัตถุข้างหน้า
  fault         สลับ sensor fault
  ldr <adc>     ตั้งค่า ADC ของ LDR (0..4095, >3000 = มืด)
  dark / light  ลัด: ldr 3900 / ldr 1200
  usb           สลับ ถอด/เสียบ USB
  run <s>       ปล่อยเวลาเดิน s วินาที
  watch <s>     ปล่อยเวลาเดินพร้อมพิมพ์ตารางทุก 0.25 s
  show          แสดง output ทั้งหมดอย่างละเอียด
  demo          รันสถานการณ์ตัวอย่าง
  help / quit"""


def run_demo(sim):
    print('=== DEMO: boot -> cruise -> car ahead -> auto brake -> stop -> night -> reset ===\n')
    print('[1] เปิดบอร์ด วัตถุอยู่ 70 cm (=70 m) ห้องสว่าง')
    sim.board.object_cm = 70.0
    sim.run(1.0)
    sim.show()
    print('[2] กด SPEED+ ค้าง 1.5 s แล้วปล่อยให้รถเร่ง 4 s')
    print(sim.table_header())
    sim.press('btn_up', 1.5)
    sim.run(4.0, every=0.5, table=True)
    print('\n[3] วัตถุเข้าใกล้จาก 70 cm -> 12 cm ใน 5 s (รถข้างหน้าช้ากว่า)')
    print(sim.table_header())
    approach(sim, 12.0, 5.0)
    print('\n[4] กด SPEED+ ระหว่างมีรถใกล้ -> ดู flag BLOCKED (X)')
    sim.board.btn_up = True
    sim.run(0.3, every=0.1, table=True)
    sim.board.btn_up = False
    print('\n[5] ปล่อยเวลาเดินจนจบ sequence')
    sim.run(6.0, every=0.5, table=True)
    sim.show()
    print('[6] ปิด LDR (มืด) 0.6 s -> NIGHT')
    sim.board.ldr_value = 3900
    sim.run(0.6, every=0.1, table=True)
    print('\n[7] ยกวัตถุออก (no echo) แล้วกด RESET')
    sim.board.no_echo = True
    sim.press('btn_reset', 0.1)
    sim.run(1.0, every=0.25, table=True)
    sim.show()


def approach(sim, cm, seconds):
    start = sim.board.object_cm if not sim.board.no_echo else 120.0
    sim.board.no_echo = False
    steps = max(1, int(seconds / 0.01))
    every = 0.25
    next_print = sim.t
    for i in range(1, steps + 1):
        sim.board.object_cm = start + (cm - start) * i / steps
        sim.run(0.01)
        if sim.t >= next_print - 1e-9:
            print(sim.table_row())
            next_print += every


def run_console(demo_only):
    sim = ConsoleSim()
    if demo_only:
        run_demo(sim)
        return
    print(__doc__.split('วิธีรัน')[0])
    print(HELP)
    sim.run(0.5)
    sim.show()
    while True:
        try:
            line = input('fametest> ').strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, args = line[0].lower(), line[1:]
        try:
            num = float(args[0]) if args else None
            if cmd in ('quit', 'exit', 'q'):
                break
            elif cmd == 'help':
                print(HELP)
                continue
            elif cmd == 'up':
                sim.press('btn_up', num or 0.1)
            elif cmd == 'down':
                sim.press('btn_down', num or 0.1)
            elif cmd == 'both':
                sim.board.btn_up = sim.board.btn_down = True
                sim.run(num or 0.1)
                sim.board.btn_up = sim.board.btn_down = False
            elif cmd == 'reset':
                sim.press('btn_reset', 0.1)
            elif cmd == 'dist':
                sim.board.object_cm, sim.board.no_echo = num, False
                sim.run(0.1)
            elif cmd == 'approach':
                print(sim.table_header())
                approach(sim, num, float(args[1]) if len(args) > 1 else 3.0)
            elif cmd == 'noecho':
                sim.board.no_echo = True
                sim.run(0.1)
            elif cmd == 'fault':
                sim.board.sensor_fault = not sim.board.sensor_fault
                sim.run(0.1)
            elif cmd == 'ldr':
                sim.board.ldr_value = int(num)
                sim.run(0.1)
            elif cmd == 'dark':
                sim.board.ldr_value = 3900
                sim.run(0.1)
            elif cmd == 'light':
                sim.board.ldr_value = 1200
                sim.run(0.1)
            elif cmd == 'usb':
                sim.board.usb_connected = not sim.board.usb_connected
                sim.run(0.1)
            elif cmd == 'run':
                sim.run(num or 1.0)
            elif cmd == 'watch':
                print(sim.table_header())
                sim.run(num or 2.0, every=0.25, table=True)
                continue
            elif cmd == 'demo':
                run_demo(sim)
                continue
            elif cmd == 'show':
                pass
            else:
                print('ไม่รู้จักคำสั่ง — พิมพ์ help')
                continue
        except (ValueError, TypeError, IndexError):
            print('รูปแบบคำสั่งไม่ถูกต้อง — พิมพ์ help')
            continue
        sim.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fake STM32 board for the road simulator (no COM port)')
    parser.add_argument('--console', action='store_true', help='text mode, type inputs as commands')
    parser.add_argument('--demo', action='store_true', help='run the scripted scenario in text mode')
    parser.add_argument('--print-frames', action='store_true', help='GUI mode: also print every RD1 frame')
    opts = parser.parse_args()
    if opts.console or opts.demo:
        run_console(opts.demo)
    else:
        try:
            import ursina  # noqa: F401
        except ImportError:
            sys.exit('ต้องติดตั้ง ursina ก่อน: python -m pip install ursina  (หรือใช้ --console)')
        run_gui(opts.print_frames)
