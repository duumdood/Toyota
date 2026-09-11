"""Read-only UART boundary for road_simulator_board.py. No driving model here.

BOARD INPUT VARIABLES are the named fields in BoardTelemetry below. C produces
ALL control results (IDM, speed, lead estimate, light mode, brake, reset).
Only SI unit conversion and render interpolation happen on this side.
Wire contract and pin/calibration instructions: stm32_road/README.md.
"""
from dataclasses import dataclass
import re
import threading
import time
import zlib

SERIAL_PORT = None  # Set to e.g. "COM5", or pass --port COM5. Never auto-select.
SERIAL_BAUD = 115200
LINK_TIMEOUT_S = 0.5
MAX_FRAME_BYTES = 240
RECONNECT_DELAY_S = 1.0


@dataclass(frozen=True)
class BoardTelemetry:
    sequence: int
    board_ms: int
    speed_mps: float         # INPUT: board IDM integrated vehicle speed
    target_mps: float        # INPUT: board speed-button setpoint, already capped
    acceleration_mps2: float # INPUT: board IDM output
    odometer_m: float        # INPUT: board integrated distance (visual road phase)
    gap_m: float | None      # INPUT: US-100 scaled to road metres ON THE BOARD
    lead_speed_mps: float | None # INPUT: signed board-estimated lead velocity
    night: bool             # INPUT: board LDR threshold result; no PC threshold
    braking: bool           # INPUT: braking state, not inferred from PC speed
    done: bool              # INPUT: sequence finished, wait for PHYSICAL Reset
    fault: bool             # INPUT: board sensor/data fault; freeze display
    ldr_adc: int             # INPUT: raw 0..4095, diagnostic/calibration only
    distance_mm: int | None  # INPUT: real US-100 millimetres, before world scaling
    reset_id: int           # INPUT: physical reset counter
    done_reason: int        # INPUT: 0 running / 1 stopped / 2 settled
    range_status: int       # INPUT: 0 echo / 1 no return / 2 invalid or not ready


def parse_frame(line: bytes) -> BoardTelemetry:
    """Reject partial/corrupt/out-of-contract frames; never fabricate a value."""
    if len(line) > MAX_FRAME_BYTES or not line.endswith(b'\n'):
        raise ValueError('partial or oversized frame')
    body, separator, checksum = line.rstrip(b'\r\n').partition(b'*')
    if not separator or not re.fullmatch(rb'[0-9A-Fa-f]{8}', checksum):
        raise ValueError('missing CRC32')
    if zlib.crc32(body) != int(checksum, 16):
        raise ValueError('CRC32 mismatch')
    fields = body.split(b',')
    if len(fields) != 15 or fields[0] != b'RD1':
        raise ValueError('unsupported protocol or field count')
    if any(not re.fullmatch(rb'-?\d{1,10}', f) for f in fields[1:]):
        raise ValueError('integer field required')
    seq, ms, v, target, acc, odo, gap, lead, flags, ldr, mm, reset, reason, status = map(int, fields[1:])
    for value in (seq, ms, odo, reset):
        if not 0 <= value <= 0xFFFFFFFF:
            raise ValueError('uint32 out of range')
    if not (0 <= v <= 40000 and 0 <= target <= 33334 and -7001 <= acc <= 3001
            and -80000 <= lead <= 120000 and 0 <= flags <= 63 and 0 <= ldr <= 4095
            and 0 <= reason <= 2 and 0 <= status <= 2 and (mm == -1 or 1 <= mm <= 5000)):
        raise ValueError('telemetry outside physical/protocol bounds')
    present, valid = bool(flags & 2), bool(flags & 4)
    if (present and not 2000 <= gap <= 100000) or (not present and gap != -1):
        raise ValueError('inconsistent range')
    if valid and not present:
        raise ValueError('lead estimate without target')
    if bool(flags & 16) != (reason != 0):
        raise ValueError('inconsistent sequence state')
    if flags & 1 and target > 27778:
        raise ValueError('night target exceeds board cap')
    return BoardTelemetry(seq, ms, v/1000, target/1000, acc/1000, odo/1000,
        gap/1000 if present else None, lead/1000 if valid else None,
        bool(flags & 1), bool(flags & 8), bool(flags & 16), bool(flags & 32),
        ldr, mm if mm >= 0 else None, reset, reason, status)


class LineDecoder:
    """Bounded streaming framing: recover after noise/overlong/partial input."""
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False
        self.invalid = 0

    def feed(self, data):
        frames = []
        for byte in data:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            self.buffer.append(byte)
            if len(self.buffer) > MAX_FRAME_BYTES:
                self.invalid += 1
                self.buffer.clear()
                self.discarding = byte != 10
            elif byte == 10:
                try:
                    frames.append(parse_frame(bytes(self.buffer)))
                except ValueError:
                    self.invalid += 1
                self.buffer.clear()
        return frames


class SerialReceiver:
    """Background read only; no serial writes, no fake input, no render blocking."""
    def __init__(self, port, baud=SERIAL_BAUD):
        self.port, self.baud = port, baud
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = None
        self.message = 'WAITING FOR BOARD'
        self.invalid_frames = 0
        self.thread = threading.Thread(target=self._run, name='board-telemetry', daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.latest, self.received_at, self.message, self.invalid_frames

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _run(self):
        import serial
        while not self.stop_event.is_set():
            try:
                # Set modem controls BEFORE opening. Never deliberately reset
                # the board; some adapters/OS drivers may still toggle on open.
                link = serial.Serial(port=None, baudrate=self.baud, timeout=.1)
                link.dtr = False
                link.rts = False
                link.port = self.port
                with link:  # pyserial opens the configured port on __enter__
                    decoder = LineDecoder()
                    previous = None
                    with self.lock:
                        self.received_at = None
                        self.message = 'WAITING FOR TELEMETRY'
                    while not self.stop_event.is_set():
                        data = link.read(min(max(link.in_waiting, 1), 4096))
                        for frame in decoder.feed(data):
                            now = time.monotonic()
                            # Repeated packets never refresh link freshness.
                            # A restart near uptime zero is a physical MCU boot;
                            # modular comparisons also accept counter rollover.
                            if previous is not None:
                                reboot = (frame.board_ms < previous.board_ms and
                                          frame.sequence < previous.sequence and
                                          frame.board_ms < 1000 and frame.sequence < 20)
                                seq_forward = 0 < ((frame.sequence-previous.sequence)&0xFFFFFFFF) < 0x80000000
                                ms_forward = 0 < ((frame.board_ms-previous.board_ms)&0xFFFFFFFF) < 0x80000000
                                if not reboot and not (seq_forward and ms_forward):
                                    continue
                            previous = frame
                            with self.lock:
                                self.latest, self.received_at = frame, now
                                self.message = 'BOARD CONNECTED'
                        with self.lock:
                            self.invalid_frames = decoder.invalid
            except (serial.SerialException, OSError) as exc:
                with self.lock:
                    self.received_at = None
                    self.message = 'SERIAL DISCONNECTED'
                print(f'Serial {self.port}: {exc}')
                self.stop_event.wait(RECONNECT_DELAY_S)


class BoardView:
    """Renderer adapter. NO IDM, speed integration, derivative or LDR decision.

    Only interpolate between received board odometer/gap samples for smooth
    drawing. Never extrapolate movement beyond the latest board measurement.
    """
    def __init__(self):
        self.speed = self.target = self.acceleration = self.odometer = 0.0
        self.gap = 100.0
        self.lead_speed = None
        self.night = self.present = self.done = self.braking = False
        self.reason = ''
        self.packet = None
        self.connected = False
        self.status = 'WAITING FOR BOARD'
        self.last_received = None
        self.origin_odo = self.origin_gap = 0.0
        self.blend_duration = .05

    def refresh(self, receiver, now=None):
        now = time.monotonic() if now is None else now
        packet, received, message, invalid = receiver.snapshot()
        live = packet is not None and received is not None and 0 <= now-received <= LINK_TIMEOUT_S
        self.status = ('SENSOR FAULT / CHECK BOARD' if packet and packet.fault else 'BOARD CONNECTED') if live else (
            'TELEMETRY TIMEOUT' if received is not None else message)
        self.connected = live
        if not live:
            return
        if received != self.last_received or packet is not self.packet:
            previous = self.packet
            continuous = (self.last_received is not None and previous is not None
                          and received-self.last_received <= LINK_TIMEOUT_S
                          and packet.reset_id == previous.reset_id
                          and packet.board_ms >= previous.board_ms
                          and packet.odometer_m >= previous.odometer_m)
            self.origin_odo = self.odometer if continuous else packet.odometer_m
            self.origin_gap = self.gap if continuous and self.present and packet.gap_m is not None else (packet.gap_m or 100)
            self.blend_duration = min(.15, max(.01, (packet.board_ms-previous.board_ms)/1000)) if continuous else .05
            self.last_received, self.packet = received, packet
            self.speed, self.target = packet.speed_mps, packet.target_mps
            self.acceleration, self.lead_speed = packet.acceleration_mps2, packet.lead_speed_mps
            self.night, self.present, self.done = packet.night, packet.gap_m is not None, packet.done
            self.braking = packet.braking
            self.reason = {0:'', 1:'STOP COMPLETE', 2:'DECELERATION COMPLETE'}[packet.done_reason]
        alpha = min(1.0, max(0.0, (now-received)/self.blend_duration))
        if packet.fault or packet.done:
            alpha = 1.0
        self.odometer = self.origin_odo+(packet.odometer_m-self.origin_odo)*alpha
        self.gap = self.origin_gap+((packet.gap_m or 100)-self.origin_gap)*alpha
