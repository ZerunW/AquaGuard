import errno
import select
import socket
import struct
import time

CAN_FRAME = struct.Struct("=IB3x8s")
FILTER_MASK = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG

C620_FULL_SCALE = 16384      # command units for 20 A
GEAR_RATIO = 3591 / 187      # M3508 P19 gearbox, rotor : output
PHASE_TIME = 5.0
TORQUE_PHASES = [("100%", 1.0), ("50%", 0.5)]   # fraction of --max-current
SPEED_LIMIT_DEFAULT = 8192   # current limit of the speed loop unless --max-current (10 A)

KP, KI = 0.6, 0.25           # current units per rpm of (predicted) error, and per rpm*s
ACCEL = 3000.0               # rpm/s slew of the speed target
EASE_TIME = 0.25             # s: the target eases exponentially into the goal
INTEGRATE_BAND = 0.05        # integrate only once the target is this close to the goal (0 = always)
FRICTION = (124, 0.013)      # running friction: units + units per rpm (144 at 1500 rpm, 163 at 3000)
MODEL_GAIN = 11.4            # rpm/s per current unit above friction
BREAKAWAY_FAST = 700         # breakaway boost ramps fast up to this, then slowly
BREAKAWAY_RATES = (5000.0, 1500.0)  # boost ramp, units/s (breakaway measured at 640-1040)
STUCK_RPM = 5                # the C620 reports exactly 0 when the rotor is still
MOVING_RPM = 30              # ...and a twitch while breaking free reads a few rpm
SEND_PERIOD = 0.002          # never command faster than 500 Hz
STALL_TIMEOUT = 1.0          # no echo for this long -> assume echoes were lost, resync
STOP_TIMEOUT = 3.0           # give up waiting for confirmed zeros after this long
ZERO_CONFIRMS = 3            # zero-current frames that must be confirmed on the wire
STATUS_PERIOD = 0.5
SETTLE_TIME = 1.0            # ignore the first second of a phase in the rpm stats



def amps(raw):
    return raw * 20.0 / C620_FULL_SCALE


def clamp(x, limit):
    return max(-limit, min(limit, x))



class CanIO:

    def __init__(self, iface, ids):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
        filters = b"".join(struct.pack("=II", i, FILTER_MASK) for i in ids)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        self.sock.bind((iface,))
        self.sock.setblocking(False)

    def send(self, can_id, data):
        try:
            self.sock.send(CAN_FRAME.pack(can_id, len(data), data))
            return True
        except BlockingIOError:
            return False
        except OSError as e:
            if e.errno == errno.ENOBUFS:
                return False
            raise

    def recv(self):
        while True:
            try:
                frame, _, flags, _ = self.sock.recvmsg(CAN_FRAME.size)
            except BlockingIOError:
                return
            can_id, dlc, data = CAN_FRAME.unpack(frame)
            yield (can_id, data[:dlc], bool(flags & socket.MSG_CONFIRM),
                   bool(flags & socket.MSG_DONTROUTE))

    def wait(self, timeout):
        select.select([self.sock], [], [], max(0.0, timeout))

    def close(self):
        self.sock.close()



class Link:

    def __init__(self, io, esc_id, window):
        self.io = io
        self.cmd_id = 0x200 if esc_id <= 4 else 0x1FF
        self.slot = (esc_id - 1) % 4
        self.fb_id = 0x200 + esc_id
        self.window = window
        self.in_flight = 0
        self.last_echo = 0.0
        self.next_send = 0.0
        self.sent = self.confirmed = self.zero_confirmed = 0
        self.skipped = self.resyncs = self.foreign = 0
        self.wire_value = None       # setpoint of the newest frame confirmed sent
        self.fb_frames = 0
        self.angle = self.rpm = self.current = self.temp = None
        self.rpm_mean = None         # mean rpm of the latest feedback burst
        self.burst_n = 0

    def frame(self, setpoint):
        data = bytearray(8)
        struct.pack_into(">h", data, self.slot * 2, setpoint)
        return bytes(data)

    def service(self, setpoint, now):
        burst_sum = burst_n = 0
        for can_id, data, own, local in self.io.recv():
            if can_id == self.cmd_id:
                if own:
                    self.in_flight = max(0, self.in_flight - 1)
                    self.confirmed += 1
                    self.last_echo = now
                    self.wire_value = struct.unpack_from(">h", data, self.slot * 2)[0]
                    if self.wire_value == 0:
                        self.zero_confirmed += 1
                else:
                    self.foreign += 1
            elif can_id == self.fb_id and len(data) >= 7:
                self.angle, self.rpm, self.current, self.temp = struct.unpack_from(">HhhB", data)
                self.fb_frames += 1
                burst_sum += self.rpm
                burst_n += 1
        if burst_n:
            self.rpm_mean = burst_sum / burst_n
            self.burst_n = burst_n

        if self.in_flight and now - self.last_echo > STALL_TIMEOUT:
            self.in_flight = 0
            self.resyncs += 1

        if self.in_flight < self.window and now >= self.next_send:
            self.next_send = now + SEND_PERIOD
            if self.io.send(self.cmd_id, self.frame(setpoint)):
                if not self.in_flight:
                    self.last_echo = now
                self.in_flight += 1
                self.sent += 1
            else:
                self.skipped += 1

    def send_deadline(self):
        return self.next_send if self.in_flight < self.window else None



def sign(x):
    return (x > 0) - (x < 0)


class SpeedProfile:

    def __init__(self, targets, limit, kp, ki, accel):
        self.phases = [(f"{r}rpm", PHASE_TIME, r) for r in targets]
        self.total = PHASE_TIME * len(self.phases)
        self.limit, self.kp, self.ki, self.accel = limit, kp, ki, accel
        self.target = 0.0
        self.last_target = None
        self.predicted = 0.0
        self.integral = 0.0
        self.boost = 0.0
        self.stuck = True
        self.output = 0
        self.trace = []              
        self.fb_seen = 0
        self.last_update = None

    def describe(self):
        return (", ".join(f"{d:.0f} s at {r} rpm ({r / GEAR_RATIO:.0f} rpm at the shaft)"
                          for _, d, r in self.phases)
                + f"; current limit {self.limit} ({amps(self.limit):.1f} A), "
                  f"kp {self.kp} ki {self.ki}")

    def status(self):
        return f" target {self.target:6.0f} pred {self.predicted:6.0f}"

    def friction(self, rpm):
        return sign(rpm) * (FRICTION[0] + FRICTION[1] * abs(rpm))

    def update(self, goal, rpm, now):
        dt = 0.0 if self.last_update is None else min(now - self.last_update, 0.2)
        self.last_update = now
        breaking_free = self.stuck and abs(rpm) < MOVING_RPM
        if (abs(rpm) < STUCK_RPM or breaking_free) and abs(goal) >= STUCK_RPM:
            self.stuck = True
            if self.boost < BREAKAWAY_FAST:
                self.boost = min(BREAKAWAY_FAST, self.boost + BREAKAWAY_RATES[0] * dt)
            else:
                self.boost += BREAKAWAY_RATES[1] * dt
            self.integral = 0.0
            self.predicted = float(rpm)
            return clamp(sign(goal) * (FRICTION[0] + self.boost), self.limit)
        self.predicted = rpm + MODEL_GAIN * (self.output - self.friction(rpm)) * dt
        if self.stuck:
            self.stuck = False
            self.boost = 0.0
            self.target = self.predicted
            self.last_target = None
        previous = self.target
        step = min(self.accel * dt, abs(goal - self.target) * dt / EASE_TIME)
        self.target += clamp(goal - self.target, step)
        accel = (self.target - previous) / dt if dt else 0.0
        feedforward = self.friction(self.target) + accel / MODEL_GAIN
        err = (rpm if self.last_target is None else self.last_target) - rpm
        self.last_target = self.target
        p_term = self.kp * (self.target - self.predicted)
        unclamped = feedforward + p_term + self.integral + self.ki * err * dt
        on_goal = INTEGRATE_BAND <= 0 or abs(goal - self.target) <= INTEGRATE_BAND * abs(goal)
        if on_goal and (abs(unclamped) < self.limit or (unclamped > 0) != (err > 0)):
            self.integral = clamp(self.integral + self.ki * err * dt, self.limit)
        return clamp(feedforward + p_term + self.integral, self.limit)

    def __call__(self, t, link, now):
        start = 0.0
        for name, duration, goal in self.phases:
            if t < start + duration:
                break
            start += duration
        else:
            return None
        if link.rpm_mean is not None and link.fb_frames != self.fb_seen:
            self.fb_seen = link.fb_frames
            self.output = round(self.update(goal, link.rpm_mean, now))
            self.trace.append((round(t, 4), goal, round(self.target), round(link.rpm_mean),
                               link.burst_n, round(self.predicted), self.output, link.wire_value))
        return name, self.output



def status_line(t, name, setpoint, link, last, note=""):
    dt = max(t - last["t"], 1e-6)
    tx_rate = (link.confirmed - last["confirmed"]) / dt
    fb_rate = (link.fb_frames - last["fb"]) / dt
    last.update(t=t, confirmed=link.confirmed, fb=link.fb_frames)
    wire = "-" if link.wire_value is None else link.wire_value
    fb = "no feedback"
    if link.rpm_mean is not None:
        fb = f"rpm {link.rpm_mean:6.0f}  I {amps(link.current):5.1f}A  {link.temp}C"
    extra = ""
    if link.skipped or link.resyncs:
        extra = f"  [queue-full skips {link.skipped}, resyncs {link.resyncs}]"
    return (f"{t:5.1f}s {name:<7}{note} cmd {setpoint:6d} wire {wire:>6}  "
            f"tx {tx_rate:4.0f}/s  fb {fb_rate:4.0f}/s  {fb}{extra}")


def stop_motor(link, io, out):
    t0 = time.monotonic()
    # Echoes of frames already in flight carry the old setpoint, so only zeros count.
    z0 = link.zero_confirmed
    while link.zero_confirmed - z0 < ZERO_CONFIRMS:
        now = time.monotonic()
        if now - t0 > STOP_TIMEOUT:
            out(f"WARNING: zero current NOT confirmed on the wire within {STOP_TIMEOUT:.0f} s "
                "- check the motor (the C620 should still time out on its own).")
            return False
        link.service(0, now)
        deadline = link.send_deadline()
        io.wait(0.05 if deadline is None else min(0.05, deadline - time.monotonic()))
    out(f"stop: zero current confirmed on the wire after {time.monotonic() - t0:.2f} s")
    return True



def read_stat(iface, name):
    try:
        with open(f"/sys/class/net/{iface}/statistics/{name}") as f:
            return int(f.read())
    except OSError:
        return 0


