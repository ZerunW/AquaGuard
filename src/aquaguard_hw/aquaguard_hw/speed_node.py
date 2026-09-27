#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from . import m3508_can as mc


class M3508SpeedNode(Node):

    def __init__(self):
        super().__init__("m3508_speed_node")
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("esc_id", 1)
        self.declare_parameter("target_hz", 200.0)        
        self.declare_parameter("gear_ratio", 1.0)      
        self.declare_parameter("window", 4)
        self.declare_parameter("kp", mc.KP)
        self.declare_parameter("ki", mc.KI)
        self.declare_parameter("accel", mc.ACCEL)
        self.declare_parameter("max_current", mc.SPEED_LIMIT_DEFAULT)
        self.declare_parameter("run_on_start", False)
        self.declare_parameter("trigger_topic", "/motor/trigger")
        self.declare_parameter("state_topic", "/motor/state")
        p = lambda n: self.get_parameter(n).value  

        self.iface = p("can_iface")
        self.esc_id = int(p("esc_id"))
        self.gear_ratio = float(p("gear_ratio"))
        self.target_hz = float(p("target_hz"))
        self.enabled = bool(p("run_on_start"))
        self.kp, self.ki, self.accel = float(p("kp")), float(p("ki")), float(p("accel"))
        self.limit = max(0, min(mc.C620_FULL_SCALE, int(p("max_current"))))
        window = max(1, min(16, int(p("window"))))

        self.io = mc.CanIO(self.iface, [0x200 if self.esc_id <= 4 else 0x1FF, 0x200 + self.esc_id])
        self.link = mc.Link(self.io, self.esc_id, window)
        self.speed = self._new_speed()
        self.fb_seen = 0
        self.output = 0

        self.create_subscription(Bool, p("trigger_topic"), self._on_trigger, 10)
        self.state_pub = self.create_publisher(JointState, p("state_topic"), 10)
        self.create_timer(0.05, self._publish_state)
        self.create_timer(1.0, self._log_status)
        self.add_on_set_parameters_callback(self._on_params)
        self._last = {"t": time.monotonic(), "confirmed": 0, "fb": 0}

        self.get_logger().info(f"iface={self.iface} esc_id={self.esc_id} target={self.target_hz} Hz "
                               f"(= {self.target_hz * 60 * self.gear_ratio:.0f} rpm) window={window} "
                               f"kp={self.kp} ki={self.ki} limit={self.limit} run_on_start={self.enabled}")
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="m3508-loop", daemon=True)
        self.thread.start()

    def _new_speed(self):
        return mc.SpeedProfile([0], self.limit, self.kp, self.ki, self.accel)

    def _on_trigger(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info(f"trigger={int(msg.data)}")

    def _on_params(self, params):
        for prm in params:
            if prm.name == "target_hz":
                self.target_hz = float(prm.value)
                self.get_logger().info(f"target_hz -> {self.target_hz} Hz")
            elif prm.name in ("kp", "ki", "accel", "max_current"):
                setattr(self, {"max_current": "limit"}.get(prm.name, prm.name), prm.value)
                self.speed.kp, self.speed.ki, self.speed.accel = self.kp, self.ki, self.accel
                self.speed.limit = self.limit
            else:
                return SetParametersResult(successful=False, reason=f"{prm.name}: restart to change")
        return SetParametersResult(successful=True)

    def _loop(self):
        link, speed = self.link, self.speed
        while self.running:
            now = time.monotonic()
            goal = self.target_hz * 60.0 * self.gear_ratio if self.enabled else 0.0
            if link.rpm_mean is not None and link.fb_frames != self.fb_seen:
                self.fb_seen = link.fb_frames
                if not self.enabled and abs(link.rpm_mean) < mc.STUCK_RPM and abs(speed.target) < 1.0:
                    speed = self.speed = self._new_speed()   # trigger=0 且已停稳: 不出力, 下次启动重新脱困
                    self.output = 0
                else:
                    # 脚本 __call__ 同款: 取整后的输出要存回 speed.output, update() 用它做下一拍的预测
                    self.output = speed.output = round(speed.update(goal, link.rpm_mean, now))
            link.service(self.output, now)
            deadline = link.send_deadline()
            self.io.wait(0.01 if deadline is None else max(0.0, min(0.01, deadline - time.monotonic())))

    def _publish_state(self):
        link = self.link
        if link.rpm is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["m3508"]
        msg.position = [link.angle * 2.0 * math.pi / 8192.0 / self.gear_ratio]
        msg.velocity = [link.rpm / self.gear_ratio * 2.0 * math.pi / 60.0]
        msg.effort = [float(link.current)]
        self.state_pub.publish(msg)

    def _log_status(self):
        link, sp = self.link, self.speed
        now = time.monotonic()
        dt = max(now - self._last["t"], 1e-6)
        tx = (link.confirmed - self._last["confirmed"]) / dt
        fb = (link.fb_frames - self._last["fb"]) / dt
        self._last.update(t=now, confirmed=link.confirmed, fb=link.fb_frames)
        meas = "no feedback" if link.rpm_mean is None else \
            f"rpm {link.rpm_mean:6.0f} ({link.rpm_mean / self.gear_ratio / 60:.2f} Hz)  I {mc.amps(link.current):4.1f}A  {link.temp}C"
        wire = "-" if link.wire_value is None else link.wire_value
        self.get_logger().info(f"{'on ' if self.enabled else 'off'} goal {self.target_hz:.2f}Hz target {sp.target:6.0f} pred {sp.predicted:6.0f} "
                               f"{meas} | cmd {self.output:6d} wire {wire:>6} | tx {tx:3.0f}/s fb {fb:4.0f}/s"
                               + (f"  [skips {link.skipped}, resyncs {link.resyncs}]" if link.skipped or link.resyncs else "")
                               + ("  WARNING: another sender on cmd id" if link.foreign else ""))

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        link, io = self.link, self.io
        t0 = time.monotonic()
        z0 = link.zero_confirmed
        ok = True
        while link.zero_confirmed - z0 < mc.ZERO_CONFIRMS:
            now = time.monotonic()
            if now - t0 > mc.STOP_TIMEOUT:
                self.get_logger().warn(f"zero current NOT confirmed within {mc.STOP_TIMEOUT:.0f} s (C620 will time out by itself)")
                ok = False
                break
            link.service(0, now)
            deadline = link.send_deadline()
            io.wait(0.05 if deadline is None else min(0.05, max(0.0, deadline - time.monotonic())))
        for _ in range(3):
            try:
                io.send(link.cmd_id, link.frame(0))
            except OSError:
                pass
        io.close()
        if ok:
            self.get_logger().info(f"stop: zero current confirmed after {time.monotonic() - t0:.2f} s")


def main(args=None):
    rclpy.init(args=args)
    node = M3508SpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
