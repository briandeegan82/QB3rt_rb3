#!/usr/bin/env python3
"""cmd_vel -> Wave Rover bridge with speed-scaled open-loop control, dead-reckoning
odometry and tank-drive joystick.

Subscribes to /cmd_vel (geometry_msgs/Twist), converts it to per-wheel motor
fractions through a CALIBRATED open-loop drive model (see below), and sends them
to the rover as JSON ({"T":1,"L":..,"R":..}). The rover firmware accepts the
identical command set over either transport:

    transport: http    -> HTTP over WiFi   (http://<rover_ip>/js?json=...)
    transport: serial  -> USB/UART @115200 (writes the JSON line to a tty)

Drive model (docs/odometry_calibration.md has the full calibration procedure):

  Layer A - skid-steer kinematics -> per-wheel speeds (m/s):
      boost = 1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)
      turn  = angular.z * boost * track_width / 2
      v_l   = (linear.x - turn)
      v_r   = (linear.x + turn)
      if max(|v_l|,|v_r|) > max_speed: scale BOTH down (preserves curvature).
      # straight trim is applied per-wheel in Layer B (it is speed-scheduled on
      # the post-saturation forward speed, so it lives with the feedforward):
      trim  = straight_trim + straight_trim_slope * (|v| - straight_trim_ref_speed)
      v_l  *= (1 - trim) ;  v_r *= (1 + trim)

    Turn gain is exponential in forward speed: at standstill boost = spin_boost_max
    (maximum scrub compensation), decaying toward 1.0 as |v| rises (rolling wheels
    redirect more easily). `spin_boost_k` (1/m/s) sets the decay rate — larger k
    drops the boost faster with speed. This replaces the old linear schedule
    (spin_boost + slope * (|v| - ref)), which needed a separate zero-speed floor
    and could not naturally give high crawl-turn authority without overshooting
    at cruise.

    `straight_trim` corrects a left/right drivetrain imbalance. The imbalance is
    NOT a constant fraction of speed (drift grows toward full throttle), so the
    trim is likewise scheduled: `straight_trim` is the value at
    `straight_trim_ref_speed` and `straight_trim_slope` (per m/s) extends it.
    slope 0.0 = constant trim. Calibrate with the odom_square_test `trim` mode.

  Layer B - speed-scaled feedforward -> motor fraction in [-1, 1]:
      f(v) = sign(v) * (motor_deadband + (1 - motor_deadband) * |v| / max_speed)

    This inverts the measured speed law of the drivetrain, which is affine and
    NOT through the origin (nothing moves until static friction is cleared):
        v_actual ~= V_full * (fraction - f0)
    with f0 = motor_deadband and V_full = max_speed. A small commanded speed is
    lifted onto the friction floor instead of being sent as an unusably small
    fraction, and every speed in between lands on the measured line - the
    open-loop gain is correct across the whole speed range, not just at full
    output.

  Layer C - firmware scaling:
      L = f_l * motor_cmd_max ; R = f_r * motor_cmd_max
    The encoder-less WAVE ROVER "T":1 command takes -0.5..0.5 where 0.5 = 100%
    PWM; values beyond 0.5 drive the firmware out of range and SLOW/stall the
    motors. motor_cmd_max MUST stay 0.5 on this platform.

Also:
  - Publishes dead-reckoning /odom (and optionally odom->base_link TF).
    Integrates the post-saturation commanded velocities through the same
    calibrated model (no encoders), so pair with an EKF/SLAM to correct drift.
    Leave publish_tf false when robot_localization owns odom->base_footprint.
  - Subscribes to /joy for tank-drive teleoperation. While the deadman button
    is held the left/right stick Y drive the wheel sides directly (through the
    same feedforward, so stick fraction == speed fraction). Releasing the
    deadman hands control back to cmd_vel (Nav2 or teleop_twist_joy).

Run with:
    ros2 run wave_rover_controller wave_rover_bridge.py
or via the launch file (loads config/wave_rover_bridge.yaml):
    ros2 launch wave_rover_controller wave_rover_bridge_launch.py

NOTE: this file is deployed from the QB3rt project
(/root/QB3rt/vendor_overrides/wave_rover_controller/) by /root/QB3rt/deploy.sh.
Edit it THERE - the copy under /usr/lib is overwritten on deploy and lost on
reflash.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import tf2_ros


class HttpTransport:
    """Send JSON commands over HTTP (WiFi).

    Uses `requests` if installed, otherwise falls back to the stdlib `urllib`
    so it works on the QIR image with no extra packages.
    """

    name = 'http'

    def __init__(self, node):
        self._ip = node.get_parameter('rover_ip').value
        self._timeout = node.get_parameter('request_timeout').value
        self._session = None
        try:
            import requests
            self._session = requests.Session()
            self._send_impl = self._send_requests
            backend = 'requests'
            self.exceptions = (requests.exceptions.RequestException,)
        except ImportError:
            import urllib.request
            self._urlopen = urllib.request.urlopen
            self._send_impl = self._send_urllib
            backend = 'urllib'
            # urllib.error.URLError and socket.timeout both subclass OSError.
            self.exceptions = (OSError,)
        self.summary = f'http://{self._ip} (timeout {self._timeout}s, {backend})'

    def _send_requests(self, cmd):
        self._session.get(
            f'http://{self._ip}/js?json={cmd}', timeout=self._timeout)

    def _send_urllib(self, cmd):
        self._urlopen(
            f'http://{self._ip}/js?json={cmd}', timeout=self._timeout)

    def send(self, cmd):
        self._send_impl(cmd)

    def close(self):
        if self._session is not None:
            self._session.close()


class _RawSerial:
    """Minimal write-only serial port via termios (pyserial fallback).

    Used when pyserial is not installed (e.g. the QIR Yocto image). We only ever
    write newline-terminated JSON, so a raw fd in 8N1 raw mode is sufficient.
    """

    def __init__(self, port, baud):
        import os
        import termios
        baud_const = getattr(termios, f'B{baud}', None)
        if baud_const is None:
            raise ValueError(f'unsupported baud_rate {baud} for termios fallback')
        self._os = os
        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0  # iflag: no input processing
        attrs[1] = 0  # oflag: no output processing
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0  # lflag: raw (no canonical/echo)
        attrs[4] = baud_const  # ispeed
        attrs[5] = baud_const  # ospeed
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)

    def write(self, data):
        self._os.write(self._fd, data)

    def close(self):
        self._os.close(self._fd)


class SerialTransport:
    """Send JSON commands over USB/UART (pyserial, falling back to termios)."""

    name = 'serial'

    def __init__(self, node):
        self._port = node.get_parameter('serial_port').value
        self._baud = node.get_parameter('baud_rate').value
        try:
            import serial
            # timeout=0 -> non-blocking writes; we never read back.
            self._ser = serial.Serial(self._port, self._baud, timeout=0)
            backend = 'pyserial'
            self.exceptions = (serial.SerialException, OSError)
        except ImportError:
            self._ser = _RawSerial(self._port, self._baud)
            backend = 'termios'
            self.exceptions = (OSError,)
        self.summary = f'{self._port} @ {self._baud} ({backend})'

    def send(self, cmd):
        self._ser.write((cmd + '\n').encode('ascii'))

    def close(self):
        self._ser.close()


TRANSPORTS = {t.name: t for t in (HttpTransport, SerialTransport)}

# Live-tunable drive parameters (ros2 param set /waverover_bridge <name> <value>).
TUNABLE = (
    'max_speed', 'track_width', 'spin_boost_max', 'spin_boost_k',
    'straight_trim', 'straight_trim_slope', 'straight_trim_ref_speed',
    'motor_deadband', 'motor_cmd_max',
)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('waverover_bridge')

        # Transport: 'http' (WiFi) or 'serial' (USB/UART).
        self.declare_parameter('transport', 'http')

        # HTTP params.
        # rover_ip: 192.168.4.1 in AP mode (rover's own hotspot), or the
        # router-assigned IP shown on the OLED when in STA mode.
        self.declare_parameter('rover_ip', '192.168.4.1')
        # Per-command HTTP timeout, seconds.
        self.declare_parameter('request_timeout', 0.2)

        # Serial params.
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # --- Calibrated drive model (docs/odometry_calibration.md) ---
        # Defaults mirror the calibrated wave_rover_bridge.yaml (2026-07-13
        # tape-measured campaign) so running WITHOUT the yaml does not silently
        # revert the calibration. The yaml stays the canonical record.
        # Effective wheel separation (m).
        self.declare_parameter('track_width', 0.15)
        # Ground speed (m/s) at full motor output (V_full from mode:=straight).
        self.declare_parameter('max_speed', 0.56)
        # Static-friction breakaway fraction f0 (mode:=deadband).
        self.declare_parameter('motor_deadband', 0.12)
        # Left/right imbalance trim; + curves left (mode:=straight). This is the
        # trim at straight_trim_ref_speed; straight_trim_slope schedules it on
        # speed (below).
        self.declare_parameter('straight_trim', 0.12)
        # Speed scheduling of the straight trim: trim change per m/s of |linear.x|
        # relative to straight_trim_ref_speed. The needed L/R correction is NOT
        # constant with speed (measured drift grows from ~1.5 deg/2 m at 0.3 m/s
        # to much larger at full throttle), so a single constant can't zero drift
        # at both ends. 0.0 = constant trim (the pre-2026-07-19 behavior).
        # Calibrate with mode:=trim at two linear_speed values v1 < v2:
        #   slope = (trim_at_v2 - trim_at_v1) / (v2 - v1)
        self.declare_parameter('straight_trim_slope', 0.0)
        # Linear speed (m/s) the straight_trim value was measured at.
        self.declare_parameter('straight_trim_ref_speed', 0.20)
        # Exponential skid-steer turn gain (replaces constant/linear spin_boost):
        #   boost(|v|) = 1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)
        # At v=0: boost = spin_boost_max (crawl/near-pivot authority). As |v|
        # rises, boost decays toward 1. Past ~6.7 on carpet the inside wheel can
        # pin on the friction floor and anchor; >=9 can reverse/stall — keep
        # spin_boost_max below that for your floor, or accept that crawl turns
        # sit near the scrub ceiling by design.
        self.declare_parameter('spin_boost_max', 10.0)
        # Decay rate (1/(m/s)). Larger = boost drops faster with speed.
        # Default 1.4 with max 10 → ~7.8 at |v|=0.20. Calibrate with mode:=turn
        # at two speeds (carpet crawl can anchor above ~6.7).
        self.declare_parameter('spin_boost_k', 1.4)
        # Firmware full-scale for the T:1 command. The encoder-less WAVE ROVER
        # takes -0.5..0.5 (0.5 = 100% PWM); beyond that it slows/stalls. Keep 0.5.
        self.declare_parameter('motor_cmd_max', 0.5)

        # Safety watchdog: if no cmd_vel arrives within this many seconds, send a
        # stop. Set to 0.0 to disable.
        self.declare_parameter('cmd_timeout', 0.5)

        # Odometry frames.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # False: robot_localization owns odom->base_footprint on QB3rt. Enabling
        # this gives base_link two parents and detaches Nav2 from the sensors.
        self.declare_parameter('publish_tf', False)

        # Tank-drive joystick.
        # Axis indices into sensor_msgs/Joy.axes[].  The scale flips sign:
        # most controllers report stick-up as -1, so scale = -1.0 gives
        # positive = forward.  Set joy_deadman_button to -1 to require no button.
        self.declare_parameter('joy_left_axis', 1)       # left stick Y
        self.declare_parameter('joy_left_scale', -1.0)
        self.declare_parameter('joy_right_axis', 3)      # right stick Y
        self.declare_parameter('joy_right_scale', -1.0)
        self.declare_parameter('joy_deadman_button', 5)  # RB (matches existing teleop)

        for name in TUNABLE:
            setattr(self, name, self.get_parameter(name).value)
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value

        self._joy_left_axis   = self.get_parameter('joy_left_axis').value
        self._joy_left_scale  = self.get_parameter('joy_left_scale').value
        self._joy_right_axis  = self.get_parameter('joy_right_axis').value
        self._joy_right_scale = self.get_parameter('joy_right_scale').value
        self._joy_deadman_btn = self.get_parameter('joy_deadman_button').value

        self.add_on_set_parameters_callback(self._on_params)

        transport = self.get_parameter('transport').value
        if transport not in TRANSPORTS:
            raise ValueError(
                f"Unknown transport '{transport}', "
                f"expected one of {sorted(TRANSPORTS)}")
        self.transport = TRANSPORTS[transport](self)

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Joy,   'joy',     self._joy_cb,     10)

        # Watchdog: stop rover if cmd_vel goes silent and joy is not active,
        # or if the /joy stream itself dies while tank drive is engaged.
        self._last_cmd_time = self.get_clock().now()
        self._last_joy_time = self.get_clock().now()
        self._stopped = True
        self._joy_active = False
        if self.cmd_timeout > 0.0:
            self.create_timer(self.cmd_timeout / 2.0, self._watchdog_cb)

        # Dead-reckoning odometry state.
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_th  = 0.0
        self._odom_vx  = 0.0
        self._odom_vth = 0.0
        self._last_odom_time = None

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Publish odom at 20 Hz so TF stays fresh even when the rover is idle.
        self.create_timer(0.05, self._odom_timer_cb)

        self.get_logger().info(
            f'waverover_bridge ready: transport={self.transport.name} '
            f'-> {self.transport.summary} | '
            f'max_speed={self.max_speed} deadband={self.motor_deadband} '
            f'trim={self.straight_trim}'
            f'{"" if self.straight_trim_slope == 0.0 else f" slope={self.straight_trim_slope}/m/s @ {self.straight_trim_ref_speed}"} '
            f'spin_boost=1+({self.spin_boost_max}-1)*exp(-{self.spin_boost_k}*|v|) '
            f'motor_cmd_max={self.motor_cmd_max} track={self.track_width} | '
            f'cmd_timeout={self.cmd_timeout}s | '
            f'odom {self._odom_frame}->{self._base_frame} '
            f'publish_tf={self._publish_tf} | '
            f'tank-drive axes L={self._joy_left_axis} R={self._joy_right_axis} '
            f'deadman_btn={self._joy_deadman_btn}')

    # ── parameter callback (live calibration tuning) ─────────────────────────

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in TUNABLE:
                setattr(self, p.name, p.value)
                self.get_logger().info(f'{p.name} -> {p.value}')
        return SetParametersResult(successful=True)

    # ── drive model ───────────────────────────────────────────────────────────

    def _boost_at(self, v):
        """Exponential skid-steer turn gain vs forward speed.

        boost = 1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)

        High at crawl (static-friction / scrub dominated), asymptotes to 1.0 as
        the wheels roll freely. spin_boost_k <= 0 falls back to a constant
        spin_boost_max (no decay).
        """
        peak = max(1.0, float(self.spin_boost_max))
        k = float(self.spin_boost_k)
        if k <= 0.0:
            return peak
        return 1.0 + (peak - 1.0) * math.exp(-k * abs(v))

    def _trim_at(self, v):
        """Speed-scheduled left/right straight trim.

        Clamped to (-0.9, 0.9) so the (1 -/+ trim) wheel factors stay same-sign
        and bounded regardless of a mis-set slope.
        """
        trim = (self.straight_trim
                + self.straight_trim_slope * (abs(v) - self.straight_trim_ref_speed))
        return max(-0.9, min(0.9, trim))

    def _wheel_speeds(self, v, w):
        """cmd_vel -> saturated per-wheel target speeds (m/s) + effective boost.

        Saturation scales BOTH wheels by the same factor so the commanded
        curvature (turn radius) is preserved at the speed limit.
        """
        boost = self._boost_at(v)
        turn = w * boost * self.track_width / 2.0
        v_l = v - turn
        v_r = v + turn
        peak = max(abs(v_l), abs(v_r))
        if peak > self.max_speed > 0.0:
            scale = self.max_speed / peak
            v_l *= scale
            v_r *= scale
        return v_l, v_r, boost

    def _feedforward(self, v):
        """Wheel speed (m/s) -> motor fraction, inverting the affine speed law.

        v_actual = max_speed * (f - motor_deadband) / (1 - motor_deadband)
        so f(v) = motor_deadband + (1 - motor_deadband) * |v| / max_speed.
        """
        if abs(v) < 1e-4 or self.max_speed <= 0.0:
            return 0.0
        f = (self.motor_deadband
             + (1.0 - self.motor_deadband) * min(abs(v) / self.max_speed, 1.0))
        return math.copysign(min(f, 1.0), v)

    def _drive(self, v_l, v_r):
        """Per-wheel speeds -> conditioned motor fractions, sent to the rover."""
        trim = self._trim_at((v_l + v_r) / 2.0)
        left = self._feedforward(v_l * (1.0 - trim))
        right = self._feedforward(v_r * (1.0 + trim))
        self._send(left, right)

    # ── cmd_vel (Nav2 / teleop_twist_joy) ────────────────────────────────────

    def _cmd_vel_cb(self, msg):
        # Always update the timestamp so the watchdog knows cmd_vel is alive,
        # even while the joystick has priority.
        self._last_cmd_time = self.get_clock().now()
        self._stopped = (abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3)

        if self._joy_active:
            return  # joystick has priority; ignore the motion command

        v_l, v_r, boost = self._wheel_speeds(msg.linear.x, msg.angular.z)

        # Integrate what was actually commanded post-saturation, mapped back
        # through the calibrated model, not the raw request.
        vx = (v_l + v_r) / 2.0
        vth = (v_r - v_l) / (boost * self.track_width)
        self._integrate_odom(vx, vth)

        self._drive(v_l, v_r)

    # ── tank-drive joystick ───────────────────────────────────────────────────

    def _joy_cb(self, msg):
        self._last_joy_time = self.get_clock().now()
        deadman = (
            self._joy_deadman_btn < 0
            or (self._joy_deadman_btn < len(msg.buttons)
                and msg.buttons[self._joy_deadman_btn])
        )

        if not deadman:
            if self._joy_active:
                self._joy_active = False
                self._stopped = True
                self._odom_vx  = 0.0
                self._odom_vth = 0.0
                self._send(0.0, 0.0)
                self.get_logger().info('Tank drive released — cmd_vel resumed')
            return

        if not self._joy_active:
            self.get_logger().info('Tank drive active')
        self._joy_active = True

        # Stick fraction == speed fraction; the shared feedforward puts it on
        # the friction floor so a small deflection actually creeps.
        v_l = self._axis(msg, self._joy_left_axis,  self._joy_left_scale) * self.max_speed
        v_r = self._axis(msg, self._joy_right_axis, self._joy_right_scale) * self.max_speed

        # Derive vx / vth for odom integration. The boost divisor converts the
        # wheel differential to the actual (scrub-limited) yaw rate, same as
        # the forward model.
        vx = (v_l + v_r) / 2.0
        vth = (v_r - v_l) / (self._boost_at(vx) * self.track_width)
        self._integrate_odom(vx, vth)

        self._drive(v_l, v_r)

    def _axis(self, msg, idx, scale):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx]) * scale

    # ── watchdog ─────────────────────────────────────────────────────────────

    def _watchdog_cb(self):
        now = self.get_clock().now()
        if self._joy_active:
            # Deadman held but the /joy stream died (joystick unplugged,
            # joy_node crash, teleop link drop): no release message will ever
            # arrive, so without this check the rover would keep driving the
            # last wheel command forever.
            if (now - self._last_joy_time).nanoseconds * 1e-9 > self.cmd_timeout:
                self.get_logger().warn('joy stream lost — stopping rover')
                self._joy_active = False
                self._stopped = True
                self._odom_vx  = 0.0
                self._odom_vth = 0.0
                self._send(0.0, 0.0)
            return
        if self._stopped:
            return
        elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn('cmd_vel timeout — stopping rover')
            self._stopped = True
            self._odom_vx  = 0.0
            self._odom_vth = 0.0
            self._send(0.0, 0.0)

    # ── odometry ──────────────────────────────────────────────────────────────

    def _integrate_odom(self, vx, vth):
        now = self.get_clock().now()
        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9
            # After a watchdog stop or a joy-priority period the gap since the
            # last command can be minutes; integrating the NEW velocity across
            # it would teleport the dead-reckoned pose. The robot was stopped
            # during the gap, so skip integration for stale intervals.
            if dt <= 0.5:
                self._odom_x  += vx * math.cos(self._odom_th) * dt
                self._odom_y  += vx * math.sin(self._odom_th) * dt
                self._odom_th += vth * dt
        self._odom_vx  = vx
        self._odom_vth = vth
        self._last_odom_time = now

    def _odom_timer_cb(self):
        now = self.get_clock().now()

        q = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self._odom_th / 2.0),
            w=math.cos(self._odom_th / 2.0),
        )

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame
            t.child_frame_id  = self._base_frame
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame
        odom.pose.pose.position.x  = self._odom_x
        odom.pose.pose.position.y  = self._odom_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = self._odom_vx
        odom.twist.twist.angular.z = self._odom_vth

        # Diagonal pose covariance [x, y, z, rx, ry, yaw]. Open-loop dead
        # reckoning: the pose drifts unbounded, so keep it weak.
        odom.pose.covariance[0]  = 0.5   # x   (m²)
        odom.pose.covariance[7]  = 0.5   # y   (m²)
        odom.pose.covariance[35] = 0.5   # yaw (rad²)
        # Diagonal twist covariance [vx, vy, vz, vrx, vry, vyaw]. vx now comes
        # through the calibrated affine model, so it is trustworthy enough for
        # the EKF to fuse (ekf.yaml odom0); vyaw stays weak (scrub makes the
        # commanded yaw rate optimistic - the gyro owns rotation).
        odom.twist.covariance[0]  = 0.04  # vx   (m/s)²
        odom.twist.covariance[35] = 0.5   # vyaw (rad/s)²

        self._odom_pub.publish(odom)

    # ── motor output ──────────────────────────────────────────────────────────

    def _send(self, left, right):
        left  = max(-1.0, min(1.0, left)) * self.motor_cmd_max
        right = max(-1.0, min(1.0, right)) * self.motor_cmd_max

        cmd = f'{{"T":1,"L":{left:.3f},"R":{right:.3f}}}'
        self.get_logger().debug(f'-> {cmd}')
        try:
            self.transport.send(cmd)
        except self.transport.exceptions as e:
            self.get_logger().warn(
                f'Send failed: {e}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.transport.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
