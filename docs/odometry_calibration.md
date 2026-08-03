# QB3rt Odometry & Drive Calibration

End-to-end procedure for calibrating the WAVE ROVER open-loop drivetrain and
checking odometry accuracy. The robot has **no wheel encoders**, so every drive
parameter is open-loop and tuned against an external sensor (ORB-SLAM3 VIO /
the EKF).

All steps use one tool: `scripts/odom_square_test.py`, driven by
`launch/odom_square_test.launch.py`. Pass `bringup:=true` to also start the robot
stack (bridge + robot_state_publisher + joint_state_publisher + EKF + ORB-SLAM3
VIO); omit it if the stack is already running.

## RB3 preflight (before any file edits under `/usr/share`)

`/usr/share` is mounted read-only by default on RB3. If you need to edit files
there (for example deployed configs), run:

```bash
source /root/rover_env.sh
mount | grep ' on /usr '
```

Expected: `/usr` remounted as `rw`. If you only run launch/test commands, this
step is optional.

---

## 1. The drive math (what each parameter does)

`cmd_vel` (`linear.x` m/s, `angular.z` rad/s) becomes two motor commands in the
bridge (`wave_rover_bridge.py`). Two layers:

**Layer A - kinematics -> normalised wheel commands in [-1, 1]:**

```
turn  = angular.z * boost(|v|) * track_width / 2
left  = (linear.x - turn) / max_speed
right = (linear.x + turn) / max_speed
```

- **`max_speed`** - speed (m/s) that maps to full output. Sets the overall scale:
  too low overdrives, too high underdrives.
- **`boost(|v|)`** - exponential skid-steer turn gain
  `1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)`. Peak at crawl, decays
  toward 1 as speed rises (rolling wheels scrub less).
- **`track_width`** - effective wheel separation (geometry).

**Layer B - per-wheel conditioning -> firmware command:**

```
left  *= (1 - straight_trim);  right *= (1 + straight_trim)   # imbalance trim
left, right = clamp(left, right, -1, 1)
left, right = friction_floor(left, right)                     # static friction
L = left  * motor_cmd_max;  R = right * motor_cmd_max         # firmware range
send {"T":1,"L":L,"R":R}
```

- **`straight_trim`** - corrects left/right motor imbalance. `+` curves left
  (fixes a rover veering right), `-` curves right. Typical +-0.2.
- **`motor_deadband`** - static-friction floor: any nonzero command is lifted to at
  least this magnitude so the wheels actually start moving (critical for turning
  in place, where the differential is tiny). `|cmd|` in (0,1] -> [deadband, 1].
- **`motor_cmd_max`** - firmware full-scale. **The encoder-less WAVE ROVER `T:1`
  command takes -0.5..0.5, where 0.5 = 100% PWM.** Sending 1.0 drives the firmware
  out of range (it *slows/stalls* above 0.5). So this MUST be `0.5`. (Encoder
  models like the UGV02 interpret `T:1` as m/s - set differently there.)

> **Why this bug bites:** before `motor_cmd_max` was added the bridge sent the raw
> [-1, 1] command, so fractions above 0.5 were physically *slower*, not faster.
> The speed sweep looked nonsensical until this was fixed.

Calibrate in this order, because each layer depends on the ones below it:
**`motor_deadband` -> `max_speed` -> `straight_trim` -> `spin_boost_max`/`spin_boost_k` -> square test.**

---

## 2. Prerequisites

- ~4-5 m of clear floor for the speed/straight runs; ~2 x 2 m for the square.
- ORB-SLAM3 VIO must be publishing `/vio/odometry` (OV9282 + RB3 IMU) — it and
  the EKF are the rulers. The test waits up to ~25 s for it. Do the figure-8
  init + stand-still until `/vio/ready` before trusting speed numbers (see
  [orbslam3_calibration.md](orbslam3_calibration.md)).
- The RB3 IMU must be publishing `/imu/data` (started by `base.launch.py` with
  `bringup:=true`). It is the EKF's `imu0` gyro and the **only** trustworthy
  yaw source for `mode:=turn` — VIO alone under-observes rotation. Verify with
  `ros2 topic hz /imu/data`.
- `driver_max_speed` in the test config **must match** `max_speed` in
  `vendor_overrides/wave_rover_controller/wave_rover_bridge.yaml` (and the
  mirrors in `config/odom_square_test.yaml`). The sweep commands
  `linear.x = fraction * driver_max_speed`, so `fraction` == the true motor
  fraction only if they agree.

> **/odom is never used as a reference.** It is dead-reckoned from the commands, so
> it just echoes the commanded speed/path and would report a falsely perfect
> result. The test explicitly excludes it from all calibration; only VIO / EKF
> count. It is still *recorded* for comparison.

---

## 3. `motor_deadband` (static-friction floor)

Ramps the raw motor fraction until the wheels break static friction. The bridge's
own floor is auto-disabled for the sweep (set back to 0.0 over a live parameter
call) so it doesn't mask the result.

```
ros2 launch QB3rt odom_square_test.launch.py mode:=deadband bringup:=true
```

Each step holds a fraction for `step_hold` s; when measured motion exceeds
`move_threshold` (default 2 cm via VIO/EKF), that fraction is the breakaway point.

Output ends with:

```
RESULT: wheels break free at motor fraction ~ 0.12
        recommended motor_deadband: 0.12
```

Set it in `vendor_overrides/wave_rover_controller/wave_rover_bridge.yaml`
(`motor_deadband: 0.12`).

Tuning knobs: `deadband_step` (resolution), `deadband_max` (raise if no breakaway),
`step_hold`, `move_threshold`.

---

## 4. `max_speed` (overall speed scale)

Two ways; **`mode:=straight` is the definitive one**, `mode:=speed` is a quick scan.

### 4a. Quick scan - `mode:=speed`

Sweeps fractions, holds each on a long single-direction baseline, and measures the
steady-state speed by VIO **position displacement** (robust to VIO twist noise).

```
ros2 launch QB3rt odom_square_test.launch.py mode:=speed bringup:=true
```

Speed vs fraction is **affine, not through the origin** (nothing moves until static
friction is cleared):

```
v ~= slope * fraction + intercept = V_full * (fraction - f0)
```

The report fits a line, extrapolates to fraction = 1.0 to get **`V_full`**, and
recommends that as `max_speed`. Watch the `implied_Vfull` column:

- **rising** across the sweep -> you're still near breakaway; raise `speed_max` to
  capture the linear region.
- **collapsing** on the top rows -> VIO is losing tracking at speed; lower
  `speed_max` and trust the lower, stable rows.

Defaults are VIO-friendly: `speed_alternate: false` (one direction = clean, long
baseline), `speed_hold: 3.5 s`, `accel_skip: 0.8 s`.

### 4b. Definitive - `mode:=straight`

One long straight leg; VIO net displacement / duration = actual speed. Use as much
runway as you have:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=straight bringup:=true \
    side_length:=3.0 linear_speed:=0.42
```

Report gives measured distance, actual speed, and the `max_speed` that makes
commanded == actual. Set it in
`vendor_overrides/wave_rover_controller/wave_rover_bridge.yaml`, then re-run to
verify the ratio is ~1.0.

> **Sanity-check the ruler first:** if VIO-measured speed seems implausibly low,
> run `mode:=vio_check` (Section 7) and hand-push the rover a known distance. We
> confirmed VIO scale was good (~0.93 m reported for a 1.0 m push), i.e. the
> rover's real top speed (~0.4 m/s) is genuinely lower than it looks by eye.

---

## 5. `straight_trim` (left/right imbalance)

With `max_speed` set, drive straight and read the heading drift:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=straight bringup:=true \
    side_length:=2.0 linear_speed:=0.42
```

Look at `head_err_deg` (VIO/EKF). A rover that veers right shows up as heading
drift; increase `straight_trim` (positive curves left) to cancel it. It's roughly
linear, so interpolate between two runs:

- run with trim `t1` -> error `e1`, run with `t2` -> error `e2`
- target trim ~= `t1 + (t2 - t1) * e1 / (e1 - e2)`

We landed on `straight_trim: 0.12`. Set live to iterate quickly:

```
ros2 param set /waverover_bridge straight_trim 0.12
```

then persist it in `vendor_overrides/wave_rover_controller/wave_rover_bridge.yaml`.

---

## 6. Exponential turn gain (`spin_boost_max`, `spin_boost_k`) — via ARC turns

> **A skid-steer can't reliably spin in place.** An in-place rotation forces all
> four wheels to scrub sideways at once (zero rolling) — the highest-torque
> maneuver there is. On a high-friction floor the motors stall, and no amount of
> boost fixes it. So we **never command a pure rotation**: all turns
> (square corners *and* this calibration) are driven as **arcs** — roll forward
> while steering, so the wheels keep rolling and the scrub per corner is small.
> A bonus: with forward motion the wheels are already above the `motor_deadband`
> floor, so the turn differential has a clean, near-linear effect.

The bridge uses an **exponential** speed schedule (2026-07-19):

```
boost(|v|) = 1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)
```

- At standstill: `boost = spin_boost_max` (maximum crawl-turn authority).
- As `|v|` rises: boost decays toward `1.0` (rolling wheels redirect more easily).
- `spin_boost_k` (1/m/s) sets how fast it decays — larger k = less boost at cruise.

Calibrated with **`mode:=turn`**: the rover drives an arc of a known heading
change while rolling at `turn_linear_speed`; rotation is measured by the
**IMU-backed EKF**.

```
ros2 launch QB3rt odom_square_test.launch.py mode:=turn bringup:=true
```

By default it drives one full revolution (`turn_target: 6.2832` rad) at
`angular_speed` while rolling `turn_linear_speed` — a circle of radius
`turn_linear_speed / angular_speed` (default `0.20/0.4 ~ 0.5 m`, so clear ~1.1 m).
In a tight space, sweep a smaller angle:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=turn bringup:=true \
    angular_speed:=0.4 turn_target:=1.5708 direction:=ccw
```

The report prints the effective boost at the test speed and recommends either a
new `spin_boost_k` (preferred, keep max fixed) or a new `spin_boost_max` if the
needed peak is out of k-only range. Mirrors
`driver_spin_boost_max` / `driver_spin_boost_k` in the test config **must match**
the bridge yaml.

```
ros2 param set /waverover_bridge spin_boost_k 1.4
ros2 param set /waverover_bridge spin_boost_max 10.0
```

> **Two-speed fit for k (recommended).** Run `mode:=turn` at two forward speeds,
> read needed boost at each (`boost_now / ratio`), then solve for max and k:
>
> ```
> boost(v) = 1 + (M - 1) * exp(-k * v)
> # from two (v, boost) points: k = ln((b1-1)/(b2-1)) / (v2 - v1)
> #                           M = 1 + (b1-1) * exp(k * v1)
> ```
>
> Persist both in `vendor_overrides/wave_rover_controller/wave_rover_bridge.yaml`.

> **The scrub ceiling (important).** Past ~6.7 boost on carpet the inside wheel
> pins on the friction floor and **anchors**; >=9 can reverse and stall. Crawl
> turns (v→0 → boost→max) sit nearest that ceiling — keep `spin_boost_max`
> honest for your floor.

> **Directional asymmetry.** Turning is ~25–35% stronger one way than the other.
> A single schedule cannot make both cw and ccw hit target open-loop — pick
> values that never *over*-rotate and let the EKF/Nav2 heading closed loop own
> the exact final angle.

**Defaults (bridge yaml):** `spin_boost_max: 10.0`, `spin_boost_k: 1.4`.
Recalibrate after changing floor/tires.

Knobs: `turn_linear_speed`, `turn_target`, `angular_speed`, `direction`.

> **Where the bridge lives:** the canonical `wave_rover_bridge.py` +
> `wave_rover_bridge.yaml` are in
> `/root/QB3rt/vendor_overrides/wave_rover_controller/` and are installed by
> `deploy.sh`. The copies under `/usr/lib` / `/usr/share` are overwritten on
> deploy and reverted by a reflash — never edit them directly.

## 7. `mode:=vio_check` (verify the ruler)

Sanity-checks VIO scale before you trust any speed number. Commands no motion; you
push the rover by hand a known distance and compare:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=vio_check bringup:=true
# push the rover exactly 1.0 m, read the logged net displacement, Ctrl-C
```

If reported displacement ~= the real push, VIO scale is trustworthy.

### `mode:=turn_check` (verify the rotational ruler)

The rotational analogue: before trusting any turn-gain number, confirm the EKF
yaw actually matches reality. Commands no motion; you rotate the rover by hand a
known angle and compare the logged cumulative yaw:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=turn_check bringup:=true
# rotate the rover exactly 360 deg against a floor mark, read cumulative yaw, Ctrl-C
```

If a hand 360 deg reads ~360 deg, the gyro/EKF rotational scale is trustworthy and
any under-rotation seen in `mode:=turn` is real drive behaviour. If it reads low,
the ruler itself is off — suspect IMU mounting tilt (yaw axis not vertical reads
low by ~`cos(tilt)`). Confirm `/imu/data` `angular_velocity.z` while turning.

### `mode:=oneside` (one side full, other static — sanity check)

A behavioural sanity check, not a calibration: drive one wheel side at **full**
and the other at **exactly static** for a fixed time, then read the rotation. This
is the maximum-asymmetry powered turn the rover can do; the static side scrubs, so
expect slip. It answers "how sharply can it actually turn under power, and how
much does it slide?".

```
ros2 launch QB3rt odom_square_test.launch.py mode:=oneside bringup:=true \
    oneside_side:=left oneside_duration:=2.0
```

- `oneside_side`: `left` drives the left side full → turns **cw**; `right` →
  turns **ccw**.
- `oneside_duration`: seconds to hold the drive (default 2.0). The run adds
  `turn_settle` of coast/settle afterwards before measuring.
- The bridge's `motor_deadband` and `straight_trim` are auto-zeroed for the run so
  the static side truly stops (relaunch the bridge to restore them). The
  exponential boost and `track_width` are folded into the commanded twist, so
  `driver_spin_boost_max` / `driver_spin_boost_k` and `driver_track_width`
  **must match** `wave_rover_bridge.yaml`.

The report gives the rotation (deg + rad), mean yaw rate, net base displacement,
and an **implied turn radius** (chord/`2·sin(θ/2)`). A radius near `track_width/2`
(~0.075 m) means it pivoted cleanly about the static side; a much larger radius
means it scrubbed/slid outward instead of pivoting.

---

## 8. Square accuracy test

The headline test once the drive is calibrated:

```
ros2 launch QB3rt odom_square_test.launch.py mode:=square bringup:=true \
    side_length:=1.0 num_sides:=4
```

Per odometry source the report prints:

| column         | meaning                                   | ideal      |
|----------------|-------------------------------------------|------------|
| `close_err_m`  | distance between start and end pose        | 0          |
| `head_err_deg` | heading drift start -> end                 | 0          |
| `path_m`       | total measured path length                 | perimeter  |

A clean square closes (`close_err_m` small) with near-zero `head_err_deg`. Compare
`/vio/odometry`, `/odometry/filtered` and `/odom` to see how well dead reckoning
and the EKF track ground truth.

---

## 9. Results & current calibration

Every run writes a JSON summary (and per-source CSV trajectories for square/
straight) to `results/` (override with `output_dir:=`).

Current calibrated values (`vendor_overrides/wave_rover_controller/
wave_rover_bridge.yaml` — recalibrated **2026-07-13 on the SERIAL transport,
by tape measure**; the 2026-06-29 HTTP-era values baked WiFi latency in as
drivetrain slowness and are kept below only for history):

| parameter        | value | notes                                        |
|------------------|-------|----------------------------------------------|
| `max_speed`      | 0.56  | speed at full output (m/s); tape-measured, was 0.42 over HTTP |
| `motor_cmd_max`  | 0.5   | firmware T:1 full-scale — RE-CONFIRMED on serial 2026-07-13: sending 1.0 adds no speed and veers hard |
| `motor_deadband` | 0.12  | static-friction breakaway fraction           |
| `straight_trim`  | 0.12  | + curves left; 1.5 deg over 2 m at 0.3 m/s (worsens with speed: ~17 deg at full — Nav2 speeds are fine) |
| `spin_boost_max` | 10.0  | peak boost at v=0 in exponential schedule; carpet anchor ~6.7, stall >=9 |
| `spin_boost_k`   | 1.4   | decay rate (1/m/s); must match `odom_square_test.yaml` mirrors |
| `track_width`    | 0.15  | effective wheel separation (m)                |

> **2026-07-13 session lessons:** (1) calibrate with a TAPE MEASURE — VIO is
> unreliable while the robot drives on the floor and mis-recommended max_speed
> by up to 13x. (2) The `driver_*` mirrors exist in BOTH
> `config/odom_square_test.yaml` AND as hardcoded defaults in
> `launch/odom_square_test.launch.py` — the launch defaults win; update both.
> (3) The onboard oneside ceiling is healthy (~27 deg/s with the outside side
> at true full PWM), so mid-arc under-rotation is a torque/scrub limit, not a
> motor fault.

> **Keep the test mirrors in sync with the bridge:** `driver_max_speed` ==
> `max_speed`, `driver_spin_boost_max` == `spin_boost_max`,
> `driver_spin_boost_k` == `spin_boost_k`, `driver_track_width` ==
> `track_width`. If you change one, change its mirror, or the sweeps mis-scale and
> the turn recommendation / one-side twist come out wrong.
