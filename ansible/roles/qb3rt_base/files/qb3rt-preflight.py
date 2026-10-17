#!/usr/bin/env python3
"""qb3rt-preflight — hardware sanity check run BEFORE any ROS launch.

WHY: the convention probe kept burning 10+ minutes on failures that are
detectable in seconds over raw serial: stale bridges holding /dev/wave_rover
(two writers interleave garbage the ESP32 ignores), flapping CP2102 adapters,
dead USB buses (missing renesas_usb_fw.mem), and udev links pointing at the
wrong adapter after hardware swaps. This script catches all of those with no
ROS dependency, so a failed unit is diagnosed BEFORE the stack is launched.

Checks (each prints PASS/FAIL/WARN; exit 1 on any FAIL):
  1. kill switch:  no process holds /dev/wave_rover or /dev/rplidar
  2. symlinks:     /dev/wave_rover and /dev/rplidar exist and resolve
  3. stability:    the resolved ttyUSB nodes stay bound for 2 s
                   (a flapping adapter re-enumerates and the node vanishes)
  4. lidar alive:  RPLIDAR GET_INFO gets an 0xA5 response at 460800 baud
  5. rover alive:  ESP32 emits a boot banner after a DTR/RTS reset toggle;
                   WARN (not FAIL) if silent — some firmware boots quiet

Run:  qb3rt-preflight            (destructive: stops all qb3rt stacks first)
Exit: 0 = hardware path is sound, 1 = something failed (read output)
"""

import os
import subprocess
import sys
import time

fails = []


def check(name, ok, detail="", fatal=True):
    tag = "PASS" if ok else ("FAIL" if fatal else "WARN")
    print(f"  {tag:4}  {name}" + (f"  — {detail}" if detail else ""))
    if not ok and fatal:
        fails.append(name)
    return ok


def stop_stacks():
    """Tear down every ROS thing that could hold a serial port. Mirrors
    qb3rt-down but also reaps the probe's old transient units and bare
    processes — this script may run on units that predate qb3rt-stack@."""
    subprocess.run(
        "systemctl list-units --no-legend 'qb3rt-*' 2>/dev/null | "
        "awk '{print $1}' | xargs -r systemctl stop 2>/dev/null; "
        "systemctl list-units --no-legend 'qb3rt-*' | awk '{print $1}' | "
        "xargs -r systemctl reset-failed 2>/dev/null; "
        "pkill -f 'ros2 launch' 2>/dev/null; "
        "pkill -f 'waverover_bridge|wave_rover_bridge' 2>/dev/null; "
        "pkill -f 'rplidar' 2>/dev/null; "
        "pkill -f 'imu_container' 2>/dev/null; true",
        shell=True,
    )
    time.sleep(2)


def port_holders(path):
    """PIDs holding the device open (via /proc/*/fd — no lsof/fuser dep)."""
    try:
        target = os.path.realpath(path)
    except OSError:
        return []
    holders = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(f"{fd_dir}/{fd}") == target:
                        holders.append(int(pid))
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return holders


def main():
    print("qb3rt-preflight")
    stop_stacks()

    # --- 1. serial ports free -------------------------------------------
    # Stack teardown cascades SIGINT -> SIGTERM -> SIGKILL (up to ~25 s);
    # poll so a slow-clean-shutdown bridge doesn't read as "held".
    for dev in ("/dev/wave_rover", "/dev/rplidar"):
        holders = port_holders(dev) if os.path.exists(dev) else []
        t0 = time.time()
        while holders and time.time() - t0 < 25:
            time.sleep(1)
            holders = port_holders(dev) if os.path.exists(dev) else []
        check(f"{dev} not held", not holders,
              f"still held by pid(s) {holders} — inspect manually"
              if holders else "")

    # --- 2. symlinks ------------------------------------------------------
    resolved = {}
    for dev in ("/dev/wave_rover", "/dev/rplidar"):
        if os.path.exists(dev):
            resolved[dev] = os.path.realpath(dev)
            check(f"{dev} resolves", True, resolved[dev])
        else:
            check(f"{dev} resolves", False,
                  "missing — udev rules not matched (adapter unplugged? "
                  "serials changed?) — re-run stamp_unit.sh")
    if len(resolved) < 2:
        print("\nFAIL: serial symlinks missing — nothing else can work.")
        return 1

    # --- 3. stability ------------------------------------------------------
    time.sleep(2.0)
    for dev, tty in resolved.items():
        ok = os.path.exists(tty) and os.path.realpath(dev) == tty
        check(f"{dev} stable", ok,
              "adapter re-enumerating — reseat the USB cable/adapter"
              if not ok else "")
    if fails:
        print("\nFAIL: fix the items above, then re-run.")
        return 1

    import serial  # pyserial — always present on provisioned units

    # --- 4. lidar alive -----------------------------------------------------
    lidar_tty = resolved["/dev/rplidar"]
    try:
        with serial.Serial(lidar_tty, 460800, timeout=1.0) as s:
            s.reset_input_buffer()
            s.write(bytes([0xA5, 0x50]))          # GET_INFO
            resp = s.read(32)
        check("lidar GET_INFO", len(resp) >= 7 and resp[0] == 0xA5,
              f"{len(resp)} bytes" if resp else "no response — dead adapter "
              "or wrong device on this port")
    except Exception as e:
        check("lidar GET_INFO", False, str(e))

    # --- 5. rover alive ------------------------------------------------------
    rover_tty = resolved["/dev/wave_rover"]
    try:
        with serial.Serial(rover_tty, 115200, timeout=0.5) as s:
            s.reset_input_buffer()
            # DTR+RTS toggle resets the ESP32 -> boot banner on the wire.
            for _ in range(2):
                s.setDTR(False)
                s.setRTS(False)
                time.sleep(0.05)
                s.setDTR(True)
                s.setRTS(True)
                time.sleep(0.05)
            time.sleep(1.0)
            banner = s.read(2000)
        if banner:
            check("rover ESP32 banner", True, f"{len(banner)} bytes")
        else:
            check("rover ESP32 banner", False,
                  "silent after reset — firmware may boot quietly; "
                  "verify wheels physically", fatal=False)
    except Exception as e:
        check("rover ESP32 banner", False, str(e))

    if fails:
        print("\nFAIL: fix the items above, then re-run.")
        return 1
    print("\nPASS: hardware path is sound — safe to launch the stack.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
