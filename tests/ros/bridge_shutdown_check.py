"""Run wave_rover_bridge on a pty, signal it, check it exits cleanly and zeroes the wheels."""
import os
import pty
import select
import signal
import subprocess
import sys
import time

BRIDGE = sys.argv[1]
results = {}
for sig in (signal.SIGINT, signal.SIGTERM):
    master, slave = pty.openpty()
    tty = os.ttyname(slave)
    p = subprocess.Popen(
        ["python3", BRIDGE, "--ros-args", "-p", "transport:=serial",
         "-p", f"serial_port:={tty}", "-p", "cmd_timeout:=0.0"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(3.0)
    buf = b""
    while select.select([master], [], [], 0.1)[0]:
        buf += os.read(master, 4096)
    t0 = time.monotonic()
    p.send_signal(sig)
    out, _ = p.communicate(timeout=15)
    dt = time.monotonic() - t0
    buf = b""
    while select.select([master], [], [], 0.2)[0]:
        buf += os.read(master, 4096)
    zeros = buf.decode(errors="replace").count('{"T":1,"L":0.000,"R":0.000}')
    tb = "Traceback" in out
    results[sig.name] = dict(rc=p.returncode, exit_s=round(dt, 2), zero_cmds=zeros, traceback=tb)
    if tb:
        print(out[-1500:])
print(results)
ok = all(r["rc"] == 0 and r["zero_cmds"] >= 1 and not r["traceback"] for r in results.values())
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
