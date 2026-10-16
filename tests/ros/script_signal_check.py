"""Start each node script, signal it, and check it exits 0 without a traceback."""
import signal
import subprocess
import sys
import time
res = {}
for f in sys.argv[1:]:
    for sig in (signal.SIGINT, signal.SIGTERM):
        p = subprocess.Popen(["python3", f], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(2.5)
        p.send_signal(sig)
        out, _ = p.communicate(timeout=15)
        res[(f.split("/")[-1], sig.name)] = (p.returncode, "Traceback" in out)
for k, v in res.items():
    print(k, "rc=%s traceback=%s" % v)
ok = all(rc == 0 and not tb for rc, tb in res.values())
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
