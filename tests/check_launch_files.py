"""Import every launch file and build its LaunchDescription (no nodes start).

Catches syntax errors, bad imports and errors in generate_launch_description()
without hardware. Run with ROS 2 Jazzy sourced:
    python3 tests/check_launch_files.py
"""
import importlib.util
import pathlib
import sys
import traceback

from launch import LaunchDescription

root = pathlib.Path(__file__).resolve().parents[1] / "launch"
failed = []
for path in sorted(root.glob("*.launch.py")):
    try:
        spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        assert isinstance(ld, LaunchDescription), type(ld)
        print(f"ok   {path.name} ({len(ld.entities)} entities)")
    except Exception:
        failed.append(path.name)
        print(f"FAIL {path.name}\n{traceback.format_exc()}")
print(f"{len(failed)} failed" if failed else "all launch files OK")
sys.exit(1 if failed else 0)
