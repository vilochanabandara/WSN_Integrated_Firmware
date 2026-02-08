#!/usr/bin/env python3
"""
WSN Device Manager: build, flash, monitor, and optimize MS Node devices.

Usage:
  python device_manager.py build
  python device_manager.py flash --port /dev/tty.usbmodem123
  python device_manager.py flash --name Node1
  python device_manager.py flash-all
  python device_manager.py monitor --port /dev/tty.usbmodem123
  python device_manager.py monitor-all
  python device_manager.py optimize --port /dev/tty.usbmodem123 audio_interval_ms=300000 inmp441_enabled=0
  python device_manager.py list-devices
  python device_manager.py list-ports

Requires: ESP-IDF (idf.py) on PATH, PyYAML for devices.yaml.
Optional: pyserial for list-ports (serial.tools.list_ports).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

# Project root: esp_project/ms_node (where idf.py is run)
SCRIPT_DIR = Path(__file__).resolve().parent
# tools/ is under esp_project/; ms_node is esp_project/ms_node
MS_NODE_ROOT = SCRIPT_DIR.parent / "ms_node"
if not MS_NODE_ROOT.is_dir():
    MS_NODE_ROOT = Path.cwd()
if not (MS_NODE_ROOT / "main").is_dir() and (SCRIPT_DIR.parent.parent / "ms_node" / "main").is_dir():
    MS_NODE_ROOT = SCRIPT_DIR.parent.parent / "ms_node"
DEVICES_FILE = MS_NODE_ROOT / "devices.yaml"
DEVICES_EXAMPLE = MS_NODE_ROOT / "devices.example.yaml"


def load_devices():
    if not DEVICES_FILE.exists():
        if DEVICES_EXAMPLE.exists():
            print(f"Create {DEVICES_FILE} from {DEVICES_EXAMPLE} and set your ports.", file=sys.stderr)
        else:
            print(f"Missing {DEVICES_FILE}. Create it with a 'devices:' list (name, port, optional mac).", file=sys.stderr)
        return []
    with open(DEVICES_FILE) as f:
        data = yaml.safe_load(f) or {}
    return data.get("devices", []), data.get("baud", 115200)


def get_idf_cmd():
    if os.environ.get("IDF_PATH"):
        return "idf.py"
    # Try common locations
    for base in [Path.home() / "esp", Path.home() / "esp-idf", Path("/opt/esp-idf")]:
        export = base / "export.sh"
        if (base / "export.sh").exists():
            return f"bash -c 'source {export} && idf.py'"
    return "idf.py"


def run_idf(ms_node_dir: Path, *args, timeout=300):
    cmd = ["idf.py"] + list(args)
    env = os.environ.copy()
    env["IDF_PATH"] = env.get("IDF_PATH", "")
    return subprocess.run(
        cmd,
        cwd=str(ms_node_dir),
        env=env,
        timeout=timeout,
    )


def run_idf_background(ms_node_dir: Path, *args):
    cmd = ["idf.py"] + list(args)
    return subprocess.Popen(
        cmd,
        cwd=str(ms_node_dir),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
    )


def cmd_build(_):
    print("Building firmware...")
    r = run_idf(MS_NODE_ROOT, "build")
    sys.exit(r.returncode)


def cmd_flash(args):
    port = args.port
    if args.name and not port:
        devices, _ = load_devices()
        if not devices:
            sys.exit(1)
        for d in devices:
            if d.get("name") == args.name:
                port = d.get("port")
                break
        if not port:
            print(f"No device named '{args.name}' in devices.yaml", file=sys.stderr)
            sys.exit(1)
    if not port:
        print("Specify --port or --name", file=sys.stderr)
        sys.exit(1)
    print(f"Flashing {port}...")
    r = run_idf(MS_NODE_ROOT, "-p", port, "app-flash" if getattr(args, "app_only", False) else "flash")
    sys.exit(r.returncode)


def cmd_flash_all(_):
    devices, _ = load_devices()
    if not devices:
        sys.exit(1)
    for d in devices:
        port = d.get("port")
        name = d.get("name", port)
        if not port:
            print(f"Skipping {name}: no port", file=sys.stderr)
            continue
        print(f"Flashing {name} ({port})...")
        r = run_idf(MS_NODE_ROOT, "-p", port, "app-flash")
        if r.returncode != 0:
            print(f"Flash failed for {name}", file=sys.stderr)
            sys.exit(1)
        time.sleep(2)
    print("All devices flashed.")


def cmd_monitor(args):
    port = args.port
    if args.name and not port:
        devices, _ = load_devices()
        if not devices:
            sys.exit(1)
        for d in devices:
            if d.get("name") == args.name:
                port = d.get("port")
                break
        if not port:
            print(f"No device named '{args.name}'", file=sys.stderr)
            sys.exit(1)
    if not port:
        print("Specify --port or --name", file=sys.stderr)
        sys.exit(1)
    run_idf(MS_NODE_ROOT, "-p", port, "monitor", timeout=3600 * 24)


def _monitor_one(device, baud):
    port = device.get("port")
    name = device.get("name", port)
    if not port:
        return
    try:
        run_idf(MS_NODE_ROOT, "-p", port, "monitor", timeout=60)
    except subprocess.TimeoutExpired:
        pass
    except Exception as e:
        print(f"[{name}] Error: {e}", file=sys.stderr)


def cmd_monitor_all(args):
    devices, baud = load_devices()
    if not devices:
        sys.exit(1)
    print("Starting monitor on all devices (Ctrl+C to stop).")
    threads = []
    for d in devices:
        t = threading.Thread(target=_monitor_one, args=(d, baud))
        t.start()
        threads.append(t)
        time.sleep(1)
    for t in threads:
        t.join(timeout=65)


def cmd_optimize(args):
    port = args.port
    if args.name and not port:
        devices, _ = load_devices()
        if not devices:
            sys.exit(1)
        for d in devices:
            if d.get("name") == args.name:
                port = d.get("port")
                break
        if not port:
            print(f"No device named '{args.name}'", file=sys.stderr)
            sys.exit(1)
    if not port:
        print("Specify --port or --name", file=sys.stderr)
        sys.exit(1)
    pairs = args.config
    if not pairs:
        print("Usage: optimize --port PORT key=value [key=value ...]", file=sys.stderr)
        sys.exit(1)
    try:
        import serial
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    ser = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(0.5)
    for kv in pairs:
        line = f"CONFIG {kv}\n"
        ser.write(line.encode())
        time.sleep(0.3)
        out = ser.read(256).decode("utf-8", errors="ignore")
        if out:
            print(out.strip())
    ser.close()
    print("Config sent. Device applies and saves to NVS; main loop reloads within ~30s.")


def cmd_list_devices(_):
    if yaml is None:
        print("Install PyYAML for devices.yaml support: pip install pyyaml", file=sys.stderr)
        sys.exit(1)
    devices, baud = load_devices()
    if not devices:
        sys.exit(1)
    for d in devices:
        print(f"  {d.get('name', '?')}: {d.get('port', 'no port')}  (mac: {d.get('mac', '-')})")


def cmd_list_ports(_):
    try:
        import serial.tools.list_ports
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}: {p.description or p.hwid}")


def main():
    if yaml is None and "list-ports" not in sys.argv and "build" not in sys.argv:
        print("Install PyYAML for device list: pip install pyyaml", file=sys.stderr)
    ap = argparse.ArgumentParser(description="WSN device manager: build, flash, monitor, optimize")
    ap.add_argument("--project", type=Path, default=MS_NODE_ROOT, help="Path to ms_node project")
    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("build").set_defaults(run=cmd_build)
    p_flash = sub.add_parser("flash")
    p_flash.add_argument("--port", "-p", help="Serial port")
    p_flash.add_argument("--name", "-n", help="Device name from devices.yaml")
    p_flash.add_argument("--app-only", action="store_true", help="app-flash only")
    p_flash.set_defaults(run=cmd_flash)
    sub.add_parser("flash-all").set_defaults(run=cmd_flash_all)
    p_mon = sub.add_parser("monitor")
    p_mon.add_argument("--port", "-p", help="Serial port")
    p_mon.add_argument("--name", "-n", help="Device name")
    p_mon.set_defaults(run=cmd_monitor)
    sub.add_parser("monitor-all").set_defaults(run=cmd_monitor_all)
    p_opt = sub.add_parser("optimize")
    p_opt.add_argument("--port", "-p", help="Serial port")
    p_opt.add_argument("--name", "-n", help="Device name")
    p_opt.add_argument("config", nargs="*", help="key=value pairs (e.g. audio_interval_ms=300000)")
    p_opt.set_defaults(run=cmd_optimize)
    sub.add_parser("list-devices").set_defaults(run=cmd_list_devices)
    sub.add_parser("list-ports").set_defaults(run=cmd_list_ports)

    args = ap.parse_args()
    global MS_NODE_ROOT
    if getattr(args, "project", None) and args.project != MS_NODE_ROOT:
        MS_NODE_ROOT = args.project
    args.run(args)


if __name__ == "__main__":
    main()
