# WSN_main_set

## Quick start: flash and monitor nodes

1. **Environment:** `source ~/esp/esp-idf/export.sh` (or your IDF path).
2. **Build:** `cd esp_project/ms_node && idf.py build`
3. **Flash one device:** `idf.py -p /dev/cu.usbmodemXXXX app-flash` (macOS) or `-p COM3` (Windows).
4. **Flash all (from devices.yaml):** `python esp_project/tools/device_manager.py flash-all`
5. **Monitor (in a real terminal):** `idf.py -p /dev/cu.usbmodemXXXX monitor` — exit with `Ctrl+]`.

Ports are in `esp_project/ms_node/devices.yaml`. Serial CONFIG: type `CONFIG key=value` in the monitor to update sensor config.
