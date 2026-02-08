#!/usr/bin/env python3
"""Generate test MS Node log file for parser testing"""

import struct
import json
import zlib
from datetime import datetime

# Log format constants (must match logger.c)
HEADER_FMT = '<IHBBIIQQII'
LOG_MAGIC = 0x4D534C47  # 'MSLG'
LOG_VERSION = 2
NODE_ID = 0x1020BA4DF03C  # Match your device's MAC

def calc_crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF

def write_chunk(f, data, compress=True):
    """Write a log chunk (raw or compressed)"""
    raw_data = data.encode('utf-8') if isinstance(data, str) else data
    raw_len = len(raw_data)
    
    if compress and raw_len >= 1024:
        # Compress with miniz (deflate)
        compressed = zlib.compress(raw_data, level=3)
        # Remove zlib header/trailer for raw deflate
        payload = compressed[2:-4]
        
        # Check if compression worth it (5% savings)
        if len(payload) + 32 < raw_len - (raw_len // 20):
            algo = 1
            level = 3
            data_to_write = payload
        else:
            algo = 0
            level = 0
            data_to_write = raw_data
    else:
        algo = 0
        level = 0
        data_to_write = raw_data
    
    crc32 = calc_crc32(data_to_write)
    timestamp = int(datetime.now().timestamp())
    
    header = struct.pack(
        HEADER_FMT,
        LOG_MAGIC,
        LOG_VERSION,
        algo,
        level,
        raw_len,
        len(data_to_write),
        crc32,
        NODE_ID,
        timestamp,
        0  # reserved
    )
    
    f.write(header)
    f.write(data_to_write)
    
    compression_str = f"MINIZ-{level}" if algo == 1 else "RAW"
    print(f"Chunk written: {compression_str} | {raw_len} bytes | CRC32=0x{crc32:08X}")

# Generate sample sensor data
sensor_data = {
    "timestamp": int(datetime.now().timestamp()),
    "node_id": "10:20:BA:4D:F0:3C",
    "battery_pct": 28,
    "mode": "POWER_SAVE",
    "sensors": {
        "bme280": {
            "temperature_c": 30.5,
            "humidity_pct": 65.2,
            "pressure_hpa": 1013.25
        },
        "aht21": {
            "temperature_c": 30.1,
            "humidity_pct": 62.9
        },
        "ens160": {
            "aqi": 1,
            "tvoc_ppb": 35,
            "eco2_ppm": 426,
            "status": 0x8B
        },
        "gy271": {
            "x": -6222,
            "y": 1550,
            "z": -1122
        },
        "ina219": {
            "bus_voltage_v": 3.552,
            "shunt_voltage_mv": 7.49,
            "current_ma": 74.9
        }
    }
}

# Create test log file
with open('test_msn.log', 'wb') as f:
    # Write several chunks
    for i in range(5):
        sensor_data['timestamp'] = int(datetime.now().timestamp()) + (i * 60)
        sensor_data['battery_pct'] = 28 - i
        sensor_data['sensors']['bme280']['temperature_c'] = 30.5 + (i * 0.2)
        
        # Write as JSON
        json_str = json.dumps(sensor_data, indent=2)
        write_chunk(f, json_str, compress=True)
    
    # Write one large chunk to trigger compression
    large_data = json.dumps([sensor_data] * 20, indent=2)
    write_chunk(f, large_data, compress=True)

print("\nTest log file created: test_msn.log")
print("Parse with: python log_parser.py test_msn.log")
print("View JSON: python log_parser.py test_msn.log --json")
