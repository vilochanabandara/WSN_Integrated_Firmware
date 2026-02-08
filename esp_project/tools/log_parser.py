#!/usr/bin/env python3
"""
MS Node Log Parser
Parses binary log files from ESP32-S3 sensor nodes with CRC32 verification and decompression.
"""

import struct
import json
import sys
import zlib
from pathlib import Path
from datetime import datetime

# Log chunk header format (matches logger.c)
# uint32 magic, uint16 version, uint8 algo, uint8 level, uint32 raw_len, uint32 data_len, uint32 crc32, uint64 node_id, uint32 timestamp, uint32 reserved
HEADER_FMT = '<IHBBIIIQII'  # little-endian
HEADER_SIZE = struct.calcsize(HEADER_FMT)
LOG_MAGIC = 0x4D534C47  # 'MSLG'
LOG_VERSION = 2

class LogChunk:
    def __init__(self, magic, version, algo, level, raw_len, data_len, crc32, node_id, timestamp, reserved):
        self.magic = magic
        self.version = version
        self.algo = algo  # 0=raw, 1=miniz(deflate)
        self.level = level
        self.raw_len = raw_len
        self.data_len = data_len
        self.crc32 = crc32
        self.node_id = node_id
        self.timestamp = timestamp
        self.reserved = reserved

def calc_crc32(data):
    """Calculate CRC32 matching ESP32's crc32_le()"""
    return zlib.crc32(data) & 0xFFFFFFFF

def format_node_id(node_id):
    """Convert node_id to MAC address format"""
    mac_bytes = []
    for i in range(6):
        mac_bytes.append((node_id >> (i * 8)) & 0xFF)
    return ':'.join(f'{b:02X}' for b in reversed(mac_bytes))

def unspiffs(data, offset, length):
    """
    Attempt to read 'length' bytes from SPIFFS dump starting at 'offset',
    skipping page headers.
    Assumes 256-byte pages and 12-byte headers (common config).
    """
    PAGE_SIZE = 256
    HEADER_SIZE = 12
    
    out = bytearray()
    
    # Calculate where we are in the first page
    page_idx = offset // PAGE_SIZE
    
    current_pos = offset
    remaining = length
    
    while remaining > 0:
        # End of current page
        next_page_start = (page_idx + 1) * PAGE_SIZE
        
        # Bytes available in this page
        bytes_in_page = next_page_start - current_pos
        
        # Don't read past end of data buffer
        if current_pos >= len(data):
            break
            
        to_read = min(remaining, bytes_in_page)
        
        # Append data
        out.extend(data[current_pos : current_pos + to_read])
        remaining -= to_read
        
        if remaining <= 0:
            break
            
        # Move to next page
        page_idx += 1
        current_pos = (page_idx * PAGE_SIZE) + HEADER_SIZE
        
    return bytes(out)

def scan_raw_partition(data, verify_crc=True, verbose=False, force=False, spiffs_mode=False):
    """Scan raw SPIFFS partition data for log chunks"""
    chunks = []
    offset = 0
    chunk_num = 0
    
    # Search for MSLG magic bytes
    magic_bytes = struct.pack('<I', LOG_MAGIC)
    
    while offset < len(data) - HEADER_SIZE:
        # Find next magic signature
        pos = data.find(magic_bytes, offset)
        if pos == -1:
            break
            
        # Try to parse header at this position
        try:
            header_data = data[pos:pos + HEADER_SIZE]
            if len(header_data) < HEADER_SIZE:
                break
                
            hdr = struct.unpack(HEADER_FMT, header_data)
            chunk = LogChunk(*hdr)
            
            # Validate header
            if chunk.magic != LOG_MAGIC or chunk.version != LOG_VERSION:
                offset = pos + 1
                continue
            
            # Sanity check sizes
            if chunk.data_len == 0 or chunk.data_len > 1024*1024:  # Max 1MB
                if verbose:
                    print(f"  Invalid data_len={chunk.data_len} at 0x{pos:08X}", file=sys.stderr)
                offset = pos + 1
                continue
            
            if chunk.raw_len == 0 or chunk.raw_len > 1024*1024:
                if verbose:
                    print(f"  Invalid raw_len={chunk.raw_len} at 0x{pos:08X}", file=sys.stderr)
                offset = pos + 1
                continue
                
            # Read data
            if spiffs_mode:
                chunk_data = unspiffs(data, pos + HEADER_SIZE, chunk.data_len)
            else:
                data_start = pos + HEADER_SIZE
                data_end = data_start + chunk.data_len
                
                if data_end > len(data):
                    offset = pos + 1
                    continue
                    
                chunk_data = data[data_start:data_end]
            
            # Verify CRC32 on the stored payload (compressed or raw)
            computed_crc = calc_crc32(chunk_data)
            crc_valid = (computed_crc == chunk.crc32)
            
            # Decompress if needed
            if chunk.algo == 1:  # miniz/deflate
                try:
                    raw_data = zlib.decompress(chunk_data, wbits=-15)
                except:
                    if verbose:
                        print(f"  Decompression failed at 0x{pos:08X}", file=sys.stderr)
                    offset = pos + 1
                    continue
            else:
                raw_data = chunk_data
                
            if verify_crc and not crc_valid:
                if verbose:
                    print(f"  CRC FAIL at 0x{pos:08X}: expected=0x{chunk.crc32:08X}, computed=0x{computed_crc:08X}, raw_len={len(raw_data)}, data_len={chunk.data_len}, algo={chunk.algo}", file=sys.stderr)
                    if (computed_crc ^ 0xFFFFFFFF) == chunk.crc32:
                        print(f"  (Matches inverted CRC32)", file=sys.stderr)
                    print(f"  Data preview: {raw_data[:64]!r}", file=sys.stderr)
                if not force:
                    offset = pos + 1
                    continue
                
            chunk_num += 1
            if verbose:
                print(f"Chunk {chunk_num} at offset 0x{pos:08X}: {len(raw_data)} bytes | CRC32: {'PASS' if crc_valid else 'FAIL'}", file=sys.stderr)
                
            chunks.append({
                'chunk_num': chunk_num,
                'offset': pos,
                'node_id': format_node_id(chunk.node_id),
                'timestamp': chunk.timestamp,
                'timestamp_iso': datetime.fromtimestamp(chunk.timestamp).isoformat() if chunk.timestamp > 0 else 'N/A',
                'algo': 'miniz' if chunk.algo == 1 else 'raw',
                'level': chunk.level,
                'raw_len': chunk.raw_len,
                'compressed_len': chunk.data_len if chunk.algo == 1 else None,
                'crc32': f'0x{chunk.crc32:08X}',
                'crc_valid': crc_valid,
                'raw_data': raw_data
            })
            
            offset = pos + chunk.data_len # Approximate jump
            
        except Exception as e:
            if verbose:
                print(f"  Parse error at 0x{pos:08X}: {e}", file=sys.stderr)
            offset = pos + 1
            continue
            
    return chunks

def parse_log_file(filepath, verify_crc=True, verbose=False):
    """Parse binary log file and yield chunks with metadata"""
    
    with open(filepath, 'rb') as f:
        chunk_num = 0
        while True:
            # Read header
            hdr_bytes = f.read(HEADER_SIZE)
            if len(hdr_bytes) == 0:
                break  # EOF
            if len(hdr_bytes) < HEADER_SIZE:
                print(f"WARNING: Incomplete header at chunk {chunk_num}, truncated file?", file=sys.stderr)
                break
            
            hdr = LogChunk(*struct.unpack(HEADER_FMT, hdr_bytes))
            chunk_num += 1
            
            # Validate magic
            if hdr.magic != LOG_MAGIC:
                print(f"ERROR: Invalid magic 0x{hdr.magic:08X} at chunk {chunk_num}, expected 0x{LOG_MAGIC:08X}", file=sys.stderr)
                break
            
            # Validate version
            if hdr.version != LOG_VERSION:
                print(f"WARNING: Chunk {chunk_num} version {hdr.version} != expected {LOG_VERSION}", file=sys.stderr)
            
            # Read payload data
            payload = f.read(hdr.data_len)
            if len(payload) < hdr.data_len:
                print(f"ERROR: Chunk {chunk_num} truncated payload ({len(payload)}/{hdr.data_len} bytes)", file=sys.stderr)
                break
            
            # Verify CRC32 on stored payload
            actual_crc = calc_crc32(payload)
            if verify_crc and actual_crc != hdr.crc32:
                print(f"ERROR: Chunk {chunk_num} CRC32 mismatch! Expected 0x{hdr.crc32:08X}, got 0x{actual_crc:08X}", file=sys.stderr)
                print(f"       Data integrity: CORRUPTED", file=sys.stderr)
                continue  # Skip corrupted chunk
            
            # Decompress if needed
            raw_data = payload
            compression = "RAW"
            if hdr.algo == 1:  # miniz/deflate
                try:
                    raw_data = zlib.decompress(payload, -zlib.MAX_WBITS)
                    if len(raw_data) != hdr.raw_len:
                        print(f"WARNING: Chunk {chunk_num} decompressed size mismatch ({len(raw_data)}/{hdr.raw_len})", file=sys.stderr)
                    compression = f"MINIZ-{hdr.level}"
                except zlib.error as e:
                    print(f"ERROR: Chunk {chunk_num} decompression failed: {e}", file=sys.stderr)
                    continue
            
            # Format timestamp
            ts_str = datetime.fromtimestamp(hdr.timestamp).isoformat() if hdr.timestamp > 0 else "NO_TIMESTAMP"
            
            if verbose or verify_crc:
                ratio = (hdr.data_len / hdr.raw_len * 100) if hdr.raw_len > 0 else 0
                print(f"Chunk {chunk_num}: {compression} | {hdr.raw_len} bytes | "
                      f"CRC32=0x{hdr.crc32:08X} {'✓ PASS' if verify_crc else ''} | "
                      f"Node: {format_node_id(hdr.node_id)} | Time: {ts_str}", file=sys.stderr)
                if hdr.algo == 1:
                    print(f"           Compressed: {hdr.data_len} bytes ({ratio:.1f}%)", file=sys.stderr)
            
            yield {
                'chunk_num': chunk_num,
                'node_id': format_node_id(hdr.node_id),
                'timestamp': hdr.timestamp,
                'timestamp_iso': ts_str,
                'compression': compression,
                'raw_len': hdr.raw_len,
                'compressed_len': hdr.data_len if hdr.algo == 1 else None,
                'crc32': f"0x{hdr.crc32:08X}",
                'crc_valid': actual_crc == hdr.crc32 if verify_crc else None,
                'raw_data': raw_data
            }

def parse_json_sensor_data(raw_data):
    """Attempt to parse sensor data as JSON"""
    try:
        # Try to decode as UTF-8 JSON
        text = raw_data.decode('utf-8')
        return json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError):
        # Binary data or malformed JSON
        return None

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Parse MS Node binary log files')
    parser.add_argument('logfile', help='Path to msn.log binary file or SPIFFS partition dump')
    parser.add_argument('--no-verify', action='store_true', help='Skip CRC32 verification')
    parser.add_argument('--quiet', action='store_true', help='Suppress chunk metadata output')
    parser.add_argument('--json', action='store_true', help='Output sensor data as JSON')
    parser.add_argument('--hex', action='store_true', help='Output raw data as hex dump')
    parser.add_argument('--raw-partition', action='store_true', help='Scan raw SPIFFS partition dump for log chunks')
    parser.add_argument('--spiffs', action='store_true', help='Attempt to strip SPIFFS page headers (assumes 256b pages, 12b headers)')
    parser.add_argument('--force', action='store_true', help='Output chunks even if CRC verification fails')
    parser.add_argument('--extract-lines', action='store_true', help='Extract valid JSON lines from corrupted/raw chunks (implies --force)')
    args = parser.parse_args()
    
    if args.extract_lines:
        args.force = True
    
    if not Path(args.logfile).exists():
        print(f"ERROR: File not found: {args.logfile}", file=sys.stderr)
        sys.exit(1)
    
    verify = not args.no_verify
    verbose = not args.quiet
    
    # Parse based on mode
    if args.raw_partition:
        # Scan raw SPIFFS partition dump
        with open(args.logfile, 'rb') as f:
            partition_data = f.read()
        if verbose:
            print(f"Scanning {len(partition_data)} bytes for log chunks...", file=sys.stderr)
        chunks = scan_raw_partition(partition_data, verify_crc=verify, verbose=verbose, force=args.force, spiffs_mode=args.spiffs)
    else:
        # Parse sequential log file
        chunks = list(parse_log_file(args.logfile, verify_crc=verify, verbose=verbose))
    
    if not chunks:
        print("No valid chunks found", file=sys.stderr)
        sys.exit(1)
    
    if args.extract_lines:
        # Extract valid JSON lines from all chunks
        valid_lines = 0
        for chunk in chunks:
            try:
                text = chunk['raw_data'].decode('utf-8', errors='replace')
                for line in text.splitlines():
                    line = line.strip()
                    if not line: continue
                    # Heuristic: valid log lines start with { and end with }
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            # Verify it's valid JSON
                            obj = json.loads(line)
                            print(json.dumps(obj))
                            valid_lines += 1
                        except json.JSONDecodeError:
                            pass
            except Exception:
                pass
        if verbose:
            print(f"Extracted {valid_lines} valid JSON lines", file=sys.stderr)
            
    elif args.json:
        # Try to parse and output JSON sensor data
        for chunk in chunks:
            sensor_data = parse_json_sensor_data(chunk['raw_data'])
            if sensor_data:
                output = {
                    'chunk': chunk['chunk_num'],
                    'node_id': chunk['node_id'],
                    'timestamp': chunk['timestamp_iso'],
                    'sensors': sensor_data
                }
                print(json.dumps(output, indent=2))
            else:
                print(f"Chunk {chunk['chunk_num']}: Binary data ({chunk['raw_len']} bytes)", file=sys.stderr)
    
    elif args.hex:
        # Hex dump mode
        for chunk in chunks:
            print(f"\n=== Chunk {chunk['chunk_num']} ===")
            print(chunk['raw_data'].hex())
    
    else:
        # Summary mode
        total_raw = sum(c['raw_len'] for c in chunks)
        total_compressed = sum(c['compressed_len'] for c in chunks if c['compressed_len'])
        crc_valid = sum(1 for c in chunks if c['crc_valid'])
        
        print(f"\n=== Summary ===", file=sys.stderr)
        print(f"Total chunks: {len(chunks)}", file=sys.stderr)
        print(f"Total raw data: {total_raw} bytes", file=sys.stderr)
        if total_compressed > 0:
            print(f"Total compressed: {total_compressed} bytes ({total_compressed/total_raw*100:.1f}%)", file=sys.stderr)
        if verify:
            print(f"CRC32 validation: {crc_valid}/{len(chunks)} PASS", file=sys.stderr)

if __name__ == '__main__':
    main()
