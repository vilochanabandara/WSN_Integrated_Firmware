#!/usr/bin/env python3
"""
Capture hex log dump from UART and save to binary file.
Usage: python capture_uart_log.py COM3 output.bin
"""

import sys
import serial
import re

def capture_log(port, baudrate=115200):
    """Capture hex dump between BEGIN and END markers"""
    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=10)
    
    print("Waiting for '=== BEGIN LOG DUMP ===' marker...")
    capturing = False
    hex_data = ""
    
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            
            if not line:
                continue
                
            print(f"RX: {line[:100]}")  # Show first 100 chars
            
            if "=== BEGIN LOG DUMP ===" in line:
                print("Found BEGIN marker, capturing...")
                capturing = True
                continue
                
            if "=== END LOG DUMP ===" in line:
                print("Found END marker, done!")
                break
                
            if capturing:
                # Extract hex characters only
                hex_chars = re.findall(r'[0-9A-Fa-f]', line)
                hex_data += ''.join(hex_chars)
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            break
    
    ser.close()
    return hex_data

def hex_to_bin(hex_str):
    """Convert hex string to binary data"""
    return bytes.fromhex(hex_str)

def main():
    if len(sys.argv) < 3:
        print("Usage: python capture_uart_log.py <port> <output_file>")
        print("Example: python capture_uart_log.py COM3 log_dump.bin")
        sys.exit(1)
    
    port = sys.argv[1]
    output_file = sys.argv[2]
    
    hex_data = capture_log(port)
    
    if not hex_data:
        print("No data captured!")
        sys.exit(1)
    
    print(f"Captured {len(hex_data)} hex characters ({len(hex_data)//2} bytes)")
    
    # Convert to binary
    try:
        bin_data = hex_to_bin(hex_data)
        
        # Save to file
        with open(output_file, 'wb') as f:
            f.write(bin_data)
        
        print(f"Saved {len(bin_data)} bytes to {output_file}")
        
    except Exception as e:
        print(f"Error converting/saving: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
