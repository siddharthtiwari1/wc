#!/usr/bin/env python3
"""
Direct Arduino Test Script
Test wheelchair movement by sending commands directly to Arduino via serial
"""

import serial
import time
import sys

# Configuration
PORT = '/dev/ttyACM0'
BAUD = 115200

def main():
    try:
        # Open serial connection
        print(f"Opening serial port {PORT}...")
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset

        print("Connected! Testing wheelchair movement...")
        print("Sending command: rp0.7,lp0.7, (both wheels forward at 0.7 rad/s)")
        print("This should move wheelchair at ~0.1 m/s")
        print("\n** BE READY TO DISCONNECT RELAY! **\n")

        # Send command for 5 seconds
        for i in range(50):  # 50 commands at 10Hz = 5 seconds
            cmd = "rp0.7,lp0.7,\n"
            ser.write(cmd.encode())
            time.sleep(0.1)

            # Read response
            if ser.in_waiting:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if i % 10 == 0:  # Print every 10th response
                    print(f"Arduino response: {response}")

        print("\nStopping wheelchair...")
        ser.write(b"rp0.0,lp0.0,\n")

        ser.close()
        print("Test complete!")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {PORT}")
        print(f"Details: {e}")
        print("\nMake sure:")
        print("1. Arduino is connected to /dev/ttyACM0")
        print("2. You have permission: sudo chmod 666 /dev/ttyACM0")
        print("3. No other program is using the port (stop ROS2 nodes first!)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nEmergency stop! Sending stop command...")
        if 'ser' in locals() and ser.is_open:
            ser.write(b"rp0.0,lp0.0,\n")
            ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
