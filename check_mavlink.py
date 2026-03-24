import time
import sys
from pymavlink import mavutil

def check_connection(connection_string, baudrate=115200, timeout=3):
    print(f"Testing connection on {connection_string} (baud: {baudrate})...")
    try:
        if 'udp' in connection_string or 'tcp' in connection_string:
            master = mavutil.mavlink_connection(connection_string)
        else:
            master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        
        # Wait for a heartbeat
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg:
            print(f"✅ SUCCESS! Heartbeat received from {connection_string}")
            print(f"   System ID: {master.target_system}, Component ID: {master.target_component}")
            print(f"   Autopilot type: {msg.autopilot}, Vehicle type: {msg.type}")
            master.close()
            return True
        else:
            print(f"❌ No heartbeat received on {connection_string} within {timeout} seconds.")
            master.close()
            return False
    except Exception as e:
        print(f"⚠️ Error opening {connection_string}: {e}")
        return False

def main():
    # Common connection strings to test
    targets = [
        # Network
        ("udp:127.0.0.1:14550", 0),
        ("udp:127.0.0.1:14551", 0),
        # USB Serial
        ("/dev/ttyACM0", 115200),
        ("/dev/ttyACM0", 57600),
        ("/dev/ttyACM1", 115200),
        # Hardware Serial (Jetson UART)
        ("/dev/ttyTHS1", 115200),
        ("/dev/ttyTHS1", 57600),
        ("/dev/ttyTHS1", 921600),
    ]

    found = False
    for conn, baud in targets:
        if check_connection(conn, baud):
            found = True
            print("\n--- Connection Found! ---")
            print(f"You should use: '{conn}' with baudrate {baud} in your ROS 2 node.")
            break
            
    if not found:
        print("\n❌ Could not find an active ArduPilot connection on standard ports.")

if __name__ == '__main__':
    main()
