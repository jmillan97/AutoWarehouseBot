import socket
import serial
import serial.tools.list_ports
import time
import sys

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
TCP_PORT = 5006
BIND_IP = '0.0.0.0'

def find_arduino_port():
    """Automatically finds the Arduino port by scanning for common names."""
    ports = list(serial.tools.list_ports.comports())
    # 1. First try to find by 'Arduino' description
    for p in ports:
        if 'Arduino' in p.description or 'USB' in p.description:
            return p.device
            
    # 2. Fallback to common Linux names
    common_names = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    for name in common_names:
        try:
            temp_ser = serial.Serial(name)
            temp_ser.close()
            return name
        except:
            continue
            
    return None

def run_pi_server():
    # 1. Find and Open Serial to Arduino
    arduino_port = find_arduino_port()
    if not arduino_port:
        print("Error: Could not find any Arduino/USB serial device. Is it plugged in?")
        return

    try:
        ser = serial.Serial(arduino_port, 9600, timeout=1)
        print(f"✅ Connected to Arduino on: {arduino_port}")
    except Exception as e:
        print(f"❌ Error connecting to {arduino_port}: {e}")
        return

    # 2. Setup TCP Server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((BIND_IP, TCP_PORT))
    server_socket.listen(1)
    
    print(f"--- ROBOT SIGNAL SERVER (AUTO-PORT) ---")
    print(f"Listening on port {TCP_PORT}...")
    
    try:
        while True:
            print("\nReady for connection...")
            conn, addr = server_socket.accept()
            print(f"Accepted connection from {addr}")

            while True:
                data = conn.recv(1024).decode('utf-8')
                if not data:
                    break
                
                ser.write(data.encode('utf-8'))
                print(f"Relayed: {data.strip()}")

            conn.close()
            print("Client Disconnected.")
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server_socket.close()
        ser.close()

if __name__ == "__main__":
    run_pi_server()
