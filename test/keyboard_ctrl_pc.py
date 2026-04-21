import socket
import os
import sys
import threading
from pynput import keyboard

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
PORT = 5006
DEFAULT_PI_HOST = '104.194.124.29'


def resolve_pi_host():
    if len(sys.argv) > 1 and sys.argv[1].strip():
        return sys.argv[1].strip()
    return os.environ.get('PI_HOST', DEFAULT_PI_HOST)

# Global State
current_speed = 100
current_dir = "STOP"
last_ack = "WAITING"
pressed_keys = set()

print(f"--- ROBOT DASHBOARD (WASD + QE + ARROWS) ---")
PI_HOST = resolve_pi_host()
print(f"Connecting to Pi at {PI_HOST}:{PORT}...")

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((PI_HOST, PORT))
    # Non-blocking for receiving ACKs
    client_socket.setblocking(False)
except Exception as e:
    print(f"Error: {e}")
    print("Usage: python keyboard_ctrl_pc.py <pi_ip>")
    print(f"Or set PI_HOST. Current default: {DEFAULT_PI_HOST}")
    sys.exit(1)

def update_dashboard():
    """Prints a single line status dashboard."""
    status = f"\r[SPEED: {current_speed:3}] | [DIR: {current_dir:8}] | [STATUS: {last_ack:12}]"
    sys.stdout.write(status)
    sys.stdout.flush()

def receive_acks():
    """Background thread to listen for Arduino feedback."""
    global last_ack
    while True:
        try:
            data = client_socket.recv(1024).decode('utf-8').strip()
            if data:
                if "ACK: SPEED" in data:
                    last_ack = "SPEED SYNCED"
                elif "ACK: SYSTEM_READY" in data:
                    last_ack = "ROBOT ONLINE"
                update_dashboard()
        except BlockingIOError:
            continue
        except Exception:
            break

def on_press(key):
    global current_speed, current_dir, last_ack
    try:
        if key in pressed_keys:
            return  # Filter repeats

        if hasattr(key, 'char'):
            # Movement Keys
            if key.char in ['w', 's', 'a', 'd', 'q', 'e']:
                client_socket.sendall(f"{key.char}\n".encode('utf-8'))
                current_dir = {"w":"FORWARD", "s":"BACKWARD", "a":"LEFT", "d":"RIGHT", "q":"SPIN L", "e":"SPIN R"}[key.char]
                pressed_keys.add(key)
        
        # Speed Control (Arrows)
        elif key == keyboard.Key.up:
            current_speed = min(current_speed + 10, 255)
            client_socket.sendall(f"speed:{current_speed}\n".encode('utf-8'))
            last_ack = "SYNCING..."
        elif key == keyboard.Key.down:
            current_speed = max(current_speed - 10, 0)
            client_socket.sendall(f"speed:{current_speed}\n".encode('utf-8'))
            last_ack = "SYNCING..."
            
        update_dashboard()
    except Exception as e:
        last_ack = "ERR: SEND FAIL"
        update_dashboard()

def on_release(key):
    global current_dir
    if key in pressed_keys:
        pressed_keys.remove(key)
        # If no movement keys are left pressed, stop the robot
        if not any(hasattr(k, 'char') and k.char in ['w', 's', 'a', 'd', 'q', 'e'] for k in pressed_keys):
            client_socket.sendall(b"stop\n")
            current_dir = "STOP"
            update_dashboard()

    if key == keyboard.Key.esc:
        return False

# Start ACK Listener Thread
threading.Thread(target=receive_acks, daemon=True).start()

print("Connected! WASD to move. UP/DOWN for speed. ESC to exit.")
update_dashboard()

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

client_socket.close()
print("\nRemote Control stopped.")
