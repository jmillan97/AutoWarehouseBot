import argparse
import requests
import time
import subprocess
import os
import platform
import re
from pynput import keyboard

# --- Hardware Scanning Logic ---

def scan_wifi():
    """Returns a dict of {BSSID: RSSI} currently visible."""
    results = {}
    
    current_os = platform.system()
    try:
        if current_os == "Windows":
            # Command: netsh wlan show networks mode=bssid
            output = subprocess.check_output(["netsh", "wlan", "show", "networks", "mode=bssid"], 
                                           universal_newlines=True, stderr=subprocess.DEVNULL)
            # Find BSSIDs and Signal Strengths
            # Signal: 100% -> ~-30dBm, 0% -> ~-100dBm roughly
            bssids = re.findall(r"BSSID \d+ : ([\da-fA-F:]+)", output)
            signals = re.findall(r"Signal : (\d+)%", output)
            for mac, sig in zip(bssids, signals):
                # Percent to dBm conversion (approximate)
                rssi = (float(sig) / 2) - 100
                results[mac.upper()] = rssi

        elif current_os == "Linux":
            # Command: nmcli -t -f BSSID,SIGNAL dev wifi
            output = subprocess.check_output(["nmcli", "-t", "-f", "BSSID,SIGNAL", "dev", "wifi"],
                                           universal_newlines=True, stderr=subprocess.DEVNULL)
            for line in output.strip().split("\n"):
                if ":" in line:
                    mac, sig = line.split(":")
                    rssi = (float(sig) / 2) - 100
                    results[mac.upper()] = rssi

    except Exception:
        pass # Catch silently if no WiFi hardware or command fails
        
    return results

# --- Main Logic ---

class SummonClient:
    def __init__(self, server_url):
        self.server_url = server_url.rstrip("/")
        self.ble_mac = "XX:XX:XX:XX:XX:XX" # TODO: Set this to own laptop's MAC
        
        print(f"--- Summon Client Ready ---")
        print(f"Connected to: {self.server_url}")
        print(f"Self BLE MAC: {self.ble_mac}")
        print("Press [SPACE] to summon the robot. Press [ESC] to quit.")

    def on_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.trigger_summon()
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False # Stop listener

    def trigger_summon(self):
        print("\n[!] Summon triggered! Scanning environment...")
        
        # 1. WiFi Scan
        fingerprint = scan_wifi()
        print(f"Found {len(fingerprint)} access points.")

        # 2. POST to Server
        payload = {
            "wifi_fingerprint": fingerprint,
            "ble_mac": self.ble_mac
        }
        
        try:
            r = requests.post(f"{self.server_url}/summon/dynamic", json=payload, timeout=5)
            if r.status_code == 200:
                data = r.json()
                print(f"Server Response: {data['message']}")
                print(f"Targeting: {data['estimated_target']}")
                self.monitor_status()
            else:
                print(f"Server Error ({r.status_code}): {r.text}")
        except Exception as e:
            print(f"Connection failed: {e}")

    def monitor_status(self):
        print("Monitoring robot status...")
        while True:
            try:
                r = requests.get(f"{self.server_url}/summon/status", timeout=2)
                if r.status_code == 200:
                    status = r.json()
                    state = status["state"]
                    dist = status["distance_remaining"]
                    print(f"Status: {state} | Dist: {dist:.2f}m", end="\r")
                    
                    if state in ["ARRIVED", "FAILED"]:
                        print(f"\nFinal Status: {state}")
                        break
                time.sleep(1)
            except KeyboardInterrupt:
                break
            except Exception:
                pass

def main():
    parser = argparse.ArgumentParser(description="Summon Client for AutoWarehouseBot")
    parser.add_argument("--server", default="http://localhost:8080", help="Summon Server URL")
    args = parser.parse_args()

    client = SummonClient(args.server)
    
    with keyboard.Listener(on_press=client.on_press, on_release=client.on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()
