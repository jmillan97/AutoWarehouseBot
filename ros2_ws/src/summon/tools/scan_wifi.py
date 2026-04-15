import subprocess
import re
import platform

def scan_wifi_fingerprint():
    """
    Scans for ALL nearby Access Points to create a unique 'Fingerprint'.
    Works on both Windows (netsh) and Linux/Pi (iw).
    """
    current_os = platform.system()
    results = {}

    try:
        if current_os == "Windows":
            result = subprocess.run(
                ['netsh', 'wlan', 'show', 'networks', 'mode=bssid'],
                capture_output=True, text=True, check=True
            )
            output = result.stdout
            networks = re.split(r'SSID \d+ :', output)[1:]

            print("-" * 60)
            print(f"{'SSID':<20} | {'BSSID':<20} | {'Signal':<10}")
            print("-" * 60)

            for net in networks:
                ssid_match = re.search(r'^\s*(.*)$', net, re.MULTILINE)
                ssid = ssid_match.group(1).strip() if ssid_match else "Unknown"
                bssids = re.findall(
                    r'BSSID \d+\s*:\s*([0-9a-f:]{17}).*?Signal\s*:\s*(\d+%)',
                    net, re.DOTALL
                )
                for bssid, signal in bssids:
                    pct = int(signal.replace('%', ''))
                    rssi = (pct / 2) - 100
                    results[bssid.upper()] = round(rssi, 1)
                    print(f"{ssid:<20} | {bssid:<20} | {signal:<10} ({rssi:.1f} dBm)")

        elif current_os == "Linux":
            # Requires: sudo iw dev wlan0 scan
            result = subprocess.run(
                ['sudo', 'iw', 'dev', 'wlan0', 'scan'],
                capture_output=True, text=True
            )
            output = result.stdout

            print("-" * 60)
            print(f"{'SSID':<20} | {'BSSID':<20} | {'Signal':<10}")
            print("-" * 60)

            # Split by BSS block
            blocks = re.split(r'(?=BSS [0-9a-f:]{17})', output)

            for block in blocks:
                bssid_match = re.search(r'BSS ([0-9a-f:]{17})', block)
                ssid_match  = re.search(r'SSID: (.+)', block)
                signal_match = re.search(r'signal: ([-\d.]+) dBm', block)

                if bssid_match and signal_match:
                    bssid  = bssid_match.group(1).upper()
                    ssid   = ssid_match.group(1).strip() if ssid_match else "Hidden"
                    rssi   = float(signal_match.group(1))
                    results[bssid] = rssi
                    print(f"{ssid:<20} | {bssid:<20} | {rssi:.1f} dBm")

        if not results:
            print("No BSSIDs detected. Try: sudo iw dev wlan0 scan")

        print("-" * 60)
        print(f"\nTotal APs detected: {len(results)}")
        print("\n[FINGERPRINT] Unique signal pattern for this location:")
        for bssid, rssi in sorted(results.items(), key=lambda x: x[1], reverse=True):
            print(f"  {bssid}: {rssi:.1f} dBm")

        return results

    except Exception as e:
        print(f"Error scanning: {e}")
        return {}


if __name__ == "__main__":
    scan_wifi_fingerprint()