# 🏎️ Robot Speed Steering Guide (3-27-2026)

This folder contains the **Ultimate Version** of the remote control system, featuring:
- **Hold-to-Move**: Stop instantly on key release.
- **Speed Steer**: Use **UP/DOWN Arrows** to adjust speed (+10/-10).
- **Dashboard**: Live terminal feedback of speed and direction.
- **Auto-Port**: Pi automatically finds the Arduino on its own.

---

## 🛠️ Step 1: Deploy to Pi (SSH)
Since I just "Taildropped" the file, you just need to catch it:
```bash
sudo tailscale file get .
```

## 🚀 Step 2: Run the System

### 1. Start the Relay (on the Pi)
```bash
python robot_server_pi.py
```
*(It should say "✅ Connected to Arduino on: /dev/tty...")*

### 2. Start the Dashboard (on your Laptop)
Open a terminal in THIS folder and run:
```bash
python keyboard_ctrl_pc.py 104.194.126.139
```

You can also set the Pi address once and reuse it:

```bash
export PI_HOST=104.194.126.139
python keyboard_ctrl_pc.py
```

If you run from Windows PowerShell:

```powershell
$env:PI_HOST="104.194.126.139"
python \\wsl.localhost\Ubuntu-24.04\home\felix\warehouse_project\test\keyboard_ctrl_pc.py
```

### 3. Flash Arduino (If not already)
Upload [final_robot_control.ino](file:///C:/Users/felix/.gemini/antigravity/scratch/school/ECE%20441/warehouse%20robot/3-27-2026/speed%20throttling/final_robot_control.ino) to your Arduino.

Notes:

- This old bridge expects Arduino serial at `9600`
- It expects movement commands `w/s/a/d/q/e`
- It sends `stop` on key release
- The Pi IP should be the Pi's current local network IP, not an old Tailscale address

---

## 🕹️ Controls
- **WASD / QE**: Movement (Hold to go, Release to stop!)
- **Arrow UP / DOWN**: Speed Control (+/- 10)
- **SPACE**: Immediate Stop
- **ESC**: Exit Controller
