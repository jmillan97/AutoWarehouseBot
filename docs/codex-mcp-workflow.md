# Codex MCP Workflow

This project uses Codex MCP servers to avoid brittle nested commands like:

```text
PowerShell -> WSL bash -> SSH -> remote bash
```

Prefer direct MCP routes while debugging the robot.

## Configured MCP Servers

Codex global config lives at:

```text
C:\Users\felix\.codex\config.toml
```

Current useful MCP servers:

- `wsl`
  Runs commands directly in the Ubuntu WSL distro.
- `ssh`
  Runs commands directly on SSH hosts from Windows `~/.ssh/config`.

Antigravity keeps a separate MCP config at:

```text
C:\Users\felix\.gemini\antigravity\mcp_config.json
```

The Codex config mirrors the useful Antigravity entries, but Codex does not read
the Antigravity JSON file directly.

## SSH Hosts

Windows SSH config lives at:

```text
C:\Users\felix\.ssh\config
```

Current robot aliases:

```sshconfig
Host warehouse-pi
    HostName 104.194.124.29
    User ece_441
    IdentityFile ~/.ssh/warehouse_pi_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    StrictHostKeyChecking accept-new

Host warehouse-pi-tail
    HostName 100.89.188.77
    User ece_441
    IdentityFile ~/.ssh/warehouse_pi_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    StrictHostKeyChecking accept-new
```

Use `warehouse-pi` first when on the same local network. Use
`warehouse-pi-tail` as the Tailscale fallback.

Both aliases are configured for non-interactive key auth using:

```text
C:\Users\felix\.ssh\warehouse_pi_ed25519
```

The matching public key is installed in:

```text
/home/ece_441/.ssh/authorized_keys
```

## Current Network Values

Known-good values as of the latest setup:

```text
Laptop / WSL source IP: 104.194.115.152
Pi Wi-Fi IP:           104.194.124.29
Pi Tailscale IP:       100.89.188.77
```

Rediscover the Pi addresses on the Pi:

```bash
hostname -I
ip -br addr
```

Rediscover the laptop/WSL source IP from WSL:

```bash
ip route get 104.194.124.29
```

Use the `src` value as the laptop peer in `etc/fastdds_config.xml`.

## FastDDS Files

WSL repo config:

```text
/home/felix/warehouse_project/etc/fastdds_config.xml
```

Pi runtime config:

```text
/etc/fastdds_config.xml
```

Both should include the current laptop/WSL IP and the current Pi Wi-Fi IP.

## Recommended Debug Flow

Use the WSL MCP for local project commands:

```bash
cd /home/felix/warehouse_project
git status --short
./scripts/check_topics.sh --snapshot
```

Use Windows PowerShell for GitHub pushes. WSL-side pushes can stall badly in
this setup, so commit from WSL if convenient, then push from Windows:

```powershell
cd \\wsl.localhost\Ubuntu-24.04\home\felix\warehouse_project
git push
```

Use the SSH MCP for Pi commands:

```bash
hostname -I
ls -l /dev/lidar /dev/arduino /dev/serial/by-id
source ~/.ros_network_env
ros2 topic list
```

For LiDAR work, check the device first:

```bash
ls -l /dev/lidar /dev/serial/by-id
```

The expected mapping is:

```text
/dev/lidar   -> ttyUSB0
/dev/arduino -> ttyUSB1 or ttyUSB2 depending on plug order
```

Then start Pi bringup with LiDAR enabled:

```bash
source ~/.ros_network_env
ros2 launch embedded robot_bringup.launch.py enable_lidar:=true
```

## Smoke Tests

From Windows PowerShell:

```powershell
ssh -o BatchMode=yes warehouse-pi hostname
ssh -o BatchMode=yes warehouse-pi-tail hostname
```

Both should print:

```text
ece441group4
```

If `warehouse-pi` fails but `warehouse-pi-tail` works, the local network route
or Wi-Fi IP likely changed. Rediscover the Pi Wi-Fi IP and update
`C:\Users\felix\.ssh\config`, `scripts/common.sh`, and both FastDDS configs.
