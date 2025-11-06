# Troubleshooting Guide

This document collects practical tips for diagnosing common issues when working with the `franka-emika-panda-online-rl` project.

## Testing Container Network Connectivity

Use this procedure to confirm that messages from a remote machine reach a process running inside the `fep_rl` container.

1. **Start the container on your laptop**
   ```bash
   docker compose up -d fep_rl
   ```
   The Compose file exposes container port `5559` on the host.

2. **Run a listener inside the container**
   ```bash
   docker exec -it fep_rl bash -lc "while true; do nc -lk -p 5559; done"
   ```
   Keep this shell open; any received messages print here.

3. **Send a test message from the remote machine**
   ```bash
   echo "hello from remote" | nc <laptop-ip> 5559
   ```
   Replace `<laptop-ip>` with an address that can reach your laptop (public IP, VPN, etc.).

4. **If the laptop is not directly reachable (optional)**
   ```bash
   ssh -R 5559:localhost:5559 <user>@<remote-machine>
   echo "hello" | nc localhost 5559
   ```
   The reverse SSH tunnel forwards the remote port to the container listener through your laptop.

