import network
import socket
import time

ssid = 'RobotNet'

# Disable AP mode to avoid conflicts
ap = network.WLAN(network.AP_IF)
ap.active(False)

# Reset and activate STA mode
sta = network.WLAN(network.STA_IF)
sta.active(False)
time.sleep(1)
sta.active(True)
time.sleep(1)

# Optional: Assign static IP to avoid DHCP issues
sta.ifconfig(('192.168.4.20', '255.255.255.0', '192.168.4.1', '192.168.4.1'))

# Connect to open AP (no password)
print("Connecting to RobotNet...")
sta.connect(ssid)

# Wait for connection (10 seconds max)
timeout = 10
while not sta.isconnected() and timeout > 0:
    print("Waiting for connection...")
    time.sleep(1)
    timeout -= 1

if not sta.isconnected():
    print("❌ Failed to connect to RobotNet")
else:
    print("✅ Connected:", sta.ifconfig())

    # Setup UDP
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        msg = "B1"
        udp.sendto(msg.encode(), ('192.168.4.1', 1234))  # Send to receiver AP
        print("Sent:", msg)
        time.sleep(2)

