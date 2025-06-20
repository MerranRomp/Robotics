import network
import socket

# Start WiFi Access Point
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='RobotNet', password='12345678', authmode=network.AUTH_WPA_WPA2_PSK)

# Setup UDP socket
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind(('0.0.0.0', 1234))  # listen on all interfaces at port 1234

print("Receiver started. Waiting for data...")

while True:
    try:
        data, addr = udp.recvfrom(64)  # max 64 bytes
        node_id = data.decode().strip()
        print(node_id)  # For serial output
    except Exception as e:
        print("Fout:", e)
