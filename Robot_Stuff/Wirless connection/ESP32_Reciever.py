import network
import socket

# Start WiFi Access Point
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='RobotNet', password='12345678')

# Setup UDP socket
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind(('0.0.0.0', 1234))  # luistert op poort 1234

print("Receiver started. Waiting for UDP messages...")

while True:
    try:
        data, addr = udp.recvfrom(64)  # maximaal 64 bytes
        node_id = data.decode().strip()
        print(node_id)  # <-- belangrijk! Dit gaat via USB naar de PC
    except Exception as e:
        print("Fout:", e)
