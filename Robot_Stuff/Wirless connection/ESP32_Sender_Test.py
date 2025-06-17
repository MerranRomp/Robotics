import network
import socket
import time

# Connect to receiver's AP
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect('RobotNet', '12345678')

while not sta.isconnected():
    time.sleep(0.5)

print("Connected to RobotNet")

# Setup UDP socket
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver_ip = '192.168.4.1'  # default IP of ESP32 in AP mode
receiver_port = 1234

while True:
    msg = "B3"
    udp.sendto(msg.encode(), (receiver_ip, receiver_port))
    print("Sent:", msg)
    time.sleep(2)
