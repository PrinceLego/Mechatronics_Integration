import socket
import time
import random

HOST = "172.20.10.2" 
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

try:
    while True:
        data = f"{random.randint(0, 100)},{random.randint(0, 100)},{random.randint(0, 100)},{random.randint(0, 100)}\n"
        sock.sendall(data.encode())
        print(f"Sent: {data.strip()}")
        time.sleep(0.2)  # 每秒發送一次
except KeyboardInterrupt:
    print("關閉連接")
finally:
    sock.close()
