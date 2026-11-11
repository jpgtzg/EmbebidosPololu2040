"""Simula al Zumo: manda JSON por UDP broadcast al puerto 5000.
Uso: python send_test.py [IP_de_la_pico]   (sin argumento = broadcast)
La PC debe estar en la misma red Wi-Fi ("embedded")."""
import json, socket, sys, time

port = 5000
dest = sys.argv[1] if len(sys.argv) > 1 else "255.255.255.255"
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

i = 0
while True:
    msg = json.dumps({"id": 1, "x": i * 0.1, "y": 0.0, "theta": 0.0, "n": i})
    s.sendto(msg.encode(), (dest, port))
    print("sent", msg)
    i += 1
    time.sleep(0.5)
