import ultrasonicthree
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = "192.168.43.97"
port = 1118

s.bind((host, port))
# queue up to 5 requests
s.listen(5)

sc,addr = s.accept()

while (1):
    d=ultrasonicthree.ultrathree()
    if(d < 40):
        sc.send("1".encode("ascii"))
    else:
        sc.send("0".encode("ascii"))
