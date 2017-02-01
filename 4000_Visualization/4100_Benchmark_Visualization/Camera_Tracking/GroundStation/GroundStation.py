import socket
from matplotlib import pyplot as plt
from matplotlib import patches as patches

UDP_IP = '127.0.0.1'
UDP_PORT = 9155
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

#sock.bind((UDP_IP, UDP_PORT))

posx = 0
posy = 0
posz = 0
number = 0
plt.ion()

mainWindow = plt.figure()
Gauge1 = patches.Arc([100, 100], 50, 50)

fig, ax = plt.subplots(1,1)

# axis settings
ax.set_aspect('equal')
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)

angle = 180
rpm1 = patches.Wedge((-50,0),25,25,179,width=10,facecolor='#FF0000')
rpm2 = patches.Wedge((50,0),25,25,179,width=10,facecolor='#FF0000')
rpm3 = patches.Wedge((0,50),25,25,179,width=10,facecolor='#FF0000')
rpm4 = patches.Wedge((0,-50),25,25,179,width=10,facecolor='#FF0000')
ax.add_patch(rpm1)
ax.add_patch(rpm2)
ax.add_patch(rpm3)
ax.add_patch(rpm4)

while True:
    angle -=1
    plt.pause(0.01)
    rpm1.set_theta1(angle)
    rpm2.set_theta1(angle)
    rpm3.set_theta1(angle)
    rpm4.set_theta1(angle)
    plt.show()

while False:
    data, addr = sock.recvfrom(512)  # buffer size is 1024 bytes
    number += 1
    print "received message:", data
    if data[0] == 'x':
        posx = data[1:-1]
    if data[0] == 'y':
        posy = data[1:-1]
    if data[0] == 'z':
        posz = data[1:-1]

    plt.scatter(number, posx, c='r')
    plt.scatter(number, posy, c='g')
    plt.scatter(number, posz, c='b')
    plt.pause(0.0001)


