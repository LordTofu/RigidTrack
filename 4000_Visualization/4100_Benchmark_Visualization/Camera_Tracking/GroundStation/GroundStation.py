import socket
from matplotlib import pyplot as plt
from matplotlib import cm as cm
from matplotlib import patches as patches
import struct

UDP_IP = '192.168.4.5'
UDP_PORT = 9155
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

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

angle1 = 180
angle2 = 180
angle3 = 180
angle4 = 180

rpm1Value = 0
rpm2Value = 0
rpm3Value = 0
rpm4Value = 0

rpmMax1 = 11074
rpmMax2 = 11074
rpmMax3 = 5707
rpmMax4 = 5707

rpmMin1 = 765
rpmMin2 = 765
rpmMin3 = 512
rpmMin4 = 512

rpm1 = patches.Wedge((-50,0),25,0,90,width=10,facecolor='#91D100')
rpm2 = patches.Wedge((50,0),25,0,90,width=10,facecolor='#91D100')
rpm3 = patches.Wedge((0,50),25,0,90,width=10,facecolor='#91D100')
rpm4 = patches.Wedge((0,-50),25,0,90,width=10,facecolor='#91D100')
ax.add_patch(rpm1)
ax.add_patch(rpm2)
ax.add_patch(rpm3)
ax.add_patch(rpm4)

text1 = plt.text(-80, 30, "RPM 1: 0")
text2 = plt.text( 30, 30, "RPM 2: 0")
text3 = plt.text(-35, 80, "RPM 3: 0")
text4 = plt.text(-35,-20, "RPM 4: 0")

text1percent = plt.text(-60, 0, "00%")
text2percent = plt.text( 40, 0, "00%")
text3percent = plt.text(-10, 50, "00%")
text4percent = plt.text(-10,-50, "00%")

while True:
    data, addr = sock.recvfrom(54)  # buffer size is 1024 bytes
    #print "received message:", data

    dataFloat = data[4:8]
    rpm1Value = struct.unpack('<f', dataFloat)[0]
    angle1 = (struct.unpack('<f', dataFloat)[0] -rpmMin1)/(rpmMax1-rpmMin1)*360

    dataFloat = data[8:12]
    rpm2Value = struct.unpack('<f', dataFloat)[0]
    angle2 = (struct.unpack('<f', dataFloat)[0] -rpmMin2)/(rpmMax2-rpmMin2)*360

    dataFloat = data[12:16]
    rpm3Value = struct.unpack('<f', dataFloat)[0]
    angle3 = (struct.unpack('<f', dataFloat)[0] -rpmMin3)/(rpmMax3-rpmMin3)*360

    dataFloat = data[16:20]
    rpm4Value = struct.unpack('<f', dataFloat)[0]
    angle4 = (struct.unpack('<f', dataFloat)[0] -rpmMin4)/(rpmMax4-rpmMin4)*360

    text1.set_text("RPM 1: " + str(rpm1Value))
    text2.set_text("RPM 2: " + str(rpm2Value))
    text3.set_text("RPM 3: " + str(rpm3Value))
    text4.set_text("RPM 4: " + str(rpm4Value))

    text1percent.set_text('{:3.0f}%'.format(angle1/3.60))
    text2percent.set_text('{:3.0f}%'.format(angle2/3.60))
    text3percent.set_text('{:3.0f}%'.format(angle3/3.60))
    text4percent.set_text('{:3.0f}%'.format(angle4/3.60))

    rpm1.set_theta1(-angle1+90)
    rpm2.set_theta1(-angle2+90)
    rpm3.set_theta1(-angle3+90)
    rpm4.set_theta1(-angle4+90)

    rpm1.set_facecolor(plt.cm.jet(angle1 / 360))
    rpm2.set_facecolor(plt.cm.jet(angle2 / 360))
    rpm3.set_facecolor(plt.cm.jet(angle3 / 360))
    rpm4.set_facecolor(plt.cm.jet(angle4 / 360))

    plt.pause(0.01)
    plt.show()



