import socket
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib import patches as patches
import struct
import numpy as np
import math

UDP_IP = '127.0.0.1'
UDP_PORT = 9155
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

posx = 0
posy = 0
posz = 0
number = 0
plt.ion()

fig, ax = plt.subplots(2, 2, figsize=(12, 7), dpi=80)
plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
fig.set_facecolor('w')


fig.show()

# We need to draw the canvas before we start animating...
fig.canvas.draw()


# axis settings
ax[0][0].set_aspect('equal')
ax[0][0].set_xlim(-140, 140)
ax[0][0].set_ylim(-80, 100)
ax[0][0].set_title('Motor RPMs')
ax[0][0].axis('off')

ax[0][1].set_aspect('equal')
ax[0][1].set_xlim(-160, 160)
ax[0][1].set_ylim(-120, 120)
ax[0][1].set_title('Horizon')
ax[0][1].axes.get_xaxis().set_visible(False)
ax[0][1].axes.get_yaxis().set_visible(False)


ax[1][1].set_aspect('equal')
ax[1][1].set_xlim(-170, 170)
ax[1][1].set_ylim(-120, 120)
ax[1][1].set_title('Stick Commands')
ax[1][1].axis('off')

ax[1][0].set_title('RPMs')

fig.canvas.set_window_title('5TOL Telemetry')

angle1 = 180
angle2 = 180
angle3 = 180
angle4 = 180

angleTL = 0
angleTR = 0

pitch = 0
roll = 0

stickLUD = 0
stickLLR = 0

stickRUD = 0
stickRLR = 0

rpm1Value = 0
rpm2Value = 0
rpm3Value = 0
rpm4Value = 0

scatterColors = ['r', 'g', 'b', 'black']
rpms = np.zeros((1, 4))
#timeArray = np.arange(0,100,0.1)
#timeArray = np.resize(timeArray, (100, 4))

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
tiltLeft = patches.Wedge((-80,0),25,160,180,width=5,facecolor='#91D100')
tiltRight = patches.Wedge((80,0),25,0,30,width=5,facecolor='#91D100')

stickLeft = plt.Circle((-80,0), 10)
stickRight = plt.Circle((80,0), 10)

horizon = patches.Ellipse((0,0), 5000,200, facecolor='#9b4712')
horizonLine = patches.FancyArrowPatch((-170, 0), (180,0))

ax[1][1].add_artist(stickLeft)
ax[1][1].add_artist(stickRight)

stickLeftBorder  = plt.Circle((-80,0), 80, fill=False)
stickRightBorder = plt.Circle((80,0), 80, fill=False)
ax[1][1].add_artist(stickLeftBorder)
ax[1][1].add_artist(stickRightBorder)

ax[0][0].add_patch(rpm1)
ax[0][0].add_patch(rpm2)
ax[0][0].add_patch(rpm3)
ax[0][0].add_patch(rpm4)
ax[0][0].add_patch(tiltLeft)
ax[0][0].add_patch(tiltRight)

ax[0][1].add_patch(horizon)
ax[0][1].add_patch(horizonLine)

text1 = ax[0][0].text(-72, 30, "RPM 1: 0", fontsize=10)
text2 = ax[0][0].text( 28, 30, "RPM 2: 0", fontsize=10)
text3 = ax[0][0].text(-24, 80, "RPM 3: 0", fontsize=10)
text4 = ax[0][0].text(-24,-20, "RPM 4: 0", fontsize=10)

textPitch = ax[0][1].text(-170,-140, "Roll: 0")
textRoll = ax[0][1].text(-50,-140, "Pitch : 0")

text1percent = ax[0][0].text(-60, 0, "00%" , fontsize=10)
text2percent = ax[0][0].text( 40, 0, "00%" , fontsize=10)
text3percent = ax[0][0].text(-10, 47, "00%", fontsize=10)
text4percent = ax[0][0].text(-10,-53, "00%", fontsize=10)

actualTime = 0
while True:
    actualTime += 1
    if actualTime == 100:
        actualTime = 0
        ax[1][0].cla()
        ax[1][0].set_xlim(0, 100)

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

    dataFloat = data[20:24]
    angleTL = struct.unpack('<f', dataFloat)[0]*180.0/3.1415

    dataFloat = data[24:28]
    angleTR = struct.unpack('<f', dataFloat)[0]*180.0/3.1415

    dataFloat = data[28:32]
    roll = struct.unpack('<f', dataFloat)[0] * 180.0 / 3.1415

    dataFloat = data[32:36]
    pitch = struct.unpack('<f', dataFloat)[0] * 180.0 / 3.1415

    dataFloat = data[36:40]
    stickLUD = struct.unpack('<f', dataFloat)[0]*80

    dataFloat = data[40:44]
    stickLLR = struct.unpack('<f', dataFloat)[0]*80

    dataFloat = data[44:48]
    stickRUD = struct.unpack('<f', dataFloat)[0]*80

    dataFloat = data[48:52]
    stickRLR = struct.unpack('<f', dataFloat)[0]*80

    text1.set_text('RPM 1: {:3.0f}'.format(rpm1Value))
    text2.set_text('RPM 2: {:3.0f}'.format(rpm2Value))
    text3.set_text('RPM 3: {:3.0f}'.format(rpm3Value))
    text4.set_text('RPM 4: {:3.0f}'.format(rpm4Value))

    text1percent.set_text('{:3.0f}%'.format(angle1/3.60))
    text2percent.set_text('{:3.0f}%'.format(angle2/3.60))
    text3percent.set_text('{:3.0f}%'.format(angle3/3.60))
    text4percent.set_text('{:3.0f}%'.format(angle4/3.60))

    rpm1.set_theta1(-angle1+90)
    rpm2.set_theta1(-angle2+90)
    rpm3.set_theta1(-angle3+90)
    rpm4.set_theta1(-angle4+90)

    if angleTL > 0:
        tiltLeft.set_theta1(180 - angleTL)
        tiltLeft.set_theta2(180)
    else:
        tiltLeft.set_theta1(180)
        tiltLeft.set_theta2(180 - angleTL)

    if angleTR > 0:
        tiltRight.set_theta1(0)
        tiltRight.set_theta2(angleTR)
    else:
        tiltRight.set_theta1(angleTR)
        tiltRight.set_theta2(0)

    stickLeft.center = stickLLR-80, stickLUD
    stickRight.center = stickRLR+80, stickRUD

    horizon.center = 0,  math.sin(math.radians(pitch))*10-100
    horizon.angle = -roll

    rpm1.set_facecolor(plt.cm.jet(angle1 / 360))
    rpm2.set_facecolor(plt.cm.jet(angle2 / 360))
    rpm3.set_facecolor(plt.cm.jet(angle3 / 360))
    rpm4.set_facecolor(plt.cm.jet(angle4 / 360))

    rpms = [rpm1Value, rpm2Value, rpm3Value, rpm4Value]

    ax[1][0].scatter([actualTime, actualTime, actualTime, actualTime, ], rpms, c=scatterColors)
    ax[0][0].draw_artist(ax[0][0].patch)
    ax[1][1].draw_artist(ax[1][1].patch)
    ax[0][1].draw_artist(ax[0][1].patch)

    fig.canvas.flush_events()




