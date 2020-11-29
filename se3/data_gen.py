import holodeck
import numpy as np
import holodeck
import sys
from pynput import keyboard
np.set_printoptions(suppress=True,
   formatter={'float_kind':'{:0.5f}'.format}) 
# get filename to save things as
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.npz"

# get number of ticks per second
if len(sys.argv) > 2:
    ticks = int(sys.argv[2])
else:
    ticks = 60

# install holodeck worlds if needed
if "Ocean" not in holodeck.packagemanager.installed_packages():
    holodeck.packagemanager.install("Ocean", "https://robots.et.byu.edu/jenkins/job/holodeck-ocean-engine/job/develop/lastSuccessfulBuild/artifact/Ocean.zip")

# These control the quadcopter
command = np.array([0, 0, 0, 0])
val = 2
def on_press(key):
    try:
        if key.char == "w":
            command[3] = 100
        if key.char == "s":
            command[3] = -100

        if key.char == "y":
            command[0] = val
        if key.char == "u":
            command[0] = -val

        if key.char == "h":
            command[1] = val
        if key.char == "j":
            command[1] = -val

        if key.char == "n":
            command[2] = val
        if key.char == "m":
            command[2] = -val

    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    try:
        if key.char == "w":
            command[3] = 0
        if key.char == "s":
            command[3] = 0

        if key.char == "y":
            command[0] = 0
        if key.char == "u":
            command[0] = 0

        if key.char == "h":
            command[1] = 0
        if key.char == "j":
            command[1] = 0

        if key.char == "n":
            command[2] = 0
        if key.char == "m":
            command[2] = 0

        if key.char == "q":
            command[3] = 1
        if key.char == "r":
            command[3] = -1
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

# set things up to save
x = []
z = []
u = []
record = False

# This is where the magic actually uhappens
with holodeck.make("Rooms-IEKF", ticks_per_sec=ticks) as env:
   # start keyboard listener
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    #listen till we need to quit (by pressing q)
    while True:
        if command[3] == 1:
            break
        if command[3] == -1:
            record = True

        #send to holodeck
        env.act("uav0", command)
        state = env.tick()

        #make state
        if record:
            temp = np.eye(5)
            temp[:4,:4] = state['PoseSensor']
            temp[:3, 4] = state['VelocitySensor']
            # flip velocity and position columns
            temp[:3, [3,4]] = temp[:3, [4,3]]

            #save stuff we'll need
            x.append(temp)
            z.append(state['VelocitySensor'])
            u.append(state['IMUSensor'])

if record:
    np.savez(filename, x=np.array(x), z=np.array(z), u=np.array(u), ticks=ticks)
    print("Data Saved.")

# test Pose Sensor
# import matplotlib.pyplot as plt
# from pytransform3d.rotations import plot_basis
# ax = plot_basis()
# ax.set_xlim([-10,10])
# ax.set_ylim([-10,10])
# ax.set_zlim([-10,10])
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# for xi in x[100:]:
#     plot_basis(ax, xi[:3,:3], xi[:3,3], s=1, strict_check=False)
# plt.show()
