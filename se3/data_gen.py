import holodeck
import numpy as np
import holodeck
import sys
from pynput import keyboard

# get filename to save things as
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.npz"

# install holodeck worlds if needed
if "DefaultWorlds" not in holodeck.packagemanager.installed_packages():
    holodeck.packagemanager.install("DefaultWorlds")

# These control the quadcopter
command = np.array([0, 0, 0, 0])
val = 5
def on_press(key):
    try:
        if key.char == "w":
            command[3] = 100
        if key.char == "s":
            command[3] = -val0

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
        command[0] = 10000

# set things up to save
position = []
velocity = []
imu      = []
pose     = []
controls = []

# This is where the magic actually happens
with holodeck.make("RedwoodForest-MaxDistance") as env:
   # start keyboard listener
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    #listen till we need to quit (by pressing q)
    while True:
        if command[0] == 10000:
            break

        #send to holodeck
        env.act("uav0", command)
        state = env.tick()

        #save stuff we'll need
        position.append( state['LocationSensor'] )
        velocity.append( state['VelocitySensor'] )
        imu.append( state['IMUSensor'] )
        pose.append( state['OrientationSensor'] )
        controls.append( command )


position = np.array(position)
velocity = np.array(velocity)
imu = np.array(imu)
pose = np.array(pose)

# since we're using the default holodeck... switch to RHS
# TODO: Double check that this flipped everything (and the right things)
position[:,1] *= -1
velocity[:,1] *= -1
imu[:,:,1] *= -1
pose[:,:,1] *= -1

np.savez(filename, position=position, velocity=velocity, imu=imu, pose=pose, controls=controls)