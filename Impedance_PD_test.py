import pyCandle

import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import json

# Generate reference stepping trajectory
def step_ref (t, T, ref_angl):
    if t <= T:
        theta = ref_angl*(6*(t/T)**5 - 15*(t/T)**4 + 10*(t/T)**3) # ref position
        theta_d = ref_angl*(30*(t/T)**4 - 60*(t/T)**3 + 30*(t/T)**2)# ref velocity
        return theta, theta_d
    else:
        return ref_angl, 0

def sin_ref (sine_freq, t):
    angl_ref = math.sin(2*np.pi*sine_freq*t) * 2 # ref position
    angl_vel_ref = 2*np.pi*sine_freq*math.cos(2*np.pi*sine_freq*t) * 2 # ref velocity
    return angl_ref, angl_vel_ref

# Set up candle object
# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_8M,True)
# Ping FDCAN bus in search of drives
ids = candle.ping() # ids contains the drivers'id
if len(ids) == 0: # If no drives found -> quit
    sys.exit("EXIT FALIURE") 

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

test_motor_id = 110 # ankle_motor_5
test_md80_index = 0
for i in range(len(ids)):
    if ids[i] == test_motor_id:
        candle.controlMd80SetEncoderZero(ids[i])       #  Reset encoder at current position
        candle.controlMd80Mode(ids[i], pyCandle.IMPEDANCE)   # Set mode to position Impedence PD control
        candle.controlMd80Enable(ids[i], True)    # Enable the drive <=> Servo ON
        test_md80_index = i

sampling_time = 0.002 # sec
T = 0.5 # sec, for stepping test
duration = 2
loop_range = int((duration)/sampling_time)
theta_d = 20*np.pi/180 # in rad, ref angle for stepping test
sine_freq = 0.3 # Hz, frequency of the sinewave signal

t = 0.0

data = []
time_arr = []
ref_angl = []
ref_vel = []
motor_angl = []
fb_vel = []

print(candle.md80s[0].getPosition())

candle.begin()

for i in range(loop_range):
    t += sampling_time

    #angle_ref, angle_vel_ref = sin_ref(sine_freq,t)
    angle_ref, angle_vel_ref = step_ref(t,T,theta_d)

    candle.md80s[test_md80_index].setTargetPosition(angle_ref)
    candle.md80s[test_md80_index].setTargetVelocity(angle_vel_ref)

    fb_angl = candle.md80s[test_md80_index].getPosition()
    fb_vel = candle.md80s[test_md80_index].getVelocity()
    fb_tqr = candle.md80s[test_md80_index].getTorque()

    time_arr.append(t)
    ref_angl.append(angle_ref)
    motor_angl.append(fb_angl)

    time.sleep(sampling_time)

candle.end()


ref_angl = np.array(ref_angl)*180/np.pi
fb_angl = np.array(fb_angl)*180/np.pi

plt.figure()
plt.plot(time_arr,ref_angl, label = "ref_angle")
plt.plot(time_arr,motor_angl, label = "feedback_angle")
plt.show()
