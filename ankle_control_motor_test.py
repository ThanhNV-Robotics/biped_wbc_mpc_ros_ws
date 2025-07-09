import pyCandle

import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import json
from low_level_fcn import ankle_kinematics

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

def init_motor (candle_device):
    # Ping FDCAN bus in search of drives
    ids = candle_device.ping() # ids contains the drivers'id
    if len(ids) == 0: # If no drives found -> quit
        sys.exit("EXIT FALIURE") 

    # Add all found to the update list
    for id in ids:
        candle_device.addMd80(id)

    for i in range(len(ids)):    
        candle_device.controlMd80SetEncoderZero(ids[i])             #  Reset encoder at current position
        candle_device.controlMd80Mode(ids[i], pyCandle.IMPEDANCE)   # Set mode to position Impedence PD control
        candle_device.controlMd80Enable(ids[i], True)               # Enable the drive <=> Servo ON
    
    

def main ():
    sampling_time = 0.002 # sec
    T = 0.5 # sec, for stepping test
    duration = 2
    t = 0.0
    loop_range = int((duration)/sampling_time)

    data = []
    time_arr = []
    ref_angl = []
    ref_vel = []
    fb_angl = []
    fb_vel = []

    # ──────────────────────────────────────────────────────────────────────
    #   init candle hat and motors
    # ──────────────────────────────────────────────────────────────────────
    candle = pyCandle.Candle(pyCandle.CAN_BAUD_8M,True)
    init_motor(candle_device= candle)
    candle.begin() # servo ON

    for i in range(loop_range):
        t += sampling_time

        #angle_ref, angle_vel_ref = sin_ref(sine_freq,t)
        angle_ref, angle_vel_ref = step_ref(t,T,np.pi/2)

        candle.md80s[0].setTargetPosition(angle_ref)
        candle.md80s[0].setTargetVelocity(angle_vel_ref)

        fb_angl = candle.md80s[0].getPosition()
        fb_vel = candle.md80s[0].getVelocity()
        fb_tqr = candle.md80s[0].getTorque()

        data.append([t, angle_ref, angle_vel_ref, fb_angl, fb_vel, fb_tqr])

        time.sleep(sampling_time)

    candle.end()

if __name__ == "__main__":
    main()
