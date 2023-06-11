"""  controller without the optimization """

"""
reciver listens to channel 4

emitter is on channel 5
"""

import math
import numpy as np
import csv
import time

from vehicle import Driver

# initialization
driver = Driver()
gps = driver.getDevice("gps")
acelo = driver.getDevice("accelerometer")
receiver = driver.getDevice("cmd receiver")
receiver_ref = driver.getDevice("ref receiver")
emitter = driver.getDevice("emitter")

ts = 0.1
time_step = int(1000*ts)
""" do moving average of 5/4 eliments since time step here is 1/5th """

gps.enable(time_step)  ##  time step corresponds to wait time in mili seconds
acelo.enable(time_step)
receiver.enable(time_step)
receiver_ref.enable(time_step) 

driver.setGear(1)
T_hw = 1.6 ## also mentioned in Supervisor
d0 = 3
previous_message = ''

xr_d = -55
xr_v = 0

commands = np.array([0,0]) 

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number, np.ndarray):
        return np.round(number, precision)
a = 0
with open('step_input_accel.csv', 'w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(['time','pos', 'vel','vel_gps', 'accel'])      

    while driver.step() != -1:
        a += 1
        if a > 100:
            driver.setThrottle(1)
    
            # Vehicle parameters
            gps_car = gps.getValues()
            xh = gps_car[0]  ## cars y-axis is along the lane
            vh = driver.getCurrentSpeed()
            vh2 = gps.getSpeed()
            ah = acelo.getValues()
    
            # Handle NaN values
            if math.isnan(vh):
                vh = 0
            if math.isnan(ah[0]):
                ah[0] = 0
            if math.isnan(xh):
                xh = 0       
            t = time.time()
            writer.writerow([t,xh, vh, vh2 ,ah])
    

