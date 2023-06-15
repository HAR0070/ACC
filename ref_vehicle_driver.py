"""
this is  working type_check - or communicating controller
emitter is at 8

"""

import math
import numpy as np
from vehicle import Driver


driver = Driver()
# receiver = driver.getDevice("receiver")
emitter = driver.getDevice("emitter")
gps = driver.getDevice("gps")

ts = 0.05
time_step = int(1000*ts)

# receiver.enable(time_step)
gps.enable(time_step)  ##  number corresponds to frequency
# emitter.enable(time_step)

previous_message = ''

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
a = 0
while driver.step() != -1:
    a += 1
    # if a > 300:
        # driver.setCruisingSpeed(10)

    gps_car = gps.getValues()
    xh = gps_car[0]
    vh = driver.getCurrentSpeed()

    if math.isnan(vh):
        vh = 0
    if math.isnan(xh):
        xh = 0

    xc = rnd(np.array([xh, vh, 0]) )# referance state
    # print("this is xc" , xc)

    message = xc.tobytes()

    #send reference
    if message != '' : # and message != previous_message:
        previous_message = message
        emitter.send(message)
