"""  controller without the optimization """

"""
reciver listens to channel 4

emitter is on channel 5
"""

import math
import numpy as np


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
acelo = driver.getDevice("accelerometer")
acelo.enable(time_step)
ah = acelo.getValues()




ts = 0.1
time_step = int(1000*ts)
""" do moving average of 5/4 eliments since time step here is 1/5th """

gps.enable(time_step)  ##  time step corresponds to wait time in mili seconds
acelo.enable(time_step)
receiver.enable(time_step)
receiver_ref.enable(time_step)

driver.setGear(1)
T_hw = 1.3 ## also mentioned in Supervisor
d0 = 3
previous_message = ''

xr_d = -55
xr_v = 0

commands = np.array([0,0])

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)


while driver.step() != -1:
    if receiver_ref.getQueueLength() > 0:
        message = receiver_ref.getBytes()
        coord = rnd(np.frombuffer(message, dtype=np.float64))
        xr_d = coord[0]
        xr_v = coord[1]
        print(f"this is the ref vehicle coord from straightlien_rec-- {coord} ")
        receiver_ref.nextPacket()

    #vehicle paramters
    gps_car = gps.getValues()
    xh = gps_car[0]  ## cars y axis is along the lane
    # print("this is xh", xh)
    vh = driver.getCurrentSpeed()
    ah = acelo.getValues()
    # print("this is ah" , ah)

    if math.isnan(vh):
        vh = 0
    if math.isnan(ah[0]):
        ah[0] = 0
    if math.isnan(xh):
        xh = 0

    #create reference  #
    d_c = -xr_d + xh
    # d_c = -xr_d - (-xh + T_hw*vh + d0) # reference del d
    print(f"this is dc {rnd(d_c)} xrd {xr_d} xh {xh} Thw*vh {rnd(T_hw*vh)}")
    v_c = xr_v - vh # reference del v
    xc = rnd(np.array([d_c, v_c, -ah[0]])) # referance state
    # print("this is the states" , xc)
    message = xc.tobytes()

    #send reference
    if message != '' : # and message != previous_message:
        previous_message = message
        emitter.send(message)
        # print("sent")

    ## recive command and execute command
    # print("this is q length" ,receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        commands = rnd(np.frombuffer(message, dtype=np.float64))
        print(f"this is the commands from straightlien_rec-- {commands} ")
        receiver.nextPacket()

    # apply the commands
    # driver.setThrottle(1)
    # 0 - accel and 1 - brake
    if math.isnan(commands[1]):
        driver.setThrottle(commands[0])
        print("setting throttle to", commands[0])
    elif commands[1] == -10:
        driver.setBrakeIntensity(1)
        print("brake intensity to" , int(1))
    else :
        driver.setBrakeIntensity(commands[1])
        print("brake intensity to",commands[1])
