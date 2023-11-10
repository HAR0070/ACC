"""  controller without the optimization """

"""
supervisor reciver listens to channel 4
reference reciver at channel 8

emitter is on channel 5
"""

import math
import numpy as np
import time as time


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
T_hw = 0.5 ## also mentioned in Supervisor
d0 = 1.5
previous_message = ''

xr_d = -55
xr_v = 0

commands = np.array([0,0])

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)


def get_filtered_acel(acel, t):
    """Filter the speed command to avoid abrupt speed changes."""
    get_filtered_acel.previousAcel = np.append(get_filtered_acel.previousAcel, acel)
    get_filtered_acel.prevTime =  np.append(get_filtered_acel.prevTime , t)
    if len(get_filtered_acel.previousAcel) > 10:  # keep only 10 values
        get_filtered_acel.previousAcel = get_filtered_acel.previousAcel[1:]
        get_filtered_acel.prevTime = get_filtered_acel.prevTime[1:]

    sum = np.sum(np.diff(get_filtered_acel.previousAcel))
    t_diff = np.sum(np.diff(get_filtered_acel.prevTime))
    ah = sum / t_diff/10
    if math.isnan(ah):
        ah = 0
    return ah

get_filtered_acel.previousAcel = np.array([0,0])
get_filtered_acel.prevTime = np.array([0,0])


cmd = [[0,0]]

while driver.step() != -1:
    # driver.setThrottle(1)
    if receiver_ref.getQueueLength() > 0:
        message = receiver_ref.getBytes()
        coord = rnd(np.frombuffer(message, dtype=np.float64))
        xr_d = rnd(coord[0])
        xr_v = rnd(coord[1])
        # print(f"this is the ref vehicle coord from reference controller-- {coord} ")
        receiver_ref.nextPacket()

    #vehicle paramters
    gps_car = gps.getValues()
    xh = rnd(gps_car[0])
    # print("this is xh", xh)
    vh = rnd(gps.getSpeed())
    t = time.time()
    ah = get_filtered_acel(vh , t)

    # print("this is ah" , ah)

    if math.isnan(vh):
        vh = 0
        # print("vh is nan")
    if math.isnan(ah):
        ah = 0
        # print("ah is nan")
    if math.isnan(xh):
        xh = 0

    d_c = -xr_d - (-xh +d0 + T_hw*vh) # reference del d
    # print(f"this is dc {rnd(d_c)} xrd {xr_d} xh {xh} vh = {vh} Thw*vh {rnd(T_hw*vh)}")
    v_c = xr_v - vh # reference del v


    # for MPC the message is
    xc = rnd(np.array([[d_c], [v_c], [ah]]))

    ### For Astar the message is
    # xc = rnd(np.array([abs(xr_d)-d0, xr_v, abs(xh),vh, ah])) # referance state

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
        path = np.frombuffer(message, dtype=np.float64)
        path = np.array(path, dtype=np.float64)

        # path = path.reshape((-1,1))
       
        if np.isnan(path[0]):
            path[0] = -3
        # print(f"this is the commands from cmd device -- {commands} ")
        receiver.nextPacket()
        cmd = np.empty((0,2))
        if not isinstance(path, str) and len(path) >= 0 and path[0]!= -100:
            for a in path:
                if  a >=0:
                    brake = np.nan
                    accel = a/2.5    ########### don't divide by 2.5 for A*
                    cmd = np.vstack((cmd,[accel, brake]))
                else:
                    accel = np.nan
                    brake = a/-3.5
                    cmd = np.vstack((cmd,[accel, brake]))
            print(f"this is cmd {cmd}")
            commands = cmd[0]
            cmd = cmd[1:]
            
            if math.isnan(commands[1]):
                driver.setBrakeIntensity(0)
                driver.setThrottle(commands[0])
                print("setting throttle to", commands[0])
            else :
                driver.setBrakeIntensity(commands[1])
                print("brake intensity to",commands[1])
            
        else:
            driver.setBrakeIntensity(1)
            print("No solution brake intensity to" , int(1))
        
    elif len(cmd)>0:
        # print("this is alternate path try")
        commands = cmd[0]
        cmd = cmd[1:]
        print(f"this is commands {commands} in alternate path")
        if math.isnan(commands[1]):
            driver.setBrakeIntensity(0)
            driver.setThrottle(commands[0])
            print("setting throttle in alternate path to", commands[0])
        else :
            driver.setBrakeIntensity(commands[1])
            print("brake intensity in alternate path to",commands[1])
            
    else:
        pass
