

"""
driver.setThrottle(1.0)
driver.setGear(1)
driver.setBrakeIntensity(0.5)

"""



"""vehicle_driver controller."""

import math
from vehicle import Driver
from controller import Emitter
from controller import Receiver

import struct


def get_filtered_speed(speed):
    """Filter the speed command to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))

get_filtered_speed.previousSpeeds = []
# Create a driver instance
driver = Driver()
gps = driver.getDevice("gps")
gps.enable(10) # Get a reference to the GPS device
# Enable the GPS device
# gps.enable(driver.getBasicTimeStep())

# Get a reference to the emitter device for sending GPS data to the first car
emitter = driver.getDevice("emitter")

driver.setCruisingSpeed(15)
driver.setSteeringAngle(0)
delimiter = ","

a = 0
b = 0

while driver.step() != -1:
    # Get the GPS data
    a += 1
    if a == 100:
        a = 0
        b += 1
        driver.setCruisingSpeed(15 + b)

    gps_data = gps.getValues()
    speed1 = driver.getCurrentSpeed()
    speed = get_filtered_speed(speed1)

    if math.isnan(speed):
        if math.isnan(speed1):
            speed = 0
        else:
            speed = speed1

    message = str(gps_data[0]) + delimiter + str(speed)
    # string_message = message.encode("utf-8")
    # print("robot handle emitter string message to send:", string_message)
    if emitter.send(message):
        print("message sent")
    # else:
        # print("emitter queue full")

######################################################################
####################33
######################
#######################################################################

"""reciver controller."""

import math
from vehicle import Driver
from controller import Emitter
from controller import Receiver

import struct


def get_filtered_speed(speed):
    """Filter the speed command to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))


driver = Driver()
gps = driver.getDevice("gps")
gps.enable(10)   # input is sampling period in milli second
# when speed is obtained from gps it is in m/s

# Get a reference to the receiver device for receiving GPS data from the second car
receiver = driver.getDevice("receiver")

# Enable the receiver device
receiver.enable(10)
# Enable the receiver device
get_filtered_speed.previousSpeeds = []
driver.setCruisingSpeed(20)
driver.setSteeringAngle(0)
delimiter = ","

while driver.step() != -1:
    # Receive GPS data from the second car
    if receiver.getQueueLength() > 0:
       gps_data = receiver.getString()
       split_numbers = gps_data.split(delimiter)
       x = float(split_numbers[0])
       v = float(split_numbers[1])

       # print("this is the data type" , type(gps_data))
       print("Received GPS data:", x,v)
       driver.setCruisingSpeed(v)

    receiver.nextPacket()




if message != '' and message != previous_message:
    previous_message = message
    print('Please, ' + message)
    self.emitter.send(message.encode('utf-8'))
message = pickle.dumps(xc)
data_string = message.tostring()

if self.receiver.getQueueLength() > 0:
    message = self.receiver.getString()
    self.receiver.nextPacket()
# deserialized_data = pickle.loads(serialized_data)
deserialized_data = np.fromstring(data_string, dtype=np.float64) # converts directly from string to numpy array


###############################################33
#####################################################
####################################################33333
"""supervisor_contr controller."""


"""
this is minimal supervisor controller -- working
reciver listens to channel 5

emitter is on channel 4
"""

import numpy as np
import pickle

from controller import Supervisor
from controller import Emitter
from controller import Receiver



TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")

receiver.enable(TIME_STEP)
# emitter.enable(TIME_STEP)

# [CODE PLACEHOLDER 1]
ref = robot.getFromDef('ref_veh')
translation_field = ref.getField('translation')

pos_ref =  translation_field.getSFVec3f()

previous_message = ''

i = 0
while robot.step(TIME_STEP) != -1:
    pos_ref = np.array(translation_field.getSFVec3f())
    # print(f"this is pos_ref {pos_ref} " )
    # message = pickle.dumps(pos_ref)
    message = pos_ref.tobytes()

    #send reference
    if message != '' and message != previous_message:
        previous_message = message
        emitter.send(message)

    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        params = np.frombuffer(message, dtype=np.float64)
        # commands = np.fromstring(message, dtype=np.float64)
        print(f"this is params from supervisor controller {params} ")
        receiver.nextPacket()


####################################################3
################################################333333333
######################################################
"""
this is  working type_check - or communicating controller
emitter is at 5
reciver is at 4
"""
import math
import numpy as np
from vehicle import Driver


driver = Driver()
receiver = driver.getDevice("receiver")
emitter = driver.getDevice("emitter")
gps = driver.getDevice("gps")
acelo = driver.getDevice("accelerometer")

time_step = 20


receiver.enable(time_step)
gps.enable(time_step)  ##  number corresponds to frequency
acelo.enable(time_step)
# emitter.enable(time_step)

previous_message = ''

while driver.step() != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        commands = np.frombuffer(message, dtype=np.float64)
        # commands = np.fromstring(message, dtype=np.float64)
        print(f"this is command from type check {commands} ")
        receiver.nextPacket()

    gps_car = gps.getValues()
    xh = gps_car[0]  ## cars y axis is along the lane
    vh = driver.getCurrentSpeed()
    ah = acelo.getValues()

    if math.isnan(vh):
        vh = 0
    if math.isnan(ah[1]):
        ah[1] = 0
    if math.isnan(xh):
        xh = 0

    #create reference  #
    d_c =  xh
    # d_c = -xr_d - (-xh + T_hw*vh + d0) # reference del d
    v_c = vh # reference del v
    xc = np.array([d_c, v_c, -ah[1]]) # referance state

    message = xc.tobytes()

    #send reference
    if message != '' and message != previous_message:
        previous_message = message
        emitter.send(message)
