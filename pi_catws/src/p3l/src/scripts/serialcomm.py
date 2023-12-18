#!/usr/bin/env python3
import rospy

import serial

from std_msgs.msg import Float32

#Global variables
recvInProgress=False
ardstateoff=True #Arduino is off

#Message object
ch1val=Float32()
ch2val=Float32()
ch3val=Float32()
ch4val=Float32()
ch5val=Float32()
ch6val=Float32()

def turn_off():
  #Function for safe shutdown 
  
  print('serialcomm node turning off')

  ch1val.data=0
  ch2val.data=0
  ch3val.data=0
  ch4val.data=0
  ch5val.data=-10
  ch6val.data=0

  pub1.publish(ch1val)
  pub2.publish(ch2val)
  pub3.publish(ch3val)
  pub4.publish(ch4val)
  pub5.publish(ch5val)
  pub6.publish(ch6val)

  #ser.reset_input_buffer()

  #ser.close()

def failurecase():

    global ardstateoff
    ardstateoff=True
    print('arduino is off')
    ch1val.data=0
    ch2val.data=0
    ch3val.data=0
    ch4val.data=0
    ch5val.data=-10
    ch6val.data=0
    pub1.publish(ch1val)
    pub2.publish(ch2val)
    pub3.publish(ch3val)
    pub4.publish(ch4val)
    pub5.publish(ch5val)
    pub6.publish(ch6val)

    rospy.loginfo("ARDUINO IS DISCONNECTED")





def waitForArduino():

  # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
  # it also ensures that any bytes left over from a previous message are discarded
  global ardstateoff
  
  print("Waiting for Arduino to be ready")
 



  while(ardstateoff):
      if(ser.is_open):
          ardstateoff=False #Arduino is on
      print("TURN ON ARDUINO")

  try:
    msg = ser.readline().decode("utf-8").rstrip()
  except:
      msg = ser.readline().decode("utf-8").rstrip()


  while msg.find("Arduino is ready") == -1:
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0):
      if rospy.is_shutdown(): 
        return False

    msg = ser.readline().decode("utf-8").rstrip()
 
  return True


def recv():
    rospy.loginfo("Recieving started")
    chtemp=''
    channels=['0','0','0','0','0','0']
    k=0
    startchar=60
    endchar=62
    spacechar=32
    datavalid=False

    recvchar='!'
    recvdata=''

    recvchar=ser.read()

    while(len(recvchar)==0):
        print('WAITING FOR DATA')
        recvchar=ser.read()

    while(ord(recvchar)!=startchar):
        recvchar=ser.read()
        print(recvchar)



    rospy.loginfo(recvchar)
    rospy.loginfo(ord(recvchar))
    #recvchar=ser.read()

    while(ord(recvchar)!=endchar):
        if(ord(recvchar)!=startchar):
            if(ord(recvchar)!=32):
                chtemp+=recvchar.decode('utf-8')
                recvchar=ser.read()
            else:
                channels[k]=chtemp
                chtemp=''
                k+=1
                recvchar=ser.read()
        else:
            k=0
            chtemp=''
            channels=['0','0','0','0','0','0']
            recvchar=ser.read()
    
    return channels
    


def start():

    rospy.init_node('serial_comm')
    rospy.on_shutdown(turn_off)
    #Publishers
    global pub1,pub2,pub3,pub4,pub5,pub6
    pub1 = rospy.Publisher('ch1', Float32)
    pub2 = rospy.Publisher('ch2', Float32)
    pub3 = rospy.Publisher('ch3', Float32)
    pub4 = rospy.Publisher('ch4', Float32)
    pub5 = rospy.Publisher('ch5', Float32)
    pub6 = rospy.Publisher('ch6', Float32)


    rate = rospy.Rate(10)



    #Connecting to Serial_Input
    global ser

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    while not waitForArduino():
        pass

    
    while not rospy.is_shutdown():


            channels=recv()
            chtemp=''
            rospy.loginfo(channels)

            ch1val.data=int(channels[0])
            ch2val.data=int(channels[1])
            ch3val.data=int(channels[2])
            ch4val.data=int(channels[3])
            ch5val.data=int(channels[4])
            ch6val.data=int(channels[5])

            pub1.publish(ch1val)
            pub2.publish(ch2val)
            pub3.publish(ch3val)
            pub4.publish(ch4val)
            pub5.publish(ch5val)
            pub6.publish(ch6val)
            rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
        failurecase()

    

