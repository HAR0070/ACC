#!/usr/bin/env python3
import pygame
import serial
import time
import sys
import datetime
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from collections import namedtuple


DEADZONE = 0.1
SEND_INTERVAL = 0.01

KP = 10
KD = 0.5
KI = 0.5
SPD_REF = 4000
CUR_LIM = 2.5

fb = namedtuple("fb" , ["pos" , "spd" , "cur"  , "temp" , "err"])

class steering_fb(Node):

    def __init__(self):
        super().__init__('steering_fb')
        self.pub_str = self.create_publisher(Twist, '/steering_pub', 10)
        self.pub_accel = self.create_publisher(Twist , '/vehicle_throttle' , 10)
        self.sub_str_fb = self.create_subscription(Float32MultiArray , '/steering_feedback' ,
                                                self.read_steering_feedback, 10 )

        self.sub_str_fb

        timer_period = SEND_INTERVAL  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # State variables
        self.integral_err = 0
        self.controller = None

        self.str_fb = fb(0,0,0,0,0)

        # Initialization
        self.get_logger().info("Initializing Controller and Serial...")

        try:
            self.find_controller()
            self.steering = self.controller.get_axis(0)
            self.takeover = self.controller.get_button(7)
            self.reset = self.controller.get_button(6)
            self.throttle = self.controller.get_button(3)

        except Exception as e:
            print(f"couldn't connect to controller: {e}")



        # This is CRITICAL must pump the event queue.
        pygame.event.pump()

    def find_controller(self):
        """Initializes pygame and finds the first available joystick."""
        print("Initializing controller...")
        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("Error: No joystick or controller found.")
            return None

        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Found controller: {joystick.get_name()}")
        print(f"Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}")

        self.controller = joystick

    def to_twist(self, x , y=None):
        msg = Twist()
        msg.linear.x = float(x)
        if y:
            msg.linear.y = float(y)

        return msg

    def read_steering_feedback(self , msg):

        self.str_fb = fb(*msg.data)
        # self.str_fb.spd = msg.data[1]
        # self.str_fb.cur = msg.data[2]
        # self.str_fb.temp = msg.data[3]
        # self.str_fb.err = msg.data[4]

        self.get_logger().info("Steering feedback came in")
        # self.get_logger().info(*msg.data)

    def pid_pos_vel(self, ref_pos , pos_fb , speed_fb , curr_fb , integral_err):

        pos_err = ref_pos - pos_fb          # if ref > current - velocity -ve
        spd_err = -speed_fb

        if abs(pos_err) < 1.5:
            integral_err = 0
        else:
            integral_err += pos_err

        velocity = KP * pos_err + KD * spd_err + KI*integral_err ## positive position error -> Positive velocity

        if velocity > SPD_REF:
            velocity = SPD_REF
            integral_err -= pos_err
        elif -velocity > SPD_REF:
            velocity = -SPD_REF
            integral_err -= pos_err

        if curr_fb > CUR_LIM:
            print(f"your hitting current limit {curr_fb}")
            velocity *=0.5

        return velocity , integral_err , curr_fb > CUR_LIM

    def map_axis_to_position(self, axis_value, range, inverted=False):
        """Converts a joystick axis (-1.0 to 1.0) to a velocity (-100 to 100)."""
        # Apply deadzone
        if abs(axis_value) < DEADZONE:
            axis_value = 0.0

        # Map the value from [-1.0, 1.0] to [-520, 520]
        position = int(axis_value * range)

        if inverted:
            position = -position

        return int(position)

    def timer_callback( self ):

        try:
            # This is CRITICAL must pump the event queue.
            pygame.event.pump()
            
            self.steering = self.controller.get_axis(0)
            self.takeover = self.controller.get_button(7)
            self.reset = self.controller.get_button(6)
            self.throttle = self.controller.get_button(3)

            steering = self.map_axis_to_position(self.steering , 3100)
            throttle = self.map_axis_to_position(self.throttle , 128)

            cmd_takeover = self.takeover        # if we want to let driver takeover
            reset = self.reset              # get joystick command back
            allow_control = True

            # steering
            if self.str_fb.err == 0:
                vel_x , self.integral_err ,takeover  = self.pid_pos_vel(steering , self.str_fb.pos, self.str_fb.spd,
                                                            self.str_fb.cur , self.integral_err)

                msg_steering = self.to_twist(vel_x)
            else :
                self.get_logger().error("The steering motor has error")
                allow_control = False
                raise

            # throttle
            msg_throttle = self.to_twist(throttle)

            if cmd_takeover > 0.1 or takeover:
                allow_control = False

            if reset:
                allow_control = True

            if allow_control:
                self.pub_str.publish(msg_steering)
                self.pub_accel.publish(msg_throttle)
            
            self.get_logger().info(f" allow_control = {allow_control} vel = {vel_x} pos_ref = {steering} pos_fb = {self.str_fb.pos} takeover = {takeover} cmd_tak = {cmd_takeover}")

        except KeyboardInterrupt:
            print("\nExiting program.")
            sys.exit(1)

        except Exception as e:
            print(f"\n Serial write error: {e}")
            raise

    def cleanup(self):
        """Explicit cleanup function called on shutdown."""
        self.get_logger().info("Shutting down...")
        pygame.joystick.quit()
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    ros_node = steering_fb()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup happens here, NOT in the timer loop
        ros_node.cleanup()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
