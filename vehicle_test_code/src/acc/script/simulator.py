#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool

from nav_msgs.msg import Odometry

class VehicleSimulator:
    def __init__(self):
        rospy.init_node('vehicle_simulator', anonymous=True)
        
        # Physics Parameters
        self.dt = 0.1              # 20 Hz loop rate
        self.max_accel = 1.0        # m/s^2 maximum acceleration
        self.drag_decay = 0.99      # 1% speed decay per tick due to drag/rolling resistance
        
        # Ego Vehicle States
        self.v_ego = 0.0
        self.p_ego = 0.0
        self.throttle_cmd = 0.0
        
        # Lead Vehicle States (Hardcoded setup)
        self.v_lead = 0.0           # lead vehicle moving at constant 0 m/s
        self.p_lead = 60.0          # starts 60 meters ahead
        
        self.radar_detection_range = 30.0
        
        # Publishers (Sensor Emulators)
        self.pub_lead_valid = rospy.Publisher('/lead_valid', Bool, queue_size=10)
        self.pub_lead_dist = rospy.Publisher('/lead_distance', Float32, queue_size=10)
        self.pub_lead_vel = rospy.Publisher('/lead_relative_velocity', Float32, queue_size=10)
        self.pub_vel_fb = rospy.Publisher('/velocity_feedback', Float32, queue_size=10)
        self.pub_odom = rospy.Publisher('/fixposition/odometry_enu', Odometry, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/motor_command', Float32, self.cmd_callback)
        
    def cmd_callback(self, msg):
        # Throttle command from ACC PID node [-1 to 1]
        self.throttle_cmd = msg.data

    def run(self):
        rate = rospy.Rate(10) # 20 Hz
        
        while not rospy.is_shutdown():
            # 1. Update Ego Vehicle Physics
            # a = throttle * max_accel
            acceleration = self.throttle_cmd * self.max_accel 
            
            # v_new = drag_decay * v_old + a * dt
            self.v_ego = (self.drag_decay * self.v_ego) + (acceleration * self.dt)
            
            # Prevent reversing for this simple ACC test
            if self.v_ego < 0:
                self.v_ego = 0.0
                
            self.p_ego += self.v_ego * self.dt
            
            # 2. Update Lead Vehicle Physics
            self.p_lead += self.v_lead * self.dt
            
            # 3. Calculate Relative States
            rel_dist = self.p_lead - self.p_ego
            rel_vel = self.v_lead - self.v_ego
            
            # 4. Radar Logic
            if 0 < rel_dist <= self.radar_detection_range:
                self.pub_lead_valid.publish(Bool(True))
                self.pub_lead_dist.publish(Float32(rel_dist))
                self.pub_lead_vel.publish(Float32(rel_vel))
            else:
                self.pub_lead_valid.publish(Bool(False))
                # Publish 100m to match C++ init logic for out-of-range
                self.pub_lead_dist.publish(Float32(100.0)) 
                self.pub_lead_vel.publish(Float32(0.0))
                
            # 5. Ego Velocity / IMU Logic
            self.pub_vel_fb.publish(Float32(self.v_ego))
            
            odom_msg = Odometry()
            odom_msg.twist.twist.linear.x = self.v_ego
            self.pub_odom.publish(odom_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        sim = VehicleSimulator()
        rospy.loginfo("Vehicle Simulator Started.")
        sim.run()
    except rospy.ROSInterruptException:
        pass