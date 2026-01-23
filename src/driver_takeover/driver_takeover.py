import rclpy
from rclpy.node import Node
import json
import math
import numpy as np
import joblib 
import os # Added for path handling

from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray, Bool
from fixposition_driver_msgs.msg import FpaOdomenu

import warnings
warnings.filterwarnings("ignore")

# Load model safely
MODEL_PATH = "model/boost_tree_v3"
FEATURE_PATH = "model/feature_list"

try:
    model = joblib.load(MODEL_PATH)
except FileNotFoundError:
    print(f"WARNING: Model file {MODEL_PATH} not found. Prediction will fail.")
    model = None

debug = True

# Motor params
POLE_PAIR = 21
GR = 8
KT = 0.199

# --- FIX 1: Use a Mutable Class instead of namedtuple ---
class BaseInputState:
    def __init__(self):
        self.spd_fb = 0.0
        self.pos = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        
    def __repr__(self):
        return f"spd={self.spd_fb:.2f}, pos={self.pos:.2f}, vx={self.vx:.2f}, vy={self.vy:.2f}, yaw={self.yaw:.2f}"

class predict_steering_takeover(Node):
    
    def __init__(self):
        super().__init__('predict_steering_takeover')
        
        self.pub_takeover = self.create_publisher(Bool , '/steering_takeover' , 10)
        time_period = 0.05 
        self.timer = self.create_timer(time_period , self.timer_callback)
        
        self.sub_imu  = self.create_subscription(FpaOdomenu , '/fixposition/fpa/odomenu' , self.imu_callback , 10 )
        self.sub_steering = self.create_subscription(Float32MultiArray , '/steering_feedback' , self.steering_fb , 10)
        
        self.basex = BaseInputState()
        self.torque_now = 0.0
        
        self.base_dim = 5   
        self.max_hist = 10 

        # Indices
        self.IDX_SPD = 0
        self.IDX_POS = 1
        self.IDX_VX  = 2
        self.IDX_VY  = 3
        self.IDX_YAW = 4

        try:
            # Check if file exists to prevent crash
            if os.path.exists(FEATURE_PATH):
                with open(FEATURE_PATH, "r") as f:
                    self.features_list = json.load(f)
            else:
                print(f"Warning: {FEATURE_PATH} not found, using default.")
                raise FileNotFoundError
        except Exception as e:
            self.features_list = ['torque_lag_1', 'torque_smooth_5', 'steering_spd',
                                'pos_diff_1', 'steering_acel_diff_3', 'steering_acel_diff_4', 
                                'pos_diff_2', 'steering_spd_lag_3', 'yaw_term_diff_1', 'pos']

        self.feat_dict = dict(zip(self.features_list, [0.0]*len(self.features_list)))

        self.base_hist = np.zeros((self.max_hist, self.base_dim))
        self.torque_hist = np.zeros(self.max_hist)
        
        self.valid_steps = 0
        self.ewma_torque_5 = 0.0
        self.a5 = 2 / (5 + 1)
        self.prev_preds = []
        self.takeover_count = 0
        
    def timer_callback(self):
        msg = Bool()
        msg.data = bool(self.predict()) # Ensure bool type
        self.pub_takeover.publish(msg)
        
        if debug :
            # self.basex now works because we defined __repr__ in BaseInputState
            print(f"State: {self.basex} | Takeover: {msg.data}")
    
    def steering_fb(self, msg):
        # --- FIX 1 Usage: Now we can assign values ---
        self.basex.pos = float(msg.data[0])
        self.basex.spd_fb = float(msg.data[1]*(POLE_PAIR*GR))
        self.torque_now = float(msg.data[2]*KT)

    def imu_callback(self , msg):
        q = msg.pose.pose.orientation
        v = msg.velocity.twist.linear
        
        yaw, pitch, roll = R.from_quat(
                [q.x, q.y, q.z, q.w]
            ).as_euler("zyx", degrees=True)
        
        self.basex.vx = float(v.x)
        self.basex.vy = float(v.y)
        self.basex.yaw  = float(yaw)

    def update(self, base_in, torque):
        # 1. Shift History
        self.base_hist[1:] = self.base_hist[:-1]
        self.torque_hist[1:] = self.torque_hist[:-1]

        # 2. Add New Data
        self.base_hist[0] = [base_in.spd_fb, base_in.pos, base_in.vx, base_in.vy, base_in.yaw]
        self.torque_hist[0] = torque

        # 3. EWMA Calculation
        if self.valid_steps == 0:
            self.ewma_torque_5 = torque
        else:
            self.ewma_torque_5 = self.a5 * torque + (1 - self.a5) * self.ewma_torque_5

        self.valid_steps += 1

        # --- FEATURE CALCULATION ---
        spd_0 = self.base_hist[0][self.IDX_SPD]
        spd_1 = self.base_hist[1][self.IDX_SPD]
        spd_3 = self.base_hist[3][self.IDX_SPD]
        
        pos_0 = self.base_hist[0][self.IDX_POS]
        pos_1 = self.base_hist[1][self.IDX_POS]
        pos_2 = self.base_hist[2][self.IDX_POS]

        self.feat_dict['torque_lag_1'] = self.torque_hist[1]
        self.feat_dict['torque_smooth_5'] = self.ewma_torque_5
        self.feat_dict['steering_spd'] = spd_0
        self.feat_dict['pos'] = pos_0
        self.feat_dict['pos_diff_1'] = pos_0 - pos_1
        self.feat_dict['pos_diff_2'] = pos_0 - pos_2
        self.feat_dict['steering_spd_lag_3'] = spd_3

        # Acceleration Features
        acel_0 = spd_0 - spd_1
        acel_3 = self.base_hist[3][self.IDX_SPD] - self.base_hist[4][self.IDX_SPD]
        acel_4 = self.base_hist[4][self.IDX_SPD] - self.base_hist[5][self.IDX_SPD]

        self.feat_dict['steering_acel_diff_3'] = acel_0 - acel_3
        self.feat_dict['steering_acel_diff_4'] = acel_0 - acel_4

        # Yaw Term Diff
        def calc_yaw_term(idx):
            dy = self.base_hist[idx][self.IDX_YAW] - self.base_hist[idx+1][self.IDX_YAW]
            vx = self.base_hist[idx][self.IDX_VX]
            vy = self.base_hist[idx][self.IDX_VY]
            v_spd = math.sqrt(vx**2 + vy**2) + 1e-6 
            return (dy / 0.05) / v_spd 

        yaw_term_0 = calc_yaw_term(0)
        yaw_term_1 = calc_yaw_term(1)
        
        self.feat_dict['yaw_term_diff_1'] = yaw_term_0 - yaw_term_1

    def build_features(self):
        if self.valid_steps < 6: 
            return None
        feature_vector = []
        for name in self.features_list:
            feature_vector.append(self.feat_dict[name])
        return np.array(feature_vector)

    def predict(self):
        if model is None: return False

        self.update(self.basex, self.torque_now)
        x = self.build_features()
        if x is None: return False

        y_pred = model.predict(x.reshape(1, -1))[0]

        self.prev_preds.append(y_pred)
        if len(self.prev_preds) > 10: self.prev_preds.pop(0)
        if len(self.prev_preds) < 5: return False

        std = np.std(self.prev_preds)
        if std < 0.01: std = 0.01 

        if self.torque_now > y_pred + 0.5 * std:
            self.takeover_count += 1
        else:
            self.takeover_count = max(0, self.takeover_count - 1)

        return self.takeover_count > 3

    def cleanup(self):
        print("Shutting down node...")
        self.destroy_publisher(self.pub_takeover)
        self.destroy_subscription(self.sub_imu)
        self.destroy_subscription(self.sub_steering)
        self.destroy_timer(self.timer)

"""         How features are read while training 
            row.update({
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
                "vx":v.x,
                "vy":v.y,
                "vz":v.z,
                "v_speed": (v.x**2 + v.y**2) ** 0.5,
                "yaw_rate": w.x, 
                "pitch_rate": w.y,
                "roll_rate": w.z
                # "body_x": v_body[0],
                # "body_y": v_body[1],
                # "body_z": v_body[2]
            })

        # -------- motor feedback --------
        if c.topic == '/motor_feedback':
            row.update({
                "pos": msg.data[0],
                "rpm": msg.data[1]*(POLE_PAIR*GR),
                "torque": msg.data[2]*KT,
                "temp": msg.data[3],
                "err": msg.data[4]
            })

        # -------- steering --------
        if c.topic == '/steering_pub':
            row.update({
                "steering_vel": msg.linear.x
            })
"""

def main(args=None):
    rclpy.init(args=args)
    ros_node = predict_steering_takeover()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.cleanup()
        ros_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()