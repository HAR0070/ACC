#include "ros/ros.h"
#include "std_msgs/Float32.h"  // For throttle and brake
#include "std_msgs/Bool.h"    // For lead valid
#include "std_msgs/Float32MultiArray.h"  // To publish current states 
// #include "radar_msgs"  // For radar data  
// #include "sensor_msgs/IMU"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <system_error>

#include "mpc.hpp"
#include <cmath>
#include <iostream>

class RadarKalmanFilter {
private:
    float dr_, vr_;
    float P[2][2];
    float Q_dr, Q_vr; // Process noise Q (How much we trust our kinematic model)
    float R_dr, R_vr; // Measurement noise R (How much we trust the radar sensor)
    bool is_initialized_;
    int ghost_counter;

public:
    RadarKalmanFilter() {
        is_initialized_ = false;
        ghost_counter = 0;
        
        // Initial uncertainty
        P[0][0] = 10.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;  P[1][1] = 10.0f;
        
        // Process noise (tune these: lower means smoother but slower to react)
        Q_dr = 0.1f;  
        Q_vr = 0.1f;  
        
        // Measurement noise (tune these: based on radar spec)
        R_dr = 0.5f; 
        R_vr = 0.5f; 
    }

    // Initialize with first reliable radar reading
    void init(float initial_dr, float initial_vr) {
        dr_ = initial_dr;
        vr_ = initial_vr;
        is_initialized_ = true;
        
        // ADD THIS: Reset P matrix so old corrupted data doesn't ruin the new lock
        P[0][0] = 10.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;  P[1][1] = 10.0f;
    }

    bool is_initialized() const { return is_initialized_; }

    // Step 1: Predict where the target is based on kinematics
    void predict(float dt) {
        if (!is_initialized_) return;

        // State prediction: dr = dr + vr * dt
        dr_ = dr_ + vr_ * dt;
        // vr remains constant in this simple model
        
        // Covariance prediction: P = F * P * F^T + Q
        P[0][0] = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_dr;
        P[0][1] = P[0][1] + dt * P[1][1];
        P[1][0] = P[1][0] + dt * P[1][1];
        P[1][1] = P[1][1] + Q_vr;
    }

    // Step 2: Update based on new radar measurement
    void update(float meas_dr, float meas_vr) {
        if (!is_initialized_) {
            init(meas_dr, meas_vr);
            return;
        }

        // --- GATING / TARGET SWITCH LOGIC ---
        // Calculate the difference between measurement and prediction (Innovation)
        float diff_dr = meas_dr - dr_;
        
        // If the target jumps by more than 5 meters instantly, it's likely a target switch
        // or a sensor ghost. You can choose to ignore it, or reset the filter.
        if (std::abs(diff_dr) > 0.50f) { 
          ghost_counter++;
            
            // If jump persists for 5 frames (0.5s) - lock to that target 
            // for the first detection locking 
            if (ghost_counter > 5) {
                init(meas_dr, meas_vr);
                ghost_counter = 0;
            }

            // Ignore the spike and rely purely on the kinematic prediction
            return; 
        } else {
            ghost_counter = 0; // Valid measurement, reset counter
        }

        // KALMAN GAIN CALCULATION ---
        // S = P + R
        float S00 = P[0][0] + R_dr;
        float S01 = P[0][1];
        float S10 = P[1][0];
        float S11 = P[1][1] + R_vr;

        // Calculate Determinant of 2x2 matrix S
        float det = (S00 * S11) - (S01 * S10);
        
        // Calculate S Inverse
        float Sinv00 = S11 / det;
        float Sinv01 = -S01 / det;
        float Sinv10 = -S10 / det;
        float Sinv11 = S00 / det;

        // K = P * S^-1
        float K_00 = P[0][0] * Sinv00 + P[0][1] * Sinv10;
        float K_01 = P[0][0] * Sinv01 + P[0][1] * Sinv11;
        float K_10 = P[1][0] * Sinv00 + P[1][1] * Sinv10;
        float K_11 = P[1][0] * Sinv01 + P[1][1] * Sinv11;

        // 1. Calculate Innovation (Measurement Residual)
        float y_dr = meas_dr - dr_;
        float y_vr = meas_vr - vr_;
        
        // 2. Update States
        dr_ = dr_ + K_00 * y_dr + K_01 * y_vr;
        vr_ = vr_ + K_10 * y_dr + K_11 * y_vr;

        // 3. Update Covariance P = (I - K) * P
        float P00_new = P[0][0] - (K_00 * P[0][0] + K_01 * P[1][0]);
        float P01_new = P[0][1] - (K_00 * P[0][1] + K_01 * P[1][1]);
        float P10_new = P[1][0] - (K_10 * P[0][0] + K_11 * P[1][0]);
        float P11_new = P[1][1] - (K_10 * P[0][1] + K_11 * P[1][1]);
        
        P[0][0] = P00_new; P[0][1] = P01_new;
        P[1][0] = P10_new; P[1][1] = P11_new;
    }

    float get_dr() const { return dr_; }
    float get_vr() const { return vr_; }
};


class states {
    public: 
    float dr, vr, u_prev , vh , ah = 0 , lead_valid = 1; 
    states(float dr_, float vr_, float vh_ ,  float u_prev_ ) : dr(dr_), vr(vr_), vh(vh_), u_prev(u_prev_) {

    };

    void radar_dis(const std_msgs::Float32::ConstPtr& msg) {
        dr = msg->data;  // Distance to lead vehicle
    }
    void radar_rv(const std_msgs::Float32::ConstPtr& msg) {
        vr = msg->data;  // velocity to lead vehicle
    }

    void imu_callback(const nav_msgs::Odometry::ConstPtr& msg) { 
        vh = msg->twist.twist.linear.x;  // Longitudinal velocity
    }

    void odom_callback(const std_msgs::Float32::ConstPtr& msg) { 
        vh = msg->data;  // Longitudinal velocity
    }

    void lead_valid_(const std_msgs::Bool::ConstPtr& msg) { 
        if(msg->data) {
            lead_valid = 1;
        } 
        else lead_valid = 0;
    }

    void update_u(const float & u){
        u_prev = u; 
    }

};

std::string get_timestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);

    std::ostringstream oss;
    // Format: YYYY-MM-DD_HH-MM-SS (Safe for filenames)
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "acc_controller");
    ros::NodeHandle n;

    float dr , vr , vh, vmax , nominal_cmd; 
    float kp, kd, ki, thw, d0;
    bool use_imu , debug; 
    int controller; 

    n.getParam("/acc_node/model/thw", thw);
    n.getParam("/acc_node/model/d0", d0);
    n.getParam("/acc_node/pid/vmax", vmax);
    n.getParam("/acc_node/pid/nominal", nominal_cmd);

    vmax = vmax * (5.0f / 18.0f);  // input is in kmph -- sensor value is in m/s

    n.getParam("/acc_node/imu", use_imu);
    n.getParam("/acc_node/debug", debug);

    states current_states(100.0f, 0.0f, 0.0f, 0.0f); // Initialize dr = 100 - which is outside detection range
    // results in cruise control 
    RadarKalmanFilter kf;

    ros::Publisher state_pub = n.advertise<std_msgs::Float32MultiArray>("/states", 1);
    ros::Publisher throttle_pub = n.advertise<std_msgs::Float32>("/motor_command", 1);

    ros::Subscriber lead_dist = n.subscribe("/lead_distance", 2, &states::radar_dis, &current_states);
    ros::Subscriber lead_vel = n.subscribe("/lead_relative_velocity", 2, &states::radar_rv , &current_states);
    ros::Subscriber vel_sub;
    if(use_imu) vel_sub = n.subscribe("/fixposition/odometry_enu", 1, &states::imu_callback , &current_states); // Subscribe to imu data topic
    else vel_sub = n.subscribe("/velocity_feedback" , 1 , &states::odom_callback ,  &current_states);
    ros::Subscriber lead_valid = n.subscribe("/lead_valid", 2, &states::lead_valid_,  &current_states);
    
    ros::Rate loop_rate(10);
    float dt = 1.0f / 10.0f ; 
    std::string filename = "/home/asl-laptop2/HAR/acc/terminal_log/acc_pid_data_" + get_timestamp() + "_.csv";

    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return 0;
    }

    output_file << "Time,raw_dr,raw_vr,kf_dr,kf_vr,ego_v,d_c,v_c,del_d,del_v,del_a,mpc_mode,solver_status,control_action,final_cont_act\n";

    int target_lost_counter = 0;

     // MPC initialization   - first elem for accel and 2nd for brake
    std::vector<float> a = {0.5 , 0.5};
    std::vector<int> N = { 7 ,  7} ;
    std::vector<int> Nc = {15 , 15};
    std::vector<int> Np = {40 , 40};

    StateSpaceModel mpc;
    mpc.init_controller(a , N , Nc , Np);

    // --- STATE MEMORY VARIABLES ---
    bool imu_first_run = true; 
    bool mpc_first_run = true;
    float d_c_prev = 0.0f;
    float v_c_prev = 0.0f;
    float ah_prev = 0.0f; 
    int solver_status; 

    // --- VELOCITY DERIVATIVE VARIABLES ---
    float vh_prev = 0.0f;
    float ah_filtered = 0.0f; // Low-pass filter for smooth acceleration 

    while (ros::ok()) {
        ros::spinOnce();
        std_msgs::Float32 throttle_msg;
        std_msgs::Float32MultiArray state_msg; 

        // Grab raw data
        float raw_dr = current_states.dr; 
        float raw_vr = current_states.vr;
        vh = current_states.vh;

        float error = 0.0f;
        float control_action = 0.0f;
        float final_cont_act = 0.0f;
        float d_c = 0.0f, v_c = 0.0f;
        float del_d = 0.0f, del_v = 0.0f, del_a = 0.0f;
        int mpc_mode_int = 0;

        // --- DERIVATIVE OF VELOCITY LOGIC ---
        if (imu_first_run) { 
            vh_prev = vh; 
            imu_first_run = false;
         } 
        float raw_ah = (vh - vh_prev) / dt;

        if (raw_ah > 1.2f) raw_ah = 1.2f;
        if (raw_ah < -2.0f) raw_ah = -2.0f;

        ah_filtered = (0.1f * ah_filtered) + (0.9f * raw_ah); 
        current_states.ah = ah_filtered;
        vh_prev = vh;

        // his keeps the ghost target moving if the radar flickers.
        kf.predict(dt);

        // Only UPDATE the filter if the radar actually sees something
        if (current_states.lead_valid == 1) {
            kf.update(raw_dr, raw_vr);
            target_lost_counter = 0; // Reset counter because we see the target
        } else {
            target_lost_counter++;   // Target lost for this frame
        }

        // Extract the smoothed/predicted distance and velocity
        dr = kf.get_dr(); 
        vr = kf.get_vr();

        // --- MODE 1: CRUISE CONTROL ---
        // ONLY revert to cruise control if the road is clear OR target lost for 2 seconds (20 frames)
        if (dr > 30.0f || target_lost_counter > 10) {
            
            float speed_error = vmax - vh;
            float kp_cruise = 0.5f; 
            
            if (speed_error > 0.0f) {
                float proportional_throttle = speed_error * kp_cruise;
                final_cont_act = std::min(proportional_throttle, nominal_cmd);
            } 
            else {
                float proportional_brake = speed_error * kp_cruise; 
                final_cont_act = std::max(proportional_brake, -0.0f);
            }

            if (debug) {
                std::cout << "CRUISE MODE. Speed err: " << speed_error << " Act: " << final_cont_act << std::endl;
            }
        } 
        
        // --- MODE 2: ADAPTIVE CRUISE CONTROL ---
        else {
            error = dr - (vh * thw + d0);  
            float verror = vr;             
            /**
             * MPC start
             * 
             */

            d_c = dr - (vh * thw + d0);  // actual distance - Desired distance
            v_c = vr ; 
            float ah = current_states.ah;

            if (mpc_first_run) {
                    d_c_prev = d_c;
                    v_c_prev = v_c;
                    ah_prev = ah;
                    mpc_first_run = false;
                }

            // change variable
            del_d = d_c - d_c_prev;
            del_v = v_c - v_c_prev;
            del_a = ah  - ah_prev;

            d_c_prev = d_c;
            v_c_prev = v_c;
            ah_prev  = ah;

            std::vector<float> X0 = {current_states.u_prev, del_d, del_v, del_a,
                                d_c , v_c, current_states.ah};

            // Unpack the new 3-part tuple
            auto [mode_str, mpc_cmd, osqp_status] = mpc.mpc_osqp(X0);
            
            control_action = mpc_cmd;
            solver_status = osqp_status;
            mpc_mode_int = (mode_str == "accel") ? 1 : -1;

            if (debug) {
                std::cout << "d=" << d_c << " del_d=" << del_d 
                          << " v_c=" << v_c << " del_v=" << del_v 
                          << " ah=" << current_states.ah << " del_a=" << del_a 
                          << " Output u=" << control_action << " mode=" << mode_str 
                          << " OSQP=" << osqp_status << std::endl;
            }

            /**
             * MPC end
             * 
             */


            // Strict Speed Limiter
            if (vh > vmax) {
                control_action = std::max(std::min(control_action, 0.0f), -0.5f);   
            }

            current_states.update_u(control_action);

            // Deadband & clamp
            if (control_action > 0.02f) {
                final_cont_act = std::min(control_action, nominal_cmd);  
            } else if (control_action < -0.02f) {
                final_cont_act = std::max(control_action, -1.0f);        
            } else {
                final_cont_act = 0.0f;                                   
            }

            if (debug) {
                std::cout << "ACC MODE. dr: " << dr << " vh: " << vh << " err: " << error << " Act: " << final_cont_act << std::endl;
        }

        // --- PUBLISH AND LOG ---
        throttle_msg.data = final_cont_act;
        throttle_pub.publish(throttle_msg);

        state_msg.data = {current_states.dr, current_states.vr, current_states.vh, 
                          static_cast<float>(current_states.lead_valid), 
                          d_c, v_c, control_action, final_cont_act}; 
        state_pub.publish(state_msg);

        try {
            output_file << get_timestamp() << "," << raw_dr << "," << raw_vr << "," 
                        << dr << "," << vr << "," << vh << ","
                        << d_c << "," << v_c << "," << del_d << "," << del_v << "," 
                        << del_a << "," << mpc_mode_int << "," << solver_status << "," << control_action << "," << final_cont_act << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Error writing to CSV: " << e.what() << std::endl;
        }
        
        loop_rate.sleep();
    }
    
    output_file.close();

    return 0;

}
