// #pragma once
// #include <iostream>
// #include <glog/logging.h>
// #include <yaml-cpp/yaml.h>
// #include <vector>
// #include <unistd.h>
// #include <thread>
// #include <chrono>
// #include <cstring> 

// #include "can.hpp"

// struct vehicle_fb {
//     int16_t throttle = 0;
//     int16_t brake = 0;
//     int16_t speed = 0;
//     int16_t acceleration = 0;
//     int16_t motor_rpm = 0;
//     int16_t current = 0;
//     int16_t sys_flags = 0; 
//     int16_t main_state = 0; 
// };

// class vehicle_can {

//     private:
//     bool debug;
//     int can_line;
//     can_handler* driver;
//     CanCommand cmd_struct;

//     // COMMAND INPUTS (Write these to drive)
//     // NOTE: We write to VCL inputs, not the Command outputs!
//     const uint16_t IDX_VCL_THROTTLE  = 0x3218; 
//     const uint16_t IDX_VCL_BRAKE     = 0x3219; 

//     // FEEDBACK OUTPUTS (Read these for status)
//     const uint16_t IDX_THROTTLE_CMD  = 0x3216; // Resulting Throttle %
//     const uint16_t IDX_BRAKE_CMD     = 0x321A; // Resulting Brake %
//     // const uint16_t IDX_SPEED         = 0x320A; // Vehicle Speed
//     const uint16_t IDX_RPM           = 0x3207; // Motor RPM
//     const uint16_t IDX_CURRENT       = 0x3209; // RMS Current
//     // const uint16_t IDX_MAIN_STATE    = 0x3223; // Main Contactor State

//     // List of objects to Poll
//     std::vector<uint16_t> feedback_indices = {
//         IDX_RPM,
//         // IDX_SPEED,
//         IDX_CURRENT,
//         // IDX_MAIN_STATE,
//         IDX_THROTTLE_CMD, // Read back the result of our command
//         IDX_BRAKE_CMD
//     };

//     void send_can_frame(uint32_t id, std::vector<unsigned char> data) {
//         cmd_struct.can_line = can_line;
//         cmd_struct.can_id = id;
//         cmd_struct.data = data;
//         cmd_struct.transmit_type = 1;   // 0 - to keep retying -- 1 for send and forget
//         driver->send_command(cmd_struct);
//     }

//     // Helper: Send SDO Write (4 Bytes)
//     void send_sdo_write(uint16_t index, uint8_t sub_idx, int32_t value) {
//         std::vector<unsigned char> d;
//         d.push_back(0x2B); // Command: Write 4 bytes
//         d.push_back(index & 0xFF);
//         d.push_back((index >> 8) & 0xFF);
//         d.push_back(sub_idx);
//         d.push_back(value & 0xFF);
//         d.push_back((value >> 8) & 0xFF);
//         // d.push_back((value >> 16) & 0xFF);
//         // d.push_back((value >> 24) & 0xFF);
//         send_can_frame(0x601, d);
//     }

//     // Helper: Send SDO Read Request
//     void send_sdo_read_req(uint16_t index, uint8_t sub_idx) {
//         std::vector<unsigned char> d;
//         d.push_back(0x40); // Command: Upload Request (Read)
//         d.push_back(index & 0xFF);
//         d.push_back((index >> 8) & 0xFF);
//         d.push_back(sub_idx);
//         d.push_back(0); d.push_back(0); d.push_back(0); d.push_back(0); 
//         send_can_frame(0x601, d);
//     }

//     public:
//     vehicle_fb fb;

//     vehicle_can(can_handler* can_ptr, const std::string & config_path) : driver(can_ptr) {
//         try {
//             YAML::Node config = YAML::LoadFile(config_path);
//             std::string line = config["vehicle"]["can_line"].as<std::string>();
//             can_line = (line == "can1") ? 0 : 1;
//             debug = config["vehicle"]["debug"].as<bool>();
//         } catch (const std::exception & e) {
//             LOG(ERROR) << "Error loading vehicle config: " << e.what();
//         }
        
//         fb = {0}; 
//         configure_canopen();
//     }

//     void configure_canopen() {
//         if (debug) LOG(INFO) << "Configuring Curtis Controller via SDO...";

//         send_can_frame(0x000, {0x01, 0x00});
//     }

//     void send_drive_command(long throttle, long brake) {
//         if (brake > 0) send_sdo_write(IDX_VCL_BRAKE, 0x00, brake);
        
//         // Write to VCL_Throttle (0x3218) and VCL_Brake (0x3219)
//         else send_sdo_write(IDX_VCL_THROTTLE, 0x00, throttle);
//     }

//     void send_feedback_requests() {
//         for (uint16_t target_idx : feedback_indices) {
//             // Just send the request. Do NOT wait for the answer.
//             send_sdo_read_req(target_idx, 0x00);
//             std::this_thread::sleep_for(std::chrono::microseconds(100));
//         }
//     }

//     void read_feedback() {
//         // Synchronous Read: Ask  -> Wait -> Read  ->  repeat 
//         // because the controller can process only 1SDO at a time
//         // and 

//         for (uint16_t target_idx : feedback_indices) {
//             // Just send the request. Do NOT wait for the answer.
//             send_sdo_read_req(target_idx, 0x00);
//             // std::this_thread::sleep_for(std::chrono::microseconds(100));
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));

//             std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

//             if (msgs.empty()) {
//                     if (debug) LOG(INFO) << "feedback messages empty for idx " << target_idx;
//                     return;
//                 }
            
//                 for (const auto& msg : msgs) {
//                     // Look for SDO Response (0x581)
//                     // if(debug) LOG(INFO) << "pile has id " << msg.id << " these are maybe extended";
//                     if (msg.id == 0x581 ) {  //&& msg.data.size() >= 8
//                         uint16_t idx = msg.data[1] | (msg.data[2] << 8);
//                         uint8_t cmd = msg.data[0];
                    
//                     if (debug) {
//                         LOG(INFO) << "Steering feedback received for ID: " << msg.id << " Len" << msg.data.size() ;
//                         LOG(INFO) << " Data bytes: " << std::hex 
//                                 << static_cast<int>(msg.data[0]) << " "
//                                 << static_cast<int>(msg.data[1]) << " "
//                                 << static_cast<int>(msg.data[2]) << " "
//                                 << static_cast<int>(msg.data[3]) << " "
//                                 << static_cast<int>(msg.data[4]) << " "
//                                 << static_cast<int>(msg.data[5]) << " "
//                                 << static_cast<int>(msg.data[6]) << " "
//                                 << static_cast<int>(msg.data[7]) << std::dec;
//                         }   

//                         if ((cmd & 0x40) == 0x40) { 
//                             int32_t value = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24);
                            
//                             switch (idx) {
//                                 case 0x3216: fb.throttle     = (int16_t)value; break;
//                                 case 0x321A: fb.brake        = (int16_t)value; break;
//                                 case 0x3207: fb.motor_rpm    = (int16_t)value; break;
//                                 case 0x3209: fb.current      = (int16_t)value; break;
//                             }
//                         }
//                     }
//             }
    

//                 // send_feedback_requests(); 
//                 // std::this_thread::sleep_for(std::chrono::milliseconds(10));

//                 // std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

//                 // if (msgs.empty()) {
//                 //     if (debug) LOG(INFO) << "No steering feedback messages received.";
//                 //     return;
//                 // }

//         }
//     }
// };

#pragma once
#include <iostream>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <unistd.h>
#include <cstring> 

#include "can.hpp"

struct vehicle_fb {
    int16_t throttle = 0;
    int16_t brake = 0;
    int16_t motor_rpm = 0;
    int16_t current = 0;
};

class vehicle_can {

    private:
    bool debug;
    int can_line;
    can_handler* driver;
    CanCommand cmd_struct;

    // IDs based on Node ID 1 (Standard CANopen)
    const uint32_t ID_TPDO1 = 0x181; // Feedback from Drive
    const uint32_t ID_RPDO1 = 0x201; // Command to Drive
    const uint32_t ID_SDO_TX = 0x601; // SDO Request
    const uint32_t ID_SDO_RX = 0x581; // SDO Response

    // INDICES for SDO (Brake only)
    const uint16_t IDX_VCL_BRAKE = 0x3219; 

    void send_can_frame(uint32_t id, std::vector<unsigned char> data) {
        cmd_struct.can_line = can_line;
        cmd_struct.can_id = id;
        cmd_struct.data = data;
        cmd_struct.transmit_type = 1; // Send and Forget
        driver->send_command(cmd_struct);
    }

    void send_sdo_write(uint16_t index, uint8_t sub_idx, int32_t value) {
        std::vector<unsigned char> d;
        d.push_back(0x2B); // Write 4 bytes
        d.push_back(index & 0xFF);
        d.push_back((index >> 8) & 0xFF);
        d.push_back(sub_idx);
        d.push_back(value & 0xFF);
        d.push_back((value >> 8) & 0xFF);
        d.push_back(0); d.push_back(0);
        send_can_frame(ID_SDO_TX, d);
    }

    public:
    vehicle_fb fb;

    vehicle_can(can_handler* can_ptr, const std::string & config_path) : driver(can_ptr) {
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            std::string line = config["vehicle"]["can_line"].as<std::string>();
            can_line = (line == "can1") ? 0 : 1;
            debug = config["vehicle"]["debug"].as<bool>();
        } catch (const std::exception & e) {
            LOG(ERROR) << "Error loading vehicle config: " << e.what();
        }
        fb = {0}; 
        configure_canopen();
    }

    void configure_canopen() {
        // Just start the node (NMT Start) so it starts streaming TPDOs
        send_can_frame(0x000, {0x01, 0x00});
        if (debug) LOG(INFO) << "Sent NMT Start to Curtis...";
    }

    void send_drive_command(long throttle, long brake) {
        // 1. BRAKE: Use SDO (Legacy method)
        if (brake > 0) {
            send_sdo_write(IDX_VCL_BRAKE, 0x00, brake);
        }
        // 2. THROTTLE: Use RPDO1 (ID 0x201)
        else {
            // Mapping from your instructions:
            // Byte 0-1: 0x33D1 (Unknown/User var) -> Send 0
            // Byte 2-3: 0x3218 (Throttle) -> Send Value
            // Byte 4-7: Padding -> Send 0
            
            std::vector<unsigned char> d(8, 0);
            d[0] = 0x00;
            d[1] = 0x00;
            d[2] = throttle & 0xFF;        // Low Byte
            d[3] = (throttle >> 8) & 0xFF; // High Byte
            
            send_can_frame(ID_RPDO1, d);
        }
    }

    void read_feedback() {
        // Just Listen. No Polling.
        std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

        for (const auto& msg : msgs) {
            // --- TPDO1 (ID 0x181) ---
            if (msg.id == ID_TPDO1 && msg.data.size() >= 8) {
                // Byte 4-5: Motor RPM (0x3207)
                // Byte 6-7: Current RMS (0x3209)
                
                int16_t raw_rpm = msg.data[4] | (msg.data[5] << 8);
                int16_t raw_cur = msg.data[6] | (msg.data[7] << 8);

                fb.motor_rpm = raw_rpm;
                fb.current   = raw_cur;

                if (debug) {
                    LOG_EVERY_N(INFO, 10) << "TPDO1 RX: RPM=" << fb.motor_rpm << " Cur=" << fb.current;
                }
            }
            
            // --- SDO Response (ID 0x581) ---
            // We still listen to this just to confirm Brake Writes (0x60)
            else if (msg.id == ID_SDO_RX) {
                if (msg.data[0] == 0x60) {
                     // Write Success Confirmation
                }
            }
        }
    }
};