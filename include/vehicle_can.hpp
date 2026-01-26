#pragma once
#include <iostream>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <cstring> 

#include "can.hpp"

struct vehicle_fb {
    int16_t throttle = 0;
    int16_t brake = 0;
    int16_t speed = 0;
    int16_t acceleration = 0;
    int16_t motor_rpm = 0;
    int16_t current = 0;
    int16_t sys_flags = 0; 
    int16_t main_state = 0; 
};

class vehicle_can {

    private:
    bool debug;
    int can_line;
    can_handler* driver;
    CanCommand cmd_struct;

    // COMMAND INPUTS (Write these to drive)
    // NOTE: We write to VCL inputs, not the Command outputs!
    const uint16_t IDX_VCL_THROTTLE  = 0x3218; 
    const uint16_t IDX_VCL_BRAKE     = 0x3219; 

    // FEEDBACK OUTPUTS (Read these for status)
    const uint16_t IDX_THROTTLE_CMD  = 0x3216; // Resulting Throttle %
    const uint16_t IDX_BRAKE_CMD     = 0x321A; // Resulting Brake %
    // const uint16_t IDX_SPEED         = 0x320A; // Vehicle Speed
    const uint16_t IDX_RPM           = 0x3207; // Motor RPM
    const uint16_t IDX_CURRENT       = 0x3209; // RMS Current
    // const uint16_t IDX_MAIN_STATE    = 0x3223; // Main Contactor State

    // List of objects to Poll
    std::vector<uint16_t> feedback_indices = {
        IDX_RPM,
        // IDX_SPEED,
        IDX_CURRENT,
        // IDX_MAIN_STATE,
        IDX_THROTTLE_CMD, // Read back the result of our command
        IDX_BRAKE_CMD
    };

    void send_can_frame(uint32_t id, std::vector<unsigned char> data) {
        cmd_struct.can_line = can_line;
        cmd_struct.can_id = id;
        cmd_struct.data = data;
        cmd_struct.transmit_type = 0;
        driver->send_command(cmd_struct);
    }

    // Helper: Send SDO Write (4 Bytes)
    void send_sdo_write(uint16_t index, uint8_t sub_idx, int32_t value) {
        std::vector<unsigned char> d;
        d.push_back(0x2B); // Command: Write 4 bytes
        d.push_back(index & 0xFF);
        d.push_back((index >> 8) & 0xFF);
        d.push_back(sub_idx);
        d.push_back(value & 0xFF);
        d.push_back((value >> 8) & 0xFF);
        // d.push_back((value >> 16) & 0xFF);
        // d.push_back((value >> 24) & 0xFF);
        send_can_frame(0x601, d);
    }

    // Helper: Send SDO Read Request
    void send_sdo_read_req(uint16_t index, uint8_t sub_idx) {
        std::vector<unsigned char> d;
        d.push_back(0x40); // Command: Upload Request (Read)
        d.push_back(index & 0xFF);
        d.push_back((index >> 8) & 0xFF);
        d.push_back(sub_idx);
        d.push_back(0); d.push_back(0); d.push_back(0); d.push_back(0); 
        send_can_frame(0x601, d);
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
        if (debug) LOG(INFO) << "Configuring Curtis Controller via SDO...";

        send_can_frame(0x000, {0x01, 0x00});
    }

    void send_drive_command(long throttle, long brake) {
        if (brake > 0) send_sdo_write(IDX_VCL_BRAKE, 0x00, brake);
        
        // Write to VCL_Throttle (0x3218) and VCL_Brake (0x3219)
        else send_sdo_write(IDX_VCL_THROTTLE, 0x00, throttle);
    }

    void send_feedback_requests() {
        for (uint16_t target_idx : feedback_indices) {
            // Just send the request. Do NOT wait for the answer.
            send_sdo_read_req(target_idx, 0x00);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void read_feedback() {
        // ASynchronous Read: Ask full -> Wait -> Read full -> done
                send_feedback_requests(); 
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

                for (const auto& msg : msgs) {
                    // Look for SDO Response (0x581)
                    if (msg.id == 0x581 && msg.data.size() >= 8) {
                        uint16_t idx = msg.data[1] | (msg.data[2] << 8);
                        uint8_t cmd = msg.data[0];

                        if ((cmd & 0x40) == 0x40) { 
                            int32_t value = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24);
                            
                            switch (idx) {
                                case 0x3216: fb.throttle     = (int16_t)value; break;
                                case 0x321A: fb.brake        = (int16_t)value; break;
                                case 0x3207: fb.motor_rpm    = (int16_t)value; break;
                                case 0x3209: fb.current      = (int16_t)value; break;
                            }
                        }
                    }
            }
    }
};

