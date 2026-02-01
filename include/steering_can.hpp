#pragma once
#include <iostream>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include "can.hpp"


class steering_can {

private:

    int can_line;
    bool debug;
    int MOTOR_ID;
    int mode;
    can_handler* driver;

    struct motor_fb {
        float pos;
        float spd;
        float cur;
        int temp;
        int err;
    };
    const uint32_t EFF_FLAG = 0x80000000;
    std::set<uint32_t> expected_id ;

public:
    motor_fb fb {0};

    steering_can(can_handler* can_ptr , const std::string & config_path) : driver(can_ptr){
        LOG(INFO) << "Initializing steering can" ;

        try {
            YAML::Node config = YAML::LoadFile(config_path); 
            std::string line = config["steering"]["can_line"].as<std::string>();
            can_line = (line == "can1") ? 0 : 1;

            MOTOR_ID  = config["steering"]["motor_id"].as<int>();
            mode = config["steering"]["mode"].as<int>();
            debug = config["steering"]["debug"].as<bool>();
            expected_id = {(MOTOR_ID | (0x29 << 8) | EFF_FLAG)};
        }
        catch (const std::exception& e){
            LOG(ERROR) << "Steering YAML Error: " << e.what();
        }

    }

    void steering_feedback(){
        // Get all messages from the assigned line
        process_feedback(driver->read_feedback(can_line , expected_id));
        if(debug) LOG(INFO) << "req fb from " << can_line;
    }

    void process_feedback(const std::vector<CanMessage>& all_msgs){
        // uint32_t expected_id = MOTOR_ID | (mode << 8);
        // uint32_t id_vel = 2968 | EFF_FLAG;
        // // uint32_t expected_id_torque = 2968 | EFF_FLAG;
        // uint32_t extended_id = MOTOR_ID | (mode << 8) | EFF_FLAG ;
        // uint32_t non_extended_id = MOTOR_ID | (mode << 8) | EFF_FLAG ;
          // extended_id , non_extended_id , id_vel ,     2147494248

        if (all_msgs.empty()) {
            if (debug) LOG(INFO) << "No steering feedback messages received.";
            return;
        }

        for(const auto& msg : all_msgs) {

            LOG(INFO) << "Steering feedback received for ID: " << msg.id;
            // if(debug) LOG(INFO) << "pile has id " << msg.id << "these are maybe extended";

            if (expected_id.find(msg.id) == expected_id.end()) continue;
            if (msg.data.size() < 7) continue;

            const unsigned char* b = msg.data.data();

            fb.pos  = (int16_t)((b[0] << 8) | b[1]) * 0.1f;
            fb.spd  = (int16_t)((b[2] << 8) | b[3]) * 10.0f;
            fb.cur  = (int16_t)((b[4] << 8) | b[5]) * 0.01f;
            fb.temp = (int8_t)b[6];
            fb.err = (int8_t)b[7];

            if (debug) {
                LOG(INFO) << "[Steering] Pos:" << fb.pos
                        << " Spd:" << fb.spd
                        << " Cur:" << fb.cur
                        << " Temp:" << fb.temp
                        << "Err:"   << fb.err;
            }
        }
    }

    void steering_stop(){

        // here we command 0 torque - use while takeover is initiated
        CanCommand frame;
        frame.can_line = can_line;
        frame.transmit_type = 0;
        frame.can_id = MOTOR_ID | (0x01 << 8);

        frame.data.push_back((0 >> 24) & 0xFF);
        frame.data.push_back((0 >> 16) & 0xFF);
        frame.data.push_back((0 >> 8)  & 0xFF);
        frame.data.push_back((0)       & 0xFF);

        driver->send_ext_command(frame);
    }

    void steering_command(long rpm) {
        CanCommand frame;
        frame.can_line = can_line;
        frame.transmit_type = 0;
        frame.can_id = MOTOR_ID | (0x03 << 8);

        frame.data.push_back((rpm >> 24) & 0xFF);
        frame.data.push_back((rpm >> 16) & 0xFF);
        frame.data.push_back((rpm >> 8)  & 0xFF);
        frame.data.push_back((rpm)       & 0xFF);

        driver->send_ext_command(frame);

        if (debug) LOG(INFO) << "Sent steering RPM:" << rpm;
    }
};
