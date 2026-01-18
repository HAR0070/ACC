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
    };

public:
    motor_fb fb {0};

    steering_can(can_handler* can_ptr) : driver(can_ptr){
        LOG(INFO) << "Initializing steering can" ;

        try {
            YAML::Node config = YAML::LoadFile("can_config.yaml"); 
            std::string line = config["steering"]["can_line"].as<std::string>();
            can_line = (line == "can1") ? 0 : 1; 
            
            MOTOR_ID  = config["steering"]["motor_id"].as<int>();
            mode = config["steering"]["mode"].as<int>(); 
            debug = config["steering"]["debug"].as<bool>(); 
        }
        catch (const std::exception& e){
            LOG(ERROR) << "Steering YAML Error: " << e.what();
        } 

    }

    void steering_feedback(){
        // Get all messages from the assigned line
        process_feedback(driver->read_all_messages(can_line)); 
        if(debug) LOG(INFO) << "req fb from " << can_line;
    }

    void process_feedback(const std::vector<CanMessage>& all_msgs){
        // uint32_t expected_id = MOTOR_ID | (mode << 8);
        const uint32_t EFF_FLAG = 0x80000000; 
        uint32_t expected_id = 2968 | EFF_FLAG;

        for(const auto& msg : all_msgs) {

            if(debug) LOG(INFO) << "pile has id " << msg.id << "these are maybe extended";

            // if (msg.id != expected_id) continue;
            if (msg.data.size() < 7) continue; 

            const unsigned char* b = msg.data.data(); 

            fb.pos  = (int16_t)((b[0] << 8) | b[1]) * 0.1f;
            fb.spd  = (int16_t)((b[2] << 8) | b[3]) * 10.0f; 
            fb.cur  = (int16_t)((b[4] << 8) | b[5]) * 0.01f;
            fb.temp = (int8_t)b[6]; 

            if (debug) {
                LOG(INFO) << "[Steering] Pos:" << fb.pos 
                        << " Spd:" << fb.spd 
                        << " Cur:" << fb.cur 
                        << " Temp:" << fb.temp;
            }
        }
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
        
        driver->send_command(frame); 

        if (debug) LOG(INFO) << "Sent steering RPM:" << rpm;
    }
};