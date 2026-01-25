
#pragma once
#include <iostream>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <unistd.h>

#include "can.hpp"

struct vehicle_fb {
    int16_t throttle;
    int16_t brake;
    int16_t speed;
    int16_t acceleration;
    int16_t motor_rpm;
    int16_t current;
    int16_t sys_flags; // Regen/Interlock status
    int16_t main_state; // Main Contactor state
};

/*  Vehicle CAN uses CAN Open protocol
So we need to first set - which all variables we want to log - into 4 TPDO -  page 78 vehicle use manual  0x1A00 and 0x1a01
SImilary we need to set receiver PDO format - ie throttle and brake value mapping to 0x1600 and 0x1601

The motor controller is slave in this case

Make sure the controller has node is 1  - that why SDO is 601 and feedback is 581
*/

class vehicle_can{

    private:
    bool debug;
    int can_line;
    can_handler* driver;

    CanCommand cmd_struct;

    // List of objects to read
    std::vector<uint32_t> feedback_var = {
        0x3216, // [2] Throttle Command
        0x321A, // [3] Brake Command
        0x320A, // [4] Vehicle Speed
        0x35C1, // [5] Acceleration
        0x3207, // [6] Motor RPM
        0x3209, // [7] Current RMS
        0x322B, // [8] System Flags (Regen/Interlock)
        0x3223  // [9] Main State
    };

    void send_sdo_write(uint16_t index , uint8_t sub_idx , const uint32_t value , int bytes = 4){
        std::vector<unsigned char> d;

        // command byte depends on size of data
        //  001 - download req, 0-padding , 00 -byted without data ,  1 -transfer type, 1 - is size indicated
        uint8_t cmd_byte = ( bytes == 4 ) ? 0x23 : (bytes == 2 ? 0x2B : 0x2F) ;

        d.push_back(cmd_byte);
        d.push_back(index & 0xFF);
        d.push_back(index >> 8 & 0xFF);
        d.push_back(sub_idx);

        uint32_t pass_val = value;

        for(int i = 0 ; i < bytes ; i++){
            d.push_back(pass_val & 0xFF);
            pass_val = pass_val >> 8;
        }

        for (int i = 0; i < (4-bytes) ; i++){
            d.push_back(0);
        }

        // LOGGING: Check what SDO we are writing
        if(debug) {
            LOG(INFO) << "SDO Write -> Index: 0x" << std::hex << index 
                        << " Sub: " << (int)sub_idx 
                        << " Value: 0x" << value;
        }
        
        send_sdo(0x601 , d , 1);
    }

    void send_sdo_raw(uint16_t index, uint8_t sub, std::vector<unsigned char> data) {
        std::vector<unsigned char> d;
        d.push_back(0x23); // Write 4 bytes
        d.push_back(index & 0xFF);
        d.push_back((index >> 8) & 0xFF);
        d.push_back(sub);
        // Add data
        d.insert(d.end(), data.begin(), data.end());
        // Pad to 8 bytes total frame
        while(d.size() < 8) d.push_back(0);

        send_sdo(0x601, d, 1);
        usleep(5000);
        sdo_feedback();
    }

    void send_sdo(uint32_t id , std::vector<unsigned char> data , int type){
        cmd_struct.can_line = can_line;
        cmd_struct.can_id = id;
        cmd_struct.data = data;
        cmd_struct.transmit_type = type;

        driver->send_command(cmd_struct);
        usleep(5000);
        sdo_feedback();
    }

    void sdo_feedback(){
        std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

        // Debug: Log if no messages received
        if (msgs.empty() && debug) {
            // Uncomment if you want to see this spam
            LOG(WARNING) << "Didn't receive SDO feedback " << can_line;
            return;
        }

        for(const auto& msg : msgs) {
            if (msg.id == 0x581) {
                if (msg.data[0] != 0x60) {
                    uint32_t  error = (msg.data[4] <<24) | (msg.data[5] <<16) | (msg.data[6] <<8) | msg.data[7] ;
                    LOG(ERROR) << "SDO rejected" << msg.data[0] << " convert to hex to read err code" << error;
                } 
                if(debug) LOG(INFO) << "Received SDO Response (0x581)" << msg.data[0];
                // 06 01 00 00 (0x06010000): Object not found (Wrong Index/Sub-index).
                // 06 02 00 00 (0x06020000): Object does not exist in object dictionary.
                // 06 01 00 02 (0x06010002): Attempt to write a Read-Only object.
                // 06 04 00 41 (0x06040041): Object cannot be mapped to PDO.
                // 08 00 00 20 (0x08000020): Data cannot be transferred or stored.
                // 08 00 00 22 (0x08000022): Data cannot be transferred or stored due to
            }
        }
    }

    public:
    vehicle_fb fb ;

    vehicle_can(can_handler* can_ptr , const std::string & config_path) : driver(can_ptr){

        try{
            YAML::Node config = YAML::LoadFile(config_path);

            std::string line = config["vehicle"]["can_line"].as<std::string>();
            can_line =  (line == "can1") ? 0 : 1;
            debug = config["debug"].as<bool>();

        } catch ( const std::exception & e) {
            LOG(ERROR) << "error loading from YAML file for vehicle can" << e.what() ;
        }

        // while intilizing - we need to stop RPDO and TPDO
        // then configure messages
        // then restart
        configure_canopen();

        if(debug) LOG(INFO) << "vehicle can is configured" ;
    }

    void configure_canopen(){


        // NMT frame to entre pre-operation (80)
        send_sdo(0x0000 , {0x80 , 0x00} , 0);

        // ENABLE WRITES TO EEPROM (Turn "Save" mode ON)
        // Index: 0x332F, Sub: 0x00, Value: 1 (Non-zero)
        send_sdo_write(0x332F, 0x00, 1);

        // Disable RPDO1
        // send_sdo_write(0x1400 , 0x01, 0x201 | 0x80000000);
        send_sdo_write(0x1601 , 0x00, 0);

        // Map throttle to sub index 1 on 0x1600
        //[16-bit Length] [Sub-index] [Index Low] [Index High]
        send_sdo_write(0x1601 , 0x01 , 0x32180010);

        // for brake
        send_sdo_write(0x1601 , 0x02 , 0x32190010);

        // enable RPDO1
        send_sdo_write(0x1601 , 0x00 , 2);
        // send_sdo_write(0x1400 , 0x01, 0x201);

        // TPDO -- First Disable the COBid - then write in it - then enable
        // send_sdo_write(0x1800, 0x01, 0x181 | 0x80000000);
        configure_tpdo(0, 0x1A00,  0x1800); // Vars 0-3  1800 is communication parameter
        send_sdo_write(0x1800, 0x01, 0x181);

        send_sdo_write(0x1801, 0x01, 0x281 | 0x80000000);
        configure_tpdo(4, 0x1A01, 0x1801); // Vars 4-7
        send_sdo_write(0x1801, 0x01, 0x281);

        // DISABLE WRITES TO EEPROM (Turn "Save" mode OFF)
        // Index: 0x332F, Sub: 0x00, Value: 0
        send_sdo_write(0x332F, 0x00, 0);

        // 3. Enter Operational Mode (Start Processing)
        // ID 000 (NMT), Data: 01 (Start), 00 (All Nodes)
        send_sdo(0x000, {0x01, 0x00}, 0);
        
        if (debug) LOG(INFO) << "Sending NMT Start Node...";
    }

    void configure_tpdo(int start_idx, uint32_t map_obj,  int comm_obj) {
        // Disable TPDO
        send_sdo_write(map_obj, 0x00, 0);

        // Map Variables
        int count = 0;
        for (int i = start_idx; i < start_idx + 4; i++) {
            if (i >= feedback_var.size()) break;

            // Mapping format: 0xIIIISSLL (Index, Sub, Length) -> Written as LL SS II II
            uint32_t mapping = (feedback_var[i] << 16) | 0x0010; // Index | Sub 00 | Len 10 (16-bit)

            uint8_t sub_idx = (i % 4) + 1;

            send_sdo_write(map_obj ,sub_idx, mapping );
            count++;
        }

        // Set Event Timer (Async 20Hz = 50ms)
        // Subindex 2 (Type) = 255 (Async)
        send_sdo_write(comm_obj, 0x02, 0xFF, 1); // 1 byte
        // Subindex 5 (Timer) = 50ms
        // send_sdo_write(comm_obj, 0x05, 50, 2); // 2 bytes  // err code 518

        // Enable TPDO (Write count)
        send_sdo_write(map_obj, 0x00, count);

        if (debug) LOG(INFO) << "Configured TPDO 0x" << std::hex << map_obj << " with " << count << " entries.";
    }

    void send_drive_command(long throttle, long brake) {
        cmd_struct.can_line = can_line;
        cmd_struct.can_id = 0x201; // RPDO1
        cmd_struct.transmit_type = 0; // Normal send
        cmd_struct.data.clear(); // IMPORTANT: Clear previous data!

        if (throttle > 0 && brake > 0) {
            throttle = 0; // Prioritize brake
            LOG(ERROR) << "Both throttle and brake is set at a time";
        }
         // --- BYTES 0-1: Brake ---
        cmd_struct.data.push_back(brake & 0xFF);
        cmd_struct.data.push_back((brake >> 8) & 0xFF);

       // --- BYTES 1-0 : Throttle ---
        cmd_struct.data.push_back(throttle & 0xFF);
        cmd_struct.data.push_back((throttle >> 8) & 0xFF);

        driver->send_command(cmd_struct);
    }

    void read_feedback() {
        std::vector<CanMessage> msgs = driver->read_all_messages(can_line);

        // Debug: Log if no messages received
        if (msgs.empty() && debug) {
            // Uncomment if you want to see this spam
            // LOG(WARNING) << "No CAN messages received on line " << can_line;
            return;
        }

        for(const auto& msg : msgs) {
            // TPDO 1 (0x180 + NodeID 1 = 0x181) -> Vars 1-4
            if(msg.id == 0x181 && msg.data.size() >= 8) {
                fb.throttle = (int16_t)(msg.data[0] | (msg.data[1] << 8));
                fb.brake = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                fb.speed = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                fb.acceleration = (int16_t)(msg.data[6] | (msg.data[7] << 8));
            }
            // TPDO 2 (0x280 + NodeID 1 = 0x281) -> Vars 5-8
            else if(msg.id == 0x281 && msg.data.size() >= 8) {
                fb.motor_rpm = (int16_t)(msg.data[0] | (msg.data[1] << 8));
                fb.current = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                fb.sys_flags = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                fb.main_state = (int16_t)(msg.data[6] | (msg.data[7] << 8));
            } 

            else if (msg.id == 0x581) {
                if(debug) LOG(INFO) << "Received SDO Response (0x581)";
            }
        }
    }
};
