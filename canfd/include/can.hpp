#pragma once
#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include "controlcanfd.h"
#include <chrono>
#include <mutex>

#define DEVICE_TYPE 41

// Safe Message Struct
struct CanMessage {
    uint32_t id;
    std::vector<unsigned char> data;
    uint64_t timestamp;   // from can device - in microseconds
};

// Command Struct
struct CanCommand {
    int can_line;
    uint32_t can_id;
    std::vector<unsigned char> data;
    int transmit_type;
};

class can_handler {

private:
    DEVICE_HANDLE device_handle;
    CHANNEL_HANDLE ch_handles[2];
    bool debug;

    std::vector<CanMessage> q0; 
    std::vector<CanMessage> q1;

    uint64_t time_now;
    // std::mutex can_mutex;

    std::vector<CanMessage> process (std::vector<CanMessage> &q , std::set<uint32_t> filter_id){
        std::vector<CanMessage> output; 
        std::set<uint32_t> unique_id; 

        for (auto it = q.begin(); it !=q.end(); ){
            unique_id.insert(it->id);

            if ( filter_id.find(it->id) != filter_id.end()){
                output.push_back(*it);
                it = q.erase(it); 
            } 
            else{
                ++it;
            }
        }
        if (debug) {
            LOG(INFO) << "Processed q size " << output.size() << " " << filter_id.size() << " " << *filter_id.begin() ;
            for (const auto &id : unique_id) LOG(INFO) << "id in que " << id;
        }
        return output;
    }

    void clean_q (std::vector<CanMessage> *q){

        if (q->empty()) return;
        // if (q->begin()->timestamp < 500000) return; 

        for (auto it = q->begin(); it != q->end() ; ){
            if ((time_now - it->timestamp) > 50000){
                it = q->erase(it);
            }
            else ++it; 
        }
    }


public:
    can_handler(const std::string& config_path) {
        int baudrates[2] = {0, 0};
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            baudrates[0] = config["can1"]["baud_rate"].as<int>();
            baudrates[1] = config["can2"]["baud_rate"].as<int>();
            debug = config["debug"].as<bool>();
        } catch (std::exception& e){
            LOG(ERROR) << "YAML Config Error: " << e.what();
            return;
        }

        if (debug) {
            LOG(INFO) << "baud are " << baudrates[0] << baudrates[1]; 
            
        }

        device_handle = ZCAN_OpenDevice(DEVICE_TYPE, 0, 0);
        if (!device_handle) {
            LOG(FATAL) << "Failed to open USBCAN Device!";
            // LOG(FATAL) << "ZCAN_GetLastError" << ZCAN_GetLastError() ; 
            return;
        }

        for(int i=0; i<2; i++) {
            ZCAN_SetResistanceEnable(device_handle, i, 1);
            ZCAN_SetAbitBaud(device_handle, i, baudrates[i]);
            ZCAN_SetDbitBaud(device_handle, i, 5000000);
            ZCAN_SetCANFDStandard(device_handle, i, 0);

            ZCAN_CHANNEL_INIT_CONFIG cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.can_type = TYPE_CANFD;
            cfg.canfd.mode = 0;
            cfg.canfd.acc_mask = 0xFFFFFFFF;

            ch_handles[i] = ZCAN_InitCAN(device_handle, i, &cfg);

            ZCAN_ClearFilter(ch_handles[i]);
            ZCAN_AckFilter(ch_handles[i]);

            if (ch_handles[i] == 0 || ZCAN_StartCAN(ch_handles[i]) != 1) {
                LOG(ERROR) << "Failed to start CAN Channel " << i;
            } else {
                LOG(INFO) << "Started CAN Channel " << i << " at " << baudrates[i] << "bps";
            }
        }
    }

    ~can_handler() {
        ZCAN_CloseDevice(device_handle);
    }

    std::vector<CanMessage> read_feedback(int channel_idx , std::set<uint32_t> filter_ids){
        // std::lock_guard<std::mutex> lock(can_mutex);
        std::vector<CanMessage> &q = (channel_idx == 0) ? q0 : q1 ;

        std::vector<CanMessage> temp = read_all_messages(channel_idx); 
        if (debug) 
            LOG(INFO) << "size of que is: " << q.size() << "before cleaning";
        
        clean_q(&q); 
        if (debug) 
            LOG(INFO) << "size of que is: " << q.size() << "after cleaning";

        q.insert(q.end(), temp.begin(), temp.end()); 
        return process(q , filter_ids); 

    }

    // Returns ALL messages in buffer
    std::vector<CanMessage> read_all_messages(int channel_idx) {
        std::vector<CanMessage> output;

        uint len = ZCAN_GetReceiveNum(ch_handles[channel_idx], 1);
        if (debug) {
            LOG(INFO) << "Buffer on Ch" << channel_idx << " has " << len << " messages.";
        }

        uint size = (len > 0) ? len : 1;
        ZCAN_Receive_Data rx_msgs[size];

        int num_rx = ZCAN_Receive(ch_handles[channel_idx], rx_msgs, size, 2); // 2 ms wait
        

        for (int i = 0; i < num_rx; i++) {
            CanMessage msg;
            msg.id = rx_msgs[i].frame.can_id;

            if (debug) LOG(INFO) << "feedback call received " << msg.id ;
            msg.timestamp = rx_msgs[i].timestamp;

            for(int k=0; k<rx_msgs[i].frame.can_dlc; k++) {
                msg.data.push_back(rx_msgs[i].frame.data[k]);
            }
            output.push_back(msg);
            time_now = msg.timestamp; 
        }
        return output;
    }

    // Sends command
    void send_ext_command(CanCommand &cmd){
        // std::lock_guard<std::mutex> lock(can_mutex);
        ZCAN_Transmit_Data tx_msg;
        memset(&tx_msg, 0, sizeof(tx_msg));

        // tx_msg.frame.can_id = cmd.can_id;
        tx_msg.frame.can_id = cmd.can_id | 0x80000000;  // extended frame  - 30th bit should be high
        tx_msg.frame.can_dlc = cmd.data.size();
        tx_msg.transmit_type = cmd.transmit_type;

        if (debug) LOG(INFO) << "extebded can_id is " << cmd.can_id << "data size" << cmd.data.size();

        for(size_t i=0; i < cmd.data.size(); i++) {
            tx_msg.frame.data[i] = cmd.data[i];
        }

        // ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);

        // 5. CRITICAL: Check if it actually sent!
        uint sent_count = ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);
        
        if (sent_count == 0) {
            LOG(ERROR) << "Failed to send CAN EXT command to ID: " << cmd.can_id;
        } else if (debug) {
            LOG(INFO) << "Sent CAN EXT command. Count: " << sent_count;
        }
    }

    void send_command(CanCommand &cmd){
        // std::lock_guard<std::mutex> lock(can_mutex);
        if (ZCAN_IsDeviceOnLine(device_handle) != STATUS_ONLINE) {
            LOG(ERROR) << "Device Offline! Cannot send command to Ch" << cmd.can_line;
            return;
        }

        ZCAN_Transmit_Data tx_msg;
        memset(&tx_msg, 0, sizeof(tx_msg));

        // tx_msg.frame.can_id = cmd.can_id;
        tx_msg.frame.can_id = cmd.can_id ;// this is 11 bit identifier
        tx_msg.frame.can_dlc = cmd.data.size();
        tx_msg.transmit_type = cmd.transmit_type;

        if (debug) LOG(INFO) << "can_id is " << cmd.can_id << "data size" << cmd.data.size();

        for(size_t i=0; i < cmd.data.size(); i++) {
            tx_msg.frame.data[i] = cmd.data[i];
        }

        ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);
    }
};
