#pragma once
#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include "controlcanfd.h"

#define DEVICE_TYPE 41

// Safe Message Struct
struct CanMessage {
    uint32_t id;
    std::vector<unsigned char> data;
    uint64_t timestamp;
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
            LOG(INFO) << "baud are " << baudrates[0] << " " << baudrates[1]; 
        }

        device_handle = ZCAN_OpenDevice(DEVICE_TYPE, 0, 0);
        if (!device_handle) {
            LOG(FATAL) << "Failed to open USBCAN Device!";
            return;
        }

        for(int i=0; i<2; i++) {
            ZCAN_SetResistanceEnable(device_handle, i, 1);
            ZCAN_SetAbitBaud(device_handle, i, baudrates[i]);

            ZCAN_CHANNEL_INIT_CONFIG cfg;
            memset(&cfg, 0, sizeof(cfg));

            if (i == 0){
                // Channel 0: Standard CAN
                cfg.can_type = TYPE_CAN;
                cfg.can.mode = 0; 
                cfg.can.acc_mask = 0xFFFFFFFF;
            }
            else {
                // Channel 1: CAN FD
                ZCAN_SetDbitBaud(device_handle, 1, 5000000);
                ZCAN_SetCANFDStandard(device_handle, 1, 0);
                cfg.can_type = TYPE_CANFD;
                cfg.canfd.mode = 0;
                cfg.canfd.acc_mask = 0xFFFFFFFF;
            }

            ch_handles[i] = ZCAN_InitCAN(device_handle, i, &cfg);

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

    // Returns ALL messages in buffer
    std::vector<CanMessage> read_all_messages(int channel_idx) {
        std::vector<CanMessage> output;

        if (channel_idx == 0) {
            // --- STANDARD CAN ---
            ZCAN_Receive_Data rx_msgs[2000];
            int num_rx = ZCAN_Receive(ch_handles[channel_idx], rx_msgs, 2000, 0); // Corrected len from 20 to 2000

            for (int i = 0; i < num_rx; i++) {
                CanMessage msg;
                msg.id = rx_msgs[i].frame.can_id;
                msg.timestamp = rx_msgs[i].timestamp;

                for(int k=0; k < rx_msgs[i].frame.can_dlc; k++) {
                    msg.data.push_back(rx_msgs[i].frame.data[k]);
                }
                
                if (debug) LOG(INFO) << "Std feedback: " << msg.id;
                output.push_back(msg);
            }
        }
        else {
            // --- CAN FD ---
            ZCAN_ReceiveFD_Data rx_msgs[2000];
            int num_rx = ZCAN_ReceiveFD(ch_handles[channel_idx], rx_msgs, 2000, 0); // Corrected len

            for (int i = 0; i < num_rx; i++) {
                CanMessage msg;
                msg.id = rx_msgs[i].frame.can_id;
                msg.timestamp = rx_msgs[i].timestamp;

                for(int k=0; k < rx_msgs[i].frame.len; k++) {
                    msg.data.push_back(rx_msgs[i].frame.data[k]);
                }

                if (debug) LOG(INFO) << "FD feedback: " << msg.id;
                output.push_back(msg);
            }
        }
        return output;
    }

    // Sends Vector-based command
    void send_ext_command(CanCommand &cmd){

        if (cmd.can_line == 0) {
            // --- STANDARD CAN ---
            ZCAN_Transmit_Data tx_msg;
            memset(&tx_msg, 0, sizeof(tx_msg));
            tx_msg.frame.can_id = cmd.can_id | 0x80000000; 
            
            tx_msg.frame.can_dlc = cmd.data.size();
            
            tx_msg.transmit_type = cmd.transmit_type;

            for(size_t i=0; i < cmd.data.size(); i++) {
                tx_msg.frame.data[i] = cmd.data[i];
            }
            ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);
        }
        else {
            // --- CAN FD ---
            ZCAN_TransmitFD_Data tx_msg;
            memset(&tx_msg, 0, sizeof(tx_msg));
            tx_msg.frame.can_id = cmd.can_id | 0x80000000; 
            
            tx_msg.frame.len = cmd.data.size();
            
            tx_msg.transmit_type = cmd.transmit_type;

            for(size_t i=0; i < cmd.data.size(); i++) {
                tx_msg.frame.data[i] = cmd.data[i];
            }
            ZCAN_TransmitFD(ch_handles[cmd.can_line], &tx_msg, 1);
        } 
    }

    void send_command(CanCommand &cmd){
        if (cmd.can_line == 0) {
            // --- STANDARD CAN ---
            ZCAN_Transmit_Data tx_msg;
            memset(&tx_msg, 0, sizeof(tx_msg));
            tx_msg.frame.can_id = cmd.can_id; 
            
            tx_msg.frame.can_dlc = cmd.data.size();
            
            tx_msg.transmit_type = cmd.transmit_type;

            for(size_t i=0; i < cmd.data.size(); i++) {
                tx_msg.frame.data[i] = cmd.data[i];
            }
            ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);
        }
        else {
            // --- CAN FD ---
            ZCAN_TransmitFD_Data tx_msg;
            memset(&tx_msg, 0, sizeof(tx_msg));
            tx_msg.frame.can_id = cmd.can_id; 
            
            tx_msg.frame.len = cmd.data.size();
            
            tx_msg.transmit_type = cmd.transmit_type;

            for(size_t i=0; i < cmd.data.size(); i++) {
                tx_msg.frame.data[i] = cmd.data[i];
            }
            ZCAN_TransmitFD(ch_handles[cmd.can_line], &tx_msg, 1);
        }
    }
};
