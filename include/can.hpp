#pragma once
#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include "controlcanfd.h"
#include <algorithm> 

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

        DEVICE_HANDLE device;
        CHANNEL_HANDLE ch_handles[2];
        bool debug = true;
        int baudrates[2];
        int d_baud[2];
        uint8_t canfd[2]; 
        uint64_t curr_time;

    void process_q( std::vector<CanMessage> &Q){

        // sort based on the timestamp 
        // std::sort(Q.begin() , Q.end() , [](const CanMessage &a , const CanMessage &b){
        //     return a.timestamp < b.timestamp;
        // });

        if (debug) {
            std::set<uint32_t> id_list;
            for (auto & msg : Q) {
                id_list.insert( msg.id );
            }
            for (const auto &id : id_list){
                LOG(INFO) << "ID in Q : " << std::hex << id;
            }
        }
        if (curr_time > 500000)
        Q.erase(std::remove_if(Q.begin(), Q.end(), [this](const CanMessage &a ) { return a.timestamp < (curr_time - 500000); }), Q.end()); // time in microseconds

    }

    std::vector<CanMessage> fetch_messages( uint32_t id , std::vector<CanMessage> &Q){

        std::vector<CanMessage> output;

        for ( auto it = Q.begin() ; it != Q.end() ; ){

            if ( it->id == id ){
                output.push_back(*it);
                it = Q.erase(it); // remove from Q after fetching
            }
            else{
                ++it;
            }
        }

        return output;

    }

    std::vector<CanMessage> read_all_messages(int channel_idx ) {
        
        std::vector<CanMessage> output;

        if (debug) {
            uint frame_number = ZCAN_GetReceiveNum(ch_handles[channel_idx] , canfd[channel_idx]);
            LOG(INFO) << "Total of frames in buffer: " << frame_number << " channel" << channel_idx ;
        }

        if (canfd[channel_idx] ==1 ) {
            ZCAN_ReceiveFD_Data rx_msgs[200];

            int num_rx = ZCAN_ReceiveFD(ch_handles[channel_idx], rx_msgs, 200, 2);  // 2 ms wait time - blocking

            LOG_EVERY_N(INFO, 1000) << "Number of received frames: " << num_rx << " on channel " << channel_idx ;

            for (int i = 0; i < num_rx; i++) {
                CanMessage msg;
                msg.id = rx_msgs[i].frame.can_id;

                if (debug) LOG(INFO) << "feedback call received " << msg.id ;
                msg.timestamp = rx_msgs[i].timestamp;

                for(int k=0; k<rx_msgs[i].frame.len; k++) {
                    msg.data.push_back(rx_msgs[i].frame.data[k]);
                }
                output.push_back(msg);
            }

            if (num_rx  >0 ) curr_time = rx_msgs[0].timestamp; // microseconds  

            // clean the buffer 
            // ZCAN_ClearBuffer(ch_handles[channel_idx]);
            // u shouldnt because - while processing - new messages might come
        }
        else{
            ZCAN_Receive_Data rx_msgs[20];

            int num_rx = ZCAN_Receive(ch_handles[channel_idx], rx_msgs, 20, 2);  // 2 ms wait time - blocking

            LOG_EVERY_N(INFO, 1000) << "Number of received frames: " << num_rx << " on channel " << channel_idx ;

            for (int i = 0; i < num_rx; i++) {
                CanMessage msg;
                msg.id = rx_msgs[i].frame.can_id;

                if (debug) LOG(INFO) << "feedback call received " << msg.id ;
                msg.timestamp = rx_msgs[i].timestamp;

                for(int k=0; k<rx_msgs[i].frame.can_dlc; k++) {
                    msg.data.push_back(rx_msgs[i].frame.data[k]);
                }
                output.push_back(msg);
            }

            if (num_rx  >0 ) curr_time = rx_msgs[0].timestamp; // microseconds  

            // clean the buffer 
            // ZCAN_ClearBuffer(ch_handles[channel_idx]);
        }
        
        return output;
    }

    public:

    std::vector<CanMessage> Q1; 
    std::vector<CanMessage> Q2;

    can_handler(const std::string& config_path) {

        try{
            YAML::Node config = YAML::LoadFile(config_path);
            baudrates[0] = config["can1"]["baud_rate"].as<int>();
            baudrates[1] = config["can2"]["baud_rate"].as<int>();

            d_baud[0] = config["can1"]["canFD_dbaud"].as<int>();
            d_baud[1] = config["can2"]["canFD_dbaud"].as<int>();

            canfd[0] = config["can1"]["canFD"].as<uint8_t>();
            canfd[1] = config["can2"]["canFD"].as<uint8_t>();

            debug = config["debug"].as<bool>();
        }
        catch (std::exception& e){
            LOG(ERROR) << "YAML Config Error: " << e.what();
        }   

        device = ZCAN_OpenDevice(DEVICE_TYPE , 0 , 0);   // one device 
        if (!device) {
            LOG(FATAL) << "Failed to open USBCAN Device!";
        }

        curr_time = 0;

        ZCAN_DEVICE_INFO device_info;
        ZCAN_GetDeviceInf(device , &device_info);

        if (debug) {
            LOG(INFO) << "Device Info: ";
            LOG(INFO) << "  HW Version: " << device_info.hw_Version;
            LOG(INFO) << "  FW Version: " << device_info.fw_Version;
            LOG(INFO) << "  Driver Version: " << device_info.dr_Version;
            LOG(INFO) << "  Interface Version: " << device_info.in_Version;
            LOG(INFO) << "  IRQ Number: " << device_info.irq_Num;
            LOG(INFO) << "  CAN Number: " << (int)device_info.can_Num;
            LOG(INFO) << "  Serial Number: " << device_info.str_Serial_Num;
            LOG(INFO) << "  HW Type: " << device_info.str_hw_Type;
        }

        for (int i = 0 ; i<2 ; i++) {

            ZCAN_CHANNEL_INIT_CONFIG config; 
            // memset(&config, 0, sizeof(config)); // to set all fields to zero

            if (canfd[i] == 1) {
                config.can_type = 1;
                config.canfd.mode = 0;
                config.canfd.filter = 0;
                config.canfd.pad = 0;
                config.canfd.brp = 0;
                config.canfd.acc_code = 0;
                config.canfd.acc_mask = 0xFFFFFFFF;
                config.canfd.reserved = 0;
            }
            else {
                config.can_type = 0;
                config.can.acc_code = 0; 
                config.can.acc_mask = 0xFFFFFFFF;
                config.can.reserved = 0;
                config.can.filter = 0;
                config.can.mode = 0;
            }

            LOG(INFO) << "Initializing CAN Channel " << i ;

            ch_handles[i] = ZCAN_InitCAN(device , i , &config); 
            if (INVALID_CHANNEL_HANDLE == ch_handles[i]) {
                LOG(FATAL) << "Failed to initialize CAN channel " << i;
            }

            LOG(INFO) << "Initializing CAN Channel " << i ;

            if (STATUS_OK !=  ZCAN_SetAbitBaud(device , i , baudrates[i])) {
                LOG(ERROR) << "Failed to set baud rate for CAN channel " << i;
            }

            LOG(INFO) << "Initializing CAN Channel " << i ;

            if (canfd[i] == 1) {
                if (STATUS_OK !=  ZCAN_SetDbitBaud(device , i , d_baud[i])) {
                    LOG(ERROR) << "Failed to set CAN FD baud rate for CAN channel " << i;
                }
                ZCAN_SetCANFDStandard(device , i , 0); // CAN FD ISO
            }

            LOG(INFO) << "Initializing CAN Channel " << i ;

        
            ZCAN_ClearFilter(ch_handles[i]); // Clear garbage filters
            ZCAN_AckFilter(ch_handles[i]);   // Accept All messages
            

            LOG(INFO) << "Initializing CAN Channel " << i ;

            if (STATUS_ERR == ZCAN_StartCAN( ch_handles[i])) {
                LOG(FATAL) << "Failed to start CAN channel " << i;
            }
            else{
                LOG(INFO) << "Started CAN Channel " << i << " at " << baudrates[i] << "bps";
            }

        }
    }

    ~can_handler(){
        ZCAN_CloseDevice(device);
    }

    void send_command(CanCommand &cmd){

        if (debug) {
            if (STATUS_ONLINE == ZCAN_IsDeviceOnLine(device)){
                LOG(INFO) << "Device is online";
            }
            else{
                LOG(FATAL) << "Device is offline while asked to send command";
            }
        }

        if (canfd[cmd.can_line] == 1){
            ZCAN_TransmitFD_Data tx_msg; 
            memset(&tx_msg, 0, sizeof(tx_msg)); // to set all fields to zero

            tx_msg.frame.can_id = cmd.can_id ; // this is by default 32 bit identifier - for extended/other flags set while sending
            tx_msg.frame.len = cmd.data.size();
            tx_msg.transmit_type = cmd.transmit_type;

            for(size_t i=0; i < cmd.data.size(); i++) {
                tx_msg.frame.data[i] = cmd.data[i];
            }

            uint success_no = ZCAN_TransmitFD(ch_handles[cmd.can_line], &tx_msg, 1);
            if (debug) LOG(INFO) << "Number of successfully sent frames" << success_no ;

        }
        else{
        ZCAN_Transmit_Data tx_msg;
        memset(&tx_msg, 0, sizeof(tx_msg)); // to set all fields to zero

        // tx_msg.frame.can_id = cmd.can_id;
        tx_msg.frame.can_id = cmd.can_id ; // this is by default 32 bit identifier - for extended/other flags set while sending
        tx_msg.frame.can_dlc = cmd.data.size();
        tx_msg.transmit_type = cmd.transmit_type;

        // if (debug) LOG(INFO) << "can_id is " << cmd.can_id << "data size" << cmd.data.size();

        for(size_t i=0; i < cmd.data.size(); i++) {
            tx_msg.frame.data[i] = cmd.data[i];
        }

        uint success_no = ZCAN_Transmit(ch_handles[cmd.can_line], &tx_msg, 1);
        if (debug) LOG(INFO) << "Number of successfully sent frames" << success_no ;

        }

        if (debug && cmd.transmit_type ==3) {
            // dig in the sent message for logging
        }
    }

    /*
    
    IF RADAR IS SENDING HIGHER NUMBER OF CAN FRAMES - INCREASE THE FD BUFFER SIZE FROM 200 TO HIGHER VALUE
    
    */

    std::vector<CanMessage> read_message(int channel_idx , std::vector<uint32_t> ids) {

        std::vector<CanMessage> buffer = read_all_messages(channel_idx);

        std::vector<CanMessage> output;

        if (channel_idx == 1){
            Q1.insert(Q1.end() , buffer.begin() , buffer.end());
            process_q(Q1); 
            for ( const auto id : ids ) {
                std::vector<CanMessage> fetched = fetch_messages(id , Q1);
                output.insert( output.end() , fetched.begin() , fetched.end() );
            }

        }
        else {
            Q2.insert(Q2.end() , buffer.begin() , buffer.end());
            process_q(Q2); 
            for ( const auto id : ids ) {
                std::vector<CanMessage> fetched = fetch_messages(id , Q2);
                output.insert( output.end() , fetched.begin() , fetched.end() );
            }
        }
        // check length of the que  - maintain max que for 0.5 seconds
        // given id return messages from the que  

        return output;
    }

};

