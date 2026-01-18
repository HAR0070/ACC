#pragma once
#include "can.hpp"
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

struct RadarPoint {
    float x;
    float y;
    float z;
    float velocity;
    int id;
};

class radar_can {
private:
    can_handler* driver;
    int can_line;
    bool debug;
    
public:
    // This buffer holds the points from the latest update
    std::vector<RadarPoint> current_points;

    radar_can(can_handler* can_ptr) : driver(can_ptr) {
        LOG(INFO) << "Initializing Radar Driver";
        try {
            YAML::Node config = YAML::LoadFile("can_config.yaml"); 
            
            std::string line = config["radar"]["can_line"].as<std::string>();
            can_line = (line == "can1") ? 0 : 1; 
            debug = config["radar"]["debug"].as<bool>();

        }catch (const std::exception& e) {
            LOG(ERROR) << "Radar YAML Error: " << e.what();
            can_line = 1; 
        }
    }

    void update_radar() {

        std::vector<CanMessage> msgs = driver->read_all_messages(can_line);
        
        // New snapshot of the current scan
        current_points.clear();

        for (const auto& msg : msgs) {
            parse_message(msg);
        }
    }
    
    void parse_message(const CanMessage& msg) {
        // ID 0x600: Header (Number of objects)
        if (msg.id == 0x600) {
            if (debug && msg.data.size() > 0) {
                // msg.data[0] is num_objs
                LOG(INFO) << "Radar Detect Header: " << (int)msg.data[0] << " objects";
            }
        }
        // ID 0x701: Detection Data
        else if (msg.id == 0x701) {
            if (msg.data.size() < 7) return;
            
            const unsigned char* d = msg.data.data();
            
            // Byte 0: Object ID (7 bits) and Frame Code (1 bit)
            int obj_id = d[0] & 0x7F;
            int frame_code = d[0] & 0x80;
            
            // We only care about Subframe A (0x00) which contains position X, Y
            if (frame_code == 0x00) {
                
                // --- Decoding Logic from radar_parse.py ---
                
                // Long Dist (X): 13 bits (Byte1 + top 5 bits of Byte2)
                // Formula: (d[1] * 32 + (d[2] >> 3)) * 0.05 - 100
                float long_dist = ((d[1] << 5) | (d[2] >> 3)) * 0.05f - 100.0f;
                
                // Lat Dist (Y): 11 bits (lower 3 bits of Byte2 + Byte3)
                // Formula: (((d[2] & 0x07) * 256) + d[3]) * 0.05 - 50 
                float lat_dist = ((d[2] & 0x07) << 8 | d[3]) * 0.05f - 50.0f;
                
                // Speed decoding (Optional, helpful for visualization intensity)
                // Long Speed
                float long_speed = ((d[4] << 2) | (d[5] >> 6)) * 0.25f - 128.0f;
                // Lat Speed
                float lat_speed = ((d[5] & 0x3F) << 3 | (d[6] >> 5)) * 0.25f - 64.0f;
                
                RadarPoint p;
                p.x = long_dist;
                p.y = lat_dist;
                p.z = 0.0f; // Radar is 2D
                p.velocity = std::sqrt(long_speed*long_speed + lat_speed*lat_speed); // Resultant speed
                p.id = obj_id;
                
                current_points.push_back(p);
                
                if (debug) {
                    LOG(INFO) << "Obj[" << obj_id << "] X:" << p.x << " Y:" << p.y;
                }
            }
        }
    }
};