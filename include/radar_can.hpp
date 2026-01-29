#pragma once
#include "can.hpp"
#include <vector>
#include <map>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

struct RadarPoint {
    int id;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float velocity;
    float rcs; // Signal Strength (dB)
    bool has_position; // Flag to ensure we have at least Frame 0
};

class radar_can {
private:
    can_handler* driver;
    int can_line;
    bool debug;

    // Temporary map to merge Subframe A (pos) and Subframe B (height)
    // Until the next 0x600 frame comes - signalling completetion of 1 cycle
    std::map<int, RadarPoint> point_map;

public:
    // This buffer holds the points from the latest update
    std::vector<RadarPoint> current_points;

    radar_can(can_handler* can_ptr , const std::string &config_path ) : driver(can_ptr) {
        LOG(INFO) << "Initializing Radar Driver";
        try {
            YAML::Node config = YAML::LoadFile(config_path); 

            std::string line = config["radar"]["can_line"].as<std::string>();
            can_line = (line == "can1") ? 0 : 1;
            debug = config["radar"]["debug"].as<bool>();

        } catch (const std::exception& e) {
            LOG(ERROR) << "Radar YAML Error: " << e.what();
        }
    }

    void update_radar() {
        std::vector<uint32_t> expected_ids = {0x600, 0x701}; // Radar Status and Detection Data
        std::vector<CanMessage> msgs = driver->read_message(can_line , expected_ids);

        for (const auto& msg : msgs) {
            // ID 0x600: Point cloud status information 
            // This message comes when 1 cycle is completed 
            // so till then its the previous cycles points in the buffer
            if (msg.id == 0x600) {
                current_points.clear();
                for (auto const& [id, point] : point_map) {
                    // Only add points that received at least Frame 0 (Position)
                    if (point.has_position) {
                        current_points.push_back(point);
                        if(debug) {
                            LOG(INFO) << "Obj " << id << " [X:" << point.x << " Y:" << point.y
                                    << " Z:" << point.z << " RCS:" << point.rcs << "]";
                        }
                    }
                }
                // clear for next cycle 
                point_map.clear(); 

                if (debug && msg.data.size() > 0) {
                    LOG(INFO) << "Radar Detect Header: " << (int)msg.data[0] << " objects";
                }
            }
            // ID 0x701: Detection Data (Trace Protocol)
            else if (msg.id == 0x701) {
                if (msg.data.size() < 8) continue;

                const unsigned char* d = msg.data.data();

                // Byte 0: Object ID (7 bits) and Frame Code (1 bit) [cite: 1402]
                int obj_id = d[0] & 0x7F;
                int frame_code = (d[0] & 0x80) >> 7; // 0 = Frame A, 1 = Frame B

                // Create entry if not exists
                if (point_map.find(obj_id) == point_map.end()) {
                    RadarPoint p;
                    p.id = obj_id;
                    p.x = 0; p.y = 0; p.z = 0; // Default Z is 0 if Frame 1 missing
                    p.velocity = 0;
                    p.rcs = 0;
                    p.has_position = false;
                    point_map[obj_id] = p;
                }

                if (frame_code == 0) {
                    // Frame 0 (Subframe A): [user manual page: 38]
                    // Long Dist (X):
                    float long_dist = ((d[1] << 5) | (d[2] >> 3)) * 0.05f - 100.0f;

                    // Lat Dist (Y):
                    float lat_dist = ((d[2] & 0x07) << 8 | d[3]) * 0.05f - 50.0f;

                    // Long Speed (Vx):
                    float long_speed = ((d[4] << 2) | (d[5] >> 6)) * 0.25f - 128.0f;

                    // Lat Speed (Vy):
                    float lat_speed = ((d[5] & 0x3F) << 3 | (d[6] >> 5)) * 0.25f - 64.0f;

                    // RCS (Echo Intensity)
                    float rcs_val = (float)d[7];

                    point_map[obj_id].x = long_dist;
                    point_map[obj_id].y = lat_dist;
                    point_map[obj_id].vx = long_speed;
                    point_map[obj_id].vy = lat_speed;
                    point_map[obj_id].velocity = std::sqrt(long_speed*long_speed + lat_speed*lat_speed);
                    point_map[obj_id].rcs = rcs_val;
                    point_map[obj_id].has_position = true;

                    // if(debug){
                    //     LOG(INFO) << "found point" << long_dist << " " << lat_dist;
                    // }

                }
                else if (frame_code == 1) {
                    // --- Frame 1 (Subframe B): Height (Z) page: 39] ---
                    // Height (Z):
                    float z_height = ((d[1] << 2) | (d[2] >> 6)) * 0.1f - 30.0f;
                    point_map[obj_id].z = z_height;
                }
            }
        }
        
    }
};
