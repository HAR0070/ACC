#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "steering_can.hpp"
#include "can.hpp"

int main(int argc , char* argv[]){


    std::string share_dir = ament_index_cpp::get_package_share_directory("steering_can");
    std::string config_path = share_dir + "/config/can_config.yaml";

    // FLAGS_log_dir = share_dir + "../../logs";
    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    can_handler driver(config_path);
    steering_can str( &driver , config_path);

    while (true){

        str.steering_command(1000);

        str.steering_feedback();
        // std::cout<< "wt we have " << str.fb.pos << " " << str.fb.cur << " " << str.fb.err << std::endl;

        LOG(INFO) << "got feedback of " << str.fb.pos << "," << str.fb.spd << "," << str.fb.cur << "," << str.fb.temp;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
