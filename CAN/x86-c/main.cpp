#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <iostream>

#include "steering_can.hpp"
#include "can.hpp"



int main(int argc , char* argv[]){

    FLAGS_log_dir = "./logs";
    google::InitGoogleLogging(argv[0]);

    can_handler driver;
    steering_can str( &driver);

    while (true){

        str.steering_command(1000);

        str.steering_feedback();
        std::cout<< "wt we have " << str.fb.pos << " " << str.fb.cur << " " << std::endl;

        LOG(INFO) << "got feedback of " << str.fb.pos << "," << str.fb.spd << "," << str.fb.cur << "," << str.fb.temp;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0; 
}