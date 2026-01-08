
#include "mpc.hpp"
#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <iostream>
#include <vector>

using namespace webots;

int main() {
  Supervisor robot;
  int timestep = (int)robot.getBasicTimeStep();

  Emitter *emitter = robot.getEmitter("emitter");
  Receiver *receiver = robot.getReceiver("receiver");
  receiver->enable(timestep);

  // MPC initialization   - first elem for accel and 2nd for brake
  std::vector<float> a = {0.5 , 0.5};
  std::vector<int> N = { 7 ,  7} ;
  std::vector<int> Nc = {15 , 15};
  std::vector<int> Np = {40 , 40};

  StateSpaceModel mpc;
  mpc.init_controller(a , N , Nc , Np);

  // State variables
  //  Previous States
  float d_c_prev = 0.0f;
  float v_c_prev = 0.0f;
  float ah_prev  = 0.0f;
  bool first_run = true;

  // change variable
  float del_d = 0.0f;
  float del_v = 0.0f;
  float del_a = 0.0f;

  // Current state
  float d_c = 0.0f;
  float v_c = 0.0f;
  float ah = 0.0f;
  float u = 0.0f;
  float u_prev = 0.0f;
  std::string mode = " ";

  std::vector<float> X0 = {u_prev, del_d, del_v, del_a, d_c, v_c, ah};

  // Time tracking for Integral Error
  double last_packet_time = 0.0;

  while (robot.step(timestep) != -1) {

    if (receiver->getQueueLength() > 0) {
        bool received = false;

        // Create a local array to store the latest values safely
        float latest_data[4] = {0};

        // Drain queue, but COPY data from every packet before discarding it
        while (receiver->getQueueLength() > 0) {
            const void *buffer = receiver->getData();

            // Copy the data immediately while 'buffer' is still valid
            if (buffer) {
                const float* data = static_cast<const float*>(buffer);
                latest_data[0] = data[0]; // u_prev
                latest_data[1] = data[1]; // d_c
                latest_data[2] = data[2]; // v_c
                latest_data[3] = data[3]; // ah
                received = true;
            }

            // NOW we can release the packet
            receiver->nextPacket();
        }

        if (received) {
            u_prev = latest_data[0];
            d_c    = latest_data[1];
            v_c    = latest_data[2];
            ah     = latest_data[3];

            // 2. Handle First Run Initialization
            if (first_run) {
                d_c_prev = d_c;
                v_c_prev = v_c;
                ah_prev = ah;
                first_run = false;
            }

            // 3. Calculate Deltas (Change in state)
            del_d = d_c - d_c_prev;
            del_v = v_c - v_c_prev;
            del_a = ah  - ah_prev;

            // 4. Update Previous States for next loop
            d_c_prev = d_c;
            v_c_prev = v_c;
            ah_prev  = ah;

            X0 = {u_prev, del_d, del_v, del_a, d_c, v_c, ah};

            // Pass this corrected vector to MPC
            auto [mode , u] = mpc.mpc_osqp(X0);
            // u = mpc.mpc_qph(X0);

            std::cout << "d=" << d_c << " del_d= " << del_d 
            << " v_c " << v_c << " del_v" << del_v << "  ah" << ah << "  del_a" << del_a <<
             " Output u= " << u << " " << mode <<std::endl;

            emitter->send(&u, sizeof(float));
        }
      }

}
  return 0;

}
