#pragma once 

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include "ocu.pb.h"
#include <unistd.h>

class ocu_client {
  public:
    ocu_client();
    void send();
    void receive();

    std::string receive_status;
    std::string send_status;

    // CCC
    double ccc_speed;
    bool ccc_start;
    bool ccc_stop;

    // PP
    std::string pp_map_file;
    double pp_goal_lat;
    double pp_goal_lon;
    double pp_goal_alt;
    bool pp_start;
    bool pp_stop;

    // status 
    std::string fsm_status;
    std::string fsm_actuator;
    std::string base_status;
    std::string actuator_status;
    std::string rse_status;
    std::string ccc_status;
    std::string pp_status;
    std::string monitor_status;
    std::string gps;
    double mag_front;
    double mag_rear;


  private:
    uint64_t seq;
    uint64_t rseq;
    uint64_t rsize;
    zmq::socket_t socket_status;
    zmq::socket_t socket_command;
};

