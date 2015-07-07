#pragma once 

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include "ocu.pb.h"
#include <unistd.h>

#include "read_map.h"

class ocu_server {
  public:
    ocu_server();
    void send();
    void receive();

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

    double ccc_speed;
    bool select_joy;
    bool select_ccc;
    bool select_pp;

    double pp_goal_lat;
    double pp_goal_lon;
    double pp_goal_alt;
    std::string last_pp_map_file;
    std::string pp_map_file;
    map_t pp_map;
    size_t pp_map_size;

  private:
    uint64_t seq;
    zmq::socket_t socket_status;
    zmq::socket_t socket_command;
};

