#include "ocu_client.h"
#include "ocu_config.h"
#include "read_map.h"
#include <stdexcept>

ocu_client::ocu_client()
  : seq( 0 )
  , rseq( 0 )
  , socket_status(context_status, ZMQ_SUB)
  , socket_command(context_command, ZMQ_PUB)
{
  // Prepare subscriber context and socket
  {
    std::stringstream ss;
    ss << "tcp://localhost:" << port_status;
    socket_status.setsockopt( ZMQ_SUBSCRIBE, "", 0);
    socket_status.setsockopt( ZMQ_RCVTIMEO, &timeout_ms, sizeof( timeout_ms ) );
    socket_status.connect( ss.str().c_str() );
  }

  // Prepare publisher context and socket
  {
    std::stringstream ss;
    ss << "tcp://localhost:" << port_command;
    socket_command.connect( ss.str().c_str() );
  }
}

void ocu_client::receive()
{
  using namespace ocu;
  Status s = Status();

  // Get the reply.
  zmq::message_t msg;
  bool success = socket_status.recv( &msg );
  if ( success ) {
    while ( success ) {
      s.ParseFromArray( msg.data(), msg.size() );
      rseq = s.seq();
      rsize = msg.size();
      if ( s.has_fsm_status() ) {
        fsm_status = s.fsm_status();
      }
      if ( s.has_fsm_actuator() ) {
        fsm_actuator = s.fsm_actuator();
      }
      if ( s.has_base_status() ) {
        base_status = s.base_status();
      }
      if ( s.has_actuator_status() ) {
        actuator_status = s.actuator_status();
      }
      if ( s.has_rse_status() ) {
        rse_status = s.rse_status();
      }
      if ( s.has_ccc_status() ) {
        ccc_status = s.ccc_status();
      }
      if ( s.has_pp_status() ) {
        pp_status = s.pp_status();
      }
      if ( s.has_monitor_status() ) {
        monitor_status = s.monitor_status();
      }
      if ( s.has_gps() ) {
        gps = s.gps();
      }
      if ( s.has_mag_front() ) {
        mag_front = s.mag_front();
      }
      if ( s.has_mag_rear() ) {
        mag_rear = s.mag_rear();
      }
      success = socket_status.recv( &msg );
    }
  }
  else {
    rsize = 0;
    if ( seq % 100 == 0 ) {
      std::cout << "Did not receive message " << std::endl;
    }
  }
  std::stringstream ss;
  ss << "Last received: seq= " << rseq << ", size= " << rsize;
  receive_status = ss.str();
  //std::cout << receive_status << std::endl;
}

void ocu_client::send()
{
  std::stringstream ss;
  seq++;
  using namespace ocu;
  Command c = Command();
  c.set_seq( seq );
  if ( ccc_start ) {
    c.set_select_ccc( true );
  }
  else if ( pp_start ) {
    c.set_select_pp( true );
  }
  else if ( pp_stop || ccc_stop ) {
    c.set_select_joy( true );
  }
  pp_start = pp_stop = false;
  ccc_start = ccc_stop = false;
  c.set_ccc_speed( ccc_speed );
  c.set_pp_goal_lat( pp_goal_lat );
  c.set_pp_goal_lon( pp_goal_lon );
  c.set_pp_goal_alt( pp_goal_alt );
  c.set_pp_map_file( pp_map_file );
#if 1
  map_t pp_map;
  size_t pp_map_size( 16 );
  if ( pp_map_file.size() ) {
    try {
      read_map( pp_map_file, pp_map_size, pp_map );
      ss << "read_map( " << pp_map_file << ") succeeded!";
      std::cout << ss.str() << std::endl;
      // load map in variable
      Map *m = new Map();
      std::cout << "Loaded map (" << pp_map.points.size() << " points)" << std::endl;
      for ( size_t i = 0; i < pp_map.points.size(); i++ ) {
        Point *p = m->add_point();
        p->set_x( pp_map.points[i].x );
        p->set_y( pp_map.points[i].y );
        p->set_z( pp_map.points[i].z );
        std::cout << i + 1 << "  [" << pp_map.points[i].x << ", " << pp_map.points[i].y << ", " << pp_map.points[i].z << "]" << std::endl;
      }
      c.set_allocated_map( m );
    }
    catch (const std::invalid_argument& e) {
      ss << "read_map( " << pp_map_file << ") crashed due to " << e.what();
      std::cerr << ss.str() << std::endl;
    }

    pp_map_file = "";
  }
#endif

  std::string str;
  c.SerializeToString( &str );
  zmq::message_t cmd ( str.length() );
  memcpy ((void *) cmd.data (), str.c_str(), str.length() );
  ss << "Sending cmd: " << c.seq() << "…" << std::endl;
  socket_command.send (cmd);

  send_status = ss.str();
}

