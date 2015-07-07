#include "ocu_server.h"
#include "ocu_config.h"
#include <math.h>

ocu_server::ocu_server()
  : seq( 0 )
  , socket_status(context_status, ZMQ_PUB)
  , socket_command(context_command, ZMQ_SUB)
{
  // Prepare subscriber context and socket
  {
    std::stringstream ss;
    ss << "tcp://*:" << port_command;
    socket_command.setsockopt( ZMQ_SUBSCRIBE, "", 0);
    socket_command.setsockopt( ZMQ_RCVTIMEO, &timeout_ms, sizeof( timeout_ms ) );
    socket_command.bind( ss.str().c_str() );
  }
  // Prepare publisher context and socket
  {
    std::stringstream ss;
    ss << "tcp://*:" << port_status;
    socket_status.bind( ss.str().c_str() );
  }
}

void ocu_server::receive()
{
  using namespace ocu;
  Command c = Command();

  // Get the reply.
  zmq::message_t msg;
  std::stringstream ss;
  bool success = socket_command.recv( &msg );
  if ( success ) {
    while ( success ) {
      c.ParseFromArray( msg.data(), msg.size() );
      uint64_t count = c.seq();
      ss << "Received (msg size= " << msg.size() << ") " << count << std::endl;

      if ( c.has_ccc_speed() ) {
        if ( fabs( ccc_speed - c.ccc_speed() ) > 1e-10 ) {
          std::cout << "received ccc_speed: " << c.ccc_speed() << std::endl;
        }
        ccc_speed = c.ccc_speed();
      }
      if ( c.has_select_joy() ) {
        select_joy = c.select_joy();
      }
      else if ( c.has_select_ccc() ) {
        select_ccc = c.select_ccc();
      }
      else if ( c.has_select_pp() ) {
        select_pp = c.select_pp();
      }
      if ( c.has_pp_goal_lat() ) {
        pp_goal_lat = c.pp_goal_lat();
      }
      if ( c.has_pp_goal_lon() ) {
        pp_goal_lon = c.pp_goal_lon();
      }
      if ( c.has_pp_goal_alt() ) {
        pp_goal_alt = c.pp_goal_alt();
      }
      pp_map.data = false;
      if ( c.has_map() ) {
        Map *m = c.mutable_map();
        pp_map.points.clear();
        pp_map.data = true;
        std::cout << "Received map (" << m->point_size() << " points)" << std::endl;
        for ( int i = 0; i < m->point_size(); i++ ) {
          point_t p;
          p.x = m->mutable_point( i )->x();
          p.y = m->mutable_point( i )->y();
          p.z = m->mutable_point( i )->z();
          pp_map.points.push_back( p );
          std::cout << i + 1 << "  [" << p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
        }
      }
      success = socket_command.recv( &msg );
    }
  }
  else {
    ss << "Did not receive message" << std::endl;
  }
  if ( seq % 100 == 0 ) {
	  std::cout << ss.str() << std::endl;
  }
}

void ocu_server::send()
{
  seq++;
  using namespace ocu;
  Status s = Status();
  s.set_seq( seq );
  if ( fsm_status.size() ) {
    s.set_fsm_status( fsm_status );
  }
  if ( fsm_actuator.size() ) {
    s.set_fsm_actuator( fsm_actuator );
  }
  if ( base_status.size() ) {
    s.set_base_status( base_status );
  }
  if ( actuator_status.size() ) {
    s.set_actuator_status( actuator_status );
  }
  if ( rse_status.size() ) {
    s.set_rse_status( rse_status );
  }
  if ( ccc_status.size() ) {
    s.set_ccc_status( ccc_status );
  }
  if ( pp_status.size() ) {
    s.set_pp_status( pp_status );
  }
  if ( monitor_status.size() ) {
    s.set_monitor_status( monitor_status );
  }
  if ( gps.size() ) {
    s.set_gps( gps );
  }
  s.set_mag_front( mag_front );
  s.set_mag_rear( mag_rear );

  std::string str;
  s.SerializeToString( &str );
  zmq::message_t status( str.length() );
  memcpy( (void *) status.data(), str.c_str(), str.length() );
  if ( seq % 100 == 0 ) {
    std::cout << "Sending status: " << s.seq() << "…" << std::endl;
  }
  socket_status.send( status );

  fsm_status = "";
  fsm_actuator = "";
}

