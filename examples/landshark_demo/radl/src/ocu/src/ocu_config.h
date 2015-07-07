#pragma once

#include <zmq.hpp>

const int port_status( 5555 );
const int port_command( 5556 );
zmq::context_t context_status( 1 );
zmq::context_t context_command( 2 );
const int timeout_ms( 0 );

