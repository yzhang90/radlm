#include <iostream>
#include <cstdlib>
#include <stdexcept>
#include <sstream>
#include "read_map.h"

int main(int argc, char *argv[])
{
  if (argc < 2 ) {
    std::cout << "Usage: " << argv[0] << " <map_file> [<map_size>]" << std::endl;
    return 0;
  }
  map_t pp_map;
  size_t pp_map_size( 16 );
  if ( argc > 2 ) { 
    pp_map_size = atoi( argv[2] );
  }
  std::string pp_map_file( argv[1] );
 
  std::cout << "Reading map: " << pp_map_file << " size= " << pp_map_size << std::endl;

  std::stringstream ss;
  try {
    read_map( pp_map_file, pp_map_size, pp_map );
    ss << "read_map( " << pp_map_file << ") succeeded!";
    std::cout << ss.str() << std::endl;
  }
  catch (const std::invalid_argument& e) {
    ss << "read_map( " << pp_map_file << ") crashed due to " << e.what();
    std::cerr << ss.str() << std::endl;
  }

//print to check
  std::cout.precision(15);
  for ( size_t i = 0; i < pp_map_size; i++ ) {
      std::cout << std::endl << "obstacle " << i+1 << ": " << pp_map.points[i].x;
      std::cout << " " << pp_map.points[i].y;
      std::cout << " " << pp_map.points[i].z << std::endl ;
  }

}

