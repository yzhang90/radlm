#include "read_map.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

ReadMap::ReadMap() 
: map_size_( *RADL_THIS->array_size )
{
  this->map;
  this->map_size=map_size_;
  this->file="obstacles.map";
  this->map.points.resize( map_size_ );
  read_map( this->file, map_size_, this->map );

  std::cout.precision(15);
  for ( size_t i = 0; i < map_size_; i++ ) {
      std::cout << std::endl << "obstacle " << i+1 << ": " << this->map.points[i].x;
      std::cout << " " << this->map.points[i].y;
      std::cout << " " << this->map.points[i].z << std::endl ;
  }

}

void ReadMap::step(const radl_in_t * in, const radl_in_flags_t* iflags,
                 radl_out_t * out, radl_out_flags_t * oflags) {

// XXX We assume map is an fixed size array of points
  out->map->data = this->map.data;
  if ( map.data ) {
    assert( map_size_ == this->map_size );
    for ( size_t i = 0; i < map_size_; i++ ) {
      out->map->points[i].x = this->map.points[i].x;
      out->map->points[i].y = this->map.points[i].y;
      out->map->points[i].z = this->map.points[i].z;
    }
  }

//sim: 37.4571196065 , -122.173347433, 0.0 (15.0, 0.0, 0.0)
    out->goal->latitude = 37.4568886861; //(25.0,-25.0,0.0) meters
    out->goal->longitude = -122.173227989;
    out->goal->altitude = 0.0;

return;
}
