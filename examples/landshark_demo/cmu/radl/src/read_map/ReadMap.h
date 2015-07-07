#ifndef READMAP_H
#define READMAP_H

#include RADL_HEADER

#include <iostream>
#include <fstream>

struct point_t {
      double x;
      double y;
      double z;
    };
struct map_t {
      bool data;
      std::vector<point_t> points;
    };

class ReadMap {
 private:
   map_t map;
   const size_t map_size_;
   int map_size;
   std::string file;
//   int read_map( const std::string& , const size_t, map_t& );
 public:
  ReadMap();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};

#endif // READMAP_H
