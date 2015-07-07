#pragma once 
#include <vector>
#include <string>

struct point_t {
  double x;
  double y;
  double z;
};

struct map_t {
  bool data;
  std::vector<point_t> points;
};

int read_map( const std::string& file, const size_t map_size, map_t& map );

