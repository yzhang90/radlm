#include "read_map.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <stdexcept>

int read_map( const std::string& file, const size_t map_size, map_t& map ) 
{
    std::ifstream myfile;
    myfile.open(file.c_str());

    map_t map_blank;
    map_blank.points.resize( map_size );
    map.points.resize( map_size );
    
    int i1=0;
    if (myfile.is_open())
    {
      while (myfile.good())
      {
        std::string s;
        if (!getline(myfile, s)) 
        {
          break;
        }
        std::istringstream ss(s);
        int i2=0;
        while (ss)
        {
          std::string s;
          if (!getline(ss, s, ',')) break;
          if (i2==0) map.points[i1].x = atof(s.c_str());
          if (i2==1) map.points[i1].y = atof(s.c_str());
          if (i2==2) map.points[i1].z = atof(s.c_str());
          i2++;
        }
        if (i2!=3)
        {
          map = map_blank;
          throw std::invalid_argument( "obstacle in map file has invalid number of entries");
        }
        i1++;
      }
      if (i1!=map_size)
      {
        map = map_blank;
        std::ostringstream int2str;
        int2str << i1;
        throw std::invalid_argument( "map file has invalid number of obstacles = "+int2str.str());
      }
    }
    else {
      throw std::invalid_argument( "error opening file "+file);
    }
  myfile.close();

  map.data = true;

  return 0;  
}

