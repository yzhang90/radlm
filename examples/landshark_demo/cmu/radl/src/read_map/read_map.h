#ifndef READ_MAP_H
#define READ_MAP_H

#include "ReadMap.h"
#include <iostream>
#include <fstream>

int read_map( const std::string& file, const size_t map_size, map_t& map ) 
{
    std::ifstream myfile;
    myfile.open(file.c_str());

    if(myfile.bad())
    {
      std::cout << std::endl << "Failed to open file " << file << std::endl; 
      return(1);
    }
    std::vector <std::vector <std::string> > data;
    int i1=0;
    if (myfile.is_open())
    {
      std::cout << std::endl << "Reading map from " << file << std::endl;
      while (myfile.good())
      {
        std::string s;
        if (!getline(myfile, s)) break;
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
        i1++;
      }
    }
  myfile.close();

  map.data = true;

  return 0;  
}

#endif //READ_MAP_H

