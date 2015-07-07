//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    readGPSCoordinateFile.cpp
 * \brief   reads from text file
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================


#include "readGPSCoordinateFile.h"
#include <sstream>

readGPSCoordinateFile::readGPSCoordinateFile()
{

}

void readGPSCoordinateFile::readFile(string fileName)
{
  //Sample GPS File 
  bool success = false;
  ifstream myfile(fileName.c_str());
  cout<<"\nName of File: "<<fileName;
  string line;
  float x;
  float y;
  float z = 0;
  int tag;
  if (myfile.is_open())
  {
    while (myfile.good())
    {
      getline(myfile, line);
      stringstream ss(line);
      ss >> tag;
      if (tag == 2)
        break;
      ss >> x;
      ss >> y;
      //int tagInt = tag;
      coordinates.push_back(x);
      coordinates.push_back(y);
      coordinates.push_back(z);
      coordinates.push_back(float(tag));
      //tags.push_back(tagInt); 
      //cout<<"\n Reading Things \n"<<line;
    }
    myfile.close();
  }

}
