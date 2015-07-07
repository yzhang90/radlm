#include <iostream>
#include "hokuyo.h" 

int main(int argc, char **argv)
{
  if ( argc >= 2 ) {
    std::string port_( argv[1] );
    std::string device_id_;

    hokuyo::Laser laser_;
    try {
      hokuyo::Laser laser_;
      laser_.open(port_.c_str());
      device_id_ = laser_.getID();
    }
    catch (hokuyo::Exception& e) {
      laser_.close();
      std::cerr << "Exception thrown while opening Hokuyo: " << e.what() << std::endl;
      return -1;
    }
    std::cout << device_id_ << std::endl;
    return 0;
  }
  else {
    std::cerr << "Usage: " << argv[0] << " <device_name>" << std::endl;
  }
  return -1;
}
