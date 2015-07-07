#include <iostream>
#include <boost/algorithm/string.hpp>
#include "3dmgx2.h"

int main(int argc, char **argv)
{
  if ( argc >= 2 ) {
    std::string port_( argv[1] );
    std::string device_id_;

    microstrain_3dmgx2_imu::IMU imu_;
    try {
      imu_.openPort(port_.c_str());
      char val[64];
      imu_.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_SERIAL_NUMBER, val );
      device_id_ = std::string( val );
      boost::algorithm::trim( device_id_ );
    }
    catch (microstrain_3dmgx2_imu::Exception& e) {
      imu_.closePort();
      std::cerr << "Exception thrown while opening IMU: " << e.what() << std::endl;
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
