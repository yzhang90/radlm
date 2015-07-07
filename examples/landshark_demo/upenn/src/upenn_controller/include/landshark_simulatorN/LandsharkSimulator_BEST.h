#ifndef __LANDSHARK_SIMULATOR__
#define __LANDSHARK_SIMULATOR__

#include <landshark_simulator/LandsharkModel.h>

namespace hacms
{
  class LandsharkSimulator
  {
  public:
    LandsharkSimulator();
    ~LandsharkSimulator();

    void ResetSimulation();
    void SetTimeStep(double dt);
    double GetSimulationTime();
    void Update();

    void SetCommand(float left_cmd, float right_cmd);
    void GetEncoders(int &left_enc, int &right_enc);
    void GetIMU(double &roll_imu, double &pitch_imu, double &yaw_imu);
    void GetGPS(double &latitude_gps, double &longitude_gps);
    void GetState(double &x, double &y, double &psi,
                  double &vx, double &vy, double &wz);
    LandsharkModel& GetModel();

  private:
    double simulation_time;
    double time_step;

    LandsharkModel model;
  };
}
#endif
