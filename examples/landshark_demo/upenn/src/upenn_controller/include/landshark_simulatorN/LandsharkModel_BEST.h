#ifndef __LANDSHARK_MODEL__
#define __LANDSHARK_MODEL__

#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv.h>

#define DIMENSION 11

namespace hacms
{
  typedef struct
  {
    double mass;
    double length;
    double width;

    double wheel_diameter;
    double wheel_width;

    double B;
    double SL;
    double Br;

    double Jt;
    double Lm;
    double Rm;
    double alpha;
    double Jg;
    double r;
    double g;
    double Kt;
    double eR;
    double eL;
  } model_params_t;

  class LandsharkModel
  {
  public:
    LandsharkModel();
    ~LandsharkModel();

    void ResetSimulation();
    void Update(double simulation_time);
    void SetCommand(float left_cmd, float right_cmd);
    void GetEncoders(int &left_enc, int &right_enc);
    void GetIMU(double &roll_imu, double &pitch_imu, double &yaw_imu);
    void GetGPS(double &latitude_gps, double &longiude_gps);
    void GetPose(double &x, double &y, double &psi);
    void GetVelocity(double &vx, double &vy, double &wz);
    void SetParameters(const model_params_t &params);

  private:
    static int ODEStep(double t, const double* x, double* xdot, void* params);

    double time;
    double statespace[DIMENSION];

    model_params_t model_params;

    gsl_odeiv_step* step;
    gsl_odeiv_control* control;
    gsl_odeiv_evolve* evolve;

    double x, y, psi;
    double v, omega;
    double iR, wR, FR;
    double iL, wL, FL;

    double dt;
    bool params_loaded;

    double encoder_left, encoder_right;
    double imu_roll, imu_pitch, imu_yaw;
    double gps_latitude, gps_longitude;
  };
}
#endif
