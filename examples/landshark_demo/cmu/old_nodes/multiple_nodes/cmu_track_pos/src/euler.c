/*	Euler Integrator

	Version:
	C99, Intel C++, SSE 4.1

	inputs:
	X0[0] = current_x
	X0[1] = current_y
	X0[2] = current_z
	
	X0[3] = current_vx
	X0[4] = current_vy
	X0[5] = current_vz
	
	dt = current_time_step

	computes:
	X[0] = X0[0] + dt*X0[3]
	X[1] = X0[1] + dt*X0[4]
	X[2] = X0[2] + dt*X0[5]

	return:
	void

*/

#include "euler.h"
#include <smmintrin.h>
#include <float.h>

void euler(double  *X0, double *X, double *dt) {
  X[0] = X0[0] + dt*X0[3];
  X[1] = X0[1] + dt*X0[4];
  X[3] = X0[2] + dt*X0[5];
return void;
}
