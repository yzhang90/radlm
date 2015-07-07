/*	Dynamic Window Approach Safety Monitor

	Version:
	C99, Intel C++, SSE 4.1

	inputs:
	D[0] = (A/b+1)*(A/2*eps^2+eps*V)
	D[1] = V/b + eps*(A/b+1)
	D[2] = 1/(2*b)

	X[0] = vr
	X[1..2] = pr.x, pr.y
	x[3..4] = po.x, po.y

	computes:
	(A/b+1)*(A/2*eps^2+eps*V) + (V/b + eps*(A/b+1))*vr + (1/(2*b))*vr^2 < || pr - po||_oo

	return:
	1 => true
	0 => false
	-1 => unknown

*/

#ifndef __DWMONITOR2_H__
#define __DWMONITOR2_H__

#ifdef __cplusplus
extern "C" {
#endif

void dwmonitor_all(int  *Y, float  *X, double  *D, float  *obs);
void dwmonitor2_err(int  *Y, float  *X, double  *D, float  *obs);
void dwmonitor2(int  *Y, float  *X, double  *D, float  *obs);

#ifdef __cplusplus
}
#endif

#endif //__DWMONITOR_H__
