#ifndef __SENSOR_FUSION_H__
#define __SENSOR_FUSION_H__

#ifdef __cplusplus
extern "C" {
#endif

void fused(int  *Y, double  *X, double  *Vertex, double  *Norm, double  *BoundRadius, double  *state);

#ifdef __cplusplus
}
#endif

#endif //__SENSOR_FUSION_H__

