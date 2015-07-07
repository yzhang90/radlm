/*
 * File: PID_d.h
 *
 * Code generated for Simulink model 'PID_d'.
 *
 * Model version                  : 1.56
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Dec 29 2011)
 * C/C++ source code generated on : Thu Jan 24 12:34:03 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PID_d_h_
#define RTW_HEADER_PID_d_h_
#ifndef PID_d_COMMON_INCLUDES_
# define PID_d_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* PID_d_COMMON_INCLUDES_ */

#include "PID_d_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Delay1_DSTATE;                /* '<S1>/Delay1' */
  real_T Integrator1_DSTATE;           /* '<S1>/Integrator1' */
  real_T Filter_DSTATE;                /* '<S1>/Filter' */
  real_T Y[12];                        /* '<S1>/Data Store Memory1' */
  real_T U[21];                        /* '<S1>/Data Store Memory6' */
  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<S1>/Scope1' */
} D_Work_PID_d;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T v[3];                         /* '<Root>/v' */
  real_T r;                            /* '<Root>/r' */
} ExternalInputs_PID_d;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExternalOutputs_PID_d;

/* Parameters (auto storage) */
struct Parameters_PID_d_ {
  real_T SFunction_p1[49];             /* Expression: A
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T SFunction_p2[14];             /* Expression: B
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T SFunction_p3[21];             /* Expression: C
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T SFunction_p4[84];             /* Expression: CAks
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T SFunction_p5;                 /* Expression: T
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S1>/Delay1'
                                        */
  real_T ProportionalGain_Gain;        /* Expression: Kp
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  real_T Integrator1_gainval;          /* Computed Parameter: Integrator1_gainval
                                        * Referenced by: '<S1>/Integrator1'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<S1>/Integrator1'
                                        */
  real_T DerivativeGain_Gain;          /* Expression: Kd
                                        * Referenced by: '<S1>/Derivative Gain'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S1>/Filter'
                                        */
  real_T Filter_IC;                    /* Expression: 0
                                        * Referenced by: '<S1>/Filter'
                                        */
  real_T FilterCoefficient_Gain;       /* Expression: N
                                        * Referenced by: '<S1>/Filter Coefficient'
                                        */
  real_T IntegralGain_Gain;            /* Expression: Ki
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
  real_T DataStoreMemory1_InitialValue[12];/* Expression: zeros(p,T)
                                            * Referenced by: '<S1>/Data Store Memory1'
                                            */
  real_T DataStoreMemory6_InitialValue[21];/* Expression: zeros(n*(T-1),1)
                                            * Referenced by: '<S1>/Data Store Memory6'
                                            */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S1>/Delay1'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_PID_d {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_PID_d PID_d_P;

/* Block states (auto storage) */
extern D_Work_PID_d PID_d_DWork;

/* External inputs (root inport signals with auto storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern ExternalInputs_PID_d PID_d_U;

#ifdef __cplusplus

}
#endif

/* External outputs (root outports fed by signals with auto storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern ExternalOutputs_PID_d PID_d_Y;

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void PID_d_initialize(void);
  extern void PID_d_step(void);
  extern void PID_d_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct RT_MODEL_PID_d *const PID_d_M;

#ifdef __cplusplus

}
#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('LandSharkModel_rev5_3outs_DT_fnctCommented/PID_d')    - opens subsystem LandSharkModel_rev5_3outs_DT_fnctCommented/PID_d
 * hilite_system('LandSharkModel_rev5_3outs_DT_fnctCommented/PID_d/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LandSharkModel_rev5_3outs_DT_fnctCommented'
 * '<S1>'   : 'LandSharkModel_rev5_3outs_DT_fnctCommented/PID_d'
 * '<S2>'   : 'LandSharkModel_rev5_3outs_DT_fnctCommented/PID_d/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_PID_d_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
