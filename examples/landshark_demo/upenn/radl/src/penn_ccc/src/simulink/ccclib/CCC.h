/*
 * File: CCC.h
 *
 * Code generated for Simulink model 'CCC'.
 *
 * Model version                  : 1.161
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Tue Mar  3 15:38:38 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_CCC_h_
#define RTW_HEADER_CCC_h_
#include <stddef.h>
#include <string.h>
#ifndef CCC_COMMON_INCLUDES_
# define CCC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* CCC_COMMON_INCLUDES_ */

#include "CCC_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
  real_T Filter1_DSTATE;               /* '<S1>/Filter1' */
  int8_T Integrator_PrevResetState;    /* '<S1>/Integrator' */
  int8_T Filter1_PrevResetState;       /* '<S1>/Filter1' */
} DW_CCC_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T ref;                          /* '<Root>/ref' */
  real_T v_estim;                      /* '<Root>/x' */
  real_T RESET;                        /* '<Root>/RESET' */
} ExtU_CCC_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T ctrl_u;                       /* '<Root>/ctrl_u' */
} ExtY_CCC_T;

/* Parameters (auto storage) */
struct P_CCC_T_ {
  real_T FilterCoeff;                  /* Variable: FilterCoeff
                                        * Referenced by: '<S1>/Filter Coefficient1'
                                        */
  real_T Kd;                           /* Variable: Kd
                                        * Referenced by: '<S1>/Derivative Gain1'
                                        */
  real_T Ki;                           /* Variable: Ki
                                        * Referenced by: '<S1>/Integral Gain1'
                                        */
  real_T Kp;                           /* Variable: Kp
                                        * Referenced by: '<S1>/Proportional Gain1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T Filter1_gainval;              /* Computed Parameter: Filter1_gainval
                                        * Referenced by: '<S1>/Filter1'
                                        */
  real_T Filter1_IC;                   /* Expression: 0
                                        * Referenced by: '<S1>/Filter1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_CCC_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_CCC_T CCC_P;

/* Block states (auto storage) */
extern DW_CCC_T CCC_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_CCC_T CCC_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_CCC_T CCC_Y;

/* Model entry point functions */
extern void CCC_initialize(void);
extern void CCC_step(void);
extern void CCC_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CCC_T *const CCC_M;

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
 * hilite_system('LandSharkModel_rev5_3outs_DT/PID_Jun/CCC')    - opens subsystem LandSharkModel_rev5_3outs_DT/PID_Jun/CCC
 * hilite_system('LandSharkModel_rev5_3outs_DT/PID_Jun/CCC/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LandSharkModel_rev5_3outs_DT/PID_Jun'
 * '<S1>'   : 'LandSharkModel_rev5_3outs_DT/PID_Jun/CCC'
 */

/*-
 * Requirements for '<Root>': CCC
 */
#endif                                 /* RTW_HEADER_CCC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
