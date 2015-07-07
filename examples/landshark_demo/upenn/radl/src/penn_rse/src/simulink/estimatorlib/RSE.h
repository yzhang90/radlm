/*
 * File: RSE.h
 *
 * Code generated for Simulink model 'RSE'.
 *
 * Model version                  : 1.159
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Wed Jan 28 16:08:34 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_RSE_h_
#define RTW_HEADER_RSE_h_
#include <stddef.h>
#include <string.h>
#ifndef RSE_COMMON_INCLUDES_
# define RSE_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* RSE_COMMON_INCLUDES_ */

#include "RSE_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T y1[3];                        /* '<Root>/_DataStoreBlk_1' */
  real_T y2[3];                        /* '<Root>/_DataStoreBlk_2' */
  real_T y3[3];                        /* '<Root>/_DataStoreBlk_3' */
  real_T B_u1[7];                      /* '<Root>/_DataStoreBlk_4' */
  real_T B_u2[7];                      /* '<Root>/_DataStoreBlk_5' */
} DW_RSE_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T y[3];                         /* '<Root>/y' */
  real_T u[2];                         /* '<Root>/u' */
} ExtU_RSE_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T v_estim;                      /* '<Root>/v_estim' */
  real_T yout[3];                      /* '<Root>/yout' */
} ExtY_RSE_T;

/* Parameters (auto storage) */
struct P_RSE_T_ {
  real_T B[14];                        /* Variable: B
                                        * Referenced by: '<S1>/RSE_Jun'
                                        */
  real_T CAs_0[21];                    /* Variable: CAs_0
                                        * Referenced by: '<S1>/RSE_Jun'
                                        */
  real_T CAs_1[21];                    /* Variable: CAs_1
                                        * Referenced by: '<S1>/RSE_Jun'
                                        */
  real_T CAs_2[21];                    /* Variable: CAs_2
                                        * Referenced by: '<S1>/RSE_Jun'
                                        */
  real_T CAs_3[21];                    /* Variable: CAs_3
                                        * Referenced by: '<S1>/RSE_Jun'
                                        */
  real_T _DataStoreBlk_1_InitialValue[3];/* Expression: zeros(p,1)
                                          * Referenced by: '<Root>/_DataStoreBlk_1'
                                          */
  real_T _DataStoreBlk_2_InitialValue[3];/* Expression: zeros(p,1)
                                          * Referenced by: '<Root>/_DataStoreBlk_2'
                                          */
  real_T _DataStoreBlk_3_InitialValue[3];/* Expression: zeros(p,1)
                                          * Referenced by: '<Root>/_DataStoreBlk_3'
                                          */
  real_T _DataStoreBlk_4_InitialValue[7];/* Expression: zeros(n,1)
                                          * Referenced by: '<Root>/_DataStoreBlk_4'
                                          */
  real_T _DataStoreBlk_5_InitialValue[7];/* Expression: zeros(n,1)
                                          * Referenced by: '<Root>/_DataStoreBlk_5'
                                          */
};

/* Real-time Model Data Structure */
struct tag_RTM_RSE_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_RSE_T RSE_P;

/* Block states (auto storage) */
extern DW_RSE_T RSE_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_RSE_T RSE_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_RSE_T RSE_Y;

/* Model entry point functions */
extern void RSE_initialize(void);
extern void RSE_step(void);
extern void RSE_terminate(void);

/* Real-time Model object */
extern RT_MODEL_RSE_T *const RSE_M;

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
 * hilite_system('LandSharkModel_rev5_3outs_DT/PID_Jun/RSE')    - opens subsystem LandSharkModel_rev5_3outs_DT/PID_Jun/RSE
 * hilite_system('LandSharkModel_rev5_3outs_DT/PID_Jun/RSE/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LandSharkModel_rev5_3outs_DT/PID_Jun'
 * '<S1>'   : 'LandSharkModel_rev5_3outs_DT/PID_Jun/RSE'
 * '<S2>'   : 'LandSharkModel_rev5_3outs_DT/PID_Jun/RSE/RSE_Jun'
 */

/*-
 * Requirements for '<Root>': RSE
 */
#endif                                 /* RTW_HEADER_RSE_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
