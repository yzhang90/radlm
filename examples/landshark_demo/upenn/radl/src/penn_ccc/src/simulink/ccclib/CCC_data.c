/*
 * File: CCC_data.c
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

#include "CCC.h"
#include "CCC_private.h"

/* Block parameters (auto storage) */
P_CCC_T CCC_P = {
  1.0,                                 /* Variable: FilterCoeff
                                        * Referenced by: '<S1>/Filter Coefficient1'
                                        */
  0.0,                                 /* Variable: Kd
                                        * Referenced by: '<S1>/Derivative Gain1'
                                        */
  0.2,                                 /* Variable: Ki
                                        * Referenced by: '<S1>/Integral Gain1'
                                        */
  0.05,                                /* Variable: Kp
                                        * Referenced by: '<S1>/Proportional Gain1'
                                        */
  0.02,                                /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.02,                                /* Computed Parameter: Filter1_gainval
                                        * Referenced by: '<S1>/Filter1'
                                        */
  0.0                                  /* Expression: 0
                                        * Referenced by: '<S1>/Filter1'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
