/*
 * File: CCC.c
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

/* Block states (auto storage) */
DW_CCC_T CCC_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_CCC_T CCC_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_CCC_T CCC_Y;

/* Real-time model */
RT_MODEL_CCC_T CCC_M_;
RT_MODEL_CCC_T *const CCC_M = &CCC_M_;

/* Model step function */
void CCC_step(void)
{
  real_T rtb_Sum2;
  real_T rtb_FilterCoefficient1;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Inport: '<Root>/ref'
   *  Inport: '<Root>/x'
   */
  rtb_Sum2 = CCC_U.ref - CCC_U.v_estim;

  /* DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Inport: '<Root>/RESET'
   */
  if ((CCC_U.RESET > 0.0) && (CCC_DW.Integrator_PrevResetState <= 0)) {
    CCC_DW.Integrator_DSTATE = CCC_P.Integrator_IC;
  }

  /* DiscreteIntegrator: '<S1>/Filter1' incorporates:
   *  Inport: '<Root>/RESET'
   */
  if ((CCC_U.RESET > 0.0) && (CCC_DW.Filter1_PrevResetState <= 0)) {
    CCC_DW.Filter1_DSTATE = CCC_P.Filter1_IC;
  }

  /* Gain: '<S1>/Filter Coefficient1' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter1'
   *  Gain: '<S1>/Derivative Gain1'
   *  Sum: '<S1>/SumD1'
   */
  rtb_FilterCoefficient1 = (CCC_P.Kd * rtb_Sum2 - CCC_DW.Filter1_DSTATE) *
    CCC_P.FilterCoeff;

  /* Outport: '<Root>/ctrl_u' incorporates:
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Gain: '<S1>/Proportional Gain1'
   *  Sum: '<S1>/Sum1'
   */
  CCC_Y.ctrl_u = (CCC_P.Kp * rtb_Sum2 + CCC_DW.Integrator_DSTATE) +
    rtb_FilterCoefficient1;

  /* Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Gain: '<S1>/Integral Gain1'
   *  Update for Inport: '<Root>/RESET'
   */
  CCC_DW.Integrator_DSTATE += CCC_P.Ki * rtb_Sum2 * CCC_P.Integrator_gainval;
  if (CCC_U.RESET > 0.0) {
    CCC_DW.Integrator_PrevResetState = 1;
  } else if (CCC_U.RESET < 0.0) {
    CCC_DW.Integrator_PrevResetState = -1;
  } else if (CCC_U.RESET == 0.0) {
    CCC_DW.Integrator_PrevResetState = 0;
  } else {
    CCC_DW.Integrator_PrevResetState = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Integrator' */

  /* Update for DiscreteIntegrator: '<S1>/Filter1' incorporates:
   *  Update for Inport: '<Root>/RESET'
   */
  CCC_DW.Filter1_DSTATE += CCC_P.Filter1_gainval * rtb_FilterCoefficient1;
  if (CCC_U.RESET > 0.0) {
    CCC_DW.Filter1_PrevResetState = 1;
  } else if (CCC_U.RESET < 0.0) {
    CCC_DW.Filter1_PrevResetState = -1;
  } else if (CCC_U.RESET == 0.0) {
    CCC_DW.Filter1_PrevResetState = 0;
  } else {
    CCC_DW.Filter1_PrevResetState = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Filter1' */
}

/* Model initialize function */
void CCC_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(CCC_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&CCC_DW, 0,
                sizeof(DW_CCC_T));

  /* external inputs */
  (void) memset((void *)&CCC_U, 0,
                sizeof(ExtU_CCC_T));

  /* external outputs */
  CCC_Y.ctrl_u = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Integrator' */
  CCC_DW.Integrator_DSTATE = CCC_P.Integrator_IC;
  CCC_DW.Integrator_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Filter1' */
  CCC_DW.Filter1_DSTATE = CCC_P.Filter1_IC;
  CCC_DW.Filter1_PrevResetState = 2;
}

/* Model terminate function */
void CCC_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
