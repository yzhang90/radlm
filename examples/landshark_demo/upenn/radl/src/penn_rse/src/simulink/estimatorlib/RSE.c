/*
 * File: RSE.c
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

#include "RSE.h"
#include "RSE_private.h"

#include "solver.h" // this line added manually

/* Block states (auto storage) */
DW_RSE_T RSE_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_RSE_T RSE_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_RSE_T RSE_Y;

/* Real-time model */
RT_MODEL_RSE_T RSE_M_;
RT_MODEL_RSE_T *const RSE_M = &RSE_M_;

/* Model step function */
void RSE_step(void)
{
  real_T B_u3[7];
  real_T CAs_0_B_u1[3];
  real_T CAs_1_B_u2[3];
  Params params;
  int32_T i;
  int32_T i_0;
  real_T y_til_1_idx_0;
  real_T y_til_1_idx_1;
  real_T y_til_1_idx_2;
  real_T y_til_2_idx_0;
  real_T y_til_2_idx_1;
  real_T y_til_2_idx_2;
  real_T y_til_3_idx_0;
  real_T y_til_3_idx_1;
  real_T y_til_3_idx_2;

  /* Outputs for Atomic SubSystem: '<Root>/RSE' */
  /* MATLAB Function: '<S1>/RSE_Jun' incorporates:
   *  Inport: '<Root>/u'
   *  Inport: '<Root>/y'
   */
  /* MATLAB Function 'RSE/RSE_Jun': '<S2>:1' */
  /*  A:7x7, B:7x2, C:3x7 */
  /* '<S2>:1:7' coder.extrinsic('csolve_mine'); */
  /* '<S2>:1:8' coder.extrinsic('struc'); */
  /*  Assign y4,u3 */
  /* '<S2>:1:11' y4 = y; */
  /* '<S2>:1:12' B_u3 = B*u; */
  for (i = 0; i < 7; i++) {
    B_u3[i] = RSE_P.B[i + 7] * RSE_U.u[1] + RSE_P.B[i] * RSE_U.u[0];
  }

  /* '<S2>:1:14' CAs_0_B_u1 = CAs_0*B_u1; */
  for (i = 0; i < 3; i++) {
    CAs_0_B_u1[i] = 0.0;
    for (i_0 = 0; i_0 < 7; i_0++) {
      CAs_0_B_u1[i] += RSE_P.CAs_0[3 * i_0 + i] * RSE_DW.B_u1[i_0];
    }
  }

  /* '<S2>:1:15' CAs_1_B_u2 = CAs_1*B_u2; */
  for (i = 0; i < 3; i++) {
    CAs_1_B_u2[i] = 0.0;
    for (i_0 = 0; i_0 < 7; i_0++) {
      CAs_1_B_u2[i] += RSE_P.CAs_1[3 * i_0 + i] * RSE_DW.B_u2[i_0];
    }
  }

  /* '<S2>:1:16' CAs_2_B_u3 = CAs_2*B_u3; */
  /*  Compute y_til */
  /* '<S2>:1:19' y_til_1 = y1; */
  y_til_1_idx_0 = RSE_DW.y1[0];
  y_til_1_idx_1 = RSE_DW.y1[1];
  y_til_1_idx_2 = RSE_DW.y1[2];

  /* '<S2>:1:20' y_til_2 = y2 - (CAs_0_B_u1); */
  y_til_2_idx_0 = RSE_DW.y2[0] - CAs_0_B_u1[0];
  y_til_2_idx_1 = RSE_DW.y2[1] - CAs_0_B_u1[1];
  y_til_2_idx_2 = RSE_DW.y2[2] - CAs_0_B_u1[2];

  /* '<S2>:1:21' y_til_3 = y3 - (CAs_0_B_u1) - (CAs_1_B_u2); */
  y_til_3_idx_0 = (RSE_DW.y3[0] - CAs_0_B_u1[0]) - CAs_1_B_u2[0];
  y_til_3_idx_1 = (RSE_DW.y3[1] - CAs_0_B_u1[1]) - CAs_1_B_u2[1];
  y_til_3_idx_2 = (RSE_DW.y3[2] - CAs_0_B_u1[2]) - CAs_1_B_u2[2];

  /* '<S2>:1:22' y_til_4 = y4 - (CAs_0_B_u1) - (CAs_1_B_u2) - (CAs_2_B_u3); */
  /*  shift values */
  /* '<S2>:1:25' y1 = y2; */
  RSE_DW.y1[0] = RSE_DW.y2[0];
  RSE_DW.y1[1] = RSE_DW.y2[1];
  RSE_DW.y1[2] = RSE_DW.y2[2];

  /* '<S2>:1:26' y2 = y3; */
  RSE_DW.y2[0] = RSE_DW.y3[0];
  RSE_DW.y2[1] = RSE_DW.y3[1];
  RSE_DW.y2[2] = RSE_DW.y3[2];

  /* '<S2>:1:27' y3 = y4; */
  RSE_DW.y3[0] = RSE_U.y[0];
  RSE_DW.y3[1] = RSE_U.y[1];
  RSE_DW.y3[2] = RSE_U.y[2];

  /* '<S2>:1:29' B_u1 = B_u2; */
  /* '<S2>:1:30' B_u2 = B_u3; */
  for (i = 0; i < 7; i++) {
    RSE_DW.B_u1[i] = RSE_DW.B_u2[i];
    RSE_DW.B_u2[i] = B_u3[i];
  }

  /*  Set CVX solver input */
  /* '<S2>:1:33' params.CAs_0 = CAs_0; */
  /* '<S2>:1:34' params.CAs_1 = CAs_1; */
  /* '<S2>:1:35' params.CAs_2 = CAs_2; */
  /* '<S2>:1:36' params.CAs_3 = CAs_3; */
  for (i = 0; i < 21; i++) {
    params.CAs_0[i] = RSE_P.CAs_0[i];
    params.CAs_1[i] = RSE_P.CAs_1[i];
    params.CAs_2[i] = RSE_P.CAs_2[i];
    params.CAs_3[i] = RSE_P.CAs_3[i];
  }

  /* '<S2>:1:38' params.y_0 = y_til_1; */
  params.y_0[0] = y_til_1_idx_0;
  params.y_0[1] = y_til_1_idx_1;
  params.y_0[2] = y_til_1_idx_2;

  /* '<S2>:1:39' params.y_1 = y_til_2; */
  params.y_1[0] = y_til_2_idx_0;
  params.y_1[1] = y_til_2_idx_1;
  params.y_1[2] = y_til_2_idx_2;

  /* '<S2>:1:40' params.y_2 = y_til_3; */
  params.y_2[0] = y_til_3_idx_0;
  params.y_2[1] = y_til_3_idx_1;
  params.y_2[2] = y_til_3_idx_2;

  /* '<S2>:1:41' params.y_3 = y_til_4; */
  for (i = 0; i < 3; i++) {
    y_til_1_idx_0 = 0.0;
    for (i_0 = 0; i_0 < 7; i_0++) {
      y_til_1_idx_0 += RSE_P.CAs_2[3 * i_0 + i] * B_u3[i_0];
    }

    params.y_3[i] = ((RSE_U.y[i] - CAs_0_B_u1[i]) - CAs_1_B_u2[i]) -
      y_til_1_idx_0;
  }

  /*  output */
  /* '<S2>:1:44' state_x = zeros(7,1); */
  /*  n = 7 */
  /* '<S2>:1:45' yout = zeros(3,1); */
  /*  p = 3 */
  /* '<S2>:1:47' if coder.target('Sfun') */
  /* '<S2>:1:50' else */
  /*  if target is code generation */
  /* '<S2>:1:52' coder.cstructname(params,'Params', 'extern'); */
  /* coder.cinclude('solver.h'); */
  /* '<S2>:1:54' coder.ceval('set_defaults'); */
  set_defaults();

  /* '<S2>:1:55' coder.ceval('setup_indexing'); */
  setup_indexing();

  /* '<S2>:1:56' coder.ceval('load_default_data'); */
  load_default_data();

  /* '<S2>:1:57' coder.ceval('solveP',params,coder.wref(state_x)); */
  solveP(params, B_u3);

  /* Outport: '<Root>/v_estim' incorporates:
   *  MATLAB Function: '<S1>/RSE_Jun'
   */
  /*  state_x = argmin_x (norm( Ytil - Phi(x) )) */
  /*    where Phi(x) = [C*x, ..., C*A^(T-1)*x] */
  /* '<S2>:1:62' v_estim = state_x(1); */
  RSE_Y.v_estim = B_u3[0];

  /* Outport: '<Root>/yout' incorporates:
   *  MATLAB Function: '<S1>/RSE_Jun'
   */
  RSE_Y.yout[0] = 0.0;
  RSE_Y.yout[1] = 0.0;
  RSE_Y.yout[2] = 0.0;

  /* End of Outputs for SubSystem: '<Root>/RSE' */
}

/* Model initialize function */
void RSE_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(RSE_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&RSE_DW, 0,
                sizeof(DW_RSE_T));

  /* external inputs */
  (void) memset((void *)&RSE_U, 0,
                sizeof(ExtU_RSE_T));

  /* external outputs */
  (void) memset((void *)&RSE_Y, 0,
                sizeof(ExtY_RSE_T));

  {
    int32_T i;

    /* Start for DataStoreMemory: '<Root>/_DataStoreBlk_1' */
    RSE_DW.y1[0] = RSE_P._DataStoreBlk_1_InitialValue[0];
    RSE_DW.y1[1] = RSE_P._DataStoreBlk_1_InitialValue[1];
    RSE_DW.y1[2] = RSE_P._DataStoreBlk_1_InitialValue[2];

    /* Start for DataStoreMemory: '<Root>/_DataStoreBlk_2' */
    RSE_DW.y2[0] = RSE_P._DataStoreBlk_2_InitialValue[0];
    RSE_DW.y2[1] = RSE_P._DataStoreBlk_2_InitialValue[1];
    RSE_DW.y2[2] = RSE_P._DataStoreBlk_2_InitialValue[2];

    /* Start for DataStoreMemory: '<Root>/_DataStoreBlk_3' */
    RSE_DW.y3[0] = RSE_P._DataStoreBlk_3_InitialValue[0];
    RSE_DW.y3[1] = RSE_P._DataStoreBlk_3_InitialValue[1];
    RSE_DW.y3[2] = RSE_P._DataStoreBlk_3_InitialValue[2];
    for (i = 0; i < 7; i++) {
      /* Start for DataStoreMemory: '<Root>/_DataStoreBlk_4' */
      RSE_DW.B_u1[i] = RSE_P._DataStoreBlk_4_InitialValue[i];

      /* Start for DataStoreMemory: '<Root>/_DataStoreBlk_5' */
      RSE_DW.B_u2[i] = RSE_P._DataStoreBlk_5_InitialValue[i];
    }
  }
}

/* Model terminate function */
void RSE_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
