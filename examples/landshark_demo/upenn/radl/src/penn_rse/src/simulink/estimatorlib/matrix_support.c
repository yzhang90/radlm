/* Produced by CVXGEN, 2013-01-15 16:44:11 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
  lhs[16] = 0;
  lhs[17] = 0;
  lhs[18] = 0;
  lhs[19] = 0;
  lhs[20] = 0;
  lhs[21] = 0;
  lhs[22] = 0;
  lhs[23] = 0;
  lhs[24] = 0;
  lhs[25] = 0;
  lhs[26] = 0;
  lhs[27] = 0;
  lhs[28] = 0;
  lhs[29] = 0;
  lhs[30] = 0;
  lhs[31] = 0;
  lhs[32] = 0;
  lhs[33] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[3]*(-1)-rhs[15]*(1);
  lhs[1] = -rhs[4]*(-1)-rhs[16]*(1);
  lhs[2] = -rhs[5]*(-1)-rhs[17]*(1);
  lhs[3] = -rhs[15]*(-1)-rhs[27]*(-params.CAs_0[0])-rhs[28]*(-params.CAs_0[3])-rhs[29]*(-params.CAs_0[6])-rhs[30]*(-params.CAs_0[9])-rhs[31]*(-params.CAs_0[12])-rhs[32]*(-params.CAs_0[15])-rhs[33]*(-params.CAs_0[18]);
  lhs[4] = -rhs[16]*(-1)-rhs[27]*(-params.CAs_0[1])-rhs[28]*(-params.CAs_0[4])-rhs[29]*(-params.CAs_0[7])-rhs[30]*(-params.CAs_0[10])-rhs[31]*(-params.CAs_0[13])-rhs[32]*(-params.CAs_0[16])-rhs[33]*(-params.CAs_0[19]);
  lhs[5] = -rhs[17]*(-1)-rhs[27]*(-params.CAs_0[2])-rhs[28]*(-params.CAs_0[5])-rhs[29]*(-params.CAs_0[8])-rhs[30]*(-params.CAs_0[11])-rhs[31]*(-params.CAs_0[14])-rhs[32]*(-params.CAs_0[17])-rhs[33]*(-params.CAs_0[20]);
  lhs[6] = -rhs[15]*(-1)-rhs[27]*(params.CAs_0[0])-rhs[28]*(params.CAs_0[3])-rhs[29]*(params.CAs_0[6])-rhs[30]*(params.CAs_0[9])-rhs[31]*(params.CAs_0[12])-rhs[32]*(params.CAs_0[15])-rhs[33]*(params.CAs_0[18]);
  lhs[7] = -rhs[16]*(-1)-rhs[27]*(params.CAs_0[1])-rhs[28]*(params.CAs_0[4])-rhs[29]*(params.CAs_0[7])-rhs[30]*(params.CAs_0[10])-rhs[31]*(params.CAs_0[13])-rhs[32]*(params.CAs_0[16])-rhs[33]*(params.CAs_0[19]);
  lhs[8] = -rhs[17]*(-1)-rhs[27]*(params.CAs_0[2])-rhs[28]*(params.CAs_0[5])-rhs[29]*(params.CAs_0[8])-rhs[30]*(params.CAs_0[11])-rhs[31]*(params.CAs_0[14])-rhs[32]*(params.CAs_0[17])-rhs[33]*(params.CAs_0[20]);
  lhs[9] = -rhs[6]*(-1)-rhs[18]*(1);
  lhs[10] = -rhs[7]*(-1)-rhs[19]*(1);
  lhs[11] = -rhs[8]*(-1)-rhs[20]*(1);
  lhs[12] = -rhs[18]*(-1)-rhs[27]*(-params.CAs_1[0])-rhs[28]*(-params.CAs_1[3])-rhs[29]*(-params.CAs_1[6])-rhs[30]*(-params.CAs_1[9])-rhs[31]*(-params.CAs_1[12])-rhs[32]*(-params.CAs_1[15])-rhs[33]*(-params.CAs_1[18]);
  lhs[13] = -rhs[19]*(-1)-rhs[27]*(-params.CAs_1[1])-rhs[28]*(-params.CAs_1[4])-rhs[29]*(-params.CAs_1[7])-rhs[30]*(-params.CAs_1[10])-rhs[31]*(-params.CAs_1[13])-rhs[32]*(-params.CAs_1[16])-rhs[33]*(-params.CAs_1[19]);
  lhs[14] = -rhs[20]*(-1)-rhs[27]*(-params.CAs_1[2])-rhs[28]*(-params.CAs_1[5])-rhs[29]*(-params.CAs_1[8])-rhs[30]*(-params.CAs_1[11])-rhs[31]*(-params.CAs_1[14])-rhs[32]*(-params.CAs_1[17])-rhs[33]*(-params.CAs_1[20]);
  lhs[15] = -rhs[18]*(-1)-rhs[27]*(params.CAs_1[0])-rhs[28]*(params.CAs_1[3])-rhs[29]*(params.CAs_1[6])-rhs[30]*(params.CAs_1[9])-rhs[31]*(params.CAs_1[12])-rhs[32]*(params.CAs_1[15])-rhs[33]*(params.CAs_1[18]);
  lhs[16] = -rhs[19]*(-1)-rhs[27]*(params.CAs_1[1])-rhs[28]*(params.CAs_1[4])-rhs[29]*(params.CAs_1[7])-rhs[30]*(params.CAs_1[10])-rhs[31]*(params.CAs_1[13])-rhs[32]*(params.CAs_1[16])-rhs[33]*(params.CAs_1[19]);
  lhs[17] = -rhs[20]*(-1)-rhs[27]*(params.CAs_1[2])-rhs[28]*(params.CAs_1[5])-rhs[29]*(params.CAs_1[8])-rhs[30]*(params.CAs_1[11])-rhs[31]*(params.CAs_1[14])-rhs[32]*(params.CAs_1[17])-rhs[33]*(params.CAs_1[20]);
  lhs[18] = -rhs[9]*(-1)-rhs[21]*(1);
  lhs[19] = -rhs[10]*(-1)-rhs[22]*(1);
  lhs[20] = -rhs[11]*(-1)-rhs[23]*(1);
  lhs[21] = -rhs[21]*(-1)-rhs[27]*(-params.CAs_2[0])-rhs[28]*(-params.CAs_2[3])-rhs[29]*(-params.CAs_2[6])-rhs[30]*(-params.CAs_2[9])-rhs[31]*(-params.CAs_2[12])-rhs[32]*(-params.CAs_2[15])-rhs[33]*(-params.CAs_2[18]);
  lhs[22] = -rhs[22]*(-1)-rhs[27]*(-params.CAs_2[1])-rhs[28]*(-params.CAs_2[4])-rhs[29]*(-params.CAs_2[7])-rhs[30]*(-params.CAs_2[10])-rhs[31]*(-params.CAs_2[13])-rhs[32]*(-params.CAs_2[16])-rhs[33]*(-params.CAs_2[19]);
  lhs[23] = -rhs[23]*(-1)-rhs[27]*(-params.CAs_2[2])-rhs[28]*(-params.CAs_2[5])-rhs[29]*(-params.CAs_2[8])-rhs[30]*(-params.CAs_2[11])-rhs[31]*(-params.CAs_2[14])-rhs[32]*(-params.CAs_2[17])-rhs[33]*(-params.CAs_2[20]);
  lhs[24] = -rhs[21]*(-1)-rhs[27]*(params.CAs_2[0])-rhs[28]*(params.CAs_2[3])-rhs[29]*(params.CAs_2[6])-rhs[30]*(params.CAs_2[9])-rhs[31]*(params.CAs_2[12])-rhs[32]*(params.CAs_2[15])-rhs[33]*(params.CAs_2[18]);
  lhs[25] = -rhs[22]*(-1)-rhs[27]*(params.CAs_2[1])-rhs[28]*(params.CAs_2[4])-rhs[29]*(params.CAs_2[7])-rhs[30]*(params.CAs_2[10])-rhs[31]*(params.CAs_2[13])-rhs[32]*(params.CAs_2[16])-rhs[33]*(params.CAs_2[19]);
  lhs[26] = -rhs[23]*(-1)-rhs[27]*(params.CAs_2[2])-rhs[28]*(params.CAs_2[5])-rhs[29]*(params.CAs_2[8])-rhs[30]*(params.CAs_2[11])-rhs[31]*(params.CAs_2[14])-rhs[32]*(params.CAs_2[17])-rhs[33]*(params.CAs_2[20]);
  lhs[27] = -rhs[12]*(-1)-rhs[24]*(1);
  lhs[28] = -rhs[13]*(-1)-rhs[25]*(1);
  lhs[29] = -rhs[14]*(-1)-rhs[26]*(1);
  lhs[30] = -rhs[24]*(-1)-rhs[27]*(-params.CAs_3[0])-rhs[28]*(-params.CAs_3[3])-rhs[29]*(-params.CAs_3[6])-rhs[30]*(-params.CAs_3[9])-rhs[31]*(-params.CAs_3[12])-rhs[32]*(-params.CAs_3[15])-rhs[33]*(-params.CAs_3[18]);
  lhs[31] = -rhs[25]*(-1)-rhs[27]*(-params.CAs_3[1])-rhs[28]*(-params.CAs_3[4])-rhs[29]*(-params.CAs_3[7])-rhs[30]*(-params.CAs_3[10])-rhs[31]*(-params.CAs_3[13])-rhs[32]*(-params.CAs_3[16])-rhs[33]*(-params.CAs_3[19]);
  lhs[32] = -rhs[26]*(-1)-rhs[27]*(-params.CAs_3[2])-rhs[28]*(-params.CAs_3[5])-rhs[29]*(-params.CAs_3[8])-rhs[30]*(-params.CAs_3[11])-rhs[31]*(-params.CAs_3[14])-rhs[32]*(-params.CAs_3[17])-rhs[33]*(-params.CAs_3[20]);
  lhs[33] = -rhs[24]*(-1)-rhs[27]*(params.CAs_3[0])-rhs[28]*(params.CAs_3[3])-rhs[29]*(params.CAs_3[6])-rhs[30]*(params.CAs_3[9])-rhs[31]*(params.CAs_3[12])-rhs[32]*(params.CAs_3[15])-rhs[33]*(params.CAs_3[18]);
  lhs[34] = -rhs[25]*(-1)-rhs[27]*(params.CAs_3[1])-rhs[28]*(params.CAs_3[4])-rhs[29]*(params.CAs_3[7])-rhs[30]*(params.CAs_3[10])-rhs[31]*(params.CAs_3[13])-rhs[32]*(params.CAs_3[16])-rhs[33]*(params.CAs_3[19]);
  lhs[35] = -rhs[26]*(-1)-rhs[27]*(params.CAs_3[2])-rhs[28]*(params.CAs_3[5])-rhs[29]*(params.CAs_3[8])-rhs[30]*(params.CAs_3[11])-rhs[31]*(params.CAs_3[14])-rhs[32]*(params.CAs_3[17])-rhs[33]*(params.CAs_3[20]);
  lhs[36] = -rhs[0]*(-1)-rhs[3]*(1)-rhs[6]*(1)-rhs[9]*(1)-rhs[12]*(1);
  lhs[37] = -rhs[1]*(-1)-rhs[4]*(1)-rhs[7]*(1)-rhs[10]*(1)-rhs[13]*(1);
  lhs[38] = -rhs[2]*(-1)-rhs[5]*(1)-rhs[8]*(1)-rhs[11]*(1)-rhs[14]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[36]*(-1);
  lhs[1] = -rhs[37]*(-1);
  lhs[2] = -rhs[38]*(-1);
  lhs[3] = -rhs[0]*(-1)-rhs[36]*(1);
  lhs[4] = -rhs[1]*(-1)-rhs[37]*(1);
  lhs[5] = -rhs[2]*(-1)-rhs[38]*(1);
  lhs[6] = -rhs[9]*(-1)-rhs[36]*(1);
  lhs[7] = -rhs[10]*(-1)-rhs[37]*(1);
  lhs[8] = -rhs[11]*(-1)-rhs[38]*(1);
  lhs[9] = -rhs[18]*(-1)-rhs[36]*(1);
  lhs[10] = -rhs[19]*(-1)-rhs[37]*(1);
  lhs[11] = -rhs[20]*(-1)-rhs[38]*(1);
  lhs[12] = -rhs[27]*(-1)-rhs[36]*(1);
  lhs[13] = -rhs[28]*(-1)-rhs[37]*(1);
  lhs[14] = -rhs[29]*(-1)-rhs[38]*(1);
  lhs[15] = -rhs[0]*(1)-rhs[3]*(-1)-rhs[6]*(-1);
  lhs[16] = -rhs[1]*(1)-rhs[4]*(-1)-rhs[7]*(-1);
  lhs[17] = -rhs[2]*(1)-rhs[5]*(-1)-rhs[8]*(-1);
  lhs[18] = -rhs[9]*(1)-rhs[12]*(-1)-rhs[15]*(-1);
  lhs[19] = -rhs[10]*(1)-rhs[13]*(-1)-rhs[16]*(-1);
  lhs[20] = -rhs[11]*(1)-rhs[14]*(-1)-rhs[17]*(-1);
  lhs[21] = -rhs[18]*(1)-rhs[21]*(-1)-rhs[24]*(-1);
  lhs[22] = -rhs[19]*(1)-rhs[22]*(-1)-rhs[25]*(-1);
  lhs[23] = -rhs[20]*(1)-rhs[23]*(-1)-rhs[26]*(-1);
  lhs[24] = -rhs[27]*(1)-rhs[30]*(-1)-rhs[33]*(-1);
  lhs[25] = -rhs[28]*(1)-rhs[31]*(-1)-rhs[34]*(-1);
  lhs[26] = -rhs[29]*(1)-rhs[32]*(-1)-rhs[35]*(-1);
  lhs[27] = -rhs[3]*(-params.CAs_0[0])-rhs[4]*(-params.CAs_0[1])-rhs[5]*(-params.CAs_0[2])-rhs[6]*(params.CAs_0[0])-rhs[7]*(params.CAs_0[1])-rhs[8]*(params.CAs_0[2])-rhs[12]*(-params.CAs_1[0])-rhs[13]*(-params.CAs_1[1])-rhs[14]*(-params.CAs_1[2])-rhs[15]*(params.CAs_1[0])-rhs[16]*(params.CAs_1[1])-rhs[17]*(params.CAs_1[2])-rhs[21]*(-params.CAs_2[0])-rhs[22]*(-params.CAs_2[1])-rhs[23]*(-params.CAs_2[2])-rhs[24]*(params.CAs_2[0])-rhs[25]*(params.CAs_2[1])-rhs[26]*(params.CAs_2[2])-rhs[30]*(-params.CAs_3[0])-rhs[31]*(-params.CAs_3[1])-rhs[32]*(-params.CAs_3[2])-rhs[33]*(params.CAs_3[0])-rhs[34]*(params.CAs_3[1])-rhs[35]*(params.CAs_3[2]);
  lhs[28] = -rhs[3]*(-params.CAs_0[3])-rhs[4]*(-params.CAs_0[4])-rhs[5]*(-params.CAs_0[5])-rhs[6]*(params.CAs_0[3])-rhs[7]*(params.CAs_0[4])-rhs[8]*(params.CAs_0[5])-rhs[12]*(-params.CAs_1[3])-rhs[13]*(-params.CAs_1[4])-rhs[14]*(-params.CAs_1[5])-rhs[15]*(params.CAs_1[3])-rhs[16]*(params.CAs_1[4])-rhs[17]*(params.CAs_1[5])-rhs[21]*(-params.CAs_2[3])-rhs[22]*(-params.CAs_2[4])-rhs[23]*(-params.CAs_2[5])-rhs[24]*(params.CAs_2[3])-rhs[25]*(params.CAs_2[4])-rhs[26]*(params.CAs_2[5])-rhs[30]*(-params.CAs_3[3])-rhs[31]*(-params.CAs_3[4])-rhs[32]*(-params.CAs_3[5])-rhs[33]*(params.CAs_3[3])-rhs[34]*(params.CAs_3[4])-rhs[35]*(params.CAs_3[5]);
  lhs[29] = -rhs[3]*(-params.CAs_0[6])-rhs[4]*(-params.CAs_0[7])-rhs[5]*(-params.CAs_0[8])-rhs[6]*(params.CAs_0[6])-rhs[7]*(params.CAs_0[7])-rhs[8]*(params.CAs_0[8])-rhs[12]*(-params.CAs_1[6])-rhs[13]*(-params.CAs_1[7])-rhs[14]*(-params.CAs_1[8])-rhs[15]*(params.CAs_1[6])-rhs[16]*(params.CAs_1[7])-rhs[17]*(params.CAs_1[8])-rhs[21]*(-params.CAs_2[6])-rhs[22]*(-params.CAs_2[7])-rhs[23]*(-params.CAs_2[8])-rhs[24]*(params.CAs_2[6])-rhs[25]*(params.CAs_2[7])-rhs[26]*(params.CAs_2[8])-rhs[30]*(-params.CAs_3[6])-rhs[31]*(-params.CAs_3[7])-rhs[32]*(-params.CAs_3[8])-rhs[33]*(params.CAs_3[6])-rhs[34]*(params.CAs_3[7])-rhs[35]*(params.CAs_3[8]);
  lhs[30] = -rhs[3]*(-params.CAs_0[9])-rhs[4]*(-params.CAs_0[10])-rhs[5]*(-params.CAs_0[11])-rhs[6]*(params.CAs_0[9])-rhs[7]*(params.CAs_0[10])-rhs[8]*(params.CAs_0[11])-rhs[12]*(-params.CAs_1[9])-rhs[13]*(-params.CAs_1[10])-rhs[14]*(-params.CAs_1[11])-rhs[15]*(params.CAs_1[9])-rhs[16]*(params.CAs_1[10])-rhs[17]*(params.CAs_1[11])-rhs[21]*(-params.CAs_2[9])-rhs[22]*(-params.CAs_2[10])-rhs[23]*(-params.CAs_2[11])-rhs[24]*(params.CAs_2[9])-rhs[25]*(params.CAs_2[10])-rhs[26]*(params.CAs_2[11])-rhs[30]*(-params.CAs_3[9])-rhs[31]*(-params.CAs_3[10])-rhs[32]*(-params.CAs_3[11])-rhs[33]*(params.CAs_3[9])-rhs[34]*(params.CAs_3[10])-rhs[35]*(params.CAs_3[11]);
  lhs[31] = -rhs[3]*(-params.CAs_0[12])-rhs[4]*(-params.CAs_0[13])-rhs[5]*(-params.CAs_0[14])-rhs[6]*(params.CAs_0[12])-rhs[7]*(params.CAs_0[13])-rhs[8]*(params.CAs_0[14])-rhs[12]*(-params.CAs_1[12])-rhs[13]*(-params.CAs_1[13])-rhs[14]*(-params.CAs_1[14])-rhs[15]*(params.CAs_1[12])-rhs[16]*(params.CAs_1[13])-rhs[17]*(params.CAs_1[14])-rhs[21]*(-params.CAs_2[12])-rhs[22]*(-params.CAs_2[13])-rhs[23]*(-params.CAs_2[14])-rhs[24]*(params.CAs_2[12])-rhs[25]*(params.CAs_2[13])-rhs[26]*(params.CAs_2[14])-rhs[30]*(-params.CAs_3[12])-rhs[31]*(-params.CAs_3[13])-rhs[32]*(-params.CAs_3[14])-rhs[33]*(params.CAs_3[12])-rhs[34]*(params.CAs_3[13])-rhs[35]*(params.CAs_3[14]);
  lhs[32] = -rhs[3]*(-params.CAs_0[15])-rhs[4]*(-params.CAs_0[16])-rhs[5]*(-params.CAs_0[17])-rhs[6]*(params.CAs_0[15])-rhs[7]*(params.CAs_0[16])-rhs[8]*(params.CAs_0[17])-rhs[12]*(-params.CAs_1[15])-rhs[13]*(-params.CAs_1[16])-rhs[14]*(-params.CAs_1[17])-rhs[15]*(params.CAs_1[15])-rhs[16]*(params.CAs_1[16])-rhs[17]*(params.CAs_1[17])-rhs[21]*(-params.CAs_2[15])-rhs[22]*(-params.CAs_2[16])-rhs[23]*(-params.CAs_2[17])-rhs[24]*(params.CAs_2[15])-rhs[25]*(params.CAs_2[16])-rhs[26]*(params.CAs_2[17])-rhs[30]*(-params.CAs_3[15])-rhs[31]*(-params.CAs_3[16])-rhs[32]*(-params.CAs_3[17])-rhs[33]*(params.CAs_3[15])-rhs[34]*(params.CAs_3[16])-rhs[35]*(params.CAs_3[17]);
  lhs[33] = -rhs[3]*(-params.CAs_0[18])-rhs[4]*(-params.CAs_0[19])-rhs[5]*(-params.CAs_0[20])-rhs[6]*(params.CAs_0[18])-rhs[7]*(params.CAs_0[19])-rhs[8]*(params.CAs_0[20])-rhs[12]*(-params.CAs_1[18])-rhs[13]*(-params.CAs_1[19])-rhs[14]*(-params.CAs_1[20])-rhs[15]*(params.CAs_1[18])-rhs[16]*(params.CAs_1[19])-rhs[17]*(params.CAs_1[20])-rhs[21]*(-params.CAs_2[18])-rhs[22]*(-params.CAs_2[19])-rhs[23]*(-params.CAs_2[20])-rhs[24]*(params.CAs_2[18])-rhs[25]*(params.CAs_2[19])-rhs[26]*(params.CAs_2[20])-rhs[30]*(-params.CAs_3[18])-rhs[31]*(-params.CAs_3[19])-rhs[32]*(-params.CAs_3[20])-rhs[33]*(params.CAs_3[18])-rhs[34]*(params.CAs_3[19])-rhs[35]*(params.CAs_3[20]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
  lhs[16] = 0;
  lhs[17] = 0;
  lhs[18] = 0;
  lhs[19] = 0;
  lhs[20] = 0;
  lhs[21] = 0;
  lhs[22] = 0;
  lhs[23] = 0;
  lhs[24] = 0;
  lhs[25] = 0;
  lhs[26] = 0;
  lhs[27] = 0;
  lhs[28] = 0;
  lhs[29] = 0;
  lhs[30] = 0;
  lhs[31] = 0;
  lhs[32] = 0;
  lhs[33] = 0;
}
void fillq(void) {
  work.q[0] = 1;
  work.q[1] = 1;
  work.q[2] = 1;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
  work.q[13] = 0;
  work.q[14] = 0;
  work.q[15] = 0;
  work.q[16] = 0;
  work.q[17] = 0;
  work.q[18] = 0;
  work.q[19] = 0;
  work.q[20] = 0;
  work.q[21] = 0;
  work.q[22] = 0;
  work.q[23] = 0;
  work.q[24] = 0;
  work.q[25] = 0;
  work.q[26] = 0;
  work.q[27] = 0;
  work.q[28] = 0;
  work.q[29] = 0;
  work.q[30] = 0;
  work.q[31] = 0;
  work.q[32] = 0;
  work.q[33] = 0;
}
void fillh(void) {
  work.h[0] = 0;
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = -params.y_0[0];
  work.h[4] = -params.y_0[1];
  work.h[5] = -params.y_0[2];
  work.h[6] = params.y_0[0];
  work.h[7] = params.y_0[1];
  work.h[8] = params.y_0[2];
  work.h[9] = 0;
  work.h[10] = 0;
  work.h[11] = 0;
  work.h[12] = -params.y_1[0];
  work.h[13] = -params.y_1[1];
  work.h[14] = -params.y_1[2];
  work.h[15] = params.y_1[0];
  work.h[16] = params.y_1[1];
  work.h[17] = params.y_1[2];
  work.h[18] = 0;
  work.h[19] = 0;
  work.h[20] = 0;
  work.h[21] = -params.y_2[0];
  work.h[22] = -params.y_2[1];
  work.h[23] = -params.y_2[2];
  work.h[24] = params.y_2[0];
  work.h[25] = params.y_2[1];
  work.h[26] = params.y_2[2];
  work.h[27] = 0;
  work.h[28] = 0;
  work.h[29] = 0;
  work.h[30] = -params.y_3[0];
  work.h[31] = -params.y_3[1];
  work.h[32] = -params.y_3[2];
  work.h[33] = params.y_3[0];
  work.h[34] = params.y_3[1];
  work.h[35] = params.y_3[2];
  work.h[36] = 0;
  work.h[37] = 0;
  work.h[38] = 0;
}
void fillb(void) {
}
void pre_ops(void) {
}
