/* Produced by CVXGEN, 2013-01-15 16:44:11 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int hello(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.y_0[0] = 0.20319161029830202;
  params.y_0[1] = 0.8325912904724193;
  params.y_0[2] = -0.8363810443482227;
  params.CAs_0[0] = 0.04331042079065206;
  params.CAs_0[1] = 1.5717878173906188;
  params.CAs_0[2] = 1.5851723557337523;
  params.CAs_0[3] = -1.497658758144655;
  params.CAs_0[4] = -1.171028487447253;
  params.CAs_0[5] = -1.7941311867966805;
  params.CAs_0[6] = -0.23676062539745413;
  params.CAs_0[7] = -1.8804951564857322;
  params.CAs_0[8] = -0.17266710242115568;
  params.CAs_0[9] = 0.596576190459043;
  params.CAs_0[10] = -0.8860508694080989;
  params.CAs_0[11] = 0.7050196079205251;
  params.CAs_0[12] = 0.3634512696654033;
  params.CAs_0[13] = -1.9040724704913385;
  params.CAs_0[14] = 0.23541635196352795;
  params.CAs_0[15] = -0.9629902123701384;
  params.CAs_0[16] = -0.3395952119597214;
  params.CAs_0[17] = -0.865899672914725;
  params.CAs_0[18] = 0.7725516732519853;
  params.CAs_0[19] = -0.23818512931704205;
  params.CAs_0[20] = -1.372529046100147;
  params.y_1[0] = 0.17859607212737894;
  params.y_1[1] = 1.1212590580454682;
  params.y_1[2] = -0.774545870495281;
  params.CAs_1[0] = -1.1121684642712744;
  params.CAs_1[1] = -0.44811496977740495;
  params.CAs_1[2] = 1.7455345994417217;
  params.CAs_1[3] = 1.9039816898917352;
  params.CAs_1[4] = 0.6895347036512547;
  params.CAs_1[5] = 1.6113364341535923;
  params.CAs_1[6] = 1.383003485172717;
  params.CAs_1[7] = -0.48802383468444344;
  params.CAs_1[8] = -1.631131964513103;
  params.CAs_1[9] = 0.6136436100941447;
  params.CAs_1[10] = 0.2313630495538037;
  params.CAs_1[11] = -0.5537409477496875;
  params.CAs_1[12] = -1.0997819806406723;
  params.CAs_1[13] = -0.3739203344950055;
  params.CAs_1[14] = -0.12423900520332376;
  params.CAs_1[15] = -0.923057686995755;
  params.CAs_1[16] = -0.8328289030982696;
  params.CAs_1[17] = -0.16925440270808823;
  params.CAs_1[18] = 1.442135651787706;
  params.CAs_1[19] = 0.34501161787128565;
  params.CAs_1[20] = -0.8660485502711608;
  params.y_2[0] = -0.8880899735055947;
  params.y_2[1] = -0.1815116979122129;
  params.y_2[2] = -1.17835862158005;
  params.CAs_2[0] = -1.1944851558277074;
  params.CAs_2[1] = 0.05614023926976763;
  params.CAs_2[2] = -1.6510825248767813;
  params.CAs_2[3] = -0.06565787059365391;
  params.CAs_2[4] = -0.5512951504486665;
  params.CAs_2[5] = 0.8307464872626844;
  params.CAs_2[6] = 0.9869848924080182;
  params.CAs_2[7] = 0.7643716874230573;
  params.CAs_2[8] = 0.7567216550196565;
  params.CAs_2[9] = -0.5055995034042868;
  params.CAs_2[10] = 0.6725392189410702;
  params.CAs_2[11] = -0.6406053441727284;
  params.CAs_2[12] = 0.29117547947550015;
  params.CAs_2[13] = -0.6967713677405021;
  params.CAs_2[14] = -0.21941980294587182;
  params.CAs_2[15] = -1.753884276680243;
  params.CAs_2[16] = -1.0292983112626475;
  params.CAs_2[17] = 1.8864104246942706;
  params.CAs_2[18] = -1.077663182579704;
  params.CAs_2[19] = 0.7659100437893209;
  params.CAs_2[20] = 0.6019074328549583;
  params.y_3[0] = 0.8957565577499285;
  params.y_3[1] = -0.09964555746227477;
  params.y_3[2] = 0.38665509840745127;
  params.CAs_3[0] = -1.7321223042686946;
  params.CAs_3[1] = -1.7097514487110663;
  params.CAs_3[2] = -1.2040958948116867;
  params.CAs_3[3] = -1.3925560119658358;
  params.CAs_3[4] = -1.5995826216742213;
  params.CAs_3[5] = -1.4828245415645833;
  params.CAs_3[6] = 0.21311092723061398;
  params.CAs_3[7] = -1.248740700304487;
  params.CAs_3[8] = 1.808404972124833;
  params.CAs_3[9] = 0.7264471152297065;
  params.CAs_3[10] = 0.16407869343908477;
  params.CAs_3[11] = 0.8287224032315907;
  params.CAs_3[12] = -0.9444533161899464;
  params.CAs_3[13] = 1.7069027370149112;
  params.CAs_3[14] = 1.3567722311998827;
  params.CAs_3[15] = 0.9052779937121489;
  params.CAs_3[16] = -0.07904017565835986;
  params.CAs_3[17] = 1.3684127435065871;
  params.CAs_3[18] = 0.979009293697437;
  params.CAs_3[19] = 0.6413036255984501;
  params.CAs_3[20] = 1.6559010680237511;
}
