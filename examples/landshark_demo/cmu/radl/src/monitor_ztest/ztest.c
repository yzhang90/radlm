#include <stdlib.h>
#include <stdio.h>
#include <math.h>
int ztest(double  *X, double  *states, int idx1, double accptDev1) {
  double s1, s2, u13, u14, u15, u16, u17, u18
    , u19, u20, u21, u22, u23, u24, u25, u26
    , u27, u28, u29, u34, u39, u44;
  u15 = (X[0] - states[(8 + ((idx1 + 224) % 256))]);
  u16 = (X[0] - states[1]);
  u13 = (states[0] + (u15*(((states[(8 + ((idx1 + 224) % 256))] - states[1]) + u16) - (u15 / 32))));
  u14 = (states[1] + (u15 / 32));
  u19 = (X[0] - states[(8 + ((idx1 + 192) % 256))]);
  u20 = (X[0] - states[3]);
  u17 = (states[2] + (u19*(((states[(8 + ((idx1 + 192) % 256))] - states[3]) + u20) - (u19 / 64))));
  u18 = (states[3] + (u19 / 64));
  u23 = (X[0] - states[(8 + ((idx1 + 128) % 256))]);
  u24 = (X[0] - states[5]);
  u21 = (states[4] + (u23*(((states[(8 + ((idx1 + 128) % 256))] - states[5]) + u24) - (u23 / 128))));
  u22 = (states[5] + (u23 / 128));
  u27 = (X[0] - states[(8 + (idx1 % 256))]);
  u28 = (X[0] - states[7]);
  u25 = (states[6] + (u27*(((states[(8 + (idx1 % 256))] - states[7]) + u28) - (u27 / 256))));
  u26 = (states[7] + (u27 / 256));
  s2 = X[0];
  states[(8 + (idx1 % 256))] = s2;
  /* skip */
  states[0] = u13;
  states[1] = u14;
  /* skip */
  u29 = (((abs(u14) - accptDev1)*(abs(u14) - accptDev1)) / (u13 / 31));
  states[2] = u17;
  states[3] = u18;
  /* skip */
  u34 = (((abs(u18) - accptDev1)*(abs(u18) - accptDev1)) / (u17 / 63));
  states[4] = u21;
  states[5] = u22;
  /* skip */
  u39 = (((abs(u22) - accptDev1)*(abs(u22) - accptDev1)) / (u21 / 127));
  states[6] = u25;
  states[7] = u26;
  /* skip */
  u44 = (((abs(u26) - accptDev1)*(abs(u26) - accptDev1)) / (u25 / 255));
  s1 = 0;
  s1 = ((((u29 >= s1))) ? (u29) : (s1));
  s1 = ((((u34 >= s1))) ? (u34) : (s1));
  s1 = ((((u39 >= s1))) ? (u39) : (s1));
  s1 = ((((u44 >= s1))) ? (u44) : (s1));
  int w1;
  w1 = ((s1 >= 2.7060249999999999));
  return w1;
}

/*int main(){*/
/*  double states[256+4*2];*/
/*  double err;*/

/*  int i, test;*/

/*  for (i = 0; i != 256+4*2; ++i)*/
/*    states[i] = 0;*/

/*  for (i = 0; i != 300; ++i){*/
/*    err = ((double)random()/(RAND_MAX/2.0))-1;*/
/*    test = ztest(&err, states, i % 256, 0.1);*/
/*    printf("%d: %d\n", i, test);*/
/*  }*/
/*  printf("\n\n");*/
/*  for (i = 301; i != 510; ++i){*/
/*    err = ((double)random()/(RAND_MAX/2.0))+5;*/
/*    test = ztest(&err, states, i % 256, 0.1);*/
/*    printf("%d: %d\n", i, test);*/
/*  }*/
/*  */
/*  return 0;*/
/*}*/
