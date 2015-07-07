#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

void fused(int  *Y, double  *X, double  *Vertex, double  *Norm, double  *BoundRadius, double  *state) {
  double s1;
  s1 = 1;
  for(int i24 = 0; i24 <= 7; i24++) {
    double s2;
    double s3;
    s3 = X[i24];    
    s2 = ((((s3 == s3))) && (((s3 >= -(FLT_MAX)))) && (((s3 <= FLT_MAX))));
    s1 = ((s1) && (s2));
  }
  Y[0] = s1;
  static double T4[12];
  static double T5[16];
  for(int i1 = 0; i1 <= 3; i1++) {
    for(int i17 = 0; i17 <= 3; i17++) {
      static double T7[2];
      double s4;
      for(int i48 = 0; i48 <= 1; i48++) {
	double s5;
	s5 = X[((2*i1) + i48)];
	T7[i48] = s5;
      }
      s4 = 0;
      for(int i47 = 0; i47 <= 1; i47++) {
	double s6;
	double s7;
	s7 = T7[i47];
	s6 = (Norm[((i17*2) + i47)]*(s7 + (Vertex[((i17*2) + i47)]*BoundRadius[i1])));
	s4 = (s4 + s6);
      }
      T5[((4*i1) + i17)] = s4;
    }
  }
  for(int i14 = 0; i14 <= 3; i14++) {
    double s8;
    s8 = 10000.0;
    for(int i44 = 0; i44 <= 1; i44++) {
      double s9;
      s9 = T5[((i44*4) + i14)];
      s8 = ((((s9 <= s8))) ? (s9) : (s8));
    }
    T4[i14] = s8;
  }
  for(int i15 = 0; i15 <= 3; i15++) {
    double s10;
    s10 = 10000.0;
    double s11;
    s11 = T5[(i15 + 8)];
    s10 = ((((s11 <= s10))) ? (s11) : (s10));
    T4[(4 + i15)] = s10;
  }
  for(int i16 = 0; i16 <= 3; i16++) {
    double s12;
    s12 = 10000.0;
    double s13;
    s13 = T5[(i16 + 12)];
    s12 = ((((s13 <= s12))) ? (s13) : (s12));
    T4[(8 + i16)] = s12;
  }
  static double T18[3];
  double s14;
  static double T21[2];
  double s15, s16;
  for(int i19 = 0; i19 <= 1; i19++) {
    static double T23[2];
    static double T24[4];
    double s17;
    for(int i28 = 0; i28 <= 3; i28++) {
      double s18;
      s18 = T4[i28];
      T24[i28] = s18;
    }
    for(int i27 = 0; i27 <= 1; i27++) {
      static double T27[4];
      double s19;
      for(int i66 = 0; i66 <= 3; i66++) {
	T27[i66] = ((((state[i66] <= T24[i66]))) ? (state[i66]) : (T24[i66]));
      }
      s19 = T27[(i19 + (2*i27))];
      T23[i27] = s19;
    }
    s17 = (T23[0] + T23[1]);
    T21[i19] = s17;
  }
  s16 = 1000.0;
  for(int i26 = 0; i26 <= 1; i26++) {
    double s20;
    s20 = T21[i26];
    s16 = ((((s20 <= s16))) ? (s20) : (s16));
  }
  s15 = ((s16 <= 0));
  T18[0] = s15;
  static double T31[2];
  double s21, s22;
  for(int i20 = 0; i20 <= 1; i20++) {
    static double T33[2];
    static double T34[4];
    double s23;
    for(int i31 = 0; i31 <= 3; i31++) {
      double s24;
      s24 = T4[(4 + i31)];
      T34[i31] = s24;
    }
    for(int i30 = 0; i30 <= 1; i30++) {
      static double T37[4];
      double s25;
      for(int i71 = 0; i71 <= 3; i71++) {
	T37[i71] = ((((state[(i71 + 4)] <= T34[i71]))) ? (state[(i71 + 4)]) : (T34[i71]));
      }
      s25 = T37[(i20 + (2*i30))];
      T33[i30] = s25;
    }
    s23 = (T33[0] + T33[1]);
    T31[i20] = s23;
  }
  s22 = 1000.0;
  for(int i29 = 0; i29 <= 1; i29++) {
    double s26;
    s26 = T31[i29];
    s22 = ((((s26 <= s22))) ? (s26) : (s22));
  }
  s21 = ((s22 <= 0));
  T18[1] = s21;
  static double T41[2];
  double s27, s28;
  for(int i21 = 0; i21 <= 1; i21++) {
    static double T43[2];
    double s29;
    for(int i33 = 0; i33 <= 1; i33++) {
      double s30;
      s30 = T4[((i21 + 8) + (2*i33))];
      T43[i33] = s30;
    }
    s29 = (T43[0] + T43[1]);
    T41[i21] = s29;
  }
  s28 = 1000.0;
  for(int i32 = 0; i32 <= 1; i32++) {
    double s31;
    s31 = T41[i32];
    s28 = ((((s31 <= s28))) ? (s31) : (s28));
  }
  s27 = ((s28 <= 0));
  T18[2] = s27;
  s14 = 0;
  for(int i25 = 0; i25 <= 2; i25++) {
    double s32;
    s32 = T18[i25];
    s14 = ((s14) || (s32));
  }
  Y[1] = s14;
  double s33;
  static double T48[12];
  for(int i22 = 0; i22 <= 3; i22++) {
    double s34;
    s34 = T4[i22];
    T48[i22] = s34;
  }
  static double T50[4];
  for(int i37 = 0; i37 <= 3; i37++) {
    double s35;
    s35 = T4[(4 + i37)];
    T50[i37] = s35;
  }
  for(int i36 = 0; i36 <= 3; i36++) {
    double s36, s37;
    s37 = T50[i36];
    s36 = (0.10000000000000001*s37);
    T48[(4 + i36)] = s36;
  }
  static double T54[4];
  for(int i39 = 0; i39 <= 3; i39++) {
    double s38;
    s38 = T4[(8 + i39)];
    T54[i39] = s38;
  }
  for(int i38 = 0; i38 <= 3; i38++) {
    double s39, s40;
    s40 = T54[i38];
    s39 = (0.005000000000000001*s40);
    T48[(8 + i38)] = s39;
  }
  for(int i9 = 0; i9 <= 3; i9++) {
    double s41;
    s41 = 0.0;
    for(int i34 = 0; i34 <= 2; i34++) {
      double s42;
      s42 = T48[((i34*4) + i9)];
      s41 = (s41 + s42);
    }
    state[i9] = s41;
  }
  static double T60[8];
  for(int i23 = 0; i23 <= 3; i23++) {
    double s43;
    s43 = T4[(i23 + 4)];
    T60[i23] = s43;
  }
  static double T62[4];
  for(int i43 = 0; i43 <= 3; i43++) {
    double s44;
    s44 = T4[(8 + i43)];
    T62[i43] = s44;
  }
  for(int i42 = 0; i42 <= 3; i42++) {
    double s45, s46;
    s46 = T62[i42];
    s45 = (0.10000000000000001*s46);
    T60[(4 + i42)] = s45;
  }
  for(int i12 = 0; i12 <= 3; i12++) {
    double s47;
    s47 = 0.0;
    for(int i40 = 0; i40 <= 1; i40++) {
      double s48;
      s48 = T60[((i40*4) + i12)];
      s47 = (s47 + s48);
    }
    state[(4 + i12)] = s47;
  }
}


/*int main(){*/
/*  */
/*  double sensors[(2+1+1)*2];*/
/*  double radius[4] = {4.0, 3.0, 2.0, 1.0};*/

/*  double vtx[8] = {-1.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 1.0};*/
/*  double nrm[8] = {-0.7071, -0.7071, 0.7071, -0.7071, 0.7071, 0.7071, -0.7071, 0.7071};*/

/*  double state[8];*/
/*  for (int i = 0; i != 8; ++i){*/
/*    sensors[i] = 0.0;*/
/*    state[i] = 10000.0;*/
/*  }*/
/*  */
/*  for (int i =0; i != 10; ++i){*/
/*    int out[2];*/
/*    fused(out, sensors, vtx, nrm, radius, &state[0]) ;*/
/*    printf("Result %d: %d %d\n", i, out[0], out[1]);*/
/*  }*/
/*  printf("\n\n");*/
/*  for (int i = 0; i != 8; ++i){*/
/*    sensors[i] = 0.0;*/
/*    state[i] = 10000.0;*/
/*  }*/
/*  for (int i=10; i != 20; ++i){*/
/*    int out[2];*/
/*    sensors[2] = 1.0/0;*/
/*    fused(out, sensors, vtx, nrm, radius, state) ;*/
/*    printf("Result %d: %d %d\n", i, out[0], out[1]);*/
/*  }*/
/*  printf("\n\n");*/
/*  for (int i = 0; i != 8; ++i){*/
/*    sensors[i] = 0.0;*/
/*    state[i] = 10000.0;*/
/*  }*/
/*  for (int i=20; i != 30; ++i){*/
/*    int out[2];*/
/*    sensors[2] += 1;*/
/*    fused(out, sensors, vtx, nrm, radius, state) ;*/
/*    printf("Result %d: %d %d\n", i, out[0], out[1]);*/
/*  }*/
/*  */

/*  return 0;*/
/*}*/
