#include <smmintrin.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <assert.h>

void dwmonitor2_err(int  *Y, float  *X, double  *D, float  *obs) {
    float s1, s13, s8, s9, u10, u11, u12, u13
            , u14, u15, u16, u17, u18, u19, u20, u21
            , u4, u5, u6, u7, u8, u9;
    s9 = 1;
    for(int i4 = 0; i4 <= 15; i4++) {
        s1 = 1;
        u4 = X[0];
        u5 = ((((obs[(3*i4)] == obs[(3*i4)]))) && (((obs[(3*i4)] >= -(FLT_MAX)))) && (((obs[(3*i4)] <= FLT_MAX))));
        s1 = ((s1) && (u5));
        u6 = X[1];
        u7 = ((((obs[((3*i4) + 1)] == obs[((3*i4) + 1)]))) && (((obs[((3*i4) + 1)] >= -(FLT_MAX)))) && (((obs[((3*i4) + 1)] <= FLT_MAX))));
        s1 = ((s1) && (u7));
        u8 = X[2];
        u9 = ((((obs[((3*i4) + 2)] == obs[((3*i4) + 2)]))) && (((obs[((3*i4) + 2)] >= -(FLT_MAX)))) && (((obs[((3*i4) + 2)] <= FLT_MAX))));
        s1 = ((s1) && (u9));
        s9 = ((s9) && (s1));
    }
    s8 = 1;
    u10 = X[0];
    u11 = ((((D[0] == D[0]))) && (((D[0] >= -(FLT_MAX)))) && (((D[0] <= FLT_MAX))));
    s8 = ((s8) && (u11));
    u12 = X[1];
    u13 = ((((D[1] == D[1]))) && (((D[1] >= -(FLT_MAX)))) && (((D[1] <= FLT_MAX))));
    s8 = ((s8) && (u13));
    u14 = X[2];
    u15 = ((((D[2] == D[2]))) && (((D[2] >= -(FLT_MAX)))) && (((D[2] <= FLT_MAX))));
    s8 = ((s8) && (u15));
    s13 = 1;
    u16 = X[0];
    u17 = ((((u16 == u16))) && (((u16 >= -(FLT_MAX)))) && (((u16 <= FLT_MAX))));
    s13 = ((s13) && (u17));
    u18 = X[1];
    u19 = ((((u18 == u18))) && (((u18 >= -(FLT_MAX)))) && (((u18 <= FLT_MAX))));
    s13 = ((s13) && (u19));
    u20 = X[2];
    u21 = ((((u20 == u20))) && (((u20 >= -(FLT_MAX)))) && (((u20 <= FLT_MAX))));
    s13 = ((s13) && (u21));
    Y[0] = 1;
    Y[0] = ((Y[0]) && (s9));
    Y[0] = ((Y[0]) && (s8));
    Y[0] = ((Y[0]) && (s13));
}

void dwmonitor2(int  *Y, float  *X, double  *D, float  *obs) {
    static __m128d U30[16];
    static __m128d U34[5];
    __m128d u23, u24, u25, u26, u27, u28, u29, u30
            , u31, u33, u34, u35, u36, u37, u38, u39
            , u40, u41, u42, u43, u44, u45, u46, u47
            , u48, u49, u50, u51, u52, x10, x11, x12
            , x13, x14, x15, x18, x19, x2, x20, x21
            , x22, x23, x24, x25, x26, x27, x28, x29
            , x3, x30, x31, x32, x33, x36, x37, x38
            , x39, x4, x40, x41, x42, x43, x44, x45
            , x46, x47, x48, x49, x5, x50, x6, x7
            , x8, x9;
    {
        unsigned _xm = _mm_getcsr();
        _mm_setcsr(_xm & 0xffff0000 | 0x0000dfc0);
        for(int i8 = 0; i8 <= 2; i8++) {
            U34[i8] = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[i8])));
        }
        u36 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[1])));
        u35 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[2])));
        for(int i3 = 0; i3 <= 15; i3++) {
            u48 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(obs[((3)*(i3))])));
            u47 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(obs[(((3)*(i3)) + 1)])));
            u46 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[1])));
            u45 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[2])));
            u41 = u48;
            u37 = u46;
            x4 = u41;
            x5 = u37;
            x3 = _mm_add_pd(x4, _mm_shuffle_pd(x5, x5, _MM_SHUFFLE2(0, 1)));
            x2 = _mm_shuffle_pd(x3, x3, _MM_SHUFFLE2(0, 1));
            u50 = _mm_shuffle_pd(_mm_min_pd(x3, x2), _mm_max_pd(x3, x2), _MM_SHUFFLE2(1, 0));
            u41 = u47;
            u37 = u45;
            x8 = u41;
            x9 = u37;
            x7 = _mm_add_pd(x8, _mm_shuffle_pd(x9, x9, _MM_SHUFFLE2(0, 1)));
            x6 = _mm_shuffle_pd(x7, x7, _MM_SHUFFLE2(0, 1));
            u49 = _mm_shuffle_pd(_mm_min_pd(x6, x7), _mm_max_pd(x6, x7), _MM_SHUFFLE2(1, 0));
            x10 = _mm_set1_pd(0.0);
            x11 = u50;
            x12 = _mm_shuffle_pd(_mm_min_pd(x10, x11), _mm_max_pd(x10, x11), _MM_SHUFFLE2(1, 0));
            x13 = u49;
            x14 = _mm_shuffle_pd(_mm_min_pd(x12, x13), _mm_max_pd(x12, x13), _MM_SHUFFLE2(1, 0));
            x15 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(obs[(((3)*(i3)) + 2)])));
            U30[i3] = _mm_add_pd(x14, _mm_shuffle_pd(x15, x15, _MM_SHUFFLE2(0, 1)));
        }
        u26 = _mm_set_pd(10000.0, (-10000.0));
        for(int i19 = 0; i19 <= 15; i19++) {
            u27 = U30[i19];
            u26 = _mm_shuffle_pd(_mm_max_pd(u27, u26), _mm_min_pd(u27, u26), _MM_SHUFFLE2(1, 0));
        }
        u44 = u26;
        u43 = u26;
        u42 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[1])));
        u40 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[2])));
        u52 = u44;
        u51 = u42;
        x18 = u52;
        x19 = u51;
        U34[3] = _mm_add_pd(x18, x19);
        u52 = u43;
        u51 = u40;
        x20 = u52;
        x21 = u51;
        U34[4] = _mm_add_pd(x20, x21);
        u28 = _mm_set1_pd(0.0);
        u23 = _mm_set_pd(1.0, (-1.0));
        for(int i14 = 0; i14 <= 2; i14++) {
            x26 = u23;
            x27 = _mm_addsub_pd(_mm_set1_pd((DBL_MIN + DBL_MIN)), _mm_loaddup_pd(&(D[i14])));
            x22 = _mm_addsub_pd(_mm_set1_pd(0.0), x26);
            x23 = _mm_mul_pd(x22, x27);
            x24 = _mm_mul_pd(_mm_shuffle_pd(x22, x22, _MM_SHUFFLE2(0,1)), x27);
            x25 = _mm_sub_pd(_mm_set1_pd(0.0), _mm_min_pd(x23, x24));
            u31 = _mm_add_pd(_mm_max_pd(x23, _mm_max_pd(x24, _mm_shuffle_pd(x25, x25, _MM_SHUFFLE2(0, 1)))), _mm_set1_pd(DBL_MIN));
            x28 = u28;
            x29 = u31;
            u28 = _mm_add_pd(x28, x29);
            x30 = _mm_addsub_pd(_mm_set1_pd(0.0), u23);
            x31 = _mm_mul_pd(x30, u24);
            x32 = _mm_mul_pd(_mm_shuffle_pd(x30, x30, _MM_SHUFFLE2(0,1)), u24);
            x33 = _mm_sub_pd(_mm_set1_pd(0.0), _mm_min_pd(x31, x32));
            u23 = _mm_add_pd(_mm_max_pd(x31, _mm_max_pd(x32, _mm_shuffle_pd(x33, x33, _MM_SHUFFLE2(0, 1)))), _mm_set1_pd(DBL_MIN));
        }
        u34 = U34[1];
        u33 = U34[3];
        x38 = u34;
        x39 = u33;
        x37 = _mm_add_pd(x38, _mm_shuffle_pd(x39, x39, _MM_SHUFFLE2(0, 1)));
        x36 = _mm_shuffle_pd(x37, x37, _MM_SHUFFLE2(0, 1));
        u39 = _mm_shuffle_pd(_mm_min_pd(x36, x37), _mm_max_pd(x36, x37), _MM_SHUFFLE2(1, 0));
        u34 = U34[2];
        u33 = U34[4];
        x42 = u34;
        x43 = u33;
        x41 = _mm_add_pd(x42, _mm_shuffle_pd(x43, x43, _MM_SHUFFLE2(0, 1)));
        x40 = _mm_shuffle_pd(x41, x41, _MM_SHUFFLE2(0, 1));
        u38 = _mm_shuffle_pd(_mm_min_pd(x40, x41), _mm_max_pd(x40, x41), _MM_SHUFFLE2(1, 0));
        u25 = _mm_set1_pd(0.0);
        u30 = u39;
        x44 = u30;
        x45 = u25;
        u25 = _mm_shuffle_pd(_mm_min_pd(x44, x45), _mm_max_pd(x44, x45), _MM_SHUFFLE2(1, 0));
        u29 = u38;
        x46 = u29;
        x47 = u25;
        u25 = _mm_shuffle_pd(_mm_min_pd(x46, x47), _mm_max_pd(x46, x47), _MM_SHUFFLE2(1, 0));
        x48 = _mm_addsub_pd(_mm_set1_pd(0.0), u25);
        x49 = _mm_addsub_pd(_mm_set1_pd(0.0), u28);
        x50 = _mm_cmpge_pd(x48, _mm_shuffle_pd(x49, x49, _MM_SHUFFLE2(0, 1)));
        Y[0] = (_mm_testc_si128(_mm_castpd_si128(x50), _mm_set_epi32(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff)) - (_mm_testnzc_si128(_mm_castpd_si128(x50), _mm_set_epi32(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff))));
        // BASIC BLOCK BARRIER
        if (_mm_getcsr() & 0x0d) {
            _mm_setcsr(_xm);
            Y[0] = -1;
        }
        _mm_setcsr(_xm);
    }
}


void dwmonitor_all(int  *Y, float  *X, double  *D, float  *obs) {
    dwmonitor2_err(Y, X, D, obs);
    dwmonitor2((Y + 1), X, D, obs);
}



/*int main(){*/

/*	double max_accv = 1.0;*/
/*	double dt = 0.1;*/
/*	double radius = 1.0;*/
/*	double accv = max_accv;*/
/*	double f_v = 0.5;*/
/*	*/
/*	int max_obs = 16;*/
/*	*/
/*	float obs[max_obs * 3];*/
/*	printf("Obstacles\n");*/
/*	for (int i = 0; i != 16; ++i){*/
/*		obs[0+i*3] = obs[1+i*3] = 0.0;*/
/*		obs[i*3+2] = (1+i)/10.0;*/
/*		*/
/*		printf("%f %f %f\n", obs[i*3], obs[i*3+1], obs[i*3+2]);*/
/*	}*/
/*	*/
/*	printf("\n");*/
/*	*/
/*	double d0 = ((accv/accv+1) * (accv/2*dt*dt) + f_v * dt);*/
/*	*/
/*	double D[3];*/
/*	D[0] = d0 + radius;*/
/*	D[1] = (f_v/accv) + dt*(accv/accv+1);*/
/*	D[2] = 1/(2*accv);*/
/*	*/
/*	float X[3];*/
/*	X[0] = f_v;*/
/*	X[1] = -5.0;*/
/*	X[2] = 0.0;*/
/*	*/
/*	int output = 1;*/
/*	int i= 0;*/
/*	while (output==1 && i < 30){*/
/*		*/
/*		printf("%d: cur_pos(%f %f)\n", i, X[1],X[2]);*/
/*		dwmonitor2(&output, X, D, obs);*/
/*		printf("\nResult: %d \n\n", output);*/
/*		*/
/*		X[1] += f_v;*/
/*		i += 1;*/
/*	}*/

/*	double D[3];*/
/*	float X[3];*/
/*	float obs[16];*/
/*	int out;*/
/*	for (int i = 0; i != 512; ++i){*/
/*			X[0] = pow(-1, (0x1 & i))*FLT_MAX;*/
/*			X[1] = pow(-1, (0x2 & i))*FLT_MAX;*/
/*			X[2] = pow(-1, (0x4 & i))*FLT_MAX;*/
/*			D[0] = pow(-1, (0x8 & i))*FLT_MAX;*/
/*			D[1] = pow(-1, (0x10 & i))*FLT_MAX;*/
/*			D[2] = pow(-1, (0x20 & i))*FLT_MAX;*/
/*			obs[0] = pow(-1, (0x40 & i))*FLT_MAX;*/
/*			obs[1] = pow(-1, (0x80 & i))*FLT_MAX;*/
/*			obs[2] = pow(-1, (0x100 & i))*FLT_MAX;*/
/*			*/
/*			for (int j= 1; j != 16; ++j){*/
/*				obs[j*3] = obs[0];*/
/*				obs[j*3+1] = obs[1];*/
/*				obs[j*3+2] = obs[2];*/
/*			}*/
/*			*/
/*			printf("%d  ", i);*/
/*			dwmonitor2(&out, X, D, obs);*/
/*	}*/

/*}*/
