#include <stdio.h>
#include <stdlib.h>
#include <smmintrin.h>
#include <float.h>
#include <time.h>


void ftest(int  *Y, float  *X,  int idx1) {
	
		 static __m128d  ivstate[16] = {0,0,0,0,
									0,0,0,0,
									0,0,0,0,
									0,0,0,0};
	
    __m128d u5, u6, u7, u8, u9, x10, x11, x12
            , x14, x15, x16, x17, x19, x20, x21, x22
            , x23, x24, x26, x27, x28, x29, x3, x30
            , x31, x32, x34, x35, x36, x37, x4, x5
            , x6, x7, x8, x9;
    {
        unsigned _xm = _mm_getcsr();
        _mm_setcsr(_xm & 0xffff0000 | 0x0000dfc0);
        x3 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[0])));
        x4 = ivstate[(2 + (((idx1)&(15))))];
        u5 = _mm_add_pd(x3, _mm_shuffle_pd(x4, x4, _MM_SHUFFLE2(0, 1)));
        x5 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[0])));
        x6 = ivstate[1];
        u6 = _mm_add_pd(x5, _mm_shuffle_pd(x6, x6, _MM_SHUFFLE2(0, 1)));
        x7 = ivstate[0];
        x19 = ivstate[(2 + (((idx1)&(15))))];
        x20 = ivstate[1];
        x17 = _mm_add_pd(x19, _mm_shuffle_pd(x20, x20, _MM_SHUFFLE2(0, 1)));
        x15 = _mm_add_pd(x17, u6);
        x26 = _mm_set_pd(0.0625, (-0.0625));
        x21 = _mm_addsub_pd(_mm_set1_pd(0.0), u5);
        x22 = _mm_mul_pd(x21, x26);
        x23 = _mm_mul_pd(_mm_shuffle_pd(x21, x21, _MM_SHUFFLE2(0, 1)), x26);
        x24 = _mm_sub_pd(_mm_set1_pd(0.0), _mm_min_pd(x22, x23));
        x16 = _mm_add_pd(_mm_max_pd(x22, _mm_max_pd(x23, _mm_shuffle_pd(x24, x24, _MM_SHUFFLE2(0, 1)))), _mm_set1_pd(DBL_MIN));
        x14 = _mm_add_pd(x15, _mm_shuffle_pd(x16, x16, _MM_SHUFFLE2(0, 1)));
        x9 = _mm_addsub_pd(_mm_set1_pd(0.0), u5);
        x10 = _mm_mul_pd(x9, x14);
        x11 = _mm_mul_pd(_mm_shuffle_pd(x9, x9, _MM_SHUFFLE2(0, 1)), x14);
        x12 = _mm_sub_pd(_mm_set1_pd(0.0), _mm_min_pd(x10, x11));
        x8 = _mm_add_pd(_mm_max_pd(x10, _mm_max_pd(x11, _mm_shuffle_pd(x12, x12, _MM_SHUFFLE2(0, 1)))), _mm_set1_pd(DBL_MIN));
        u7 = _mm_add_pd(x7, x8);
        x27 = ivstate[1];
        x34 = _mm_set_pd(0.0625, (-0.0625));
        x29 = _mm_addsub_pd(_mm_set1_pd(0.0), u5);
        x30 = _mm_mul_pd(x29, x34);
        x31 = _mm_mul_pd(_mm_shuffle_pd(x29, x29, _MM_SHUFFLE2(0, 1)), x34);
        x32 = _mm_sub_pd(_mm_set1_pd(0.0), _mm_min_pd(x30, x31));
        x28 = _mm_add_pd(_mm_max_pd(x30, _mm_max_pd(x31, _mm_shuffle_pd(x32, x32, _MM_SHUFFLE2(0, 1)))), _mm_set1_pd(DBL_MIN));
        u8 = _mm_add_pd(x27, x28);
        u9 = _mm_addsub_pd(_mm_cvtps_pd(_mm_set1_ps(FLT_MIN)), _mm_cvtps_pd(_mm_set1_ps(X[0])));
        ivstate[(2 + (((idx1)&(15))))] = u9;
        x35 = _mm_addsub_pd(_mm_set1_pd(0.0), u7);
        x36 = _mm_set1_pd(0.080000000000000002);
        x37 = _mm_cmpge_pd(x35, _mm_shuffle_pd(x36, x36, _MM_SHUFFLE2(0, 1)));
        Y[0] = (_mm_testc_si128(_mm_castpd_si128(x37), _mm_set_epi32(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff)) - (_mm_testnzc_si128(_mm_castpd_si128(x37), _mm_set_epi32(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff))));
        ivstate[0] = u7;
        ivstate[1] = u8;
        // BASIC BLOCK BARRIER
        if (_mm_getcsr() & 0x0d) {
            _mm_setcsr(_xm);
            Y[0] = -1;
        }
        _mm_setcsr(_xm);
    }
}

/*int main(){*/
/*  //__m128d  states[16+2];*/
/* */
/*  float err, prob;*/

/*  int i, j, test, test2;*/
/*  */
/*  srand(time(NULL));*/

/*//  for (i = 0; i != (16+2); ++i){*/
/*//    states[i] = _mm_setzero_pd();*/
/*//  }*/


/*  */
/*  for (i = 0; i != 60*10; ++i){*/
/*    err = ((double)rand()/(RAND_MAX/2.0))-1;*/
/*    for (j = 0; j != 10; ++j){*/
/*        ftest(&test, &err,  i % 16);  */
/*       printf(" Sensor OK? = %d \n", test );*/
/*    }*/
/*  }*/

/*  err = ((double)rand()/(RAND_MAX/2.0))-1;*/
/*  for (i = 0; i != 32; ++i){*/
/*    ftest(&test, &err, i % 16);  */
/*    printf(" Sensor OK? =%d\n", test);*/
/*  }*/

/*  return 0;*/
/*}*/
