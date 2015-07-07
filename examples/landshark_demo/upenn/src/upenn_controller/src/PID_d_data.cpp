/*
 * File: PID_d_data.cpp
 *
 * Code generated for Simulink model 'PID_d'.
 *
 * Model version                  : 1.56
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Dec 29 2011)
 * C/C++ source code generated on : Thu Jan 24 12:34:03 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include <landshark_simulatorN/PID_d.h>
#include <landshark_simulatorN/PID_d_private.h>



//manually modified by MP

/* Block parameters (auto storage) */
Parameters_PID_d PID_d_P = {
  /*  Expression: A
   * Referenced by: '<S1>/MATLAB Function'
   */
  /*
  { 0.999995416130429, -0.0058871165957384295, 0.055480730784095857,
    -0.00666096284979329, -0.00588711659573843, 0.055480730784095877,
    -0.0066609628497932925, 4.8069866871384238E-10, -0.0039036659123141667,
    0.03568557774296667, 1.8980977965731405E-5, -5.9317156938538152E-13,
    5.7041313601279944E-12, -1.1513112100624592E-12, 4.2917197943802595E-9,
    -0.033807389440705271, 0.309051357845887, 0.0001674140074554152,
    -5.4039139201212584E-12, 5.1943697445216935E-11, -1.0394676811966677E-11,
    4.2938310454933911E-5, 1.4984982604524797, -13.951167287951264,
    0.99765176944324929, -9.0892990268088954E-8, 8.662230676638898E-7,
    -1.4312130243694937E-7, 4.8069866871384248E-10, -5.9317156938538162E-13,
    5.7041313601279944E-12, -1.1513112100624596E-12, -0.0039036659123141667,
    0.03568557774296667, 1.8980977965731398E-5, 4.29171979438026E-9,
    -5.40391392012126E-12, 5.1943697445216935E-11, -1.0394676811966676E-11,
    -0.033807389440705271, 0.309051357845887, 0.0001674140074554152,
    4.2938310454933924E-5, -9.0892990268088941E-8, 8.6622306766388991E-7,
    -1.4312130243694937E-7, 1.4984982604524795, -13.951167287951264,
    0.99765176944324907 },
	*/
	{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.019410235704523639, 0.94160905144045348,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

  /*  Expression: B
   * Referenced by: '<S1>/MATLAB Function'
   */
  /*
  { 9.0920025964728474E-8, 1.7529109420122573, 35.40665341450277,
    0.0058871258456177581, -6.570265644169564E-11, 6.3880186445345875E-10,
    -1.5787674085583573E-10, 9.092002596472858E-8, -6.5702656441695692E-11,
    6.3880186445345937E-10, -1.5787674085583586E-10, 1.7529109420122677,
    35.406653414502856, 0.0058871258456177676 },
	*/

  { 0.0017528897200048425, 0.17354881307063305, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },
//  { 0.001899658879743429, 0.18808002583165917, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//    0.0, 0.0, 0.0, 0.0, 0.0 },

/*  Expression: B
  * Referenced by: '<Root>/MATLAB Function'
  */
// { 0.0017528897200048425, 0.17354881307063305, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },




  /*  Expression: C
   * Referenced by: '<S1>/MATLAB Function'
   */
  /*
  { 1.0, 0.9, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /*  Expression: CAks
   * Referenced by: '<S1>/MATLAB Function'
   */

  /*
  { 1.0, 0.9, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.999995416130429, 0.899995874517386,
    1.1999944993565148, 4.8069866871384238E-10, 4.3262880184245813E-10,
    5.7683840245661087E-10, 4.2917197943802595E-9, 3.8625478149422338E-9,
    5.1500637532563109E-9, 4.2938310454933911E-5, 3.8644479409440522E-5,
    5.1525972545920694E-5, 4.8069866871384248E-10, 4.3262880184245823E-10,
    5.76838402456611E-10, 4.29171979438026E-9, 3.8625478149422346E-9,
    5.1500637532563125E-9, 4.2938310454933924E-5, 3.8644479409440536E-5,
    5.1525972545920708E-5, 0.99999026073144393, 0.89999123465829955,
    1.1999883128777327, 1.446983553839747E-9, 1.3022851984557723E-9,
    1.7363802646076964E-9, 1.2790284965062649E-8, 1.1511256468556385E-8,
    1.5348341958075177E-8, 8.5716434717189E-5, 7.71447912454701E-5,
    0.00010285972166062679, 1.446983553839747E-9, 1.3022851984557723E-9,
    1.7363802646076964E-9, 1.2790284965062651E-8, 1.1511256468556385E-8,
    1.534834195807518E-8, 8.5716434717189035E-5, 7.7144791245470137E-5,
    0.00010285972166062684, 0.99998453640073348, 0.89998608276066017,
    1.1999814436808802, 2.5584558152148766E-9, 2.3026102336933889E-9,
    3.0701469782578517E-9, 2.2545745145795116E-8, 2.0291170631215604E-8,
    2.7054894174954138E-8, 0.00012827676167339753, 0.00011544908550605779,
    0.00015393211400807703, 2.5584558152148766E-9, 2.3026102336933889E-9,
    3.0701469782578517E-9, 2.254574514579512E-8, 2.0291170631215607E-8,
    2.7054894174954141E-8, 0.00012827676167339756, 0.00011544908550605781,
    0.00015393211400807706 },
  */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.94160905144045348,
    0.94160905144045348, 0.94160905144045348, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.88662760575459054,
    0.88662760575459054, 0.88662760575459054, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.83485657883550035,
    0.83485657883550035, 0.83485657883550035, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  4.0,                                 /* Expression: T
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  0.0,                                 /* Expression: 0.0
                                        * Referenced by: '<S1>/Delay1'
                                        */
  0.05,                                /* Expression: Kp 11.0
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  0.01,                                /* Computed Parameter: Integrator1_gainval
                                        * Referenced by: '<S1>/Integrator1'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Integrator1'
                                        */
  0.0,                                 /* Expression: Kd
                                        * Referenced by: '<S1>/Derivative Gain'
                                        */
  0.01,                                /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S1>/Filter'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Filter'
                                        */
  100.0,                               /* Expression: N
                                        * Referenced by: '<S1>/Filter Coefficient'
                                        */
  0.2,                                 /* Expression: Ki 0.2
                                        * Referenced by: '<S1>/Integral Gain'
                                        */

  /*  Expression: zeros(p,T)
   * Referenced by: '<S1>/Data Store Memory1'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /*  Expression: zeros(n*(T-1),1)
   * Referenced by: '<S1>/Data Store Memory6'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  1U                                   /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S1>/Delay1'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */