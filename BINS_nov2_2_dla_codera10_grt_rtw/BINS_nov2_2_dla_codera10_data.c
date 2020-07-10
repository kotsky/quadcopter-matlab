/*
 * BINS_nov2_2_dla_codera10_data.c
 *
 * Code generation for model "BINS_nov2_2_dla_codera10".
 *
 * Model version              : 1.46
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Sun May 13 16:34:54 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "BINS_nov2_2_dla_codera10.h"
#include "BINS_nov2_2_dla_codera10_private.h"

/* Block parameters (auto storage) */
P_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_P = {
  /*  Variable: C0
   * Referenced by: '<Root>/Integrator6'
   */
  { 6.123233995736766E-17, -1.0, -0.0, 1.0, 6.123233995736766E-17, 0.0, 0.0,
    -0.0, 1.0 },
  7.27220521664304E-5,                 /* Variable: Uz
                                        * Referenced by:
                                        *   '<Root>/Constant4'
                                        *   '<Root>/Constant5'
                                        *   '<Root>/Constant6'
                                        *   '<Root>/Gain7'
                                        *   '<Root>/Gain8'
                                        */
  6.378245E+6,                         /* Variable: a
                                        * Referenced by:
                                        *   '<Root>/Constant7'
                                        *   '<Root>/Constant9'
                                        */
  0.0066934274898192208,               /* Variable: e2
                                        * Referenced by:
                                        *   '<Root>/Constant9'
                                        *   '<Root>/Gain4'
                                        */
  9.81,                                /* Variable: g
                                        * Referenced by: '<Root>/Constant'
                                        */

  /*  Expression: [Ve0;Vn0;Vv0]
   * Referenced by: '<Root>/Integrator8'
   */
  { 10.0, 0.0, 0.0 },
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<Root>/Constant8'
                                        */

  /*  Expression: [lamda0;fi0;H0]
   * Referenced by: '<Root>/Integrator7'
   */
  { 0.53232542185827048, 0.87993683121380772, 0.0 },
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain2'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain3'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain6'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain9'
                                        */

  /*  Expression: [0;0;-g]
   * Referenced by: '<Root>/g2'
   */
  { 0.0, 0.0, -9.81 },
  -1.0                                 /* Expression: -1
                                        * Referenced by: '<Root>/Gain10'
                                        */
};
