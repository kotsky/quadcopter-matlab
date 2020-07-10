/*
 * BINS_nov2_2_dla_codera10.c
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

/* Block signals (auto storage) */
B_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_B;

/* Continuous states */
X_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_X;

/* Block states (auto storage) */
DW_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_DW;

/* Real-time model */
RT_MODEL_BINS_nov2_2_dla_code_T BINS_nov2_2_dla_codera10_M_;
RT_MODEL_BINS_nov2_2_dla_code_T *const BINS_nov2_2_dla_codera10_M =
  &BINS_nov2_2_dla_codera10_M_;

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 15;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  BINS_nov2_2_dla_codera10_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  BINS_nov2_2_dla_codera10_step();
  BINS_nov2_2_dla_codera10_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  BINS_nov2_2_dla_codera10_step();
  BINS_nov2_2_dla_codera10_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void BINS_nov2_2_dla_codera10_step(void)
{
  real_T tmp[9];
  real_T tmp_0[3];
  real_T tmp_1[9];
  real_T Bias;
  int32_T i;
  int32_T i_0;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* set solver stop time */
    if (!(BINS_nov2_2_dla_codera10_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&BINS_nov2_2_dla_codera10_M->solverInfo,
                            ((BINS_nov2_2_dla_codera10_M->Timing.clockTickH0 + 1)
        * BINS_nov2_2_dla_codera10_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&BINS_nov2_2_dla_codera10_M->solverInfo,
                            ((BINS_nov2_2_dla_codera10_M->Timing.clockTick0 + 1)
        * BINS_nov2_2_dla_codera10_M->Timing.stepSize0 +
        BINS_nov2_2_dla_codera10_M->Timing.clockTickH0 *
        BINS_nov2_2_dla_codera10_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    BINS_nov2_2_dla_codera10_M->Timing.t[0] = rtsiGetT
      (&BINS_nov2_2_dla_codera10_M->solverInfo);
  }

  /* Integrator: '<Root>/Integrator8' */
  BINS_nov2_2_dla_codera10_B.Integrator8[0] =
    BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[0];
  BINS_nov2_2_dla_codera10_B.Integrator8[1] =
    BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[1];
  BINS_nov2_2_dla_codera10_B.Integrator8[2] =
    BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[2];
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate1In1' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate1[0] = 0.0;

    /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate1In2' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate1[1] = 0.0;

    /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate2In2' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate2[1] = 0.0;

    /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate2In3' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate2[2] = 0.0;
  }

  /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate3In1' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate3[0] = 0.0;

  /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate4In2' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate4[1] = 0.0;

  /* Product: '<Root>/Divide3' incorporates:
   *  Constant: '<Root>/Constant9'
   */
  Bias = (1.0 - BINS_nov2_2_dla_codera10_P.e2) * BINS_nov2_2_dla_codera10_P.a;
  BINS_nov2_2_dla_codera10_B.Divide3 = BINS_nov2_2_dla_codera10_B.Integrator8[1]
    / Bias;

  /* Integrator: '<Root>/Integrator7' */
  BINS_nov2_2_dla_codera10_B.Integrator7[0] =
    BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[0];
  BINS_nov2_2_dla_codera10_B.Integrator7[1] =
    BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[1];
  BINS_nov2_2_dla_codera10_B.Integrator7[2] =
    BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[2];

  /* Trigonometry: '<Root>/Trigonometric Function3' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction3 = sin
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Math: '<Root>/Math Function' */
  BINS_nov2_2_dla_codera10_B.MathFunction =
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction3 *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction3;

  /* Gain: '<Root>/Gain4' */
  BINS_nov2_2_dla_codera10_B.Gain4 = BINS_nov2_2_dla_codera10_P.e2 *
    BINS_nov2_2_dla_codera10_B.MathFunction;

  /* Sum: '<Root>/Sum8' incorporates:
   *  Constant: '<Root>/Constant8'
   */
  BINS_nov2_2_dla_codera10_B.Sum8 = BINS_nov2_2_dla_codera10_P.Constant8_Value -
    BINS_nov2_2_dla_codera10_B.Gain4;

  /* Sqrt: '<Root>/Sqrt' */
  BINS_nov2_2_dla_codera10_B.Sqrt = sqrt(BINS_nov2_2_dla_codera10_B.Sum8);

  /* Product: '<Root>/Product13' */
  BINS_nov2_2_dla_codera10_B.Product13 = BINS_nov2_2_dla_codera10_B.Sqrt *
    BINS_nov2_2_dla_codera10_B.Sum8;

  /* Sum: '<Root>/Sum10' */
  BINS_nov2_2_dla_codera10_B.Sum10 = BINS_nov2_2_dla_codera10_B.Product13 +
    BINS_nov2_2_dla_codera10_B.Integrator7[2];

  /* Product: '<Root>/Divide4' */
  BINS_nov2_2_dla_codera10_B.Divide4 = BINS_nov2_2_dla_codera10_B.Divide3 /
    BINS_nov2_2_dla_codera10_B.Sum10;

  /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate4In3' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate4[2] =
    BINS_nov2_2_dla_codera10_B.Divide4;

  /* SignalConversion: '<Root>/ConcatBufferAtVector Concatenate5In3' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate5[2] = 0.0;
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* SignalConversion: '<Root>/ConcatBufferAtVector ConcatenateIn1' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate[0] = 0.0;

    /* SignalConversion: '<Root>/ConcatBufferAtVector ConcatenateIn3' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate[2] = 0.0;
  }

  /* Product: '<Root>/Divide1' incorporates:
   *  Constant: '<Root>/Constant7'
   */
  BINS_nov2_2_dla_codera10_B.Divide1 = BINS_nov2_2_dla_codera10_P.a /
    BINS_nov2_2_dla_codera10_B.Sqrt;

  /* Sum: '<Root>/Sum9' */
  BINS_nov2_2_dla_codera10_B.Sum9 = BINS_nov2_2_dla_codera10_B.Divide1 +
    BINS_nov2_2_dla_codera10_B.Integrator7[2];

  /* Product: '<Root>/Divide' */
  BINS_nov2_2_dla_codera10_B.Divide = BINS_nov2_2_dla_codera10_B.Integrator8[0] /
    BINS_nov2_2_dla_codera10_B.Sum9;

  /* Trigonometry: '<Root>/Trigonometric Function5' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction5 = cos
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Product: '<Root>/Divide2' */
  BINS_nov2_2_dla_codera10_B.Divide2 = BINS_nov2_2_dla_codera10_B.Divide /
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction5;

  /* Integrator: '<Root>/Integrator6' */
  memcpy(&BINS_nov2_2_dla_codera10_B.Integrator6[0],
         &BINS_nov2_2_dla_codera10_X.Integrator6_CSTATE[0], 9U * sizeof(real_T));

  /* Selector: '<Root>/Selector1' */
  BINS_nov2_2_dla_codera10_B.Selector1 = BINS_nov2_2_dla_codera10_B.Integrator6
    [2];

  /* Selector: '<Root>/Selector2' */
  BINS_nov2_2_dla_codera10_B.Selector2 = BINS_nov2_2_dla_codera10_B.Integrator6
    [8];

  /* Product: '<Root>/Divide5' */
  BINS_nov2_2_dla_codera10_B.Divide5 = BINS_nov2_2_dla_codera10_B.Selector1 /
    BINS_nov2_2_dla_codera10_B.Selector2;

  /* Selector: '<Root>/Selector3' */
  BINS_nov2_2_dla_codera10_B.Selector3 = BINS_nov2_2_dla_codera10_B.Integrator6
    [3];

  /* Selector: '<Root>/Selector4' */
  BINS_nov2_2_dla_codera10_B.Selector4 = BINS_nov2_2_dla_codera10_B.Integrator6
    [4];

  /* Product: '<Root>/Divide6' */
  BINS_nov2_2_dla_codera10_B.Divide6 = BINS_nov2_2_dla_codera10_B.Selector3 /
    BINS_nov2_2_dla_codera10_B.Selector4;
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* Gain: '<Root>/Gain' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate[1] =
      BINS_nov2_2_dla_codera10_P.Gain_Gain * 0.0;

    /* Gain: '<Root>/Gain1' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate1[2] =
      BINS_nov2_2_dla_codera10_P.Gain1_Gain * 0.0;

    /* Gain: '<Root>/Gain2' */
    BINS_nov2_2_dla_codera10_B.VectorConcatenate2[0] =
      BINS_nov2_2_dla_codera10_P.Gain2_Gain * 0.0;
  }

  /* Gain: '<Root>/Gain3' */
  BINS_nov2_2_dla_codera10_B.Gain3 = BINS_nov2_2_dla_codera10_P.Gain3_Gain *
    BINS_nov2_2_dla_codera10_B.Integrator7[1];

  /* Gain: '<Root>/Gain5' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate5[1] =
    BINS_nov2_2_dla_codera10_P.Gain5_Gain * BINS_nov2_2_dla_codera10_B.Divide4;

  /* Trigonometry: '<Root>/Trigonometric Function4' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction4 = cos
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Gain: '<Root>/Gain6' */
  BINS_nov2_2_dla_codera10_B.Gain6 = BINS_nov2_2_dla_codera10_P.Gain6_Gain *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction4;

  /* Trigonometry: '<Root>/Trigonometric Function6' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction6 = cos
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Gain: '<Root>/Gain7' */
  BINS_nov2_2_dla_codera10_B.Gain7 = BINS_nov2_2_dla_codera10_P.Uz *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction6;

  /* Trigonometry: '<Root>/Trigonometric Function7' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction7 = sin
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Gain: '<Root>/Gain8' */
  BINS_nov2_2_dla_codera10_B.Gain8 = BINS_nov2_2_dla_codera10_P.Uz *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction7;

  /* Gain: '<Root>/Gain9' */
  BINS_nov2_2_dla_codera10_B.Gain9 = BINS_nov2_2_dla_codera10_P.Gain9_Gain *
    BINS_nov2_2_dla_codera10_B.Divide5;
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* Concatenate: '<Root>/Matrix Concatenate' */
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[0] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate[0];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[3] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate[1];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[6] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate[2];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[1] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate1[0];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[4] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate1[1];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[7] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate1[2];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[2] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate2[0];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[5] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate2[1];
    BINS_nov2_2_dla_codera10_B.MatrixConcatenate[8] =
      BINS_nov2_2_dla_codera10_B.VectorConcatenate2[2];
  }

  /* Sum: '<Root>/Sum2' incorporates:
   *  Constant: '<Root>/Constant4'
   */
  BINS_nov2_2_dla_codera10_B.Sum2 = BINS_nov2_2_dla_codera10_B.Divide2 +
    BINS_nov2_2_dla_codera10_P.Uz;

  /* Trigonometry: '<Root>/Trigonometric Function' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction = sin
    (BINS_nov2_2_dla_codera10_B.Gain3);

  /* Product: '<Root>/Product' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate3[1] =
    BINS_nov2_2_dla_codera10_B.Sum2 *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction;

  /* Trigonometry: '<Root>/Trigonometric Function1' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction1 = cos
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Product: '<Root>/Product1' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate3[2] =
    BINS_nov2_2_dla_codera10_B.Sum2 *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction1;

  /* Sum: '<Root>/Sum3' incorporates:
   *  Constant: '<Root>/Constant5'
   */
  BINS_nov2_2_dla_codera10_B.Sum3 = BINS_nov2_2_dla_codera10_B.Divide2 +
    BINS_nov2_2_dla_codera10_P.Uz;

  /* Trigonometry: '<Root>/Trigonometric Function2' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction2 = sin
    (BINS_nov2_2_dla_codera10_B.Integrator7[1]);

  /* Product: '<Root>/Product2' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate4[0] =
    BINS_nov2_2_dla_codera10_B.Sum3 *
    BINS_nov2_2_dla_codera10_B.TrigonometricFunction2;

  /* Sum: '<Root>/Sum4' incorporates:
   *  Constant: '<Root>/Constant6'
   */
  BINS_nov2_2_dla_codera10_B.Sum4 = BINS_nov2_2_dla_codera10_B.Divide2 +
    BINS_nov2_2_dla_codera10_P.Uz;

  /* Product: '<Root>/Product6' */
  BINS_nov2_2_dla_codera10_B.VectorConcatenate5[0] =
    BINS_nov2_2_dla_codera10_B.Sum4 * BINS_nov2_2_dla_codera10_B.Gain6;

  /* Concatenate: '<Root>/Matrix Concatenate1' */
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[0] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate3[0];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[3] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate3[1];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[6] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate3[2];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[1] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate4[0];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[4] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate4[1];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[7] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate4[2];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[2] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate5[0];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[5] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate5[1];
  BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[8] =
    BINS_nov2_2_dla_codera10_B.VectorConcatenate5[2];

  /* SignalConversion: '<Root>/TmpSignal ConversionAtpr1Inport2' */
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[0] = 0.0;
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[1] = 0.0;
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[2] = 0.0;

  /* Product: '<Root>/pr1' */
  memcpy(&tmp[0], &BINS_nov2_2_dla_codera10_B.Integrator6[0], 9U * sizeof(real_T));
  Bias = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[0];
  tmp_3 = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[1];
  tmp_4 = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtpr1Inport2[2];
  for (i = 0; i < 3; i++) {
    tmp_2 = tmp[i] * Bias;
    tmp_2 += tmp[i + 3] * tmp_3;
    tmp_2 += tmp[i + 6] * tmp_4;
    tmp_0[i] = tmp_2;
  }

  BINS_nov2_2_dla_codera10_B.pr1[0] = tmp_0[0];
  BINS_nov2_2_dla_codera10_B.pr1[1] = tmp_0[1];
  BINS_nov2_2_dla_codera10_B.pr1[2] = tmp_0[2];

  /* End of Product: '<Root>/pr1' */

  /* Sum: '<Root>/Sum14' */
  BINS_nov2_2_dla_codera10_B.Sum14[0] = (0.0 - BINS_nov2_2_dla_codera10_B.pr1[0])
    - 0.0;
  BINS_nov2_2_dla_codera10_B.Sum14[1] = (0.0 - BINS_nov2_2_dla_codera10_B.pr1[1])
    - BINS_nov2_2_dla_codera10_B.Gain7;
  BINS_nov2_2_dla_codera10_B.Sum14[2] = (0.0 - BINS_nov2_2_dla_codera10_B.pr1[2])
    - BINS_nov2_2_dla_codera10_B.Gain8;

  /* Product: '<Root>/Product10' */
  BINS_nov2_2_dla_codera10_B.Product10 = BINS_nov2_2_dla_codera10_B.Sum14[0] *
    BINS_nov2_2_dla_codera10_B.Integrator8[2];

  /* Product: '<Root>/Product11' */
  BINS_nov2_2_dla_codera10_B.Product11 = BINS_nov2_2_dla_codera10_B.Sum14[1] *
    BINS_nov2_2_dla_codera10_B.Integrator8[0];

  /* Product: '<Root>/Product12' */
  BINS_nov2_2_dla_codera10_B.Product12 = BINS_nov2_2_dla_codera10_B.Sum14[0] *
    BINS_nov2_2_dla_codera10_B.Integrator8[1];

  /* Product: '<Root>/Product3' */
  BINS_nov2_2_dla_codera10_B.Product3 = BINS_nov2_2_dla_codera10_B.Sum14[1] *
    BINS_nov2_2_dla_codera10_B.Integrator8[2];

  /* Product: '<Root>/Product4' */
  for (i = 0; i < 9; i++) {
    tmp[i] = BINS_nov2_2_dla_codera10_B.Integrator6[i];
    tmp_1[i] = BINS_nov2_2_dla_codera10_B.MatrixConcatenate[i];
  }

  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      BINS_nov2_2_dla_codera10_B.Product4[i + 3 * i_0] = 0.0;
      BINS_nov2_2_dla_codera10_B.Product4[i + 3 * i_0] += tmp_1[3 * i_0] * tmp[i];
      BINS_nov2_2_dla_codera10_B.Product4[i + 3 * i_0] += tmp_1[3 * i_0 + 1] *
        tmp[i + 3];
      BINS_nov2_2_dla_codera10_B.Product4[i + 3 * i_0] += tmp_1[3 * i_0 + 2] *
        tmp[i + 6];
    }
  }

  /* End of Product: '<Root>/Product4' */

  /* Product: '<Root>/Product5' */
  for (i = 0; i < 9; i++) {
    tmp[i] = BINS_nov2_2_dla_codera10_B.MatrixConcatenate1[i];
    tmp_1[i] = BINS_nov2_2_dla_codera10_B.Integrator6[i];
  }

  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      BINS_nov2_2_dla_codera10_B.Product5[i + 3 * i_0] = 0.0;
      BINS_nov2_2_dla_codera10_B.Product5[i + 3 * i_0] += tmp_1[3 * i_0] * tmp[i];
      BINS_nov2_2_dla_codera10_B.Product5[i + 3 * i_0] += tmp_1[3 * i_0 + 1] *
        tmp[i + 3];
      BINS_nov2_2_dla_codera10_B.Product5[i + 3 * i_0] += tmp_1[3 * i_0 + 2] *
        tmp[i + 6];
    }
  }

  /* End of Product: '<Root>/Product5' */

  /* Product: '<Root>/Product8' */
  BINS_nov2_2_dla_codera10_B.Product8 = BINS_nov2_2_dla_codera10_B.Sum14[2] *
    BINS_nov2_2_dla_codera10_B.Integrator8[1];

  /* Product: '<Root>/Product9' */
  BINS_nov2_2_dla_codera10_B.Product9 = BINS_nov2_2_dla_codera10_B.Sum14[2] *
    BINS_nov2_2_dla_codera10_B.Integrator8[0];

  /* Selector: '<Root>/Selector' */
  BINS_nov2_2_dla_codera10_B.Selector = BINS_nov2_2_dla_codera10_B.Integrator6[5];

  /* SignalConversion: '<Root>/TmpSignal ConversionAtprInport2' incorporates:
   *  Constant: '<Root>/Constant'
   */
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[0] = 0.0;
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[1] = 0.0;
  BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[2] =
    BINS_nov2_2_dla_codera10_P.g;
  for (i = 0; i < 9; i++) {
    /* Sum: '<Root>/Sum13' */
    BINS_nov2_2_dla_codera10_B.Sum13[i] = BINS_nov2_2_dla_codera10_B.Product4[i]
      - BINS_nov2_2_dla_codera10_B.Product5[i];

    /* Product: '<Root>/pr' */
    tmp[i] = BINS_nov2_2_dla_codera10_B.Integrator6[i];
  }

  /* Product: '<Root>/pr' */
  Bias = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[0];
  tmp_3 = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[1];
  tmp_4 = BINS_nov2_2_dla_codera10_B.TmpSignalConversionAtprInport2[2];
  for (i = 0; i < 3; i++) {
    tmp_2 = tmp[i] * Bias;
    tmp_2 += tmp[i + 3] * tmp_3;
    tmp_2 += tmp[i + 6] * tmp_4;
    tmp_0[i] = tmp_2;
  }

  BINS_nov2_2_dla_codera10_B.pr[0] = tmp_0[0];
  BINS_nov2_2_dla_codera10_B.pr[1] = tmp_0[1];
  BINS_nov2_2_dla_codera10_B.pr[2] = tmp_0[2];

  /* Sum: '<Root>/Sum5' */
  BINS_nov2_2_dla_codera10_B.Sum5 = BINS_nov2_2_dla_codera10_B.Product3 -
    BINS_nov2_2_dla_codera10_B.Product8;

  /* Sum: '<Root>/Sum6' */
  BINS_nov2_2_dla_codera10_B.Sum6 = BINS_nov2_2_dla_codera10_B.Product9 -
    BINS_nov2_2_dla_codera10_B.Product10;

  /* Sum: '<Root>/Sum7' */
  BINS_nov2_2_dla_codera10_B.Sum7 = BINS_nov2_2_dla_codera10_B.Product12 -
    BINS_nov2_2_dla_codera10_B.Product11;

  /* Sum: '<Root>/Sum16' incorporates:
   *  Constant: '<Root>/g2'
   */
  BINS_nov2_2_dla_codera10_B.Sum16[0] = (BINS_nov2_2_dla_codera10_B.pr[0] +
    BINS_nov2_2_dla_codera10_B.Sum5) + BINS_nov2_2_dla_codera10_P.g2_Value[0];
  BINS_nov2_2_dla_codera10_B.Sum16[1] = (BINS_nov2_2_dla_codera10_B.pr[1] +
    BINS_nov2_2_dla_codera10_B.Sum6) + BINS_nov2_2_dla_codera10_P.g2_Value[1];
  BINS_nov2_2_dla_codera10_B.Sum16[2] = (BINS_nov2_2_dla_codera10_B.pr[2] +
    BINS_nov2_2_dla_codera10_B.Sum7) + BINS_nov2_2_dla_codera10_P.g2_Value[2];

  /* Trigonometry: '<Root>/Trigonometric Function10' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction10 = atan
    (BINS_nov2_2_dla_codera10_B.Divide6);

  /* Trigonometry: '<Root>/Trigonometric Function8' */
  Bias = BINS_nov2_2_dla_codera10_B.Selector;
  if (Bias > 1.0) {
    Bias = 1.0;
  } else {
    if (Bias < -1.0) {
      Bias = -1.0;
    }
  }

  BINS_nov2_2_dla_codera10_B.TrigonometricFunction8 = asin(Bias);

  /* End of Trigonometry: '<Root>/Trigonometric Function8' */

  /* Trigonometry: '<Root>/Trigonometric Function9' */
  BINS_nov2_2_dla_codera10_B.TrigonometricFunction9 = atan
    (BINS_nov2_2_dla_codera10_B.Gain9);
  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* Gain: '<Root>/Gain10' */
    BINS_nov2_2_dla_codera10_B.Gain10 = BINS_nov2_2_dla_codera10_P.Gain10_Gain *
      0.0;
  }

  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(BINS_nov2_2_dla_codera10_M->rtwLogInfo,
                        (BINS_nov2_2_dla_codera10_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(BINS_nov2_2_dla_codera10_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(BINS_nov2_2_dla_codera10_M)!=-1) &&
          !((rtmGetTFinal(BINS_nov2_2_dla_codera10_M)-
             (((BINS_nov2_2_dla_codera10_M->Timing.clockTick1+
                BINS_nov2_2_dla_codera10_M->Timing.clockTickH1* 4294967296.0)) *
              0.2)) > (((BINS_nov2_2_dla_codera10_M->Timing.clockTick1+
                         BINS_nov2_2_dla_codera10_M->Timing.clockTickH1*
                         4294967296.0)) * 0.2) * (DBL_EPSILON))) {
        rtmSetErrorStatus(BINS_nov2_2_dla_codera10_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&BINS_nov2_2_dla_codera10_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++BINS_nov2_2_dla_codera10_M->Timing.clockTick0)) {
      ++BINS_nov2_2_dla_codera10_M->Timing.clockTickH0;
    }

    BINS_nov2_2_dla_codera10_M->Timing.t[0] = rtsiGetSolverStopTime
      (&BINS_nov2_2_dla_codera10_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.2s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.2, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      BINS_nov2_2_dla_codera10_M->Timing.clockTick1++;
      if (!BINS_nov2_2_dla_codera10_M->Timing.clockTick1) {
        BINS_nov2_2_dla_codera10_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void BINS_nov2_2_dla_codera10_derivatives(void)
{
  XDot_BINS_nov2_2_dla_codera10_T *_rtXdot;
  _rtXdot = ((XDot_BINS_nov2_2_dla_codera10_T *)
             BINS_nov2_2_dla_codera10_M->ModelData.derivs);

  /* Derivatives for Integrator: '<Root>/Integrator8' */
  _rtXdot->Integrator8_CSTATE[0] = BINS_nov2_2_dla_codera10_B.Sum16[0];
  _rtXdot->Integrator8_CSTATE[1] = BINS_nov2_2_dla_codera10_B.Sum16[1];
  _rtXdot->Integrator8_CSTATE[2] = BINS_nov2_2_dla_codera10_B.Sum16[2];

  /* Derivatives for Integrator: '<Root>/Integrator7' */
  _rtXdot->Integrator7_CSTATE[0] = BINS_nov2_2_dla_codera10_B.Divide2;
  _rtXdot->Integrator7_CSTATE[1] = BINS_nov2_2_dla_codera10_B.Divide4;
  _rtXdot->Integrator7_CSTATE[2] = BINS_nov2_2_dla_codera10_B.Integrator8[2];

  /* Derivatives for Integrator: '<Root>/Integrator6' */
  memcpy(&_rtXdot->Integrator6_CSTATE[0], &BINS_nov2_2_dla_codera10_B.Sum13[0],
         9U * sizeof(real_T));
}

/* Model initialize function */
void BINS_nov2_2_dla_codera10_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)BINS_nov2_2_dla_codera10_M, 0,
                sizeof(RT_MODEL_BINS_nov2_2_dla_code_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
                          &BINS_nov2_2_dla_codera10_M->Timing.simTimeStep);
    rtsiSetTPtr(&BINS_nov2_2_dla_codera10_M->solverInfo, &rtmGetTPtr
                (BINS_nov2_2_dla_codera10_M));
    rtsiSetStepSizePtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
                       &BINS_nov2_2_dla_codera10_M->Timing.stepSize0);
    rtsiSetdXPtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
                 &BINS_nov2_2_dla_codera10_M->ModelData.derivs);
    rtsiSetContStatesPtr(&BINS_nov2_2_dla_codera10_M->solverInfo, (real_T **)
                         &BINS_nov2_2_dla_codera10_M->ModelData.contStates);
    rtsiSetNumContStatesPtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
      &BINS_nov2_2_dla_codera10_M->Sizes.numContStates);
    rtsiSetErrorStatusPtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
                          (&rtmGetErrorStatus(BINS_nov2_2_dla_codera10_M)));
    rtsiSetRTModelPtr(&BINS_nov2_2_dla_codera10_M->solverInfo,
                      BINS_nov2_2_dla_codera10_M);
  }

  rtsiSetSimTimeStep(&BINS_nov2_2_dla_codera10_M->solverInfo, MAJOR_TIME_STEP);
  BINS_nov2_2_dla_codera10_M->ModelData.intgData.y =
    BINS_nov2_2_dla_codera10_M->ModelData.odeY;
  BINS_nov2_2_dla_codera10_M->ModelData.intgData.f[0] =
    BINS_nov2_2_dla_codera10_M->ModelData.odeF[0];
  BINS_nov2_2_dla_codera10_M->ModelData.intgData.f[1] =
    BINS_nov2_2_dla_codera10_M->ModelData.odeF[1];
  BINS_nov2_2_dla_codera10_M->ModelData.intgData.f[2] =
    BINS_nov2_2_dla_codera10_M->ModelData.odeF[2];
  BINS_nov2_2_dla_codera10_M->ModelData.contStates =
    ((X_BINS_nov2_2_dla_codera10_T *) &BINS_nov2_2_dla_codera10_X);
  rtsiSetSolverData(&BINS_nov2_2_dla_codera10_M->solverInfo, (void *)
                    &BINS_nov2_2_dla_codera10_M->ModelData.intgData);
  rtsiSetSolverName(&BINS_nov2_2_dla_codera10_M->solverInfo,"ode3");
  rtmSetTPtr(BINS_nov2_2_dla_codera10_M,
             &BINS_nov2_2_dla_codera10_M->Timing.tArray[0]);
  rtmSetTFinal(BINS_nov2_2_dla_codera10_M, 10.0);
  BINS_nov2_2_dla_codera10_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    BINS_nov2_2_dla_codera10_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(BINS_nov2_2_dla_codera10_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(BINS_nov2_2_dla_codera10_M->rtwLogInfo, (NULL));
    rtliSetLogT(BINS_nov2_2_dla_codera10_M->rtwLogInfo, "tout");
    rtliSetLogX(BINS_nov2_2_dla_codera10_M->rtwLogInfo, "");
    rtliSetLogXFinal(BINS_nov2_2_dla_codera10_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(BINS_nov2_2_dla_codera10_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(BINS_nov2_2_dla_codera10_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(BINS_nov2_2_dla_codera10_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(BINS_nov2_2_dla_codera10_M->rtwLogInfo, 1);
    rtliSetLogY(BINS_nov2_2_dla_codera10_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(BINS_nov2_2_dla_codera10_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(BINS_nov2_2_dla_codera10_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &BINS_nov2_2_dla_codera10_B), 0,
                sizeof(B_BINS_nov2_2_dla_codera10_T));

  /* states (continuous) */
  {
    (void) memset((void *)&BINS_nov2_2_dla_codera10_X, 0,
                  sizeof(X_BINS_nov2_2_dla_codera10_T));
  }

  /* states (dwork) */
  (void) memset((void *)&BINS_nov2_2_dla_codera10_DW, 0,
                sizeof(DW_BINS_nov2_2_dla_codera10_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(BINS_nov2_2_dla_codera10_M->rtwLogInfo, 0.0,
    rtmGetTFinal(BINS_nov2_2_dla_codera10_M),
    BINS_nov2_2_dla_codera10_M->Timing.stepSize0, (&rtmGetErrorStatus
    (BINS_nov2_2_dla_codera10_M)));

  /* InitializeConditions for Integrator: '<Root>/Integrator8' */
  BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[0] =
    BINS_nov2_2_dla_codera10_P.Integrator8_IC[0];
  BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[1] =
    BINS_nov2_2_dla_codera10_P.Integrator8_IC[1];
  BINS_nov2_2_dla_codera10_X.Integrator8_CSTATE[2] =
    BINS_nov2_2_dla_codera10_P.Integrator8_IC[2];

  /* InitializeConditions for Integrator: '<Root>/Integrator7' */
  BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[0] =
    BINS_nov2_2_dla_codera10_P.Integrator7_IC[0];
  BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[1] =
    BINS_nov2_2_dla_codera10_P.Integrator7_IC[1];
  BINS_nov2_2_dla_codera10_X.Integrator7_CSTATE[2] =
    BINS_nov2_2_dla_codera10_P.Integrator7_IC[2];

  /* InitializeConditions for Integrator: '<Root>/Integrator6' */
  memcpy(&BINS_nov2_2_dla_codera10_X.Integrator6_CSTATE[0],
         &BINS_nov2_2_dla_codera10_P.C0[0], 9U * sizeof(real_T));
}

/* Model terminate function */
void BINS_nov2_2_dla_codera10_terminate(void)
{
  /* (no terminate code required) */
}
