/*
 * BINS_nov2_2_dla_codera10.h
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

#ifndef RTW_HEADER_BINS_nov2_2_dla_codera10_h_
#define RTW_HEADER_BINS_nov2_2_dla_codera10_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef BINS_nov2_2_dla_codera10_COMMON_INCLUDES_
# define BINS_nov2_2_dla_codera10_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* BINS_nov2_2_dla_codera10_COMMON_INCLUDES_ */

#include "BINS_nov2_2_dla_codera10_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetBlkStateChangeFlag
# define rtmGetBlkStateChangeFlag(rtm) ((rtm)->ModelData.blkStateChange)
#endif

#ifndef rtmSetBlkStateChangeFlag
# define rtmSetBlkStateChangeFlag(rtm, val) ((rtm)->ModelData.blkStateChange = (val))
#endif

#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->ModelData.contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->ModelData.contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->ModelData.contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->ModelData.contStates = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->ModelData.derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->ModelData.derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->ModelData.intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->ModelData.intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->ModelData.odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->ModelData.odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->ModelData.odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->ModelData.odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->ModelData.periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->ModelData.periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->ModelData.periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->ModelData.periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->ModelData.zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->ModelData.zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->ModelData.derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->ModelData.derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T Integrator8[3];               /* '<Root>/Integrator8' */
  real_T Divide3;                      /* '<Root>/Divide3' */
  real_T Integrator7[3];               /* '<Root>/Integrator7' */
  real_T TrigonometricFunction3;       /* '<Root>/Trigonometric Function3' */
  real_T MathFunction;                 /* '<Root>/Math Function' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Sum8;                         /* '<Root>/Sum8' */
  real_T Sqrt;                         /* '<Root>/Sqrt' */
  real_T Product13;                    /* '<Root>/Product13' */
  real_T Sum10;                        /* '<Root>/Sum10' */
  real_T Divide4;                      /* '<Root>/Divide4' */
  real_T Divide1;                      /* '<Root>/Divide1' */
  real_T Sum9;                         /* '<Root>/Sum9' */
  real_T Divide;                       /* '<Root>/Divide' */
  real_T TrigonometricFunction5;       /* '<Root>/Trigonometric Function5' */
  real_T Divide2;                      /* '<Root>/Divide2' */
  real_T Integrator6[9];               /* '<Root>/Integrator6' */
  real_T Selector1;                    /* '<Root>/Selector1' */
  real_T Selector2;                    /* '<Root>/Selector2' */
  real_T Divide5;                      /* '<Root>/Divide5' */
  real_T Selector3;                    /* '<Root>/Selector3' */
  real_T Selector4;                    /* '<Root>/Selector4' */
  real_T Divide6;                      /* '<Root>/Divide6' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T TrigonometricFunction4;       /* '<Root>/Trigonometric Function4' */
  real_T Gain6;                        /* '<Root>/Gain6' */
  real_T TrigonometricFunction6;       /* '<Root>/Trigonometric Function6' */
  real_T Gain7;                        /* '<Root>/Gain7' */
  real_T TrigonometricFunction7;       /* '<Root>/Trigonometric Function7' */
  real_T Gain8;                        /* '<Root>/Gain8' */
  real_T Gain9;                        /* '<Root>/Gain9' */
  real_T VectorConcatenate[3];         /* '<Root>/Vector Concatenate' */
  real_T VectorConcatenate1[3];        /* '<Root>/Vector Concatenate1' */
  real_T VectorConcatenate2[3];        /* '<Root>/Vector Concatenate2' */
  real_T MatrixConcatenate[9];         /* '<Root>/Matrix Concatenate' */
  real_T Sum2;                         /* '<Root>/Sum2' */
  real_T TrigonometricFunction;        /* '<Root>/Trigonometric Function' */
  real_T TrigonometricFunction1;       /* '<Root>/Trigonometric Function1' */
  real_T VectorConcatenate3[3];        /* '<Root>/Vector Concatenate3' */
  real_T Sum3;                         /* '<Root>/Sum3' */
  real_T TrigonometricFunction2;       /* '<Root>/Trigonometric Function2' */
  real_T VectorConcatenate4[3];        /* '<Root>/Vector Concatenate4' */
  real_T Sum4;                         /* '<Root>/Sum4' */
  real_T VectorConcatenate5[3];        /* '<Root>/Vector Concatenate5' */
  real_T MatrixConcatenate1[9];        /* '<Root>/Matrix Concatenate1' */
  real_T TmpSignalConversionAtpr1Inport2[3];
  real_T pr1[3];                       /* '<Root>/pr1' */
  real_T Sum14[3];                     /* '<Root>/Sum14' */
  real_T Product10;                    /* '<Root>/Product10' */
  real_T Product11;                    /* '<Root>/Product11' */
  real_T Product12;                    /* '<Root>/Product12' */
  real_T Product3;                     /* '<Root>/Product3' */
  real_T Product4[9];                  /* '<Root>/Product4' */
  real_T Product5[9];                  /* '<Root>/Product5' */
  real_T Product8;                     /* '<Root>/Product8' */
  real_T Product9;                     /* '<Root>/Product9' */
  real_T Selector;                     /* '<Root>/Selector' */
  real_T Sum13[9];                     /* '<Root>/Sum13' */
  real_T TmpSignalConversionAtprInport2[3];
  real_T pr[3];                        /* '<Root>/pr' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T Sum6;                         /* '<Root>/Sum6' */
  real_T Sum7;                         /* '<Root>/Sum7' */
  real_T Sum16[3];                     /* '<Root>/Sum16' */
  real_T TrigonometricFunction10;      /* '<Root>/Trigonometric Function10' */
  real_T TrigonometricFunction8;       /* '<Root>/Trigonometric Function8' */
  real_T TrigonometricFunction9;       /* '<Root>/Trigonometric Function9' */
  real_T Gain10;                       /* '<Root>/Gain10' */
} B_BINS_nov2_2_dla_codera10_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  struct {
    void *LoggedData;
  } Scope9_PWORK;                      /* '<Root>/Scope9' */
} DW_BINS_nov2_2_dla_codera10_T;

/* Continuous states (auto storage) */
typedef struct {
  real_T Integrator8_CSTATE[3];        /* '<Root>/Integrator8' */
  real_T Integrator7_CSTATE[3];        /* '<Root>/Integrator7' */
  real_T Integrator6_CSTATE[9];        /* '<Root>/Integrator6' */
} X_BINS_nov2_2_dla_codera10_T;

/* State derivatives (auto storage) */
typedef struct {
  real_T Integrator8_CSTATE[3];        /* '<Root>/Integrator8' */
  real_T Integrator7_CSTATE[3];        /* '<Root>/Integrator7' */
  real_T Integrator6_CSTATE[9];        /* '<Root>/Integrator6' */
} XDot_BINS_nov2_2_dla_codera10_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator8_CSTATE[3];     /* '<Root>/Integrator8' */
  boolean_T Integrator7_CSTATE[3];     /* '<Root>/Integrator7' */
  boolean_T Integrator6_CSTATE[9];     /* '<Root>/Integrator6' */
} XDis_BINS_nov2_2_dla_codera10_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (auto storage) */
struct P_BINS_nov2_2_dla_codera10_T_ {
  real_T C0[9];                        /* Variable: C0
                                        * Referenced by: '<Root>/Integrator6'
                                        */
  real_T Uz;                           /* Variable: Uz
                                        * Referenced by:
                                        *   '<Root>/Constant4'
                                        *   '<Root>/Constant5'
                                        *   '<Root>/Constant6'
                                        *   '<Root>/Gain7'
                                        *   '<Root>/Gain8'
                                        */
  real_T a;                            /* Variable: a
                                        * Referenced by:
                                        *   '<Root>/Constant7'
                                        *   '<Root>/Constant9'
                                        */
  real_T e2;                           /* Variable: e2
                                        * Referenced by:
                                        *   '<Root>/Constant9'
                                        *   '<Root>/Gain4'
                                        */
  real_T g;                            /* Variable: g
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Integrator8_IC[3];            /* Expression: [Ve0;Vn0;Vv0]
                                        * Referenced by: '<Root>/Integrator8'
                                        */
  real_T Constant8_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant8'
                                        */
  real_T Integrator7_IC[3];            /* Expression: [lamda0;fi0;H0]
                                        * Referenced by: '<Root>/Integrator7'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Gain2_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T Gain9_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain9'
                                        */
  real_T g2_Value[3];                  /* Expression: [0;0;-g]
                                        * Referenced by: '<Root>/g2'
                                        */
  real_T Gain10_Gain;                  /* Expression: -1
                                        * Referenced by: '<Root>/Gain10'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_BINS_nov2_2_dla_coder_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * ModelData:
   * The following substructure contains information regarding
   * the data used in the model.
   */
  struct {
    X_BINS_nov2_2_dla_codera10_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T blkStateChange;
    real_T odeY[15];
    real_T odeF[3][15];
    ODE3_IntgData intgData;
  } ModelData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
extern P_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_P;

/* Block signals (auto storage) */
extern B_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_B;

/* Continuous states (auto storage) */
extern X_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_X;

/* Block states (auto storage) */
extern DW_BINS_nov2_2_dla_codera10_T BINS_nov2_2_dla_codera10_DW;

/* Model entry point functions */
extern void BINS_nov2_2_dla_codera10_initialize(void);
extern void BINS_nov2_2_dla_codera10_step(void);
extern void BINS_nov2_2_dla_codera10_terminate(void);

/* Real-time Model object */
extern RT_MODEL_BINS_nov2_2_dla_code_T *const BINS_nov2_2_dla_codera10_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BINS_nov2_2_dla_codera10'
 */
#endif                                 /* RTW_HEADER_BINS_nov2_2_dla_codera10_h_ */
