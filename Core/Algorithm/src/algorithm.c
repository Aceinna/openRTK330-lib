/******************************************************************************
 * @file algorithm.c
 * @brief Top level algorithm configurations and functions.
 * All top-level algorithm configurations and functions are here, including
 * algorithm state, algorithm configurations, algorithm input and output.
 * @author Dong Xiaoguang
 * @date 2019.05.09
 * @version V1.0.0
 *-----------------------------------------------------------------------------
 * Change History
 * <Date>     | <Version> | <Author>       | <Description>
 * ----------------------------------------------------------------------------
 * 2019.05.09 | v1.0.0    | Dong Xiaoguang | Create file
 * ----------------------------------------------------------------------------
******************************************************************************/

#include <math.h>
#include <string.h>

#include "TransformationMath.h"
#include "QuaternionMath.h"
#include "MatrixMath.h"
#include "VectorMath.h"

#include "platformAPI.h"
#include "algorithm.h"
#include "algorithmAPI.h"
#include "TimingVars.h"
#include "buffer.h"


AlgorithmStruct gAlgorithm;
AlgoStatus      gAlgoStatus;

KalmanFilterStruct      gKalmanFilter;
EKF_InputDataStruct     gEKFInput;
EKF_OutputDataStruct    gEKFOutput;
ImuStatsStruct          gImuStats;

static real accumulatedAccelVector[3];
static real accumulatedGyroVector[3];
// static real accumulatedMagVector[3];
static real averagedAccelVector[3];
static real averagedGyroVector[3];
// static real averagedMagVector[3];

#define  INIT_P_Q    1.0e-5;
#define  INIT_P_WB   1.0e-5;
#define  INIT_P_INS  1.0e-3;

// Declare the limits
#define LIMIT_P                     500.0

#define LIMIT_ANG_ERROR_ROLL        0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_PITCH       0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_YAW         0.043633231299858    //(2.5 * ONE_DEGREE_IN_RAD)
#define LIMIT_YAW_RATE_SQ           0.001903858873667   // ( 2.5 [ deg/sec ] * DEG_TO_RAD )^2

#define LIMIT_BIAS_RATE_UPDATE_AHRS      5.0e-3
#define LIMIT_BIAS_RATE_UPDATE_INS       5.0e-4

#define LIMIT_MIN_GPS_VELOCITY_HEADING  0.45        //0.45 m/s ~= 1.0 mph
#define RELIABLE_GPS_VELOCITY_HEADING   1.0         // velocity of 1.0m/s should provide reliable GNSS heading

#define LIMIT_OBS_JACOBIAN_DENOM    1e-3;

// The following times are compared against ITOW (units in [msec])
#define LIMIT_MAX_GPS_DROP_TIME                     300      // [sec]
#define LIMIT_RELIABLE_DR_TIME                      10      // [sec]
#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000   // 60000 [ msec ] = 60 [ sec ]
#define LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID  120000  // 120sec, heading drifts much slower than pos
#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

// The following is compared against a counter (in units of the calling frequency of the EKF)
#define LIMIT_FREE_INTEGRATION_CNTR                 60   // 60 [ sec ]

#define LIMIT_QUASI_STATIC_STARTUP_RATE             0.087266462599716    // (5.0 * ONE_DEGREE_IN_RAD)

//=============================================================================
// Filter variables (Third-Order BWF w/ default 5 Hz Cutoff)
#define FILTER_ORDER 3

#define CURRENT 0
#define PASTx1  1
#define PASTx2  2
#define PASTx3  3
/* Replace this with a fuction that will compute the coefficients so the
 * input is the cutoff frequency in Hertz
 */
#define  NO_LPF              0
#define  TWO_HZ_LPF          1
#define  FIVE_HZ_LPF         2
#define  TEN_HZ_LPF          3
#define  TWENTY_HZ_LPF       4
#define  TWENTY_FIVE_HZ_LPF  5
#define  N_LPF               6

#define SAMPLES_FOR_STATS    20     /* 20 samples can give a relative good estimate of var
                                     * This value should not be below FILTER_ORDER.
                                     */

 /******************************************************************************
  * @brief Get filter coefficients of a 3rd Butterworth low-pass filter.
  * For now only a few specific cut-off frequencies are supported.
  * TRACE:
  * @param [in] lpfType   Low-pass filter cut-off frequency.
  * @param [in] callingFreq   Sampling frequency, only 100Hz and 200Hz are supported.
  * @param [out] b    coefficients of the numerator of the filter.
  * @param [out] a    coefficients of the denominator of the filter.
  * @retval None.
 ******************************************************************************/
/* Set the accelerometer filter coefficients, which are used to filter the
 * accelerometer readings prior to determining the setting of the linear-
 * acceleration switch and computing the roll and pitch from accelerometer
 * readings.
 */
static void _PopulateFilterCoefficients(uint8_t lpfType, uint8_t callingFreq, real *b, real *a)
{
    switch (lpfType)
    {
    case NO_LPF:
        b[0] = (real)(1.0);
        b[1] = (real)(0.0);
        b[2] = (real)(0.0);
        b[3] = (real)(0.0);

        a[0] = (real)(0.0);
        a[1] = (real)(0.0);
        a[2] = (real)(0.0);
        a[3] = (real)(0.0);
        break;
    case TWO_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(2.19606211225382e-4);
            b[1] = (real)(6.58818633676145e-4);
            b[2] = (real)(6.58818633676145e-4);
            b[3] = (real)(2.19606211225382e-4);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.748835809214676);
            a[2] = (real)(2.528231219142559);
            a[3] = (real)(-0.777638560238080);
        }
        else
        {
            b[0] = (real)(2.91464944656705e-5);
            b[1] = (real)(8.74394833970116e-5);
            b[2] = (real)(8.74394833970116e-5);
            b[3] = (real)(2.91464944656705e-5);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.874356892677485);
            a[2] = (real)(2.756483195225695);
            a[3] = (real)(-0.881893130592486);
        }
        break;
    case FIVE_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.002898194633721);
            b[1] = (real)(0.008694583901164);
            b[2] = (real)(0.008694583901164);
            b[3] = (real)(0.002898194633721);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.374094743709352);
            a[2] = (real)(1.929355669091215);
            a[3] = (real)(-0.532075368312092);
        }
        else
        {
            b[0] = (real)(0.000416546139076);
            b[1] = (real)(0.001249638417227);
            b[2] = (real)(0.001249638417227);
            b[3] = (real)(0.000416546139076);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.686157396548143);
            a[2] = (real)(2.419655110966473);
            a[3] = (real)(-0.730165345305723);
        }
        break;
    case TWENTY_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            // [B,A] = butter(3,20/(100/2))
            b[0] = (real)(0.098531160923927);
            b[1] = (real)(0.295593482771781);
            b[2] = (real)(0.295593482771781);
            b[3] = (real)(0.098531160923927);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-0.577240524806303);
            a[2] = (real)(0.421787048689562);
            a[3] = (real)(-0.056297236491843);
        }
        else
        {
            // [B,A] = butter(3,20/(200/2))
            b[0] = (real)(0.018098933007514);
            b[1] = (real)(0.054296799022543);
            b[2] = (real)(0.054296799022543);
            b[3] = (real)(0.018098933007514);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-1.760041880343169);
            a[2] = (real)(1.182893262037831);
            a[3] = (real)(-0.278059917634546);
        }
        break;
    case TWENTY_FIVE_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.166666666666667);
            b[1] = (real)(0.500000000000000);
            b[2] = (real)(0.500000000000000);
            b[3] = (real)(0.166666666666667);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-0.000000000000000);
            a[2] = (real)(0.333333333333333);
            a[3] = (real)(-0.000000000000000);
        }
        else
        {
            b[0] = (real)(0.031689343849711);
            b[1] = (real)(0.095068031549133);
            b[2] = (real)(0.095068031549133);
            b[3] = (real)(0.031689343849711);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-1.459029062228061);
            a[2] = (real)(0.910369000290069);
            a[3] = (real)(-0.197825187264319);
        }
        break;
    case TEN_HZ_LPF:
    default:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.0180989330075144);
            b[1] = (real)(0.0542967990225433);
            b[2] = (real)(0.0542967990225433);
            b[3] = (real)(0.0180989330075144);

            a[0] = (real)(1.0000000000000000);
            a[1] = (real)(-1.7600418803431690);
            a[2] = (real)(1.1828932620378310);
            a[3] = (real)(-0.2780599176345460);
        }
        else
        {
            b[0] = (real)(0.002898194633721);
            b[1] = (real)(0.008694583901164);
            b[2] = (real)(0.008694583901164);
            b[3] = (real)(0.002898194633721);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.374094743709352);
            a[2] = (real)(1.929355669091215);
            a[3] = (real)(-0.532075368312092);
        }
        break;
    }
}


/******************************************************************************
 * @brief Process input data through a low-pass Butterworth filter.
 * TRACE:
 * @param [in]  in      Input data
 * @param [in]  bfIn    Input data buffer
 * @param [in]  bfOut   Output data buffer
 * @param [in]  b       Numerator coef of the filter 
 * @param [in]  a       Denominator coef of the filter
 *
 * @param [out] filtered    Filtered IMU data.
 * @retval None.
******************************************************************************/
static void LowPassFilter(real *in, Buffer *bfIn, Buffer *bfOut, real *b, real *a, real *filtered)
{
    // Fill the buffer with the first input.
    if (!bfIn->full)
    {
        bfPut(bfIn, in);
        filtered[0] = in[0];
        filtered[1] = in[1];
        filtered[2] = in[2];
        return;
    }

    /* Filter accelerometer readings (Note: a[0] =  1.0 and the filter coefficients are symmetric)
     * y = filtered output; x = raw input;
     * a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
     * b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
     * b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
     */
    int i;
    real tmpIn[3];
    real tmpOut[3];
    filtered[0] = b[CURRENT] * in[0];
    filtered[1] = b[CURRENT] * in[1];
    filtered[2] = b[CURRENT] * in[2];
    for (i = PASTx1; i <= PASTx3; i++)
    {
        bfGet(bfIn, tmpIn, i-1);
        bfGet(bfOut, tmpOut, i-1);
        filtered[0] += b[i] * tmpIn[0] - a[i] * tmpOut[0];
        filtered[1] += b[i] * tmpIn[1] - a[i] * tmpOut[1];
        filtered[2] += b[i] * tmpIn[2] - a[i] * tmpOut[2];
    }

    // New data into buffer 
    bfPut(bfIn, in);
}

/******************************************************************************
 * @brief Compute mean and var of the input data.
 * Calculate mena and var of the latest n samples.
 * TRACE:
 * @param [in] bf       The buffer to hold the latest n samples.
 * @param [in] latest   The latest sample.
 * @param [in/out] mean Mean of samples already in buffer is used as input, and
 *                      mean of latest samples (remove oldest and include latest)
 *                      is returned as output.
 * @param [in/out] var  Var of samples already in buffer is used as input, and
 *                      var of latest samples (remove oldest and include latest)
 *                      is returned as output.
 * @retval None.
******************************************************************************/
static void ComputeStats(Buffer *bf, real *latest, real *mean, real *var)
{
    real lastMean[3];
    real lastVar[3];
    lastMean[0] = mean[0];
    lastMean[1] = mean[1];
    lastMean[2] = mean[2];
    lastVar[0] = var[0];
    lastVar[1] = var[1];
    lastVar[2] = var[2];
    if (bf->full)
    {
        /* when buffer is full, the var and mean are computed from
         * all data in the buffer. From now on, the var and mean
         * should be computed by removing the oldest data and including
         * the latest data.
         */
         // Get the oldest data which will be removed by following bfPut
        real oldest[3];
        bfGet(bf, oldest, bf->num - 1);
        // put this accel into buffer
        bfPut(bf, latest);
        // mean(n+1) = ( mean(n) * n - x(1) + x(n+1) ) / n
        mean[0] += (latest[0] - oldest[0]) / (real)(bf->num);
        mean[1] += (latest[1] - oldest[1]) / (real)(bf->num);
        mean[2] += (latest[2] - oldest[2]) / (real)(bf->num);

        // naive var calculation is adopted because recursive method is numerically instable
        real tmpVar[3];
        tmpVar[0] = vecVar(&(bf->d[0]), mean[0], bf->num);
        tmpVar[1] = vecVar(&(bf->d[bf->n]), mean[1], bf->num);
        tmpVar[2] = vecVar(&(bf->d[2 * bf->n]), mean[2], bf->num);
        // make var estimation smooth
        real k = 0.96f;
        int i;
        for (i = 0; i < 3; i++)
        {
            if (tmpVar[i] >= var[i])
            {
                var[i] = tmpVar[i];
            }
            else
            {
                var[i] = k * var[i] + (1.0f - k)*tmpVar[i];
            }
        }
    }
    else
    {
        // put this accel into buffer
        bfPut(bf, latest);
        /* Recursivly include new accel. The data num used to compute mean and
         * var are increasing.
         */
         // mean(n+1) = mean(n) *n / (n+1) + x(n+1) / (n+1)
        mean[0] = lastMean[0] + (latest[0] - lastMean[0]) / (real)(bf->num);
        mean[1] = lastMean[1] + (latest[1] - lastMean[1]) / (real)(bf->num);
        mean[2] = lastMean[2] + (latest[2] - lastMean[2]) / (real)(bf->num);
        var[0] = lastVar[0] + lastMean[0] * lastMean[0] - mean[0] * mean[0] +
            (latest[0] * latest[0] - lastVar[0] - lastMean[0] * lastMean[0]) / (real)(bf->num);
        var[1] = lastVar[1] + lastMean[1] * lastMean[1] - mean[1] * mean[1] +
            (latest[1] * latest[1] - lastVar[1] - lastMean[1] * lastMean[1]) / (real)(bf->num);
        var[2] = lastVar[2] + lastMean[2] * lastMean[2] - mean[2] * mean[2] +
            (latest[2] * latest[2] - lastVar[2] - lastMean[2] * lastMean[2]) / (real)(bf->num);
    }
}


void InitializeAlgorithmStruct(uint8_t callingFreq)
{
    gAlgorithm.Behavior.bit.freeIntegrate = FALSE;
    // The calling frequency drives the execution rate of the EKF and dictates
    //   the algorithm constants
    if(callingFreq == 0){
        // IMU case
        callingFreq = FREQ_200_HZ;
    }
    gAlgorithm.callingFreq = callingFreq;

    // Set dt based on the calling frequency of the EKF
    if (gAlgorithm.callingFreq == FREQ_100_HZ)
    {
        gAlgorithm.dt = (real)(0.01);
        gAlgorithm.dITOW = 10;
    }
    else if (gAlgorithm.callingFreq == FREQ_200_HZ)
    {
        gAlgorithm.dt = (real)(0.005);
        gAlgorithm.dITOW = 5;
    }
    else
    {
        while (1);
    }

    // Set up other timing variables
    gAlgorithm.dtOverTwo = (real)(0.5) * gAlgorithm.dt;
    gAlgorithm.dtSquared = gAlgorithm.dt * gAlgorithm.dt;
    gAlgorithm.sqrtDt = sqrtf(gAlgorithm.dt);

    // Set the algorithm duration periods
    gAlgorithm.Duration.Stabilize_System = (uint32_t)(gAlgorithm.callingFreq * STABILIZE_SYSTEM_DURATION);
    gAlgorithm.Duration.Initialize_Attitude = (uint32_t)(gAlgorithm.callingFreq * INITIALIZE_ATTITUDE_DURATION);
    gAlgorithm.Duration.High_Gain_AHRS = (uint32_t)(gAlgorithm.callingFreq * HIGH_GAIN_AHRS_DURATION);
    gAlgorithm.Duration.Low_Gain_AHRS = (uint32_t)(gAlgorithm.callingFreq * LOW_GAIN_AHRS_DURATION);

    // Set the initial state of the EKF
    gAlgorithm.state = STABILIZE_SYSTEM;
    gAlgorithm.stateTimer = gAlgorithm.Duration.Stabilize_System;

    // Turn-switch variable
    gAlgorithm.filteredYawRate = (real)0.0;

    // Tell the algorithm to apply the declination correction to the heading
    //  (at startup in AHRS, do not apply.  After INS becomes healthy, apply,
    //  even in AHRS, but this condition shouldn't last forever.  Question:
    //  how long to keep this set TRUE after GPS in invalid?)
    gAlgorithm.applyDeclFlag = FALSE;

    gAlgorithm.insFirstTime = TRUE;
    gAlgorithm.headingIni = HEADING_UNINITIALIZED;

    //gAlgorithm.magAlignUnderway = FALSE; // Set and reset in mag-align code

    // Increment at 100 Hz in EKF_Algorithm; sync with GPS itow when valid.
    gAlgorithm.itow = 0;

    // Limit is compared to ITOW.  Time must be in [msec].
    gAlgorithm.Limit.maxGpsDropTime = LIMIT_MAX_GPS_DROP_TIME * 1000;
    gAlgorithm.Limit.maxReliableDRTime = LIMIT_RELIABLE_DR_TIME * 1000;

    // Limit is compared to count (incremented upon loop through
    //   taskDataAcquisition).  Time must be in [count] based on ODR.
    gAlgorithm.Limit.Free_Integration_Cntr = gAlgorithm.callingFreq * LIMIT_FREE_INTEGRATION_CNTR;

    // Linear acceleration switch limits (level and time)
    gAlgorithm.Limit.accelSwitch = (real)(0.012);   // [g]
    gAlgorithm.Limit.linAccelSwitchDelay = (uint32_t)(2.0 * gAlgorithm.callingFreq);

    // Innovation error limits for EKF states
    gAlgorithm.Limit.Innov.positionError = (real)270.0;
    gAlgorithm.Limit.Innov.velocityError = (real)27.0;
    gAlgorithm.Limit.Innov.attitudeError = (real)SIX_DEGREES_IN_RAD;
    
    // Five-hertz LPF (corresponding integer value found in PredictFunctions.c)
    // Replace with a function that computes the coefficients.  Value (below) will
    //   then be the cutoff frequency.
    gAlgorithm.linAccelLPFType = 1;

    // Uing raw accel to detect linear acceleration has lower failure rate in small
    //  and smooth linear acceleration. But on some platform, there is large vibration,
    //  uing raw accel to detect linear acceleration will always detect linear accel.
    gAlgorithm.useRawAccToDetectLinAccel = TRUE;

    // Set the turn-switch threshold to a default value in [deg/sec]
    gAlgorithm.turnSwitchThreshold = 2.0;

	// default lever arm and point of interest
	gAlgorithm.leverArmB[X_AXIS] = 0.0;
	gAlgorithm.leverArmB[Y_AXIS] = 0.0;
	gAlgorithm.leverArmB[Z_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[X_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[Y_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[Z_AXIS] = 0.0;

    // For most vehicles, the velocity is always along the body x axis
    gAlgorithm.velocityAlwaysAlongBodyX = TRUE;

    // get IMU specifications
    gAlgorithm.imuSpec.arw = (real)ARW_300ZA;
    gAlgorithm.imuSpec.sigmaW = (real)(1.25 * ARW_300ZA / sqrt(1.0/RW_ODR));
    gAlgorithm.imuSpec.biW = (real)BIW_300ZA;
    gAlgorithm.imuSpec.maxBiasW = (real)MAX_BW;
    gAlgorithm.imuSpec.vrw = (real)VRW_300ZA;
    gAlgorithm.imuSpec.sigmaA = (real)(1.25 * VRW_300ZA / sqrt(1.0/RW_ODR));
    gAlgorithm.imuSpec.biA = (real)BIA_300ZA;
    gAlgorithm.imuSpec.maxBiasA = (real)MAX_BA;

    // default noise level multiplier for static detection
    gAlgorithm.staticDetectParam.staticVarGyro = (real)(gAlgorithm.imuSpec.sigmaW * gAlgorithm.imuSpec.sigmaW);
    gAlgorithm.staticDetectParam.staticVarAccel = (real)(gAlgorithm.imuSpec.sigmaA * gAlgorithm.imuSpec.sigmaA);
    gAlgorithm.staticDetectParam.maxGyroBias = MAX_BW;
    gAlgorithm.staticDetectParam.staticGnssVel = 0.2;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[0] = 4.0;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[1] = 4.0;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[2] = 1.0;
}

void GetAlgoStatus(AlgoStatus *algoStatus)
{
    algoStatus->all = gAlgoStatus.all;
}


void setAlgorithmExeFreq(int freq)
{
    gAlgorithm.callingFreq = freq;

}

void updateAlgorithmTimings(int corr, uint32_t tmrVal )
{
    // Set the counter to a value that corresponds to seconds after TIM2 is
    //   called and just after the sensors are read.
    gAlgorithm.counter = (uint16_t)( 1.25e-3 * ( ( corr + (uint16_t)(1.334489891239070E-05 * tmrVal) ) << 16 ) );
    // Increment the timer output value (FIXME: is this in the right spot?
    //   Should it be in the algorithm if-statement below?)
    gAlgorithm.timer = gAlgorithm.timer + gAlgorithm.dITOW;
}

uint32_t getAlgorithmTimer()
{
    return gAlgorithm.timer;
}

uint16_t getAlgorithmCounter()
{
    return gAlgorithm.counter;
}

uint16_t getAlgorithmFrequency()
{
    return gAlgorithm.callingFreq;
}

uint32_t getAlgorithmITOW()
{
    return gAlgorithm.itow;
}

void setLeverArm( real leverArmBx, real leverArmBy, real leverArmBz )
{
    gAlgorithm.leverArmB[0] = leverArmBx;
    gAlgorithm.leverArmB[1] = leverArmBy;
    gAlgorithm.leverArmB[2] = leverArmBz;
}

void setPointOfInterest( real poiBx, real poiBy, real poiBz )
{
    gAlgorithm.pointOfInterestB[0] = poiBx;
    gAlgorithm.pointOfInterestB[1] = poiBy;
    gAlgorithm.pointOfInterestB[2] = poiBz;
}


/* predict function.c */



//
static void _DropToHighGainAHRS(void)
{
    gAlgorithm.state      = HIGH_GAIN_AHRS;
    gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;

    gAlgoStatus.bit.highGain              = TRUE;
    gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;

    // Reset flag in case drop is from INS
    gAlgorithm.insFirstTime = TRUE;
}


/******************************************************************************
* @name: _AccumulateFieldVectors Update the running sum of the acceleration and
* @brief magnetic field vectors (the accumulation variables are 64-bits long).
*        The total is averaged to form the system ICs.
*
* @brief called in main.cpp and processUpdateAlgorithm() in algorithm.cpp
* TRACE:
*
* @param N/A
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
static BOOL _AccumulateFieldVectors(void)
{
    // Accumulate the acceleration vector readings (accels in g's)
    accumulatedAccelVector[X_AXIS] += (real)gEKFInput.accel_B[X_AXIS];
    accumulatedAccelVector[Y_AXIS] += (real)gEKFInput.accel_B[Y_AXIS];
    accumulatedAccelVector[Z_AXIS] += (real)gEKFInput.accel_B[Z_AXIS];

    // Accumulate the gyroscope vector readings (accels in rad/s)
    accumulatedGyroVector[X_AXIS] += gEKFInput.angRate_B[X_AXIS];
    accumulatedGyroVector[Y_AXIS] += gEKFInput.angRate_B[Y_AXIS];
    accumulatedGyroVector[Z_AXIS] += gEKFInput.angRate_B[Z_AXIS];

    // Accumulate the magnetic-field vector readings (or set to zero if the
    //   product does not have magnetometers)
    // if (magUsedInAlgorithm() )
    // {
    //     accumulatedMagVector[X_AXIS] += (real)gEKFInput.magField_B[X_AXIS];
    //     accumulatedMagVector[Y_AXIS] += (real)gEKFInput.magField_B[Y_AXIS];
    //     accumulatedMagVector[Z_AXIS] += (real)gEKFInput.magField_B[Z_AXIS];
    // } else {
        // accumulatedMagVector[X_AXIS] = (real)0.0;
        // accumulatedMagVector[Y_AXIS] = (real)0.0;
        // accumulatedMagVector[Z_AXIS] = (real)0.0;
    // }

//    return(magUsedInAlgorithm());
    return 1;
}


/******************************************************************************
* @name: _AverageFieldVectors Average the accumulated field vectors by shifting
*        the sum to the right
*        Note: the number of samples that are summed must be a multiple of 2:
*       Number of points accumulated, N = 2^bitsToShift
*
* TRACE:
*
* @param [in] bitsToShift
* @brief global data structure changes:
* Input:  gKalmanFilter.AccumulatedAccelVector
*         gKalmanFilter.

* Output: gKalmanFilter.AveragedAccelVector
*         gKalmanFilter.AveragedMagVector
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
static BOOL _AverageFieldVectors(uint16_t pointsToAverage)
{
    real mult = (real)(1.0 / (real)pointsToAverage);

    // Average the accumulated acceleration vector
    averagedAccelVector[X_AXIS] = accumulatedAccelVector[X_AXIS] * mult;
    averagedAccelVector[Y_AXIS] = accumulatedAccelVector[Y_AXIS] * mult;
    averagedAccelVector[Z_AXIS] = accumulatedAccelVector[Z_AXIS] * mult;

    // Average the accumulated angular rate vector
    averagedGyroVector[X_AXIS] = accumulatedGyroVector[X_AXIS] * mult;
    averagedGyroVector[Y_AXIS] = accumulatedGyroVector[Y_AXIS] * mult;
    averagedGyroVector[Z_AXIS] = accumulatedGyroVector[Z_AXIS] * mult;

    /* Average the accumulated magnetic-field vector (or set to zero if
     * magnetometer is not in use.)
     */
    // if (magUsedInAlgorithm())
    // {
    //     averagedMagVector[X_AXIS] = accumulatedMagVector[X_AXIS] * mult;
    //     averagedMagVector[Y_AXIS] = accumulatedMagVector[Y_AXIS] * mult;
    //     averagedMagVector[Z_AXIS] = accumulatedMagVector[Z_AXIS] * mult;
    // }
    // else
    // {
    //     averagedMagVector[X_AXIS] = (real)0.0;
    //     averagedMagVector[Y_AXIS] = (real)0.0;
    //     averagedMagVector[Z_AXIS] = (real)0.0;
    // }

    // return (magUsedInAlgorithm());
    return 1;
}


//
static void _ResetAlgorithm(void)
{
    int elemNum;

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    float s1 = 1.0f;
    float s2 = 0.01f; // s1=0.01, s2=0.1, HG=160sec can make ini_pitch=40 stable ,DXG
    gKalmanFilter.P[STATE_Q0][STATE_Q0] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = s1 * (real)INIT_P_Q;

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = s2 * (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = s2 * (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = s2 * (real)INIT_P_WB;


    // Reset the rate-bias and corrected-rate variables (in the body-frame)
    for (elemNum = X_AXIS; elemNum <= Z_AXIS; elemNum++)
    {
        /* Initialize gyro rate bias with averaged gyro output
         * If averaged gyro output is above 1deg/s, this means there should be rotation
         * during initializatoin and it cannot be considered as bias. A default zero bias
         * is used instead.
         */
        if (fabs(averagedGyroVector[elemNum]) < ONE_DEGREE_IN_RAD)
        {
            gKalmanFilter.rateBias_B[elemNum] = (real)averagedGyroVector[elemNum];
        }
        else
        {
            gKalmanFilter.rateBias_B[elemNum] = 0.0;
        }

        gKalmanFilter.correctedRate_B[elemNum] = (real)0.0;
    }


    GenerateProcessJacobian();
    GenerateProcessCovariance();
}

// StabilizeSystem: Run for a prescribed period to let the sensors settle.
void StabilizeSystem(void)
{
    // Decrement timer (initial value is set based on the calling frequency of
    //   the EKF)
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    /* Upon timeout prepare for transition to the next stage of the EKF
     * (initialization) by resetting the state and state-timer and
     * initializing the accumulation vectors.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To ini att. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To ini att. ");
        DebugPrintInt("", gEKFInput.itow);
        DebugPrintEndline();
#endif
        #endif
        // Set new state and timer
        gAlgorithm.state      = INITIALIZE_ATTITUDE;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;

        // Initialize the vectors
        accumulatedAccelVector[X_AXIS] = (real)0.0;
        accumulatedAccelVector[Y_AXIS] = (real)0.0;
        accumulatedAccelVector[Z_AXIS] = (real)0.0;

        accumulatedGyroVector[X_AXIS] = (real)0.0;
        accumulatedGyroVector[Y_AXIS] = (real)0.0;
        accumulatedGyroVector[Z_AXIS] = (real)0.0;

        // accumulatedMagVector[X_AXIS] = (real)0.0;
        // accumulatedMagVector[Y_AXIS] = (real)0.0;
        // accumulatedMagVector[Z_AXIS] = (real)0.0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        TimingVars_DiagnosticMsg("Transitioning to initialization mode");
#endif
    }

    // Set the bit to indicate initialization
    gAlgoStatus.bit.algorithmInit  = TRUE;
}


/* InitializeAttitude: Initialize the algorithm by collecting sensor data for
 * a prescribed period and averaging it.
 */
void InitializeAttitude(void)
{
    // Decrement timer
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    /* Sum the acceleration and magnetic-field vectors (from the end of the
     * initialization stage)
     */
    _AccumulateFieldVectors();

    /* Quasi-static check: check for motion over threshold. If detected, reset
     * the accumulation variables and restart initialization phase.
     */
    if ((fabs(gEKFInput.angRate_B[X_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInput.angRate_B[Y_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInput.angRate_B[Z_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE))
    {
        accumulatedAccelVector[X_AXIS] = (real)0.0;
        accumulatedAccelVector[Y_AXIS] = (real)0.0;
        accumulatedAccelVector[Z_AXIS] = (real)0.0;

        accumulatedGyroVector[X_AXIS] = (real)0.0;
        accumulatedGyroVector[Y_AXIS] = (real)0.0;
        accumulatedGyroVector[Z_AXIS] = (real)0.0;

        // accumulatedMagVector[X_AXIS] = (real)0.0;
        // accumulatedMagVector[Y_AXIS] = (real)0.0;
        // accumulatedMagVector[Z_AXIS] = (real)0.0;

        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;
    }

    /* Timeout...  Prepare for the transition to the next stage of the EKF
     * (High-Gain AHRS) then determine the system's Initial Conditions by
     * averaging the accumulated vectors.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To HG. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To HG. ");
        DebugPrintInt("", gEKFInput.itow);
        DebugPrintEndline();
#endif
        #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to high-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to high-gain VG mode");
        }
#endif

        // Set new state and timer
        gAlgorithm.state      = HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;

        /* Average acceleration and magnetic field-vectors to determine the
         * initial attitude of the system.
         */
        _AverageFieldVectors(gAlgorithm.Duration.Initialize_Attitude);

        /* Compute the measured Euler Angles and associated quaternion
         * Euler angles are computed from averaged field vectors 
         * (correct for hard/soft-iron effects)
         */
        // Unit gravity vector in the body frame
        real unitGravityVector[3] = {0.0f};
        UnitGravity( averagedAccelVector, unitGravityVector );
        // Roll and pitch
        UnitGravityToEulerAngles( unitGravityVector, gKalmanFilter.measuredEulerAngles );
        // Yaw
        // if ( magUsedInAlgorithm() )
        // {
        //     gKalmanFilter.measuredEulerAngles[YAW] = UnitGravityAndMagToYaw( unitGravityVector,
        //                                                                      averagedMagVector );
        // }
        // else
        {
            gKalmanFilter.measuredEulerAngles[YAW] = 0.0f;  // start from 0
        }

        /* Initial attitude quaternion is generated using Euler angles from
         * averaged gravity and magnetic fields. (DEBUG: This is used to
         * initialize the EKF state)
         */
        EulerAnglesToQuaternion( gKalmanFilter.measuredEulerAngles,
                                 gKalmanFilter.quaternion );
        gKalmanFilter.quaternion_Past[0] = gKalmanFilter.quaternion[0];
        gKalmanFilter.quaternion_Past[1] = gKalmanFilter.quaternion[1];
        gKalmanFilter.quaternion_Past[2] = gKalmanFilter.quaternion[2];
        gKalmanFilter.quaternion_Past[3] = gKalmanFilter.quaternion[3];
        // Euler angles from the initial measurement (DEBUG: initial output of the system)
        gKalmanFilter.eulerAngles[ROLL] = gKalmanFilter.measuredEulerAngles[ROLL];
        gKalmanFilter.eulerAngles[PITCH] = gKalmanFilter.measuredEulerAngles[PITCH];
        gKalmanFilter.eulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW];

        // Initialize the Kalman filter variables
        _ResetAlgorithm();

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;

        /// Update the system status
        gAlgoStatus.bit.algorithmInit         = FALSE;
        gAlgoStatus.bit.highGain              = TRUE;
        gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }
}


/* HG_To_LG_Transition_Test: Transition from high-gain to low-gain.  Only check
 * is that the bias isn't greater than 10 deg/sec (this is probably not a good check).
 */
void HG_To_LG_Transition_Test(void)
{
    /* Decrement timer if 'dynamicMotion' TRUE (setting FALSE will cause the
     * system to revert to high - gain mode once out of high - gain mode -- need
     * to set flag high to transition out of high - gain mode once this is done)
     */
    /* dynamic-motion flag switch from high-gain to low-gain AHRS. if not set
     * timer will not decrement the transition to LG AHRS will not occur.
     * set at system configuration or (Nav-View) interface
     */
    if (gAlgorithm.Behavior.bit.dynamicMotion) {
        gAlgorithm.stateTimer--;
    }

    /* Startup check (if the estimated bias is large the software never
     * transitions to the LG AHRS mode. NOTE: this seems incorrect, instead
     * the SW should check if the bias has converged, not if it is above a
     * threshold -- it is possible that the system could have a large bias.)
     *  However, this seems wrong too
     */
    if ((fabs(gKalmanFilter.rateBias_B[X_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Y_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Z_AXIS]) > TEN_DEGREES_IN_RAD))
    {
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;
    }

    /* Timeout...  Prepare for the transition to the next stage of the EKF
     * (Low-Gain AHRS) and populate the values in Q that do not change with
     * each iteration.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To LG. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To LG. ");
        DebugPrintInt("", gEKFInput.itow);
        DebugPrintEndline();
#endif
        #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to low-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to low-gain VG mode");
        }
#endif

        gAlgorithm.state      = LOW_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;

        gAlgoStatus.bit.highGain = FALSE;
    }
}


/* This logic is only called upon transition to INS from LG_AHRS then it is
 * not called unless the algorithm reverts back to HG_AHRS, which will
 * cause the system to pass through LG_AHRS on its way to INS.
 */
void LG_To_INS_Transition_Test(void)
{
#ifdef DISPLAY_DIAGNOSTIC_MSG
    // Display the diagnostic message once upon transition (DEBUG: Remove in firmware)
    static int oneTime = TRUE;
#endif

    if (gAlgorithm.stateTimer > 0) {
        // Stay in LG mode until timeout occurs then begin check for INS transition
        gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;
    } else {
        // Upon timeout, begin check for INS transition (remove msg in firmware)
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (oneTime) {
            TimingVars_DiagnosticMsg("Begin check for INS transition");
            oneTime = FALSE;
        }
#endif

        /* If GPS output is valid (GPS providing data with a good signal lock)
         * then transit to INS mode.
         */
        if ( gpsUsedInAlgorithm() && gEKFInput.gpsUpdate == 1 && gEKFInput.gpsFixType ) 
        {
            #ifdef INS_OFFLINE
            printf("To INS. %u\n", gEKFInput.itow);
            #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
            DebugPrintString("To INS. ");
            DebugPrintInt("", gEKFInput.itow);
            DebugPrintEndline();
#endif
            #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
            TimingVars_DiagnosticMsg("Transitioning to INS mode");
#endif

            // Transit to INS solution
            gAlgorithm.state = INS_SOLUTION;
            
            // Initialize the algorithm with GNSS position and lever arm
            InitINSFilter();

            // Set linear-acceleration switch variables
            gAlgorithm.linAccelSwitchCntr = 0;
        }
    }
}

/* INS_To_AHRS_Transition_Test:  Drop back to LG AHRS operation if...
 *   1) GPS drops out for more than 3 seconds
 *   2) magnetometer data not available AND at rest too long
 *   3) magnetic alignment being performed
 */
void INS_To_AHRS_Transition_Test(void)
{
    // Record last GPS velocity large enough to give a good heading measurement
    if (gEKFInput.rawGroundSpeed >= LIMIT_MIN_GPS_VELOCITY_HEADING)
    {
        gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gEKFInput.itow;
    }
    /* Determine the length of time it has been since the system 'moved' --
     * only linear motion considered (rotations ignored).
     */
    int32_t timeSinceRestBegan = (int32_t)gEKFInput.itow - gAlgorithm.timeOfLastSufficientGPSVelocity;
    if (timeSinceRestBegan < 0)
    {
        timeSinceRestBegan = timeSinceRestBegan + MAX_ITOW;
    }
    if (timeSinceRestBegan > LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID && gAlgorithm.headingIni != HEADING_UNINITIALIZED)
    {
        gAlgorithm.headingIni = HEADING_GNSS_LOW;
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("Rest for too long.");
        DebugPrintEndline();
#endif
    }

    // compute time since the last good GPS reading
    int32_t timeSinceLastGoodGPSReading = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastGoodGPSReading;
    if (timeSinceLastGoodGPSReading < 0) {
        timeSinceLastGoodGPSReading = timeSinceLastGoodGPSReading + MAX_ITOW;
    }

    if ( timeSinceLastGoodGPSReading > gAlgorithm.Limit.maxGpsDropTime )
    {
#ifdef INS_OFFLINE
        printf("GPS outage too long\n");
#endif // INS_OFFLINE

        // Currently in INS mode but requiring a transition to AHRS / VG
        gAlgorithm.insFirstTime = TRUE;
        gAlgorithm.headingIni = HEADING_UNINITIALIZED;

        /* The transition from INS to AHRS and back to INS does not seem to
         * generate a stable solution if we transition to LG AHRS for only 30
         * seconds.The interval needs to be longer(~1min).However, to
         * mitigate any unforseen issues with the transition, HG AHRS with the
         * nominal timing(1 min in HG, 30 seconds in LG) will be selected.
         */
        gAlgorithm.state      = LOW_GAIN_AHRS;            // HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;   // gAlgorithm.Duration.High_Gain_AHRS;

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to low-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to low-gain VG mode");
        }
#endif

        gAlgoStatus.bit.highGain              = ( gAlgorithm.state == HIGH_GAIN_AHRS );
        gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }
}


/* Dynamic motion logic:
 *   0) When dynamicMotion is FALSE, remain in high-gain AHRS (do not decrement
 *      counter in 'HG_To_LG_Transition_Test')
 *   1) If dynamicMotion is selected then proceed to other filter states upon
 *      timeout (else, stay in HG mode)
 *   2) When in LG or INS mode... if dynamicMotion is set FALSE then transition
 *      to HG AHRS
 *   3) Once dynamicMotion is reset TRUE (by user), the system should begin
 *      transition to LG AHRS as if beginning from nominal startup
 */
void DynamicMotion(void)
{
    static BOOL enterStatic = FALSE;   // should this be true or false?

    /* If dynamicMotion is FALSE then transition to high-gain AHRS. The
     * system stays in HG until dynamicMotion is set high.
     */
    if (gAlgorithm.state > HIGH_GAIN_AHRS) {
        if (gAlgorithm.Behavior.bit.dynamicMotion) {
            enterStatic = FALSE;
        } else {
            if (enterStatic == FALSE) {
                enterStatic = TRUE;
                _DropToHighGainAHRS();

#ifdef DISPLAY_DIAGNOSTIC_MSG
                /* Question: what if DM flag is set (by user) at the moment of
                 * transition?  Should the FW initialize enterStatic
                 * in HG_To_LG_Transition_Test?
                 */
                TimingVars_DiagnosticMsg("Transitioning to High-Gain AHRS -- dynamic-motion flag set");
#endif
            }
        }
    }
}



/* updatefunction.c */
uint8_t RLE_H[ROWS_IN_H][2] = { {STATE_Q0, STATE_Q3},
                                {STATE_Q0, STATE_Q3},
                                {STATE_Q0, STATE_Q3} };

// KxH is sparse with elements only in cols 6 through 9
uint8_t RLE_KxH[ROWS_IN_K][2] = { {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3} };

// Local functions
static void _TurnSwitch(void);

static real _UnwrapAttitudeError( real attitudeError );
static real _LimitValue( real value, real limit );
static BOOL _CheckForUpdateTrigger(uint8_t updateRate);

/******************************************************************************
 * @brief Initializa heading using GNSS heading.
 * If the GNSS heading is valid and the vehicle is drving forward, the GNSS
 * heading is considered valid, and the eading will be initialized to be
 * gEKFInput.trueCourse, and velocity will also be initiazlied as the
 * corresponding NED speed. After this, the quaternion (q0 and q3) and velocity
 * terms in the state covariance matrix P will be reset. Non-diagonal terms will be
 * set as 0s, and diagonal terms will be set according to estimated variance. The
 * cov(quaternion, velocity) should also be updated. But the positive-definiteness
 * is not guaranteed this way.
 * TRACE:
 * @retval TRUE if heading initialized/reinitialized, FALSE if not.
******************************************************************************/
static int InitializeHeadingFromGnss();

/******************************************************************************
 * @brief When heading is ready for initialization, the heading angle (yaw, and 
 * indeed quaternion in the Kalman filter) is initialized to match the value of
 * gEKFInput.trueCourse, and velocity will also be initiazlied as the
 * corresponding NED speed. After this, the quaternion (q0 and q3) and velocity
 * terms in the state covariance matrix P will be reset. Non-diagonal terms will be
 * set as 0s, and diagonal terms will be set according to estimated variance. The
 * cov(quaternion, velocity) should also be updated. But the positive-definiteness
 * is not guaranteed this way.
 * TRACE:
 * @retval None.
******************************************************************************/
static void InitializeEkfHeading();

// Update rates
#define  TEN_HERTZ_UPDATE          10
#define  TWENTY_HERTZ_UPDATE       20
#define  TWENTY_FIVE_HERTZ_UPDATE  25
#define  FIFTY_HERTZ_UPDATE        50
#define  ONE_HUNDRED_HERTZ_UPDATE  100

static BOOL useGpsHeading = 0;  /* When GPS velocity is above a certain threshold,
                                 * this is set to 1, and GPS heading measurement
                                 * is used, otherwise, this is set to 0 and magnetic
                                 * heading is used.
                                 */
static int runInsUpdate = 0;    /* To enable the update to be broken up into
                                 * two sequential calculations in two sucessive
                                 * 100 Hz periods.
                                 */

// Uncomment to run only AHRS-type updates
//#define ATT_UPDATE_ONLY

static void Update_GPS(void);
static void Update_PseudoMeasurement(void);
static void GenPseudoMeasCov(real *r);

// EKF_UpdateStage.m
void EKF_UpdateStage(void)
{
    /* Perform a VG/AHRS update, regardless of GPS availability or health,
     * when the state is HG AHRS or LG AHRS. Once GPS becomes healthy
     * (and the right conditions are met) perform an INS or reduced-order GPS update.
     */
    if( gAlgorithm.state <= LOW_GAIN_AHRS )
    {
        // Only allow the algorithm to be called on 100 Hz marks
        if(timer.oneHundredHertzFlag == 1) 
        {
            // Update the AHRS solution at a 10 Hz update rate
            // Subframe counter counts to 10 before it is reset
            if( _CheckForUpdateTrigger(TEN_HERTZ_UPDATE) )
            {
                /* The AHRS/VG solution is handled inside FieldVectorsToEulerAngles
                 * (called from the prediction function EKF_PredictionStage)
                 */
                ComputeSystemInnovation_Att();
                Update_Att();
            }
        }
    } 
    else
    {
        /* GPS-type Updates (with magnetometers: true-heading = mag-heading + mag-decl)
         * Perform the EKF update at 10 Hz (split nine mag-only updates for for every GPS/mag update)
         * 
         * Check for 'new GPS data'. If new, and GPS is valid, perform a
         * GPS-Based update and reset timer values to resync the attitude updates.
         */
        if(gEKFInput.gpsUpdate == 1)
        {
            // Sync the algorithm itow to the GPS value
            gAlgorithm.week = gEKFInput.week;
            gAlgorithm.itow = gEKFInput.itow;
            // Resync timer
            timer.tenHertzCntr = 0;
            timer.subFrameCntr = 0;

            // GNSS update
            if (gEKFInput.gpsFixType)
            {
                // GPS heading valid?
                useGpsHeading = (gEKFInput.rawGroundSpeed >= LIMIT_MIN_GPS_VELOCITY_HEADING);

                /* If GNSS outage is longer than a threshold (maxReliableDRTime), DR results get unreliable
                 * So, when GNSS comes back, the EKF is reinitialized. Otherwise, the DR results are still
                 * good, just correct the filter states with input GNSS measurement.
                 */
                int32_t timeSinceLastGoodGPSReading = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastGoodGPSReading;
                if (timeSinceLastGoodGPSReading < 0) 
                {
                    timeSinceLastGoodGPSReading = timeSinceLastGoodGPSReading + MAX_ITOW;
                }
                if (timeSinceLastGoodGPSReading > gAlgorithm.Limit.maxReliableDRTime)
                {
#ifdef INS_OFFLINE
                    printf("GPS relocked.\n");
#endif // INS_OFFLINE
                    // Since a relative long time has passed since DR begins, INS states need reinitialized.
                    InitINSFilter();
                }
                else
                {
                    // DR for a relative short time, no need to reinitialize the filter.
                    Update_GPS();
                }
                // reset the "last good reading" time
                gAlgorithm.timeOfLastGoodGPSReading = gEKFInput.itow;
            }
            //else
            if (gAlgorithm.velocityAlwaysAlongBodyX && gAlgorithm.headingIni>HEADING_UNINITIALIZED)
            {
                Update_PseudoMeasurement();
            }
            // At 1 Hz mark, update when GPS data is valid, else do an AHRS-update
            runInsUpdate = 1;
        }
        else if( runInsUpdate )
        {
            Update_Att();
            runInsUpdate = 0;  // set up for next pass
            useGpsHeading = 0;
        }
    }
}

// ----Compute the innovation vector, nu----
void ComputeSystemInnovation_Pos(void)
{
    // Position error
    gKalmanFilter.nu[STATE_RX] = gKalmanFilter.rGPS_N[X_AXIS] - gKalmanFilter.Position_N[X_AXIS];
    gKalmanFilter.nu[STATE_RY] = gKalmanFilter.rGPS_N[Y_AXIS] - gKalmanFilter.Position_N[Y_AXIS];
    gKalmanFilter.nu[STATE_RZ] = gKalmanFilter.rGPS_N[Z_AXIS] - gKalmanFilter.Position_N[Z_AXIS];

    gKalmanFilter.nu[STATE_RX] = _LimitValue(gKalmanFilter.nu[STATE_RX], gAlgorithm.Limit.Innov.positionError);
    gKalmanFilter.nu[STATE_RY] = _LimitValue(gKalmanFilter.nu[STATE_RY], gAlgorithm.Limit.Innov.positionError);
    gKalmanFilter.nu[STATE_RZ] = _LimitValue(gKalmanFilter.nu[STATE_RZ], gAlgorithm.Limit.Innov.positionError);
}


// ----Compute the innovation vector, nu----
void ComputeSystemInnovation_Vel(void)
{
    // Velocity error
    gKalmanFilter.nu[STATE_VX] = (real)gEKFInput.vNed[X_AXIS] - gKalmanFilter.Velocity_N[X_AXIS];
    gKalmanFilter.nu[STATE_VY] = (real)gEKFInput.vNed[Y_AXIS] - gKalmanFilter.Velocity_N[Y_AXIS];
    gKalmanFilter.nu[STATE_VZ] = (real)gEKFInput.vNed[Z_AXIS] - gKalmanFilter.Velocity_N[Z_AXIS];

    gKalmanFilter.nu[STATE_VX] = _LimitValue(gKalmanFilter.nu[STATE_VX], gAlgorithm.Limit.Innov.velocityError);
    gKalmanFilter.nu[STATE_VY] = _LimitValue(gKalmanFilter.nu[STATE_VY], gAlgorithm.Limit.Innov.velocityError);
    gKalmanFilter.nu[STATE_VZ] = _LimitValue(gKalmanFilter.nu[STATE_VZ], gAlgorithm.Limit.Innov.velocityError);
}


/* Compute the innovation, nu, between measured and predicted attitude.
 *   Correct for wrap-around. Then limit the error.
 */
void ComputeSystemInnovation_Att(void)
{
    // ----- Roll -----
    gKalmanFilter.nu[STATE_ROLL]  = gKalmanFilter.measuredEulerAngles[ROLL] -
                                    gKalmanFilter.eulerAngles[ROLL];
    gKalmanFilter.nu[STATE_ROLL]  = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_ROLL]);
    gKalmanFilter.nu[STATE_ROLL] = _LimitValue(gKalmanFilter.nu[STATE_ROLL], gAlgorithm.Limit.Innov.attitudeError);

    // ----- Pitch -----
    gKalmanFilter.nu[STATE_PITCH] = gKalmanFilter.measuredEulerAngles[PITCH] -
                                    gKalmanFilter.eulerAngles[PITCH];
    gKalmanFilter.nu[STATE_PITCH] = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_PITCH]);
    gKalmanFilter.nu[STATE_PITCH] = _LimitValue(gKalmanFilter.nu[STATE_PITCH], gAlgorithm.Limit.Innov.attitudeError);

    // ----- Yaw -----
    // CHANGED TO SWITCH BETWEEN GPS AND MAG UPDATES
    if ( useGpsHeading )
    {
        if (gAlgorithm.headingIni >= HEADING_GNSS_LOW)   // heading already initialized with GNSS heading
        {
            gKalmanFilter.nu[STATE_YAW] = gEKFInput.trueCourse * (real)DEG_TO_RAD -
                                          gKalmanFilter.eulerAngles[YAW];
        }
        else
        {
            gKalmanFilter.nu[STATE_YAW] = 0.0;
        }
        
    }
    // else if ( magUsedInAlgorithm() && gAlgorithm.state <= LOW_GAIN_AHRS )
    // {
    //     gKalmanFilter.nu[STATE_YAW]   = gKalmanFilter.measuredEulerAngles[YAW] -
    //                                     gKalmanFilter.eulerAngles[YAW];
    // }
    else 
    {
        gKalmanFilter.nu[STATE_YAW] = (real)0.0;
    }
    gKalmanFilter.nu[STATE_YAW] = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_YAW]);
    gKalmanFilter.nu[STATE_YAW] = _LimitValue(gKalmanFilter.nu[STATE_YAW], gAlgorithm.Limit.Innov.attitudeError);

    /* When the filtered yaw-rate is above certain thresholds then reduce the
     * attitude-errors used to update roll and pitch.
     */
    _TurnSwitch();

    //
    gKalmanFilter.nu[STATE_ROLL]  = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_ROLL];
    gKalmanFilter.nu[STATE_PITCH] = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_PITCH];
    //gKalmanFilter.nu[STATE_YAW]   = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_YAW];
}


/******************************************************************************
 * @name: _GenerateObservationJacobian_RollAndPitch roll and pitch elements of
 *        the measurement Jacobian (H)
 * @brief
 * TRACE:
 * @param N/A
 * @retval 1
******************************************************************************/
uint8_t _GenerateObservationJacobian_AHRS(void)
{
    real xPhi, yPhi;
    real uTheta;
    real xPsi, yPsi;
    real denom = 1.0;
    real multiplier = 1.0;

    // Set the values in DP to zero
    static BOOL initH = TRUE;
    if( initH ) 
    {
        initH = FALSE;
        memset(gKalmanFilter.H, 0, sizeof(gKalmanFilter.H));
    }

    /// Note: H is 3x7
    /// Roll
    yPhi = (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q3] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q1] );
    xPhi =  gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q0] + 
           -gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q1] +
           -gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
            gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3];

    denom = yPhi*yPhi + xPhi*xPhi;
    if (denom < 1e-3) 
    {
        /* Based on drive-test data, the minimum value seen was 0.98 but the minimum
         * values based on Matlab analysis is 1e-7.
         */
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the roll-angle wrt quaternions
    gKalmanFilter.H[ROLL][STATE_Q0] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q1] +
                                                    -yPhi*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[ROLL][STATE_Q1] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q0] + 
                                                     yPhi*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[ROLL][STATE_Q2] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q3] + 
                                                     yPhi*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[ROLL][STATE_Q3] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q2] +
                                                    -yPhi*gKalmanFilter.quaternion[Q3]);

    /* Pitch (including modifications for |q| = 1 constraint)
     *   mu = 2*( q(1)*q(3) - q(2)*q(4) );
     *   % 2/sqrt(1-mu*mu) * [q(3); -q(4); q(1); -q(2)]
     *   2/sqrt(1-mu*mu) * [q(3) - mu*q(1);
     *                      -q(4) - mu*q(2);
     *                     q(1)- mu*q(3);
     *                     -q(2)- mu*q(4)]
     */
    uTheta = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q3] -
                      gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q2] );
    // account for numerical accuracy to make sure abs(uTheta) <= 1
    if (uTheta > 1.0)
    {
        uTheta = 1.0;
    }
    if (uTheta < -1.0)
    {
        uTheta = -1.0;
    }
    denom = (real)sqrt(1.0f - uTheta*uTheta);
    if (denom < 1e-3) 
    {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    gKalmanFilter.H[PITCH][STATE_Q0] = multiplier * ( gKalmanFilter.quaternion[Q2] + 
                                                      uTheta * gKalmanFilter.quaternion[Q0] );
    gKalmanFilter.H[PITCH][STATE_Q1] = multiplier * (-gKalmanFilter.quaternion[Q3] + 
                                                      uTheta * gKalmanFilter.quaternion[Q1] );
    gKalmanFilter.H[PITCH][STATE_Q2] = multiplier * ( gKalmanFilter.quaternion[Q0] + 
                                                      uTheta * gKalmanFilter.quaternion[Q2] );
    gKalmanFilter.H[PITCH][STATE_Q3] = multiplier * (-gKalmanFilter.quaternion[Q1] + 
                                                      uTheta * gKalmanFilter.quaternion[Q3] );

    /// Yaw
    yPsi = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q2] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q3] );
    xPsi = (real)1.0 - (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
                                     gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3] );
    denom = yPsi*yPsi + xPsi*xPsi;
    if (denom < 1e-3) 
    {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the yaw-angle wrt quaternions
    gKalmanFilter.H[YAW][STATE_Q0] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q3] +
                                                   -yPsi*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[YAW][STATE_Q1] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q2] +
                                                   -yPsi*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[YAW][STATE_Q2] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q1] + 
                                                    yPsi*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[YAW][STATE_Q3] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q0] + 
                                                    yPsi*gKalmanFilter.quaternion[Q3]);

    return 1;
}


// 
void _GenerateObservationCovariance_AHRS(void)
{
    static real Rnom;

    // Only need to compute certain elements of R once
    static BOOL initR = TRUE;
    if (initR) 
    {
        initR = FALSE;

        /* Clear the values in R (in AHRS mode, there are 3 rows in the Jacobian)
         * Initialize the Process Covariance (Q) matrix
         */
        memset(gKalmanFilter.R, 0, sizeof(gKalmanFilter.R));

        /* Calculate accel var when static from IMU specs.
         * This accel var is the min accel var. If real-time accel var is below this value,
         * the min accel var is used.
         * Accel var is further converted to Euler angels measurement var.
         */
        Rnom = gAlgorithm.imuSpec.sigmaA * gAlgorithm.imuSpec.sigmaA;
    }

    /* Dynamically tune measurement covariance matrix R to get proper Kalman filter
     * gain. The R consists of four parts. First, its min value is Rnom; Secodd,
     * the online estimation of accel var; Thrid, the online estimation of accel
     * error; Four, the different between accel magnitude and 1g. 
     */

    // Rnom, accel var and accel error
    real totalAccelVar[3];  // [m/s/s]^2
    for (int i = 0; i < 3; i++)
    {
        // replace sensor noise var with vibration var
        if (gImuStats.accelVar[i] > Rnom)
        {
            totalAccelVar[i] = gImuStats.accelVar[i];
        }
        else
        {
            totalAccelVar[i] = Rnom;
        }
        // linear accel? (including noise and vibration)
        real errSqr;
        errSqr = gImuStats.accelErr[i] * gImuStats.accelErr[i];
        if (errSqr > totalAccelVar[i])
        {
            totalAccelVar[i] = errSqr;
        }
    }

    /* consider magnitude to further increase R
     * Notice: totalAccelVarSum just approximates accel norm var
     */
    real totalAccelVarSum = totalAccelVar[X_AXIS] + totalAccelVar[Y_AXIS] + totalAccelVar[Z_AXIS];
    real diff = gImuStats.accelNorm - GRAVITY;
    diff *= diff;
    real additionalR = 0.0;
    /* if diff is larger than estimated accel err and the estimated accel err does
     * not reach limit, diff will be used as additional measurement noise var.
     */
    if (diff > 4.0*totalAccelVarSum && gImuStats.accelErrLimit == false)
    {
        // the magnitude of diff is too big, there is linear acceleration
        additionalR = diff;
    }
    else
    {
        // the magnitude of diff is within noise level, no additional var
        additionalR = 0.0;
    }

    /* convert accel measurement var to pitch and roll var
     *  d(pitch) = 1/sqrt(1-ax^2) * d(ax) = 1/sqrt(ay^2+az^2) * d(ax)
     *  d(roll) = (az^2/(ay^2+az^2)) * d(ay) + (-ay/(ay^2+az^2)) * d(az)
     *  Notice: var(kx) = k*k*var(x)
     */
    // Get ax^2, ay^2 and az^2 of normalized accel
    real axSqr = gImuStats.lpfAccel[0] * gImuStats.lpfAccel[0];
    real aySqr = gImuStats.lpfAccel[1] * gImuStats.lpfAccel[1];
    real azSqr = gImuStats.lpfAccel[2] * gImuStats.lpfAccel[2];
    real sumSqr = axSqr + aySqr + azSqr;
    axSqr /= sumSqr;
    aySqr /= sumSqr;
    azSqr /= sumSqr;
    // pitch var
    real mult = 1.0f - axSqr;
    if (mult < 1.0e-2)
    {
        mult = 1.0e-2f;
    }
    mult = 1.0f / mult;  // mult = 1 / (1-ax^2) = 1 / (ay^2 + az^2)
    gKalmanFilter.R[STATE_PITCH] = mult * totalAccelVar[X_AXIS];
    //  roll var
    mult *= mult;   // multi = 1 / (ay^2 + az^2)^2
    gKalmanFilter.R[STATE_ROLL] = mult * azSqr * azSqr * totalAccelVar[Y_AXIS] +
        mult * aySqr * totalAccelVar[Z_AXIS];

    // additional R
    gKalmanFilter.R[STATE_ROLL] += additionalR;
    gKalmanFilter.R[STATE_PITCH] += additionalR;

    /* We are indeed using var of multiple accel samples to estimate the var of Euler
     * angles. From the formula above, accel var should be var of normalized accel.
     * However, we choose GRAVITY instead of real accel norm to normalize the accel.
     * Besides, accel var is only an estimate of Euler angles var, and Euler angels
     * var is indeed not Gaussian.
     */
    gKalmanFilter.R[STATE_ROLL] /= GRAVITY * GRAVITY;
    gKalmanFilter.R[STATE_PITCH] /= GRAVITY * GRAVITY;
    
    /* limit R
     * In previous version, Rnom is in untis of [g]^2, and maxR = 40000.0f*Rnom.
     * After accel in the algorithm is changed to [m/s/s],
     * 40000*Rnom(g^2) = 40000*Rnom([m/s/s]^2)/gravity/gravity = 400*Rnom([m/s/s]^2)
     */
    real maxR = 400.0f * Rnom;
    if (gKalmanFilter.R[STATE_ROLL] > maxR)
    {
        gKalmanFilter.R[STATE_ROLL] = maxR;
    }
    if (gKalmanFilter.R[STATE_PITCH] > maxR)
    {
        gKalmanFilter.R[STATE_PITCH] = maxR;
    }

    /* Yaw
     * ------ From NovAtel's description of BESTVEL: ------
     * Velocity (speed and direction) calculations are computed from either
     * Doppler or carrier phase measurements rather than from pseudorange
     * measurements. Typical speed accuracies are around 0.03m/s (0.07 mph,
     * 0.06 knots).
     *
     * Direction accuracy is derived as a function of the vehicle speed. A
     * simple approach would be to assume a worst case 0.03 m/s cross-track
     * velocity that would yield a direction error function something like:
     *
     * d (speed) = tan-1(0.03/speed)
     *
     * For example, if you are flying in an airplane at a speed of 120 knots
     * or 62 m/s, the approximate directional error will be:
     *
     * tan-1 (0.03/62) = 0.03 degrees
     *
     * Consider another example applicable to hiking at an average walking
     * speed of 3 knots or 1.5 m/s. Using the same error function yields a
     * direction error of about 1.15 degrees.
     *
     * You can see from both examples that a faster vehicle speed allows for a
     * more accurate heading indication. As the vehicle slows down, the
     * velocity information becomes less and less accurate. If the vehicle is
     * stopped, a GNSS receiver still outputs some kind of movement at speeds
     * between 0 and 0.5 m/s in random and changing directions. This
     * represents the noise and error of the static position.

     * ----- Yaw -----
     * CHANGED TO SWITCH BETWEEN GPS AND MAG UPDATES
     */
    if ( useGpsHeading )
    {
        float temp = (float)atan( 0.05 / gEKFInput.rawGroundSpeed );
        gKalmanFilter.R[STATE_YAW] = temp * temp;
        if (gAlgoStatus.bit.turnSwitch)
        {
            gKalmanFilter.R[STATE_YAW] *= 10.0;
        }
    }
    // else if ( magUsedInAlgorithm() && gAlgorithm.state <= LOW_GAIN_AHRS )
    // {
    //     // todo: need to further distinguish between if mag is used
    //     // MAGNETOMETERS
    //     if( (gAlgorithm.state == HIGH_GAIN_AHRS) ||
    //         (gAlgorithm.linAccelSwitch == TRUE) )
    //     {
    //         // --- High-Gain ---
    //         gKalmanFilter.R[STATE_YAW] = (real)1.0e-2;  // jun4
    //     } else {
    //         // --- Low-Gain ---
    //         gKalmanFilter.R[STATE_YAW]   = (real)1.0e-1; // v14.6 values
    //     }

    //     /* For 'large' roll/pitch angles, increase R-yaw to decrease the effect
    //      * of update due to potential uncompensated z-axis magnetometer
    //      * readings from affecting the yaw-update.
    //      */
    //     if( ( gKalmanFilter.eulerAngles[ROLL]  > TEN_DEGREES_IN_RAD ) ||
    //         ( gKalmanFilter.eulerAngles[PITCH] > TEN_DEGREES_IN_RAD ) )
    //     {
    //         gKalmanFilter.R[STATE_YAW] = (real)0.2;
    //     }
    // }
    else
    {
        gKalmanFilter.R[STATE_YAW] = (real)1.0;
    }
}

// 
void _GenerateObservationCovariance_INS(void)
{
    // Only need to compute certain elements of R once
    static BOOL initR = TRUE;
    if (initR) 
    {
        initR = FALSE;

        gKalmanFilter.R[STATE_RX] = (real)R_VALS_GPS_POS_X;
        gKalmanFilter.R[STATE_RY] = gKalmanFilter.R[STATE_RX];
        gKalmanFilter.R[STATE_RZ] = gKalmanFilter.R[STATE_RX];

        gKalmanFilter.R[STATE_VX] = (real)R_VALS_GPS_VEL_X;
        gKalmanFilter.R[STATE_VY] = gKalmanFilter.R[STATE_VX];
        gKalmanFilter.R[STATE_VZ] = gKalmanFilter.R[STATE_VX];
    }

    /* Use the GPS-provided horizontal and vertical accuracy values to populate
     * the covariance values.
     */
    gKalmanFilter.R[STATE_RX] = gEKFInput.GPSHorizAcc * gEKFInput.GPSHorizAcc;
    gKalmanFilter.R[STATE_RY] = gKalmanFilter.R[STATE_RX];
    gKalmanFilter.R[STATE_RZ] = gEKFInput.GPSVertAcc * gEKFInput.GPSVertAcc;

    /* Scale the best velocity error by HDOP then multiply by the z-axis angular
     * rate PLUS one (to prevent the number from being zero) so the velocity
     * update during high-rate turns is reduced.
     */
    float temp = (real)0.0625 * gEKFInput.HDOP;  // 0.0625 = 0.05 / 0.8
    real absFilteredYawRate = (real)fabs(gAlgorithm.filteredYawRate);
    if (absFilteredYawRate > TEN_DEGREES_IN_RAD)
    {
        temp *= (1.0f + absFilteredYawRate);
    }
    gKalmanFilter.R[STATE_VX] = temp;// *((real)1.0 + fabs(gAlgorithm.filteredYawRate) * (real)RAD_TO_DEG);
    gKalmanFilter.R[STATE_VX] = gKalmanFilter.R[STATE_VX] * gKalmanFilter.R[STATE_VX];
    gKalmanFilter.R[STATE_VY] = gKalmanFilter.R[STATE_VX];
    if (gAlgorithm.headingIni == HEADING_UNINITIALIZED)
    {
        /* When heading is not initialized, velocity measurement is not able to correct 
         * attitude/rate bias/accel bias, the larger the velocity, the more uncertain it is.
         */
        gKalmanFilter.R[STATE_VX] += SQUARE(gEKFInput.vNed[0]) + SQUARE(gEKFInput.vNed[1]);
        gKalmanFilter.R[STATE_VY] += gKalmanFilter.R[STATE_VX];
    }

    // z-axis velocity isn't really a function of yaw-rate and hdop
    gKalmanFilter.R[STATE_VZ] = (float)(0.1 * 0.1);
}


//
uint8_t rowNum, colNum, multIndex;

real S_3x3[3][3], SInverse_3x3[3][3];
real PxHTranspose[ROWS_IN_P][ROWS_IN_H];
real KxH[NUMBER_OF_EKF_STATES][COLS_IN_H] = {{ 0.0 }};
real deltaP_tmp[ROWS_IN_P][COLS_IN_P];

void Update_Att(void)
{
    static real lastYaw = 7.0;  // a values larger than 2pi means this yaw is invalid
    // which state is updated in Update_Att()
    uint8_t updatedStatesAtt[16] = { 1, 1, 1,           // Positions are not updated
                                     1, 1, 1,           // Velocities are not updated
                                     1, 1, 1, 1,        // Quaternions are updated
                                     1, 1, 1,           // Gyro biases are updated
                                     1, 1, 1};          // Accel biases are not upated

    /* Calculate the elements in the H and R matrices
     *  Matrix sizes for an Euler-angle based AHRS solution:
     */
    _GenerateObservationJacobian_AHRS();     // gKF.H: 3x16
    _GenerateObservationCovariance_AHRS();   // gKF.R: 3x3

    // In INS mode, do not do pitch and roll update while heading update is kept
    if (gAlgorithm.state == INS_SOLUTION)
    {
        if (!gImuStats.bStaticIMU)
        {
            // If neither mag or GPS headig is available, update measuremnt matrix H to 2x16
            if (gKalmanFilter.R[STATE_YAW] > 0.9)
            {
                for (colNum = 0; colNum < COLS_IN_H; colNum++)
                {
                    gKalmanFilter.H[2][colNum] = 0.0;
                }
            }
            lastYaw = 7.0;
        }
        else
        {
            if (lastYaw > TWO_PI)
            {
                lastYaw = gKalmanFilter.eulerAngles[YAW];
            }
            else
            {
                gKalmanFilter.nu[STATE_YAW] = lastYaw - gKalmanFilter.eulerAngles[YAW];
                gKalmanFilter.R[STATE_YAW] = 1e-4;
            }
        }
        for (colNum = 0; colNum < COLS_IN_H; colNum++)
        {
            gKalmanFilter.H[0][colNum] = 0.0;
            gKalmanFilter.H[1][colNum] = 0.0;
        }
    }

    /* This solution consists of an integrated roll/pitch/yaw solution
     * S = H*P*HTrans + R (However the matrix math can be simplified since
     *                     H is very sparse!  P is fully populated)
     * Update P from the P, H, and R matrices: P = HxPxHTranspose + R
     */
    // 1) PxHTranspose is computed first
    memset(PxHTranspose, 0, sizeof(PxHTranspose));
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) 
    {
        for (colNum = 0; colNum < ROWS_IN_H; colNum++) 
        {
            for (multIndex = RLE_H[colNum][0]; multIndex <= RLE_H[colNum][1]; multIndex++) 
            {
                PxHTranspose[rowNum][colNum] = PxHTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.H[colNum][multIndex];
            }
        }
    }

    /* HPH' is symmetric so only need to multiply one half and reflect the values
     * across the diagonal. S_3x3 is to hold values of HPH'.
     */
    for (rowNum = 0; rowNum < ROWS_IN_H; rowNum++) 
    {
        for (colNum = rowNum; colNum < ROWS_IN_H; colNum++) 
        {
            S_3x3[rowNum][colNum] = 0.0;
            for (multIndex = RLE_H[rowNum][0]; multIndex <= RLE_H[rowNum][1]; multIndex++) 
            {
                S_3x3[rowNum][colNum] = S_3x3[rowNum][colNum] +
                    gKalmanFilter.H[rowNum][multIndex] * PxHTranspose[multIndex][colNum];
            }
            S_3x3[colNum][rowNum] = S_3x3[rowNum][colNum];
        }
    }

    // S = HxPxHTranspose + R (rows 7:10 and cols 7:10 of P PLUS diagonal of R)
    S_3x3[ROLL][ROLL]   +=  gKalmanFilter.R[STATE_ROLL];
    S_3x3[PITCH][PITCH] += gKalmanFilter.R[STATE_PITCH];
    S_3x3[YAW][YAW]     +=  gKalmanFilter.R[STATE_YAW];

    // Invert the S-Matrix (replace with sequential update)
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute the Kalman gain: K = P*HTrans*SInv
    AxB( &PxHTranspose[0][0],
         &SInverse_3x3[0][0],
         ROWS_IN_P, ROWS_IN_H, ROWS_IN_H,
         &gKalmanFilter.K[0][0] );

    // force unupdated terms in K to be 0
    for (rowNum = STATE_RX; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesAtt[rowNum])
        {
            for (colNum = 0; colNum < 3; colNum++)
            {
                gKalmanFilter.K[rowNum][colNum] = 0.0;
            }
        }
    }

    /* Compute attitude-quaternion updates: Dx = K*nu
     * NOTE: Can access nu in the elements that the attitude error is stored BUT the
     * value of ROWS_IN_H must be correct or the multiplication will be wrong
     */
    AxV( &gKalmanFilter.K[0][0],
         &gKalmanFilter.nu[STATE_ROLL],
         NUMBER_OF_EKF_STATES, ROWS_IN_H,
         &gKalmanFilter.stateUpdate[0] );

    // Update states based on computed deltas
    // --- attitude quaternions (q = q + Dq) ---
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + gKalmanFilter.stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + gKalmanFilter.stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + gKalmanFilter.stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + gKalmanFilter.stateUpdate[STATE_Q3];

    // Normalize q
    QuatNormalize(&gKalmanFilter.quaternion[0]);
    
    // --- Angular-rate bias (wBias = wBias = DwBias) ---
    //     If magnetometers are not used then set the rate bias to zero???
    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + gKalmanFilter.stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_WBY];
    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_WBZ];

    /* Update covariance: P = P + DP = P - K*H*P
     * KxH = gKF.K * gKF.H;
     */
    memset(KxH, 0, sizeof(KxH));
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) 
    {
        for (colNum = RLE_KxH[rowNum][0]; colNum <= RLE_KxH[rowNum][1]; colNum++)
        {
            for (multIndex = 0; multIndex < ROWS_IN_H; multIndex++)
            {
                KxH[rowNum][colNum] = KxH[rowNum][colNum] +
                    gKalmanFilter.K[rowNum][multIndex] * gKalmanFilter.H[multIndex][colNum];
            }
        }
    }

    // deltaP = KxH * gKF.P;
    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    /* deltaP is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
#if 0
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) 
    {
        for (colNum = rowNum; colNum < COLS_IN_P; colNum++) 
        {
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++)
            {
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }
#else
    for (rowNum = 0; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesAtt[rowNum])
        {
            continue;
        }
        for (colNum = rowNum; colNum <= STATE_ABZ; colNum++)
        {
            if (!updatedStatesAtt[colNum])
            {
                continue;
            }
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++) 
            {
                if (!updatedStatesAtt[multIndex])
                {
                    continue;
                }
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }
#endif
    /* P is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) 
    {
        for (colNum = rowNum; colNum < COLS_IN_P; colNum++) 
        {
            gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] -
                                              deltaP_tmp[rowNum][colNum];
            gKalmanFilter.P[colNum][rowNum] = gKalmanFilter.P[rowNum][colNum];
        }
    }
}


/* The position update only allows us to update the position and velocity states (along
 * with Pr and Pv).  Want to verify this...
 */
void Update_Pos(void)
{
    // which state is updated in Update_Pos()
    uint8_t updatedStatesPos[16] = { 1, 1, 1,           // Positions are updated
                                     1, 1, 1,           // Velocities are updated
                                     1, 1, 1, 1,        // Quaternions are NOT updated
                                     1, 1, 1,           // Gyro biases are NOT updated
                                     1, 1, 1 };         // Accel biases are NOT upated


    /* S1 = H1*gKF.P*H1' + R1;   Top 3 rows of the first 3 cols of P + R
     * K1 = gKF.P*H1'*inv(S1);   ( first 3 cols of P ) * S1Inverse
     * P1 = (eye(16) - K1*H1) * gKF.P;
     * H2 = [I3x3 0_3x3 0_3x4 0_3x3 0_3x3]
     */

    // S1 = H1*gKF.P*H1' + R1;
    S_3x3[0][0] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.R[STATE_RX];
    S_3x3[0][1] = gKalmanFilter.P[STATE_RX][STATE_RY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_RX][STATE_RZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_RY][STATE_RX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.R[STATE_RY];
    S_3x3[1][2] = gKalmanFilter.P[STATE_RY][STATE_RZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_RZ][STATE_RX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_RZ][STATE_RY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.R[STATE_RZ];

    // S1_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K1 = ( gKF.P*H1' ) * S1Inverse = ( first 3 cols of P ) * S1Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++)
    {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++)
        {
            gKalmanFilter.K[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the position states are used
            //   in the calculation
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++)
            {
                gKalmanFilter.K[rowNum][colNum] = gKalmanFilter.K[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_RX][colNum];
            }
        }
    }

    // force uncorrected terms in K to be 0
    for (rowNum = STATE_RX; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesPos[rowNum])
        {
            for (colNum = 0; colNum < 3; colNum++)
            {
                gKalmanFilter.K[rowNum][colNum] = 0.0;
            }
        }
    }
    // Compute the intermediate state update, stateUpdate
    AxB(&gKalmanFilter.K[0][0], &gKalmanFilter.nu[STATE_RX], NUMBER_OF_EKF_STATES, 3, 1, &gKalmanFilter.stateUpdate[0]);

    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) 
    {
        if (!updatedStatesPos[rowNum])
        {
            continue;
        }
        for (colNum = rowNum; colNum < NUMBER_OF_EKF_STATES; colNum++) 
        {
            if (!updatedStatesPos[colNum])
            {
                continue;
            }
            /* H is sparse so only the columns of P associated with the position states are used
             * in the calculation
             */
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) 
            {
                if (!updatedStatesPos[multIndex])
                {
                    continue;
                }
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    gKalmanFilter.K[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }

    AMinusB(&gKalmanFilter.P[0][0], &deltaP_tmp[0][0], NUMBER_OF_EKF_STATES, NUMBER_OF_EKF_STATES, &gKalmanFilter.P[0][0]);

    // Update states
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_RX];
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_RY];
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + gKalmanFilter.stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + gKalmanFilter.stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + gKalmanFilter.stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + gKalmanFilter.stateUpdate[STATE_Q3];
    
    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);
    
    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + gKalmanFilter.stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_WBY];
    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_WBZ];
    
    gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + gKalmanFilter.stateUpdate[STATE_ABX];
    gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_ABY];
    gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_ABZ];
}


/* The velocity update only allows us to update the velocity, attitude, and acceleration-
 * bias states (along with Pv, Pq, and Pab).  Wwant to verify this...
 */
void Update_Vel(void)
{
    // which state is updated in Update_Vel()
    uint8_t updatedStatesVel[16] = { 1, 1, 1,           // Positions are NOT updated
                                     1, 1, 1,           // Velocities are updated
                                     1, 1, 1, 1,        // Quaternions are updated
                                     1, 1, 1,           // Gyro biases are NOT updated
                                     1, 1, 1 };         // Accel biases are upated

    /* S2 = H2*P1*H2' + R2; (4th, 5th, and 6th rows of the 4th, 5th, and 6th cols of P1)
     * K2 = P1*H2'*inv(S2);
     * P2 = (eye(16) - K2*H2) * P1;
     * H2 = [0_3x3 I3x3 0_3x4 0_3x3 0_3x3]
     */

    // S2 = H2*P1*H2' + R2;
    S_3x3[0][0] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.R[STATE_VX];
    S_3x3[0][1] = gKalmanFilter.P[STATE_VX][STATE_VY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_VX][STATE_VZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_VY][STATE_VX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.R[STATE_VY];
    S_3x3[1][2] = gKalmanFilter.P[STATE_VY][STATE_VZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_VZ][STATE_VX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_VZ][STATE_VY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.R[STATE_VZ];

    // S2_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K2 = ( P1*H2' ) * S2Inverse = ( 4th, 5th, and 6th cols of P1 ) * S2Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) 
    {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) 
        {
            gKalmanFilter.K[rowNum][colNum] = 0.0;
            /* H is sparse so only the columns of P associated with the velocity states are used
             * in the calculation
             */
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) 
            {
                gKalmanFilter.K[rowNum][colNum] = gKalmanFilter.K[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_VX][colNum];
            }
        }
    }

    // force uncorrected terms in K to be 0
    for (rowNum = STATE_RX; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesVel[rowNum])
        {
            for (colNum = 0; colNum < 3; colNum++)
            {
                gKalmanFilter.K[rowNum][colNum] = 0.0;
            }
        }
    }
    // Compute the intermediate state update
    AxB(&gKalmanFilter.K[0][0], &gKalmanFilter.nu[STATE_VX], NUMBER_OF_EKF_STATES, 3, 1, &gKalmanFilter.stateUpdate[0]);

    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) 
    {
        if (!updatedStatesVel[rowNum])
        {
            continue;
        }
        for (colNum = rowNum; colNum < NUMBER_OF_EKF_STATES; colNum++) 
        {
            if (!updatedStatesVel[colNum])
            {
                continue;
            }
            /* H is sparse so only the columns of P associated with the velocity states are used
             * in the calculation
             */
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) 
            {
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    gKalmanFilter.K[rowNum][multIndex - STATE_VX] * gKalmanFilter.P[multIndex][colNum];
            }
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }

    // P2 = P2 - KxHxP2
    AMinusB(&gKalmanFilter.P[0][0], &deltaP_tmp[0][0], NUMBER_OF_EKF_STATES, NUMBER_OF_EKF_STATES, &gKalmanFilter.P[0][0]);
    // ++++++++++++++++++++++ END OF VELOCITY ++++++++++++++++++++++

    // Update states
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_RX];
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_RY];
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + gKalmanFilter.stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + gKalmanFilter.stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + gKalmanFilter.stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + gKalmanFilter.stateUpdate[STATE_Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + gKalmanFilter.stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_WBY];
    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_WBZ];
    
    gKalmanFilter.accelBias_B[X_AXIS] += gKalmanFilter.stateUpdate[STATE_ABX];
    gKalmanFilter.accelBias_B[Y_AXIS] += gKalmanFilter.stateUpdate[STATE_ABY];
    gKalmanFilter.accelBias_B[Z_AXIS] += gKalmanFilter.stateUpdate[STATE_ABZ];
    
}

static void Update_GPS(void)
{  
    // Calculate the R-values for the INS measurements
    _GenerateObservationCovariance_INS();

    /* This Sequential-Filter (three-stage approach) is nearly as
     * good as the full implementation -- we also can split it
     * across multiple iterations to not exceed 10 ms execution on
     * the embedded 380
     */
     /* Compute the system error: z = meas, h = pred = q, nu - z - h
      * Do this at the same time even if the update is spread across time-steps
      */
    ComputeSystemInnovation_Pos();
    Update_Pos();
    ComputeSystemInnovation_Vel();
    Update_Vel();
    ComputeSystemInnovation_Att();

    // Initialize heading. If getting initial heading at this step, do not update att
    if (gAlgorithm.headingIni < HEADING_GNSS_HIGH)
    {
        if (InitializeHeadingFromGnss())
        {
            // Heading is initialized. Related elements in the EKF also need intializing.
            InitializeEkfHeading();

            /* This heading measurement is used to initialize heading, and should not be
             * used to update heading.
             */
            useGpsHeading = FALSE;
        }
    }
}

static void Update_PseudoMeasurement(void)
{
    // which state is updated in Update_Vel()
    uint8_t updatedStatesPseudo[16] = { 1, 1, 1,           // Positions are NOT updated
                                        1, 1, 1,           // Velocities are updated
                                        1, 1, 1, 1,        // Quaternions are updated
                                        1, 1, 1,           // Gyro biases are updated
                                        1, 1, 1 };         // Accel biases are upated
    
    /* Get current rb2n.
     * gKalmanFilter.R_BinN is updated every time the algo enters _PredictStateEstimate
     * After prediction and GPS update, this matrix needs updated.
     */
    real rb2n[3][3];
    QuaternionToR321(gKalmanFilter.quaternion, &rb2n[0][0]);

    // detect zero velocity using GNSS vNED
    BOOL staticGnss = DetectStaticGnssVelocity(gEKFInput.vNed, 
                                               gAlgorithm.staticDetectParam.staticGnssVel,
                                               gEKFInput.gpsFixType);

    // measurement cov
    real r[3] = { 1.0e-4f, 1.0e-4f, 1.0e-4f };
    if (!gImuStats.bStaticIMU)
    {
        /* If zero velocity is not detected by IMU, the covariance for the lateral and
         * vertical velocity measurement should be increased.
         */
        GenPseudoMeasCov(r);
        r[1] = 1.0e-1;
        r[2] = 1.0e-1;
    }

    /* Compute innovation (measured - estimated) of velocity expressed in the body frame:
     * innovation = [odo/0.0, 0.0, 0.0] - Rn2b * v_ned
     * When odometer is available, front velocity measurement is given by odometer.
     * When zero velocity detected, front velocity measurement is 0.
     * Zero velocity detection result has a higher priority to determine the front velocity because
     * odometer is also used for zero velocity detection when odometer is available.
     */
    BOOL hasOdo = FALSE;
    BOOL frontVelMeaValid = FALSE;
    real frontVelMea = 0.0;
    /* Front velocity is first determined by odometer. If odometer is not available, zero velocity
     * detection results are used to determine if front velocity is zero. If neither odometer is
     * available nor zero velocity detected, front velocity measurement is not valid.
     */
    if (hasOdo)
    {
        frontVelMeaValid = TRUE;
        frontVelMea = 0.0;  // replace with real odo output
        r[0] = 1.0e-4;      // variance of front velocity measurement should be from odo spec
    }
    else if (gImuStats.bStaticIMU)
    {
        /* Only when GNSS is invalid or zero velocity is also detected by GNSS, zero velocity
         * detected by IMU (and GNSS) can be used to determine the along-track velocity.
         * When front velocity measurement is not available, it is not necessary to readjust
         * its variance since it will not be used.
         */
        if ((!gEKFInput.gpsFixType) || staticGnss)
        {
            frontVelMeaValid = TRUE;
            frontVelMea = 0.0;
        }
    }
    // front vel error
    gKalmanFilter.nu[STATE_VX] = frontVelMea
        -rb2n[0][0] * gKalmanFilter.Velocity_N[0]
        -rb2n[1][0] * gKalmanFilter.Velocity_N[1]
        -rb2n[2][0] * gKalmanFilter.Velocity_N[2];
    // lateral (right) vel error
    gKalmanFilter.nu[STATE_VY] = 
        -rb2n[0][1] * gKalmanFilter.Velocity_N[0]
        -rb2n[1][1] * gKalmanFilter.Velocity_N[1]
        -rb2n[2][1] * gKalmanFilter.Velocity_N[2];
    // vertical (downwards) vel erro
    gKalmanFilter.nu[STATE_VZ] =
        -rb2n[0][2] * gKalmanFilter.Velocity_N[0]
        -rb2n[1][2] * gKalmanFilter.Velocity_N[1]
        -rb2n[2][2] * gKalmanFilter.Velocity_N[2];
    gKalmanFilter.nu[STATE_VY] = _LimitValue(gKalmanFilter.nu[STATE_VY], gAlgorithm.Limit.Innov.velocityError);
    gKalmanFilter.nu[STATE_VZ] = _LimitValue(gKalmanFilter.nu[STATE_VZ], gAlgorithm.Limit.Innov.velocityError);

    // p*H'. PxHTranspose is 16x3, only the last two columns are used when only lateral and vertical measurements
    memset(PxHTranspose, 0, sizeof(PxHTranspose));
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++)
    {
        for (colNum = 0; colNum < 3; colNum++)
        {
            PxHTranspose[rowNum][colNum] =
                gKalmanFilter.P[rowNum][3] * rb2n[0][colNum] +
                gKalmanFilter.P[rowNum][4] * rb2n[1][colNum] +
                gKalmanFilter.P[rowNum][5] * rb2n[2][colNum];
        }
    }
    
    // s = H*P*H' + R
    for (rowNum = 0; rowNum < 3; rowNum++)
    {
        for (colNum = rowNum; colNum < 3; colNum++)
        {
            S_3x3[rowNum][colNum] = rb2n[0][rowNum] * PxHTranspose[3][colNum] +
                                    rb2n[1][rowNum] * PxHTranspose[4][colNum] +
                                    rb2n[2][rowNum] * PxHTranspose[5][colNum];
            S_3x3[colNum][rowNum] = S_3x3[rowNum][colNum];
        }
        S_3x3[rowNum][rowNum] += r[rowNum];
    }

    // Calculate inv(H*P*H'+R) according to if front velocity measurement is available
    if (frontVelMeaValid)
    {
        matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);
    }
    else
    {
        S_3x3[0][0] = 1.0;
        S_3x3[0][1] = 0.0;
        S_3x3[0][2] = 0.0;
        S_3x3[1][0] = 0.0;
        S_3x3[2][0] = 0.0;
        matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);
        SInverse_3x3[0][0] = 0.0;
    }

    // K = P*H' * inv(H*P*H' + R). gKalmanFilter.K is 16x3, only the last two columns are used.
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++)
    {
        for (colNum = 0; colNum < 3; colNum++)
        {
            gKalmanFilter.K[rowNum][colNum] = 
                PxHTranspose[rowNum][0] * SInverse_3x3[0][colNum] +
                PxHTranspose[rowNum][1] * SInverse_3x3[1][colNum] +
                PxHTranspose[rowNum][2] * SInverse_3x3[2][colNum];
        }
    }
    // force uncorrected terms in K to be 0
    for (rowNum = STATE_RX; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesPseudo[rowNum])
        {
            for (colNum = 0; colNum < 3; colNum++)
            {
                gKalmanFilter.K[rowNum][colNum] = 0.0;
            }
        }
    }

    // dx = k * nu
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++)
    {
        gKalmanFilter.stateUpdate[rowNum] = 
            gKalmanFilter.K[rowNum][0] * gKalmanFilter.nu[STATE_VX] +
            gKalmanFilter.K[rowNum][1] * gKalmanFilter.nu[STATE_VY] +
            gKalmanFilter.K[rowNum][2] * gKalmanFilter.nu[STATE_VZ];
    }
    
    // update state
    //gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_RX];
    //gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_RY];
    //gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + gKalmanFilter.stateUpdate[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + gKalmanFilter.stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + gKalmanFilter.stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + gKalmanFilter.stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + gKalmanFilter.stateUpdate[STATE_Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + gKalmanFilter.stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + gKalmanFilter.stateUpdate[STATE_WBY];
    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + gKalmanFilter.stateUpdate[STATE_WBZ];

    gKalmanFilter.accelBias_B[X_AXIS] += gKalmanFilter.stateUpdate[STATE_ABX];
    gKalmanFilter.accelBias_B[Y_AXIS] += gKalmanFilter.stateUpdate[STATE_ABY];
    gKalmanFilter.accelBias_B[Z_AXIS] += gKalmanFilter.stateUpdate[STATE_ABZ];

    // Update covariance: P = P + DP = P - K*H*P
    // Use transpose(PxHTranspose) to hold H*P (3x16). 
    // Only the last two columns are used when only lateral and vertical measurements
    for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++)
    {
        for (rowNum = 0; rowNum < 3; rowNum++)
        {
            PxHTranspose[colNum][rowNum] = rb2n[0][rowNum] * gKalmanFilter.P[3][colNum] +
                                           rb2n[1][rowNum] * gKalmanFilter.P[4][colNum] +
                                           rb2n[2][rowNum] * gKalmanFilter.P[5][colNum];
        }
    }

    // deltaP = KxH * gKF.P;
    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    /* deltaP is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
    for (rowNum = 0; rowNum <= STATE_ABZ; rowNum++)
    {
        if (!updatedStatesPseudo[rowNum])
        {
            continue;
        }
        for (colNum = rowNum; colNum <= STATE_ABZ; colNum++)
        {
            if (!updatedStatesPseudo[colNum])
            {
                continue;
            }
            deltaP_tmp[rowNum][colNum] = gKalmanFilter.K[rowNum][0] * PxHTranspose[colNum][0] +
                                         gKalmanFilter.K[rowNum][1] * PxHTranspose[colNum][1] +
                                         gKalmanFilter.K[rowNum][2] * PxHTranspose[colNum][2];
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }

    /* P is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++)
    {
        for (colNum = rowNum; colNum < COLS_IN_P; colNum++)
        {
            gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] -
                                              deltaP_tmp[rowNum][colNum];
            gKalmanFilter.P[colNum][rowNum] = gKalmanFilter.P[rowNum][colNum];
        }
    }
}

static void GenPseudoMeasCov(real *r)
{
    real absYawRate = (real)fabs(gEKFInput.angRate_B[2]);
    r[1] = absYawRate;
    r[2] = absYawRate;
    
    real minVar = 1e-4;
    real maxVar = 1e-2;
    if (r[1] < minVar)
    {
        r[1] = minVar;
    }
    if (r[1] > maxVar)
    {
        r[1] = maxVar;
    }

    if (r[2] < minVar)
    {
        r[2] = minVar;
    }
    if (r[2] > maxVar)
    {
        r[2] = maxVar;
    }
    //printf("rr: %f,%f\n", r[1], r[2]);
}


/* Conversion from turn-rate threshold (values loaded into gConfiguration) to
 * decimal value in [rad/sec]:
 *
 *   thresh_rad = ( 10.0 * pi/180 );   % 0.1745
 *   thresh_counts = floor( thresh_rad * ( 2^16 / (2*pi) ) );   % 1820
 *   thresh_rad = thresh_counts * ( 2*pi / 2^16 )   % 1820 * (2*pi) / 2^16 = 0.1745
 */
real TILT_YAW_SWITCH_GAIN = (real)0.05;

// TurnSwitch.m
static void _TurnSwitch(void)
{
    static real minSwitch = (real)0.0, maxSwitch = (real)0.0;
    static real turnSwitchThresholdPast = (real)0.0;
    static real linInterpSF;

    real absYawRate;
    real turnSwitchScaleFactor;

    // gKF.filteredYawRate (calculated in the prediction stage)
    absYawRate = (real)fabs(gAlgorithm.filteredYawRate);

    // In case the user changes the TST during operation
    if (gAlgorithm.turnSwitchThreshold != turnSwitchThresholdPast)
    {
        turnSwitchThresholdPast = gAlgorithm.turnSwitchThreshold;

        // Example conversion: ( 1820*12868 / 2^27 ) * ( 180/pi )
        minSwitch = gAlgorithm.turnSwitchThreshold * (real)(DEG_TO_RAD);   // angle in radians
        maxSwitch = (real)2.0 * minSwitch;   // angle in radians

        linInterpSF = ((real)1.0 - TILT_YAW_SWITCH_GAIN) / (maxSwitch - minSwitch);
    }

    // Linear interpolation if the yawRate is above the specified threshold
    if ((gAlgorithm.state > HIGH_GAIN_AHRS) && (absYawRate > minSwitch))
    {
        gAlgoStatus.bit.turnSwitch = TRUE;

        /* When the rate is below the maximum rate defined by turnSwitchThreshold,
         * then generate a scale-factor that is between ( 1.0 - G ) and 0.0 (based on absYawRate).
         * If it is above 'maxSwitch' then the SF is zero.
         */
        if (absYawRate < maxSwitch) 
        {
            turnSwitchScaleFactor = linInterpSF * (maxSwitch - absYawRate);
        } 
        else
        {
            // yaw-rate is above maxSwitch ==> no gain
            turnSwitchScaleFactor = (real)0.0;
        }

        // Specify the multiplier so it is between G and 1.0
        gKalmanFilter.turnSwitchMultiplier = TILT_YAW_SWITCH_GAIN + turnSwitchScaleFactor;
    }
    else
    {
        gAlgoStatus.bit.turnSwitch = FALSE;
        gKalmanFilter.turnSwitchMultiplier = (real)1.0;
    }
}


static real _UnwrapAttitudeError(real attitudeError)
{
    while (fabs(attitudeError) > PI)
    {
        if (attitudeError > PI)
        {
            attitudeError = attitudeError - (real)TWO_PI;
        } 
        else if (attitudeError < -PI)
        {
            attitudeError = attitudeError + (real)TWO_PI;
        }
    }

    return attitudeError;
}


static real _LimitValue(real value, real limit)
{
    if (value > limit) 
    {
        return limit;
    }
    else if (value < -limit)
    {
        return -limit;
    }

    return value;
}


/* Returns true when the system is ready to update (based on the timer values
 *   and the desired update rate)
 */
static BOOL _CheckForUpdateTrigger(uint8_t updateRate)
{
    //
    uint8_t oneHundredHzCntr;
    uint8_t gpsUpdate = 0;

    //
    switch( updateRate )
    {
        // ten-hertz update
        case 10:
            if( timer.subFrameCntr == 0 ) 
            {
                gpsUpdate = 1;
            }
            break;

        // twenty-hertz update
        case 20:
            if( timer.subFrameCntr == 0 || timer.subFrameCntr == 5 )
            {
                gpsUpdate = 1;
            }
            break;

        // twenty-hertz update
        case 25:
            oneHundredHzCntr = 10 * timer.tenHertzCntr + timer.subFrameCntr;

            // 
            if( oneHundredHzCntr ==  0 ||
                oneHundredHzCntr ==  4 ||
                oneHundredHzCntr ==  8 ||
                oneHundredHzCntr == 12 ||
                oneHundredHzCntr == 16 ||
                oneHundredHzCntr == 20 ||
                oneHundredHzCntr == 24 ||
                oneHundredHzCntr == 28 ||
                oneHundredHzCntr == 32 ||
                oneHundredHzCntr == 36 ||
                oneHundredHzCntr == 40 ||
                oneHundredHzCntr == 44 ||
                oneHundredHzCntr == 48 ||
                oneHundredHzCntr == 52 ||
                oneHundredHzCntr == 56 ||
                oneHundredHzCntr == 60 ||
                oneHundredHzCntr == 64 ||
                oneHundredHzCntr == 68 ||
                oneHundredHzCntr == 72 ||
                oneHundredHzCntr == 76 ||
                oneHundredHzCntr == 80 ||
                oneHundredHzCntr == 84 ||
                oneHundredHzCntr == 88 ||
                oneHundredHzCntr == 92 ||
                oneHundredHzCntr == 96 )
            {
                gpsUpdate = 1;
            }
            break;

        // fifty-hertz update
        case 50:
            if( timer.subFrameCntr == 0 ||
                timer.subFrameCntr == 2 ||
                timer.subFrameCntr == 4 ||
                timer.subFrameCntr == 6 ||
                timer.subFrameCntr == 8 )
            {
                gpsUpdate = 1;
            }
            break;

        // fifty-hertz update
        case 100:
            gpsUpdate = 1;
            break;
    }

    return gpsUpdate;
}

static int InitializeHeadingFromGnss()
{
    /* enable declination correction, but the corrected magnetic yaw will not
     * be used if GPS is available.
     */
    gAlgorithm.applyDeclFlag = TRUE;

    /* backward drive detection for heading initialization using GNSS heading.
     * Detection happends every second. Velocity increment is relatively reliable
     * if it is accumulated for 1sec.
     */
    static real lastVelBxGnss = 0;
    static uint8_t forwardDriveConfidence = 0;
    static uint32_t lastTOW = 0;
    uint32_t timeSinceLastDetection = gAlgorithm.itow - lastTOW;
    if (timeSinceLastDetection < 0)
    {
        timeSinceLastDetection = timeSinceLastDetection + MAX_ITOW;
    }
    if (timeSinceLastDetection > 950)   // 950ms is set as the threshold for 1sec
    {
        lastTOW = gAlgorithm.itow;
        /* assume velocity is always along the body x axis. otherwise, GNSS heading
         * cannot be used to initialize fusion heading
         */
        real velBx = sqrtf(SQUARE(gEKFInput.vNed[0]) + SQUARE(gEKFInput.vNed[1]) + SQUARE(gEKFInput.vNed[2]));
        velBx = fabs(velBx);
        real dv = velBx - lastVelBxGnss;
        if ((dv * gKalmanFilter.linearAccel_B[X_AXIS]) > 0.0 && fabs(gKalmanFilter.linearAccel_B[X_AXIS]) > 0.2)
        {
            if (forwardDriveConfidence < 255)
            {
                forwardDriveConfidence++;
            }
        }
        else
        {
            forwardDriveConfidence = 0;
        }
        // record this velocity along body x axis for next run
        lastVelBxGnss = velBx;
        // reset accumulated x body axis velocity change.
        gKalmanFilter.linearAccel_B[X_AXIS] = 0.0;
    }

    // detect if GNSS heading is reliable
    static uint8_t gnssHeadingGoodCntr = 0;
    static float lastGnssHeading = 0.0;
    static float lastFusionHeading = 0.0;
    BOOL gnssHeadingGood = 0;
    float angleDiff = 0.0;
    if (useGpsHeading)
    {
        float calculatedGnssHeading  = (float)(atan2(gEKFInput.vNed[1], gEKFInput.vNed[0]) * R2D);
        float diffHeading = AngleErrDeg(gEKFInput.trueCourse - calculatedGnssHeading);
        // input GNSS heading matches heading calculated from vNED
        if (fabs(diffHeading) < 5.0)
        {
            // GNSS heading change matches fusion yaw angle
            float gnssHeadingChange = gEKFInput.trueCourse - lastGnssHeading;
            float fusionHeadingChange = gKalmanFilter.eulerAngles[2] * (float)R2D - lastFusionHeading;
            angleDiff = (float)fabs( AngleErrDeg(gnssHeadingChange - fusionHeadingChange) );
            if (angleDiff < 5.0)
            {
                gnssHeadingGood = TRUE;
            }
        }
        lastGnssHeading = gEKFInput.trueCourse;
        lastFusionHeading = gKalmanFilter.eulerAngles[2] * (float)R2D;
    }
    if (gnssHeadingGood)
    {
        gnssHeadingGoodCntr++;
    }
    else
    {    
        gnssHeadingGoodCntr = 0;
    }
    
    // Heading initialization when drive forward and GNSS heading is reliable
    BOOL thisHeadingUsedForIni = FALSE;
    if (gAlgorithm.headingIni < HEADING_GNSS_LOW)   // heading is immediately but maybe unreliably initialized
    {
        if (gnssHeadingGoodCntr >= 1 && forwardDriveConfidence >= 1)   // Only one sample is checked, so heading may be unreliable
        {
            gnssHeadingGoodCntr = 0;
            // Heading is initialized with GNSS
            gAlgorithm.headingIni = HEADING_GNSS_LOW;

#ifdef INS_OFFLINE
            printf("quick gps heading: %f\n", gEKFInput.trueCourse);
#else
#ifdef DISPLAY_DIAGNOSTIC_MSG
            DebugPrintString("quick gps heading");
            DebugPrintFloat(": ", gEKFInput.trueCourse, 9);
            DebugPrintEndline();
#endif
#endif
            thisHeadingUsedForIni = TRUE;
        }
    }
    else
    {
        /* Three points are checked, and the latest ground speed is above a certian threshold.
         * The latest GNSS heading should be reliable.
         */
        if (gnssHeadingGoodCntr >= 3 && 
            forwardDriveConfidence >= 5 && 
            gEKFInput.rawGroundSpeed > RELIABLE_GPS_VELOCITY_HEADING)
        {
            gnssHeadingGoodCntr = 0;
            forwardDriveConfidence = 0;
            gAlgorithm.headingIni = HEADING_GNSS_HIGH;
#ifdef INS_OFFLINE
            printf("reliable gps heading: %f\n", gEKFInput.trueCourse);
#else
#ifdef DISPLAY_DIAGNOSTIC_MSG
            DebugPrintString("reliable gps heading");
            DebugPrintFloat(": ", gEKFInput.trueCourse, 9);
            DebugPrintEndline();
#endif
#endif
            thisHeadingUsedForIni = TRUE;
        }
    }

    return thisHeadingUsedForIni;
}

static void InitializeEkfHeading()
{
    /* Compare the reliable heading with Kalamn filter heading. If the difference exceeds
     * a certain threshold, this means the immediate heading initialization is unreliable,
     * and the Kalman filter needs reinitialized with the reliable one.
     */
    float angleDiff = (float)fabs(AngleErrDeg(gEKFInput.trueCourse - 
                                              gKalmanFilter.eulerAngles[2] * (float)R2D));
    if (angleDiff <= 2.0)
    {
        return;
    }

#ifdef INS_OFFLINE
        printf("Reinitialize KF: %f\n", angleDiff);
#else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("Reinitialize KF: ");
        DebugPrintFloat("", angleDiff, 9);
        DebugPrintEndline();
#endif
#endif

    // initialize yaw angle with GPS heading
    gKalmanFilter.eulerAngles[YAW] = (gEKFInput.trueCourse * D2R);
    if (gKalmanFilter.eulerAngles[YAW] > PI)
    {
        gKalmanFilter.eulerAngles[YAW] -= (real)TWO_PI;
    }
    EulerAnglesToQuaternion(gKalmanFilter.eulerAngles, gKalmanFilter.quaternion);

    // reinitialize NED position
    gKalmanFilter.Position_N[0] = (real)gKalmanFilter.rGPS_N[0];
    gKalmanFilter.Position_N[1] = (real)gKalmanFilter.rGPS_N[1];
    gKalmanFilter.Position_N[2] = (real)gKalmanFilter.rGPS_N[2];

    // reinitialize NED velocity
    gKalmanFilter.Velocity_N[X_AXIS] = (real)gEKFInput.vNed[X_AXIS];
    gKalmanFilter.Velocity_N[Y_AXIS] = (real)gEKFInput.vNed[Y_AXIS];
    gKalmanFilter.Velocity_N[Z_AXIS] = (real)gEKFInput.vNed[Z_AXIS];

#if 1   // mod, DXG
    // reset quaternion and velocity terms in the P matrix
    int i, j;
    // pos row
    gKalmanFilter.P[STATE_RX][STATE_RX] = gKalmanFilter.R[STATE_RX];
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.R[STATE_RY];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gKalmanFilter.R[STATE_RZ];
    for (i = STATE_RX; i < STATE_RZ; i++)
    {
        for (j = 0; j < NUMBER_OF_EKF_STATES; j++)
        {
            if (i != j)
            {
                gKalmanFilter.P[i][j] = 0;
                gKalmanFilter.P[j][i] = 0;
            }
        }
    }
    // vel row
    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.R[STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.R[STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VZ];
    for (i = STATE_VX; i < STATE_VZ; i++)
    {
        for (j = 0; j < NUMBER_OF_EKF_STATES; j++)
        {
            if (i != j)
            {
                gKalmanFilter.P[i][j] = 0;
                gKalmanFilter.P[j][i] = 0;
            }
        }
    }
    // q0 row
    for (i = 0; i < NUMBER_OF_EKF_STATES; i++)
    {
        if (i != STATE_Q0)
        {
            gKalmanFilter.P[STATE_Q0][i] = 0;
            gKalmanFilter.P[i][STATE_Q0] = 0;
        }
    }
    // q3 row
    for (i = 0; i < NUMBER_OF_EKF_STATES; i++)
    {
        if (i != STATE_Q3)
        {
            gKalmanFilter.P[STATE_Q3][i] = 0;
            gKalmanFilter.P[i][STATE_Q3] = 0;
        }
    }

    // the initial covariance of the quaternion is estimated from ground speed.
    float temp = (float)atan(0.05 / gEKFInput.rawGroundSpeed);
    temp *= temp;   // heading var
    if (gAlgoStatus.bit.turnSwitch)
    {
        temp *= 10.0;   // when rotating, heading var increases
    }
    temp /= 4.0;        // sin(heading/2) or cos(heading/2)
    float sinYawSqr = (real)sin(gKalmanFilter.eulerAngles[YAW] / 2.0f);
    sinYawSqr *= sinYawSqr;
    //  Assume roll and pitch are close to 0deg
    gKalmanFilter.P[STATE_Q0][STATE_Q0] = temp * sinYawSqr;
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = temp * (1.0f - sinYawSqr);

    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.R[STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.R[STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VZ];

#if 0
    // reset velocity and quaternion terms in the P matrix
    real v2 = gEKFInput.rawGroundSpeed * gEKFInput.rawGroundSpeed;
    real v3by4 = 4.0 * v2 * gEKFInput.rawGroundSpeed;
    real vn2 = gEKFInput.vNed[0] * gEKFInput.vNed[0];
    real q0q0 = gKalmanFilter.quaternion[0] * gKalmanFilter.quaternion[0];
    real q3q3 = gKalmanFilter.quaternion[3] * gKalmanFilter.quaternion[3];
    if (q0q0 < 1.0e-3)
    {
        q0q0 = 1.0e-3;
    }
    if (q3q3 < 1.0e-3)
    {
        q3q3 = 1.0e-3;
    }
    real multiplerQVn = (v2 - vn2) / v3by4;
    multiplerQVn *= multiplerQVn;
    real multiplerQVe = (gEKFInput.vNed[0] * gEKFInput.vNed[1]) / v3by4;
    multiplerQVe *= multiplerQVe;
    gKalmanFilter.P[STATE_VX][STATE_Q0] = multiplerQVn / q0q0 * gKalmanFilter.R[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VX][STATE_Q3] = multiplerQVn / q3q3 * gKalmanFilter.R[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_Q0] = multiplerQVe / q0q0 * gKalmanFilter.R[STATE_VY][STATE_VY];
    gKalmanFilter.P[STATE_VY][STATE_Q3] = multiplerQVe / q3q3 * gKalmanFilter.R[STATE_VY][STATE_VY];


    gKalmanFilter.P[STATE_Q0][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_Q0];
    gKalmanFilter.P[STATE_Q3][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_Q3];
    gKalmanFilter.P[STATE_Q0][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_Q0];
    gKalmanFilter.P[STATE_Q3][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_Q3];
#endif

#endif
}


/******************************************************************************
 * @brief Remove lever arm in position and velocity.
 * GNSS measured position is the position of the antenna. For GNSS/INS integration,
 * the position of IMU is needed. Before using the GNSS position and velocity,
 * those shold be first converted to the IMU position and velocity by removing
 * lever arm effects. Lever arm introduces an offset in position. This offset
 * can be directly canceled by substracting lever arm. Combined with angular
 * velocity, lever arm further introduces relative velocity.
 * TRACE:
 * @param [in/out]  LLA         [rad, rad, m], Lat, lon and alt of the antenna, and
 *                              will be converted to LLA of the IMU.
 * @param [in/out]  vNed        [m/s], NED velocity of the antenna, and will be
 *                              converted to NED velocity of the IMU.
 * @param [in]      w           [rad/s], angular velocity of the vehicle relative to
 *                              ECEF in the body frame.
 * @param [in]      leverArmB   [m], lever arm in the body frame.
 * @param [out]     rn2e        Transformation matrix from NED to ECEF.
 * @param [out]     ecef        [m], ECEF position without lever arm.
 * @retval TRUE if heading initialized, FALSE if not.
******************************************************************************/
static void RemoveLeverArm(double *lla, double *vNed, real *w, real *leverArmB,
                           real *rn2e, double *ecef);

//=============================================================================

/*  This routine is called at either 100 or 200 Hz based upon the system configuration:
 *   -- Unaided soln: 200 Hz
 *   -- Aided soln: 100 Hz
 */

void enableFreeIntegration(BOOL enable)
{
    gAlgorithm.Behavior.bit.freeIntegrate = enable;
}


BOOL freeIntegrationEnabled()
{
    return (BOOL)gAlgorithm.Behavior.bit.freeIntegrate;
}   

void enableMagInAlgorithm(BOOL enable)
{
    if(1)
    {
        gAlgorithm.Behavior.bit.useMag = enable;
    }
    else
    {
        gAlgorithm.Behavior.bit.useMag = FALSE;
    }
}

// BOOL magUsedInAlgorithm()
// {
//     return gAlgorithm.Behavior.bit.useMag != 0;
// }

BOOL gpsUsedInAlgorithm(void)
{
    return (BOOL)gAlgorithm.Behavior.bit.useGPS;
}

void enableGpsInAlgorithm(BOOL enable)
{
    gAlgorithm.Behavior.bit.useGPS = enable;
}

// Getters based on results structure passed to WriteResultsIntoOutputStream()

/* Extract the attitude (expressed as Euler-angles) of the body-frame (B)
 * in the NED-frame (N) in [deg].
 */
void EKF_GetAttitude_EA(real *EulerAngles)
{
    // Euler-angles in [deg]
    EulerAngles[ROLL]  = (real)gEKFOutput.eulerAngs_BinN[ROLL];
    EulerAngles[PITCH] = (real)gEKFOutput.eulerAngs_BinN[PITCH];
    EulerAngles[YAW]   = (real)gEKFOutput.eulerAngs_BinN[YAW];
}


void EKF_GetAttitude_EA_RAD(real *EulerAngles)
{
    // Euler-angles in [rad]
    EulerAngles[ROLL]  = (real)gKalmanFilter.eulerAngles[ROLL];
    EulerAngles[PITCH] = (real)gKalmanFilter.eulerAngles[PITCH];
    EulerAngles[YAW]   = (real) gKalmanFilter.eulerAngles[YAW];
}


/* Extract the attitude (expressed by quaternion-elements) of the body-
 * frame (B) in the NED-frame (N).
 */
void EKF_GetAttitude_Q(real *Quaternions)
{
    Quaternions[Q0] = (real)gEKFOutput.quaternion_BinN[Q0];
    Quaternions[Q1] = (real)gEKFOutput.quaternion_BinN[Q1];
    Quaternions[Q2] = (real)gEKFOutput.quaternion_BinN[Q2];
    Quaternions[Q3] = (real)gEKFOutput.quaternion_BinN[Q3];
}


/* Extract the angular-rate of the body (corrected for estimated rate-bias)
 * measured in the body-frame (B).
 */
void EKF_GetCorrectedAngRates(real *CorrAngRates_B)
{
    // Angular-rate in [deg/s]
    CorrAngRates_B[X_AXIS] = (real)gEKFOutput.corrAngRates_B[X_AXIS];
    CorrAngRates_B[Y_AXIS] = (real)gEKFOutput.corrAngRates_B[Y_AXIS];
    CorrAngRates_B[Z_AXIS] = (real)gEKFOutput.corrAngRates_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetCorrectedAccels(real *CorrAccels_B)
{
    // Acceleration in [m/s^2]
    CorrAccels_B[X_AXIS] = (real)gEKFOutput.corrAccel_B[X_AXIS];
    CorrAccels_B[Y_AXIS] = (real)gEKFOutput.corrAccel_B[Y_AXIS];
    CorrAccels_B[Z_AXIS] = (real)gEKFOutput.corrAccel_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetEstimatedAngRateBias(real *AngRateBias_B)
{
    // Angular-rate bias in [deg/sec]
    AngRateBias_B[X_AXIS] = (real)gEKFOutput.angRateBias_B[X_AXIS];
    AngRateBias_B[Y_AXIS] = (real)gEKFOutput.angRateBias_B[Y_AXIS];
    AngRateBias_B[Z_AXIS] = (real)gEKFOutput.angRateBias_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetEstimatedAccelBias(real *AccelBias_B)
{
    // Acceleration-bias in [m/s^2]
    AccelBias_B[X_AXIS] = (real)gEKFOutput.accelBias_B[X_AXIS];
    AccelBias_B[Y_AXIS] = (real)gEKFOutput.accelBias_B[Y_AXIS];
    AccelBias_B[Z_AXIS] = (real)gEKFOutput.accelBias_B[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedPosition(real *Position_N)
{
    // Position in [m]
    Position_N[X_AXIS] = (real)gEKFOutput.position_N[X_AXIS];
    Position_N[Y_AXIS] = (real)gEKFOutput.position_N[Y_AXIS];
    Position_N[Z_AXIS] = (real)gEKFOutput.position_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedVelocity(real *Velocity_N)
{
    // Velocity in [m/s]
    Velocity_N[X_AXIS] = (real)gEKFOutput.velocity_N[X_AXIS];
    Velocity_N[Y_AXIS] = (real)gEKFOutput.velocity_N[Y_AXIS];
    Velocity_N[Z_AXIS] = (real)gEKFOutput.velocity_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedLLA(double *LLA)
{
    // Velocity in [m/s]
    LLA[X_AXIS] = (double)gEKFOutput.llaDeg[X_AXIS];
    LLA[Y_AXIS] = (double)gEKFOutput.llaDeg[Y_AXIS];
    LLA[Z_AXIS] = (double)gEKFOutput.llaDeg[Z_AXIS];
}


/* Extract the Operational Mode of the Algorithm:
 *   0: Stabilize
 *   1: Initialize
 *   2: High-Gain VG/AHRS mode
 *   3: Low-Gain VG/AHRS mode
 *   4: INS operation
 */
void EKF_GetOperationalMode(uint8_t *EKF_OperMode)
{
    *EKF_OperMode = gEKFOutput.opMode;
}


// Extract the linear-acceleration and turn-switch flags
void EKF_GetOperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch)
{
    *EKF_LinAccelSwitch = gEKFOutput.linAccelSwitch;
    *EKF_TurnSwitch     = gEKFOutput.turnSwitchFlag;
}


// SETTERS: for EKF input and output structures
static void calc_mn(double a, double e2, double lat, double *M, double *N)
{
    double slat = sin(lat);
    double v = 1.0 - e2*slat*slat;
    *M = a*(1.0 - e2) / pow(v, 1.5);
    *N = a / sqrt(v);

}

// Populate the EKF input structure with sensor and GPS data (if used)
void EKF_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [m/s/s]
    gEKFInput.accel_B[X_AXIS]    = (real)accels[X_AXIS] * GRAVITY;
    gEKFInput.accel_B[Y_AXIS]    = (real)accels[Y_AXIS] * GRAVITY;
    gEKFInput.accel_B[Z_AXIS]    = (real)accels[Z_AXIS] * GRAVITY;

    // Angular-rate signal is in [rad/s]
    gEKFInput.angRate_B[X_AXIS]  = (real)rates[X_AXIS];
    gEKFInput.angRate_B[Y_AXIS]  = (real)rates[Y_AXIS];
    gEKFInput.angRate_B[Z_AXIS]  = (real)rates[Z_AXIS];

    // Magnetometer signal is in [G]
    // gEKFInput.magField_B[X_AXIS] = (real)mags[X_AXIS];
    // gEKFInput.magField_B[Y_AXIS] = (real)mags[Y_AXIS];
    // gEKFInput.magField_B[Z_AXIS] = (real)mags[Z_AXIS];
    // real tmp[2];
    // tmp[X_AXIS] = gEKFInput.magField_B[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
    // tmp[Y_AXIS] = gEKFInput.magField_B[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];
    // gEKFInput.magField_B[X_AXIS] = gMagAlign.SF[0] * tmp[X_AXIS] + gMagAlign.SF[1] * tmp[Y_AXIS];
    // gEKFInput.magField_B[Y_AXIS] = gMagAlign.SF[2] * tmp[X_AXIS] + gMagAlign.SF[3] * tmp[Y_AXIS];

    // ----- Input from the GPS goes here -----
    gEKFInput.gpsUpdate = gps->gpsUpdate;
    // gps->gpsUpdate = 0; //Prevent duplicate execution
    if (gEKFInput.gpsUpdate == 1)
    {
        gps->gpsUpdate = 0; //Prevent duplicate execution

        // Validity data
        gEKFInput.gpsFixType = gps->gpsFixType;

        // num of satellites
        gEKFInput.numSatellites = gps->numSatellites;

        gEKFInput.rovTime.time = gps->rovTime.time;
        gEKFInput.rovTime.msec = gps->rovTime.msec;
    
            // DW DEBUG
        uint32_t delay = 0;
        double dlat = 0.0, dlon = 0.0, dhgt = 0.0;
        // delay = (g_MCU_time.time - gEKFInput.rovTime.time)*1000 + g_MCU_time.msec - gEKFInput.rovTime.msec;
        // if (delay <= 0 && delay >= 2000) {
        //     gEKFInput.gpsFixType = 0;
        //     return;
        // }


        // double M = 0.0, N = 0.0;
   
        // double ds = delay*0.001;
        // calc_mn(E_MAJOR, E_ECC_SQ, gps->latitude, &M, &N);
        // dlat = (gps->vNed[0]*ds) / (M + gps->altitude);
        // dlon =  (gps->vNed[1]*ds) / ((N + gps->altitude) * cos(gps->latitude));
        // dhgt = gps->vNed[2]*ds;

        // ITOW data
        gEKFInput.week = gps->week;
        gEKFInput.itow = gps->itow + delay;

        
        // Lat/Lon/Alt data
        gEKFInput.llaAnt[LAT] = gps->latitude + dlat;
        gEKFInput.llaAnt[LON] = gps->longitude + dlon; 
        gEKFInput.llaAnt[ALT] = gps->altitude + dhgt;
        gEKFInput.geoidAboveEllipsoid = gps->geoidAboveEllipsoid;

        // Data quality measures
        gEKFInput.GPSHorizAcc = gps->GPSHorizAcc;
        gEKFInput.GPSVertAcc = gps->GPSVertAcc;
        gEKFInput.HDOP = gps->HDOP;
        gEKFInput.age = gps->sol_age;


        // Velocity data
        gEKFInput.vNedAnt[X_AXIS] = gps->vNed[X_AXIS];
        gEKFInput.vNedAnt[Y_AXIS] = gps->vNed[Y_AXIS];
        gEKFInput.vNedAnt[Z_AXIS] = gps->vNed[Z_AXIS];

        // Course and velocity data
        gEKFInput.rawGroundSpeed = (real)sqrt(SQUARE(gEKFInput.vNed[0]) +
                                                  SQUARE(gEKFInput.vNed[1]));// gps->rawGroundSpeed;
        gEKFInput.trueCourse = (real)gps->trueCourse;
        
        /* Remove lever arm effects in LLA/Velocity. To do this requires transformation matrix
         * from the body frame to the NED frame. Before heading initialized, lever arm cannot
         * be correctly removed. After heading initialized, there would be position jump if
         * initial heading is different from uninitlized one and the lever arm is large.
         * After heading intialized, the position/velocity could also be reinitialized, and
         * lever arm effects on the position/velocity are not corrected removed.
         * LLA without lever arm is used to update Rn2e/ECEF postion, and calculate relative
         * position in NED
         */
        if (gEKFInput.gpsFixType)
        {
            gEKFInput.lla[LAT] = gEKFInput.llaAnt[LAT];
            gEKFInput.lla[LON] = gEKFInput.llaAnt[LON];
            gEKFInput.lla[ALT] = gEKFInput.llaAnt[ALT];
            gEKFInput.vNed[0] = gEKFInput.vNedAnt[0];
            gEKFInput.vNed[1] = gEKFInput.vNedAnt[1];
            gEKFInput.vNed[2] = gEKFInput.vNedAnt[2];
            /* remove lever arm. Indeed, corrected angular rate should be used. Considering angular
             * bias is small, raw angular rate is used.
             */
            RemoveLeverArm( gEKFInput.lla, 
                            gEKFInput.vNed, 
                            gEKFInput.angRate_B, 
                            gAlgorithm.leverArmB, 
                            &gKalmanFilter.Rn2e[0][0],
                            gKalmanFilter.rGPS_E);

            /* Calculate relative position in the NED frame. The initial position is rGPS0_E which.
             * is determined when the algorithm first enters the INS mode (InitINSFilter).
             */
            ECEF_To_Base( &gKalmanFilter.rGPS0_E[0],
                          &gKalmanFilter.rGPS_E[0],
                          &gKalmanFilter.Rn2e[0][0],
                          &gKalmanFilter.rGPS_N[0]);
        }
    }
}


// Populate the EKF output structure with algorithm results
void EKF_SetOutputStruct(void)
{
    // ------------------ States ------------------

    // Position in [m]
    gEKFOutput.position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS];
    gEKFOutput.position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS];
    gEKFOutput.position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS];

    // Velocity in [m/s]
    gEKFOutput.velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS];
    gEKFOutput.velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS];
    gEKFOutput.velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS];

    // Position in [N/A]
    gEKFOutput.quaternion_BinN[Q0] = gKalmanFilter.quaternion[Q0];
    gEKFOutput.quaternion_BinN[Q1] = gKalmanFilter.quaternion[Q1];
    gEKFOutput.quaternion_BinN[Q2] = gKalmanFilter.quaternion[Q2];
    gEKFOutput.quaternion_BinN[Q3] = gKalmanFilter.quaternion[Q3];

    // Angular-rate bias in [deg/sec]
    gEKFOutput.angRateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] * RAD_TO_DEG;
    gEKFOutput.angRateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] * RAD_TO_DEG;
    gEKFOutput.angRateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] * RAD_TO_DEG;

    // Acceleration-bias in [m/s^2]
    gEKFOutput.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutput.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutput.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS];

    // ------------------ Derived variables ------------------

    // Euler-angles in [deg]
    gEKFOutput.eulerAngs_BinN[ROLL]  = gKalmanFilter.eulerAngles[ROLL] * RAD_TO_DEG;
    gEKFOutput.eulerAngs_BinN[PITCH] = gKalmanFilter.eulerAngles[PITCH] * RAD_TO_DEG;
    gEKFOutput.eulerAngs_BinN[YAW]   = gKalmanFilter.eulerAngles[YAW] * RAD_TO_DEG;

    // Angular-rate in [deg/s]
    gEKFOutput.corrAngRates_B[X_AXIS] = ( gEKFInput.angRate_B[X_AXIS] -
                                              gKalmanFilter.rateBias_B[X_AXIS] ) * RAD_TO_DEG;
    gEKFOutput.corrAngRates_B[Y_AXIS] = ( gEKFInput.angRate_B[Y_AXIS] -
                                              gKalmanFilter.rateBias_B[Y_AXIS] ) * RAD_TO_DEG;
    gEKFOutput.corrAngRates_B[Z_AXIS] = ( gEKFInput.angRate_B[Z_AXIS] -
                                              gKalmanFilter.rateBias_B[Z_AXIS] ) * RAD_TO_DEG;

    // Acceleration in [m/s^2]
    gEKFOutput.corrAccel_B[X_AXIS] = gEKFInput.accel_B[X_AXIS] - gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutput.corrAccel_B[Y_AXIS] = gEKFInput.accel_B[Y_AXIS] - gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutput.corrAccel_B[Z_AXIS] = gEKFInput.accel_B[Z_AXIS] - gKalmanFilter.accelBias_B[Z_AXIS];


    // ------------------ Algorithm flags ------------------
    gEKFOutput.opMode         = gAlgorithm.state;
    gEKFOutput.linAccelSwitch = gAlgorithm.linAccelSwitch;
    gEKFOutput.turnSwitchFlag = gAlgoStatus.bit.turnSwitch;

    // ------------------ Latitude and Longitude Data ------------------
    gEKFOutput.llaDeg[LAT] = gKalmanFilter.llaDeg[LAT];
    gEKFOutput.llaDeg[LON] = gKalmanFilter.llaDeg[LON];
    gEKFOutput.llaDeg[ALT] = gKalmanFilter.llaDeg[ALT];

    gEKFOutput.week = gAlgorithm.week;
    gEKFOutput.itow = gAlgorithm.itow;
    gEKFOutput.gnss_sol_type = gEKFInput.gpsFixType;
}


//
uint8_t InitINSFilter(void)
{
    real tmp[7][7];
    int rowNum, colNum;

#ifdef INS_OFFLINE
    printf("reset INS filter.\n");
#endif // INS_OFFLINE

    gAlgorithm.insFirstTime = FALSE;

    // Sync the algorithm and GPS ITOW
    gAlgorithm.itow = gEKFInput.itow;

    /* We have a good GPS reading now - set this variable so we
     * don't drop into INS right away
     */
    gAlgorithm.timeOfLastGoodGPSReading = gEKFInput.itow;

    /* Upon the first entry into INS, save off the base position and reset the
     * Kalman filter variables.
     */
    // Save off the base ECEF location
    gKalmanFilter.rGPS0_E[X_AXIS] = gKalmanFilter.rGPS_E[X_AXIS];
    gKalmanFilter.rGPS0_E[Y_AXIS] = gKalmanFilter.rGPS_E[Y_AXIS];
    gKalmanFilter.rGPS0_E[Z_AXIS] = gKalmanFilter.rGPS_E[Z_AXIS];

    // Reset the gps position (as position is relative to starting location)
    gKalmanFilter.rGPS_N[X_AXIS] = 0.0;
    gKalmanFilter.rGPS_N[Y_AXIS] = 0.0;
    gKalmanFilter.rGPS_N[Z_AXIS] = 0.0;

    // Reset prediction values. Position_N is also IMU position.
    gKalmanFilter.Position_N[X_AXIS] = (real)0.0;
    gKalmanFilter.Position_N[Y_AXIS] = (real)0.0;
    gKalmanFilter.Position_N[Z_AXIS] = (real)0.0;

    gKalmanFilter.Velocity_N[X_AXIS] = (real)gEKFInput.vNed[X_AXIS];
    gKalmanFilter.Velocity_N[Y_AXIS] = (real)gEKFInput.vNed[Y_AXIS];
    gKalmanFilter.Velocity_N[Z_AXIS] = (real)gEKFInput.vNed[Z_AXIS];

    gKalmanFilter.accelBias_B[X_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Y_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Z_AXIS] = (real)0.0;

    gKalmanFilter.linearAccel_B[X_AXIS] = (real)0.0;

    /* Extract the Quaternion and rate-bias values from the matrix before
     * resetting
     */
    // Save off the quaternion and rate-bias covariance values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) 
    {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) 
        {
            tmp[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0];
        }
    }

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) 
    {
        gKalmanFilter.P[rowNum][rowNum] = (real)INIT_P_INS;
    }

    // Repopulate the P matrix with the quaternion and rate-bias values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) 
    {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) 
        {
            gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0] = tmp[rowNum][colNum];
        }
    }

    /* Use the GPS-provided horizontal and vertical accuracy values to populate
     *   the covariance values.
     */
    gKalmanFilter.P[STATE_RX][STATE_RX] = gEKFInput.GPSHorizAcc * gEKFInput.GPSHorizAcc;
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RX][STATE_RX];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gEKFInput.GPSVertAcc * gEKFInput.GPSVertAcc;

    /* Scale the best velocity error by HDOP then multiply by the z-axis angular
     * rate PLUS one (to prevent the number from being zero) so the velocity
     * update during high-rate turns is reduced.
     */
    float temp = (real)0.0625 * gEKFInput.HDOP;  // 0.0625 = 0.05 / 0.8
    real absFilteredYawRate = (real)fabs(gAlgorithm.filteredYawRate);
    if (absFilteredYawRate > TEN_DEGREES_IN_RAD)
    {
        temp *= (1.0f + absFilteredYawRate);
    }
    gKalmanFilter.P[STATE_VX][STATE_VX] = temp;// *((real)1.0 + fabs(gAlgorithm.filteredYawRate) * (real)RAD_TO_DEG);
    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] * gKalmanFilter.P[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VX][STATE_VX];

    // z-axis velocity isn't really a function of yaw-rate and hdop
    //gKalmanFilter.R[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = (float)(0.1 * 0.1);

    return 1;
}

static void RemoveLeverArm(double *lla, double *vNed, real *w, real *leverArmB, real *rn2e, double *ecef)
{
    // Using position with lever arm to calculate rm and rn
    double sinLat = sin(lla[LAT]);
    double cosLat = cos(lla[LAT]);
    double tmp = 1.0 - (E_ECC_SQ * sinLat * sinLat);
    double sqrtTmp = sqrt(tmp);
    double rn = E_MAJOR / sqrtTmp; // radius of Curvature [meters]
    double rm = rn * (1.0 - E_ECC_SQ) / tmp;
    // Remove lever arm from position
    real leverArmN[3];  // lever arm in the NED frame
    leverArmN[0] =  gKalmanFilter.R_BinN[0][0] * leverArmB[0] +
                    gKalmanFilter.R_BinN[0][1] * leverArmB[1] +
                    gKalmanFilter.R_BinN[0][2] * leverArmB[2];
    leverArmN[1] =  gKalmanFilter.R_BinN[1][0] * leverArmB[0] +
                    gKalmanFilter.R_BinN[1][1] * leverArmB[1] +
                    gKalmanFilter.R_BinN[1][2] * leverArmB[2];
    leverArmN[2] =  gKalmanFilter.R_BinN[2][0] * leverArmB[0] +
                    gKalmanFilter.R_BinN[2][1] * leverArmB[1] +
                    gKalmanFilter.R_BinN[2][2] * leverArmB[2];
    lla[0] -= leverArmN[0] / rm;
    lla[1] -= leverArmN[1] / rn / cosLat;
    lla[2] += leverArmN[2];     /* Notice: lever arm is now in NED frame while altitude is
                                 * in the opposite direction of the z axis of NED frame.
                                 */

    /* Remove lever arm effects from velocity
     * v_gnss = v_imu + C_b2n * cross(wB, leverArmB)
     */
    cross(w, leverArmB, leverArmN);    // use leverArmN to temporatily hold w x leverArmB in body frame
    vNed[0] -=  gKalmanFilter.R_BinN[0][0] * leverArmN[0] +
                gKalmanFilter.R_BinN[0][1] * leverArmN[1] +
                gKalmanFilter.R_BinN[0][2] * leverArmN[2];
    vNed[1] -=  gKalmanFilter.R_BinN[1][0] * leverArmN[0] +
                gKalmanFilter.R_BinN[1][1] * leverArmN[1] +
                gKalmanFilter.R_BinN[1][2] * leverArmN[2];
    vNed[2] -=  gKalmanFilter.R_BinN[2][0] * leverArmN[0] +
                gKalmanFilter.R_BinN[2][1] * leverArmN[1] +
                gKalmanFilter.R_BinN[2][2] * leverArmN[2];

    // calcualte transfromation matrix from NED to ECEF
    sinLat = sin(lla[LAT]); // recalculate with LLA without lever arm
    cosLat = cos(lla[LAT]);
    double sinLon = sin(lla[LON]);
    double cosLon = cos(lla[LON]);

    real sinLat_r = (real)sinLat;
    real cosLat_r = (real)cosLat;
    real sinLon_r = (real)sinLon;
    real cosLon_r = (real)cosLon;

    // Form the transformation matrix from NED to ECEF 
    // First row
    *(rn2e + 0 * 3 + 0) = -sinLat_r * cosLon_r;
    *(rn2e + 0 * 3 + 1) = -sinLon_r;
    *(rn2e + 0 * 3 + 2) = -cosLat_r * cosLon_r;
    // Second row
    *(rn2e + 1 * 3 + 0) = -sinLat_r * sinLon_r;
    *(rn2e + 1 * 3 + 1) = cosLon_r;
    *(rn2e + 1 * 3 + 2) = -cosLat_r * sinLon_r;
    // Third row
    *(rn2e + 2 * 3 + 0) = cosLat_r;
    *(rn2e + 2 * 3 + 1) = 0.0;
    *(rn2e + 2 * 3 + 2) = -sinLat_r;

    // calculate ECEF position
    tmp = (rn + lla[ALT]) * cosLat;
    ecef[X_AXIS] = tmp * cosLon;
    ecef[Y_AXIS] = tmp * sinLon;
    ecef[Z_AXIS] = ((E_MINOR_OVER_MAJOR_SQ * (rn)) + lla[ALT]) * sinLat;
}


/* prediction functions */
/* F is sparse and has elements in the following locations...
 * There may be some more efficient ways of implementing this as this method
 * still performs multiplication with zero values.  (Ask Andrey)
 */
uint8_t RLE_F[ROWS_IN_F][2] = { {  STATE_RX, STATE_VX  },     // Row  0: cols 0,3
                                {  STATE_RY, STATE_VY  },     // Row  1: cols 1,4
                                {  STATE_RZ, STATE_VZ  },     // Row  2: cols 2,5
                                {  STATE_VX, STATE_ABZ },     // Row  3: cols 3,6:9,13:15
                                {  STATE_VY, STATE_ABZ },     // Row  4: cols 4,6:9,13:15
                                {  STATE_VZ, STATE_ABZ },     // Row  5: cols 5,6:9,13:15
                                {  STATE_Q0, STATE_WBZ },     // Row  6: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  7: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  8: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  9: cols 6:12
                                { STATE_WBX, STATE_WBX },     // Row 10: cols 10
                                { STATE_WBY, STATE_WBY },     // Row 11: cols 11
                                { STATE_WBZ, STATE_WBZ },     // Row 12: cols 12
                                { STATE_ABX, STATE_ABX },     // Row 13: cols 13
                                { STATE_ABY, STATE_ABY },     // Row 14: cols 14
                                { STATE_ABZ, STATE_ABZ } };   // Row 15: cols 15

// Q is sparse and has elements in the following locations...
uint8_t RLE_Q[ROWS_IN_F][2] = { {  STATE_RX, STATE_RX  },
                                {  STATE_RY, STATE_RY  },
                                {  STATE_RZ, STATE_RZ  },
                                {  STATE_VX, STATE_VX  },
                                {  STATE_VY, STATE_VY  },
                                {  STATE_VZ, STATE_VZ  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                { STATE_WBX, STATE_WBX },
                                { STATE_WBY, STATE_WBY },
                                { STATE_WBZ, STATE_WBZ },
                                { STATE_ABX, STATE_ABX },
                                { STATE_ABY, STATE_ABY },
                                { STATE_ABZ, STATE_ABZ } };

// Local functions
static void _PredictStateEstimate(void);
static void _PredictCovarianceEstimate(void);

static void _UpdateProcessJacobian(void);
static void _UpdateProcessCovariance(void);

// todo tm20160603 - use filters from filter.h, or move this filter there  (Ask Andrey)
void _FirstOrderLowPass(real *output, real input);

/* 16 States: [ STATE_RX,  STATE_RY,  STATE_RZ, ...
 *              STATE_VX,  STATE_VY,  STATE_VZ, ...
 *              STATE_Q0,  STATE_Q1,  STATE_Q2,  STATE_Q3, ...
 *              STATE_WBX, STATE_WBY, STATE_WBZ, ...
 *              STATE_ABX, STATE_ABY, STATE_ABZ ]
 */
//=============================================================================
//EKF_PredictionStage.m
void EKF_PredictionStage(real *filteredAccel)
{
    // real magFieldVector[3];

    // Propagate the state (22 usec) and covariance (1.82 msec) estimates
    _PredictStateEstimate();        // x(k+1) = x(k) + f(x(k), u(k))
    _PredictCovarianceEstimate();   // P = F*P*FTrans + Q

    // Extract the predicted Euler angles from the predicted quaternion
    QuaternionToEulerAngles( gKalmanFilter.eulerAngles,
                             gKalmanFilter.quaternion );

    /* Filter the yaw-rate at 200 Hz for the TURN-SWITCH (used in the
     * update stage only -- since that is a ten-hertz routine).  The way this
     * is coded, the filter function can only be used for filtering yaw-rate
     * data as the previous input state is saved as a static in the function.
     */
    _FirstOrderLowPass( &gAlgorithm.filteredYawRate,
                        gKalmanFilter.correctedRate_B[Z_AXIS] );

    /* Extract the magnetometer readings (set to zero if the magnetometer is not
     * present or unused).
     */
    // if(magUsedInAlgorithm())
    // {
    //     magFieldVector[X_AXIS] = (real)gEKFInput.magField_B[X_AXIS];
    //     magFieldVector[Y_AXIS] = (real)gEKFInput.magField_B[Y_AXIS];
    //     magFieldVector[Z_AXIS] = (real)gEKFInput.magField_B[Z_AXIS];
    // }
    // else
    // {
    //     magFieldVector[X_AXIS] = magFieldVector[Y_AXIS] = magFieldVector[Z_AXIS] = (real)0.0;
    // }

    /* Compute the measured Euler angles from gravity and magnetic field data
     * ( phiMeas, thetaMeas, psiMeas ) = f( g_B, mMeas_B ).  Adjust for declination.
     */
    // Compute the unit gravity vector (-accel) in the body frame 
    real unitGravityVector[3] = {0.0f};
    UnitGravity(filteredAccel, unitGravityVector);
    // Compute roll and pitch from the unit gravity vector.
    UnitGravityToEulerAngles(unitGravityVector, gKalmanFilter.measuredEulerAngles);
    /* Compute measured yaw.
     * If mag is not in use, yaw is the predicted yaw in the Kalman filter
     * If mag is in use, predicted roll and pitch are used to project mag and compute yaw in LG (chage to >=LG?),
     * measured pitch and roll (indeed the unit gravity vector from measured accel) are used to project
     * mag and compute yaw in other cases.
    */
    // if ( magUsedInAlgorithm() )
    // {
    //     // Transform the magnetic field vector from the body-frame to the plane normal to the gravity vector
    //     if ( gAlgorithm.state == LOW_GAIN_AHRS )
    //     {
    //         // Using predicted pitch and roll to project the mag measurement
    //         gKalmanFilter.measuredEulerAngles[YAW] =
    //             RollPitchAndMagToYaw( gKalmanFilter.eulerAngles[ROLL],
    //                                   gKalmanFilter.eulerAngles[PITCH],
    //                                   magFieldVector );
    //     }
    //     else
    //     {
    //         // Using accel measurement to project the mag measurement
    //         gKalmanFilter.measuredEulerAngles[YAW] =
    //             UnitGravityAndMagToYaw( unitGravityVector,
    //                                     magFieldVector );
    //     }
    // }
    // else
    {
        // For VG, set the measured heading to the predicted heading (this
        //   forces the error to zero)
        gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.eulerAngles[YAW];
    }


    // Adjust for declination if the GPS signal is good
    // if( gAlgorithm.applyDeclFlag ) 
    // {
    //     gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW] +
    //                                              gWorldMagModel.decl_rad;
    // }
}


/* Predict the EKF states at 100 Hz based on readings from the:
 *  - accelerometer
 *  - angular-rate sensors
 */
static void _PredictStateEstimate(void)
{
    real aCorr_N[3];
    real deltaQuaternion[4];

    if( gAlgorithm.state > LOW_GAIN_AHRS ) 
    {
        // ================= NED Position (r_N) =================
        // r_N(k+1) = r_N(k) + dV_N = r_N(k) + v_N*DT
        gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] +
                                           gKalmanFilter.Velocity_N[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] +
                                           gKalmanFilter.Velocity_N[Y_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] +
                                           gKalmanFilter.Velocity_N[Z_AXIS] * gAlgorithm.dt;

        // ================= NED Velocity (v_N) =================
        // aCorr_B = aMeas_B - aBias_B
        // gEKFInput.accel_B in g's, convert to m/s^2 for integration
        gKalmanFilter.correctedAccel_B[X_AXIS] = gEKFInput.accel_B[X_AXIS] -
                                                 gKalmanFilter.accelBias_B[X_AXIS];
        gKalmanFilter.correctedAccel_B[Y_AXIS] = gEKFInput.accel_B[Y_AXIS] -
                                                 gKalmanFilter.accelBias_B[Y_AXIS];
        gKalmanFilter.correctedAccel_B[Z_AXIS] = gEKFInput.accel_B[Z_AXIS] -
                                                 gKalmanFilter.accelBias_B[Z_AXIS];

        /* Transform the corrected acceleration vector from the body to the NED-frame and remove gravity
         * a_N = R_BinN * a_B
         */
        aCorr_N[X_AXIS] =
                gKalmanFilter.R_BinN[X_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[X_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[X_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
        aCorr_N[Y_AXIS] =
                gKalmanFilter.R_BinN[Y_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[Y_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[Y_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
        aCorr_N[Z_AXIS] =
                gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS] +
                (real)GRAVITY;

        /* Determine the acceleration of the system by removing the gravity vector
         * v_N(k+1) = v_N(k) + dV = v_N(k) + aMotion_N*DT = v_N(k) + ( a_N - g_N )*DT
         */
        gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + aCorr_N[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + aCorr_N[Y_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + aCorr_N[Z_AXIS] * gAlgorithm.dt;
        
        // Calculate linear acceleration in the body frame.
        gKalmanFilter.linearAccel_B[X_AXIS] += (gKalmanFilter.correctedAccel_B[X_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * (real)GRAVITY)*gAlgorithm.dt;
        gKalmanFilter.linearAccel_B[Y_AXIS] = gKalmanFilter.correctedAccel_B[Y_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * (real)GRAVITY;
        gKalmanFilter.linearAccel_B[Z_AXIS] = gKalmanFilter.correctedAccel_B[Z_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * (real)GRAVITY;
    }
    else
    {
        // GPS not valid yet, do not propagate the position or velocity
        gKalmanFilter.Position_N[X_AXIS] = (real)0.0;
        gKalmanFilter.Position_N[Y_AXIS] = (real)0.0;
        gKalmanFilter.Position_N[Z_AXIS] = (real)0.0;

        gKalmanFilter.Velocity_N[X_AXIS] = (real)0.0;
        gKalmanFilter.Velocity_N[Y_AXIS] = (real)0.0;
        gKalmanFilter.Velocity_N[Z_AXIS] = (real)0.0;

        // what should this be???
        gKalmanFilter.correctedAccel_B[XACCEL] = gEKFInput.accel_B[X_AXIS];
        gKalmanFilter.correctedAccel_B[YACCEL] = gEKFInput.accel_B[Y_AXIS];
        gKalmanFilter.correctedAccel_B[ZACCEL] = gEKFInput.accel_B[Z_AXIS];
    }

    // ================= Attitude quaternion =================
    // Find the 'true' angular rate (wTrue_B = wCorr_B = wMeas_B - wBias_B)
    gKalmanFilter.correctedRate_B[X_AXIS] = gEKFInput.angRate_B[X_AXIS] -
                                            gKalmanFilter.rateBias_B[X_AXIS];
    gKalmanFilter.correctedRate_B[Y_AXIS] = gEKFInput.angRate_B[Y_AXIS] -
                                            gKalmanFilter.rateBias_B[Y_AXIS];
    gKalmanFilter.correctedRate_B[Z_AXIS] = gEKFInput.angRate_B[Z_AXIS] -
                                            gKalmanFilter.rateBias_B[Z_AXIS];

    // Placed in gKalmanFilter as wTrueTimesDtOverTwo is used to compute the Jacobian (F)
    gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] = gKalmanFilter.correctedRate_B[X_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] = gKalmanFilter.correctedRate_B[Y_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] = gKalmanFilter.correctedRate_B[Z_AXIS] * gAlgorithm.dtOverTwo;

    // Find the attitude change based on angular rate data
    deltaQuaternion[Q0] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q1] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q2] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q0] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q1] +
                           gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q3] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q2];

    // Update the attitude
    // q_BinN(k+1) = q_BinN(k) + dq = q_BinN(k) + OMEGA*q_BinN(k)
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + deltaQuaternion[Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + deltaQuaternion[Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + deltaQuaternion[Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + deltaQuaternion[Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    // ================= Angular-rate bias: wBias(k+1) = wBias(k) =================
    // N/A (predicted state is same as past state)

    // ================= Linear-acceleration bias: aBias(k+1) = aBias(k) =================
    // N/A (predicted state is same as past state)
}


// Define variables that reside on the heap
real PxFTranspose[ROWS_IN_P][ROWS_IN_F], FxPxFTranspose[ROWS_IN_F][ROWS_IN_F];

// PredictCovarianceEstimate.m
static void _PredictCovarianceEstimate(void)
{
    uint8_t rowNum, colNum, multIndex;

    /* Compute the F and Q matrices used in the prediction stage (only certain
     * elements in the process-covariance, Q, change with each time-step)
     */
    _UpdateProcessJacobian();     // gKF.F  (16x16)
    _UpdateProcessCovariance();   // gKF.Q  (16x16)

    // Update P from the P, F, and Q matrices: P = FxPxFTranspose + Q
    // 1) PxFTranspose is computed first
    memset(PxFTranspose, 0, sizeof(PxFTranspose));
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) 
    {
        for (colNum = 0; colNum < ROWS_IN_F; colNum++) 
        {
            for (multIndex = RLE_F[colNum][0]; multIndex <= RLE_F[colNum][1]; multIndex++) 
            {
                PxFTranspose[rowNum][colNum] = PxFTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.F[colNum][multIndex];
            }
        }
    }

#if 0
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < 16; rowNum++) 
    {
        for (colNum = 0; colNum < 16; colNum++) 
        {
            gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) 
            {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
        }
    }

    // P is a fully populated matrix (nominally) so all the elements of the matrix have to be
    //   considered when working with it.
    LimitValuesAndForceMatrixSymmetry_noAvg(&gKalmanFilter.P[0][0], (real)LIMIT_P, ROWS_IN_P, COLS_IN_P);
#else
    /* 2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
     * to reduce the number of "large" variables on the heap.
     * This matrix is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < 16; rowNum++) 
    {
        for (colNum = rowNum; colNum < 16; colNum++) 
        {
            //gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) 
            {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
            gKalmanFilter.P[colNum][rowNum] = gKalmanFilter.P[rowNum][colNum];
             //Limit values in P
            if(gKalmanFilter.P[rowNum][colNum] > (real)LIMIT_P) 
            {
                gKalmanFilter.P[rowNum][colNum] = (real)LIMIT_P;
            }
            else if(gKalmanFilter.P[rowNum][colNum] < -(real)LIMIT_P)
            {
                gKalmanFilter.P[rowNum][colNum] = -(real)LIMIT_P;
            }
        }
    }
#endif
    
    /* 3) Finally, add Q to FxPxFTranspose (P) to get the final value for
     * gKalmanFilter.P (only the quaternion elements of Q have nonzero off-
     * diagonal terms)
     */
    gKalmanFilter.P[STATE_RX][STATE_RX] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.Q[STATE_RX];
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.Q[STATE_RY];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.Q[STATE_RZ];

    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.Q[STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.Q[STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.Q[STATE_VZ];

    gKalmanFilter.P[STATE_Q0][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.Q[STATE_Q0];
    gKalmanFilter.P[STATE_Q0][STATE_Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1] + gKalmanFilter.Qq[0];
    gKalmanFilter.P[STATE_Q0][STATE_Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2] + gKalmanFilter.Qq[1];
    gKalmanFilter.P[STATE_Q0][STATE_Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3] + gKalmanFilter.Qq[2];

    //gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0] + gKalmanFilter.Q[STATE_Q1][STATE_Q0];
    gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.Q[STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2] + gKalmanFilter.Qq[3];
    gKalmanFilter.P[STATE_Q1][STATE_Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3] + gKalmanFilter.Qq[4];

    //gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0] + gKalmanFilter.Q[STATE_Q2][STATE_Q0];
    //gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1] + gKalmanFilter.Q[STATE_Q2][STATE_Q1];
    gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.Q[STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3] + gKalmanFilter.Qq[5];

    //gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0] + gKalmanFilter.Q[STATE_Q3][STATE_Q0];
    //gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1] + gKalmanFilter.Q[STATE_Q3][STATE_Q1];
    //gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2] + gKalmanFilter.Q[STATE_Q3][STATE_Q2];
    gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.Q[STATE_Q3];

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = gKalmanFilter.P[STATE_WBX][STATE_WBX] + gKalmanFilter.Q[STATE_WBX];
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = gKalmanFilter.P[STATE_WBY][STATE_WBY] + gKalmanFilter.Q[STATE_WBY];
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = gKalmanFilter.P[STATE_WBZ][STATE_WBZ] + gKalmanFilter.Q[STATE_WBZ];

    gKalmanFilter.P[STATE_ABX][STATE_ABX] = gKalmanFilter.P[STATE_ABX][STATE_ABX] + gKalmanFilter.Q[STATE_ABX];
    gKalmanFilter.P[STATE_ABY][STATE_ABY] = gKalmanFilter.P[STATE_ABY][STATE_ABY] + gKalmanFilter.Q[STATE_ABY];
    gKalmanFilter.P[STATE_ABZ][STATE_ABZ] = gKalmanFilter.P[STATE_ABZ][STATE_ABZ] + gKalmanFilter.Q[STATE_ABZ];
}


// GenerateProcessJacobian.m: Set the elements of F that DO NOT change with each time-step
void GenerateProcessJacobian(void)
{
    // Initialize the Process Jacobian matrix (F)
    memset(gKalmanFilter.F, 0, sizeof(gKalmanFilter.F));

    // Form the process Jacobian

    // ---------- Rows corresponding to POSITION ----------
    gKalmanFilter.F[STATE_RX][STATE_VX] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RY][STATE_VY] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RZ][STATE_VZ] = gAlgorithm.dt;

    // ---------- Rows corresponding to VELOCITY ----------
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to ATTITUDE ----------
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to RATE-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Add to I16 to get final formulation of F ----------
    // Populate the diagonals of F with 1.0
    gKalmanFilter.F[STATE_RX][STATE_RX] = (real)1.0;
    gKalmanFilter.F[STATE_RY][STATE_RY] = (real)1.0;
    gKalmanFilter.F[STATE_RZ][STATE_RZ] = (real)1.0;

    gKalmanFilter.F[STATE_VX][STATE_VX] = (real)1.0;
    gKalmanFilter.F[STATE_VY][STATE_VY] = (real)1.0;
    gKalmanFilter.F[STATE_VZ][STATE_VZ] = (real)1.0;

    gKalmanFilter.F[STATE_Q0][STATE_Q0] = (real)1.0;
    gKalmanFilter.F[STATE_Q1][STATE_Q1] = (real)1.0;
    gKalmanFilter.F[STATE_Q2][STATE_Q2] = (real)1.0;
    gKalmanFilter.F[STATE_Q3][STATE_Q3] = (real)1.0;

    gKalmanFilter.F[STATE_WBX][STATE_WBX] = (real)1.0;
    gKalmanFilter.F[STATE_WBY][STATE_WBY] = (real)1.0;
    gKalmanFilter.F[STATE_WBZ][STATE_WBZ] = (real)1.0;

    gKalmanFilter.F[STATE_ABX][STATE_ABX] = (real)1.0;
    gKalmanFilter.F[STATE_ABY][STATE_ABY] = (real)1.0;
    gKalmanFilter.F[STATE_ABZ][STATE_ABZ] = (real)1.0;
}


// _UpdateProcessJacobian.m: Update the elements of F that change with each time-step
static void _UpdateProcessJacobian(void)
{
    real q0aXdT, q1aXdT, q2aXdT, q3aXdT;
    real q0aYdT, q1aYdT, q2aYdT, q3aYdT;
    real q0aZdT, q1aZdT, q2aZdT, q3aZdT;
    real q0DtOver2, q1DtOver2, q2DtOver2, q3DtOver2;

    // ---------- Rows corresponding to POSITION ----------
    // No updates

    // ---------- Rows corresponding to VELOCITY ----------
    // Columns corresponding to the attitude-quaternion states
    q0aXdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q1aXdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q2aXdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q3aXdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;

    q0aYdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q1aYdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q2aYdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q3aYdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;

    q0aZdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q1aZdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q2aZdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q3aZdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;

#if 1
    // mod, DXG
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q0aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q1aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q2aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * ( -q3aXdT - q0aYdT + q1aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q3aZdT);

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q0aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (  q2aXdT - q1aYdT - q0aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q1aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q2aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q3aZdT);

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q0aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q1aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * ( -q0aXdT + q3aYdT - q2aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q2aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q3aZdT);
#else
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (q0aXdT - q3aYdT + q2aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * (-q2aXdT + q1aYdT + q0aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * (-q3aXdT - q0aYdT + q1aZdT);

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (q3aXdT + q0aYdT - q1aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (q2aXdT - q1aYdT - q0aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (q0aXdT - q3aYdT + q2aZdT);

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * (-q2aXdT + q1aYdT + q0aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (q3aXdT + q0aYdT - q1aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * (-q0aXdT + q3aYdT - q2aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
#endif

    // Columns corresponding to the acceleration-bias state (-R_BinN*DT)
    gKalmanFilter.F[STATE_VX][STATE_ABX] = -gKalmanFilter.R_BinN[0][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABY] = -gKalmanFilter.R_BinN[0][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABZ] = -gKalmanFilter.R_BinN[0][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VY][STATE_ABX] = -gKalmanFilter.R_BinN[1][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABY] = -gKalmanFilter.R_BinN[1][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABZ] = -gKalmanFilter.R_BinN[1][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VZ][STATE_ABX] = -gKalmanFilter.R_BinN[2][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABY] = -gKalmanFilter.R_BinN[2][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABZ] = -gKalmanFilter.R_BinN[2][2] * gAlgorithm.dt;

    // ---------- Rows corresponding to attitude-QUATERNION ----------
    // Columns corresponding to the attitude-quaternion state (0.5*Omega*DT)
    //gKalmanFilter.F[STATE_Q0][STATE_Q0] =     0;
    gKalmanFilter.F[STATE_Q0][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];

    gKalmanFilter.F[STATE_Q1][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q1][STATE_Q1] =     0;
    gKalmanFilter.F[STATE_Q1][STATE_Q2] = gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q1][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];

    gKalmanFilter.F[STATE_Q2][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q2][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    //gKalmanFilter.F[STATE_Q2][STATE_Q2] =     0;
    gKalmanFilter.F[STATE_Q2][STATE_Q3] = gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];

    gKalmanFilter.F[STATE_Q3][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q1] = gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q3,STATE.Q3] =     0;

    // Columns corresponding to the rate-bias state (-0.5*Xi*DT)
    q0DtOver2 = gKalmanFilter.quaternion_Past[Q0] * gAlgorithm.dtOverTwo;
    q1DtOver2 = gKalmanFilter.quaternion_Past[Q1] * gAlgorithm.dtOverTwo;
    q2DtOver2 = gKalmanFilter.quaternion_Past[Q2] * gAlgorithm.dtOverTwo;
    q3DtOver2 = gKalmanFilter.quaternion_Past[Q3] * gAlgorithm.dtOverTwo;

    // 
    gKalmanFilter.F[STATE_Q0][STATE_WBX] =  q1DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBY] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBZ] =  q3DtOver2;

    gKalmanFilter.F[STATE_Q1][STATE_WBX] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBY] =  q3DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBZ] = -q2DtOver2;

    gKalmanFilter.F[STATE_Q2][STATE_WBX] = -q3DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBY] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBZ] =  q1DtOver2;

    gKalmanFilter.F[STATE_Q3][STATE_WBX] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBY] = -q1DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBZ] = -q0DtOver2;

    // ---------- Rows corresponding to RATE-BIAS ----------
    // All zeros

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // All zeros
}

// 
static void _UpdateProcessCovariance(void)
{
    // Variables used to initially populate the Q-matrix
    real biSq[3] = {(real)1.0e-10, (real)1.0e-10, (real)1.0e-10};
    real sigDriftDot;

    // Variables used to populate the Q-matrix each time-step
    static real multiplier_Q, multiplier_Q_Sq;

    static BOOL initQ_HG = TRUE;
    static BOOL initQ_LG = TRUE;

    // Only need to generate Q-Bias values once
    if (initQ_HG == TRUE) 
    {
        initQ_HG = FALSE;

        // squated gyro bias instability
        biSq[X_AXIS] = gAlgorithm.imuSpec.biW * gAlgorithm.imuSpec.biW;
        biSq[Y_AXIS] = biSq[X_AXIS];
        biSq[Z_AXIS] = biSq[X_AXIS];


        /* Rate-bias terms (computed once as it does not change with attitude). 
         *   sigDriftDot = (2*pi/ln(2)) * BI^2 / ARW
         *   2*pi/ln(2) = 9.064720283654388
         */
        sigDriftDot = (real)9.064720283654388 / gAlgorithm.imuSpec.arw;

        // Rate-bias terms (Q is ultimately the squared value, which is done in the second line of the assignment)
        gKalmanFilter.Q[STATE_WBX] = sigDriftDot * biSq[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBX] = gKalmanFilter.Q[STATE_WBX] * gKalmanFilter.Q[STATE_WBX];

        gKalmanFilter.Q[STATE_WBY] = gKalmanFilter.Q[STATE_WBX];

        gKalmanFilter.Q[STATE_WBZ] = sigDriftDot * biSq[Z_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBZ] = gKalmanFilter.Q[STATE_WBZ] * gKalmanFilter.Q[STATE_WBZ];

        gKalmanFilter.Q[STATE_ABX] = (real)1.0e-12;
        gKalmanFilter.Q[STATE_ABY] = (real)1.0e-12;
        gKalmanFilter.Q[STATE_ABZ] = (real)1.0e-12;

        /* Precalculate the multiplier applied to the Q terms associated with
         * attitude.  sigRate = ARW / sqrt( dt )
         * mult = 0.5 * dt * sigRate
         *      = 0.5 * sqrt(dt) * sqrt(dt) * ( ARW / sqrt(dt) )
         *      = 0.5 * sqrt(dt) * ARW
         */
        multiplier_Q = (real)(0.5) * gAlgorithm.sqrtDt * gAlgorithm.imuSpec.arw;
        multiplier_Q_Sq = multiplier_Q * multiplier_Q;
    }

    /* Attempt to solve the rate-bias problem: Decrease the process covariance,
     * Q, upon transition to low-gain mode to limit the change in the rate-bias
     * estimate.
     */
    if( initQ_LG == TRUE ) 
    {
        if( gAlgorithm.state == LOW_GAIN_AHRS ) 
        {
            initQ_LG = FALSE;

            /* After running drive-test data through the AHRS simulation,
             * reducing Q by 1000x reduced the changeability of the rate-bias
             * estimate (the estimate was less affected by errors).  This
             * seems to have improved the solution as much of the errors
             * was due to rapid changes in the rate-bias estimate.  This
             * seemed to result in a better than nominal solution (for the
             * drive-test).  Note: this is only called upon the first-entry
             * into low-gain mode.
             */
            /*gKalmanFilter.Q[STATE_WBX] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBX];
            gKalmanFilter.Q[STATE_WBY] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBY];
            gKalmanFilter.Q[STATE_WBZ] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBZ];*/
        }
    }

    /* Update the elements of the process covariance matrix, Q, that change
     * with each time-step (the elements that correspond to the quaternion-
     * block of the Q-matrix).  The rest of the elements in the matrix are set
     * during the transition into and between EKF states (high-gain, low-gain,
     * etc) or above (upon first entry into this function).
     * The process cov matrix of quaternion is
     *          [1-q0*q0    -q0*q1      -q0*q2      -q0*q3;
     *          -q0*q1      1-q1*q1     -q1*q2      -q1*q3;
     *          -q0*q2      -q1*q2      1-q2*q2     -q2*q3;
     *          -q0*q3      -q1*q3      -q2*q3      1-q3*q3] * (0.5*dt*sigma_gyro)^2
     * The eigenvalue of the matrix is [1 1 1 1-q0^2-q1^2-q2^2-q3^2], which means it
     * is not positive defintie when quaternion norm is above or equal to 1. Quaternion
     * norm can be above 1 due to numerical accuray. A scale factor 0.99 is added here to
     * make sure the positive definiteness of the covariance matrix. The eigenvalues now
     * are [1 1 1 1-0.99*(q0^2+q1^2+q2^2+q3^2)]. Even if there is numerical accuracy issue,
     * the cov matrix is still positive definite.
     */
    real q0q0 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q0] * 0.99f;
    real q0q1 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q1] * 0.99f;
    real q0q2 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q0q3 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q1q1 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q1] * 0.99f;
    real q1q2 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q1q3 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q2q2 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q2q3 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q3q3 = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    // Note: this block of the covariance matrix is symmetric
    real tmpQMultiplier = multiplier_Q_Sq;
    /* Only considering gyro noise can underestimate the cov of the quaternion.
     * A scale factor 100 is added here. This is mainly for faster convergence
     * of the heading angle in the INS solution.
     */
    if (gAlgorithm.state == INS_SOLUTION)
    {
        tmpQMultiplier = 1.0f * multiplier_Q_Sq;
    }
    gKalmanFilter.Q[STATE_Q0] = ((real)1.0 - q0q0) * tmpQMultiplier;
    gKalmanFilter.Qq[0] = (-q0q1) * tmpQMultiplier;
    gKalmanFilter.Qq[1] = (-q0q2) * tmpQMultiplier;
    gKalmanFilter.Qq[2] = (-q0q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q1] = ((real)1.0 - q1q1) * tmpQMultiplier;
    gKalmanFilter.Qq[3] = (-q1q2) * tmpQMultiplier;
    gKalmanFilter.Qq[4] = (-q1q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q2] = ((real)1.0 - q2q2) * tmpQMultiplier;
    gKalmanFilter.Qq[5] = (-q2q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q3] = ((real)1.0 - q3q3) * tmpQMultiplier;
}


//
// GenerateProcessCovarMatrix.m
void GenerateProcessCovariance(void)
{
    // Initialize the Process Covariance (Q) matrix with values that do not change
    memset(gKalmanFilter.Q, 0, sizeof(gKalmanFilter.Q));

    /* THE FOLLOWING COVARIANCE VALUES AREN'T CORRECT, JUST SELECTED SO THE
     * PROGRAM COULD RUN
     */

    // Acceleration based values
    real dtSigAccelSq = (real)(gAlgorithm.dt * gAlgorithm.imuSpec.sigmaA);
    dtSigAccelSq = dtSigAccelSq * dtSigAccelSq;

    // Position
    gKalmanFilter.Q[STATE_RX] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RY] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RZ] = gAlgorithm.dtSquared * dtSigAccelSq;

    /* Velocity, todo. 100 is to take under-estimated accel bias, gyro bias and
     * attitude error since none of them is Gaussian. Non-Gaussian error produces
     * velocity drift. High-freq vibration can also be handled by this.
     */
    gKalmanFilter.Q[STATE_VX] = 100 * dtSigAccelSq;//(real)1e-10;
    gKalmanFilter.Q[STATE_VY] = 100 * dtSigAccelSq;
    gKalmanFilter.Q[STATE_VZ] = 100 * dtSigAccelSq;

    // Acceleration - bias
    gKalmanFilter.Q[STATE_ABX] = (real)5e-11; //(real)1e-10; // dtSigAccelSq; //%1e-8 %sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABY] = (real)5e-11; //(real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABZ] = (real)5e-11; //(real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
}


/** ****************************************************************************
 * @name: firstOrderLowPass_float  implements a low pass yaw axis filter
 * @brief floating point version
 * TRACE:
 * @param [out] - output pointer to the filtered value
 * @param [in] - input pointer to a new raw value
 * @retval N/A
 * @details  This is a replacement for 'lowPass2Pole' found in the 440 code.
 *           Note, the implementation in the 440 SW is not a second-order filter
 *           but is an implementation of a first-order filter.
 ******************************************************************************/
void _FirstOrderLowPass( real *output,   // <-- INITIALLY THIS IS OUTPUT PAST (FILTERED VALUED IS RETURNED HERE)
                         real input )   // <-- CURRENT VALUE OF SIGNAL TO BE FILTERED
{
    static real inputPast;

    // 0.25 Hz LPF
    if( gAlgorithm.callingFreq == FREQ_100_HZ ) 
    {
        *output = (real)(0.984414127416097) * (*output) + 
                  (real)(0.007792936291952) * (input + inputPast);
    } 
    else 
    {
        *output = (real)(0.992176700177507) * (*output) + 
                  (real)(0.003911649911247) * (input + inputPast);
    }

    inputPast = input;
}







/******************************************************************************
 * @brief Detect zero velocity using IMU data.
 * TRACE:
 * @param [in]  gyroVar     variance of gyro    [rad/s]^2
 * @param [in]  gyroMean    mean of gyro        [rad/s]
 * @param [in]  accelVar    variance of accel   [m/s/s]^2
 * @param [in]  threshold   threshold to detect zero velocity
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticIMU(real *gyroVar, real *gyroMean, real *accelVar, STATIC_DETECT_SETTING *threshold);


void MotionStatusImu(real *gyro, real *accel, ImuStatsStruct *imuStats, BOOL reset)
{
    static BOOL bIni = false;                       // indicate the routine is initialized or not
    // Buffer to store input IMU data. Data in buffer is ued to calcualte filtered IMU dat.
    static real dAccel[3][FILTER_ORDER];            // a section in memory as buffer to store accel data
    static real dGyro[3][FILTER_ORDER];             // a section in memory as buffer to store gyro data
    static Buffer bfAccel;                          // a ring buffer of accel
    static Buffer bfGyro;                           // a ring buffer of gyro
    // Buffer to store filtered IMU data. Data in buffer is used to calculate IMU stats.
    static real dLpfAccel[3][SAMPLES_FOR_STATS];    // a section in memory as buffer to store accel data
    static real dLpfGyro[3][SAMPLES_FOR_STATS];     // a section in memory as buffer to store gyro data
    static Buffer bfLpfAccel;                       // a ring buffer of accel
    static Buffer bfLpfGyro;                        // a ring buffer of gyro
    // filter coefficients. y/x = b/a
    static real b_AccelFilt[FILTER_ORDER + 1];
    static real a_AccelFilt[FILTER_ORDER + 1];

    // reset the calculation of motion stats
    if (reset)
    {
        bIni = false;
    }

    // initialization
    if (!bIni)
    {
        bIni = true;
        // reset stats
        imuStats->bValid = false;
        imuStats->accelMean[0] = 0.0;
        imuStats->accelMean[1] = 0.0;
        imuStats->accelMean[2] = 0.0;
        imuStats->accelVar[0] = 0.0;
        imuStats->accelVar[1] = 0.0;
        imuStats->accelVar[2] = 0.0;
        imuStats->gyroMean[0] = 0.0;
        imuStats->gyroMean[1] = 0.0;
        imuStats->gyroMean[2] = 0.0;
        imuStats->gyroVar[0] = 0.0;
        imuStats->gyroVar[1] = 0.0;
        imuStats->gyroVar[2] = 0.0;
        // create/reset buffer
        bfNew(&bfGyro, &dGyro[0][0], 3, FILTER_ORDER);
        bfNew(&bfAccel, &dAccel[0][0], 3, FILTER_ORDER);
        bfNew(&bfLpfGyro, &dLpfGyro[0][0], 3, SAMPLES_FOR_STATS);
        bfNew(&bfLpfAccel, &dLpfAccel[0][0], 3, SAMPLES_FOR_STATS);
        // Set the filter coefficients based on selected cutoff frequency and sampling rate
        _PopulateFilterCoefficients(gAlgorithm.linAccelLPFType, gAlgorithm.callingFreq, b_AccelFilt, a_AccelFilt);
    }

    /* Low-pass filter.
     * The input IMU data is put into the buffer, and then filtered.
     */
    LowPassFilter(gyro, &bfGyro, &bfLpfGyro, b_AccelFilt, a_AccelFilt, imuStats->lpfGyro);
    LowPassFilter(accel, &bfAccel, &bfLpfAccel, b_AccelFilt, a_AccelFilt, imuStats->lpfAccel);

    /* Compute accel norm using raw accel data. 
     * The norm will be used to detect static periods via magnitude.
     */
    imuStats->accelNorm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    /* Compute mean/variance from input data.
     * The latest will be put into the buffer, and stats of data in buffer is then calculated.
     * When the input data buffer is full, the stats can be assumed valid.
     */
    ComputeStats(&bfLpfGyro, imuStats->lpfGyro, imuStats->gyroMean, imuStats->gyroVar);
    ComputeStats(&bfLpfAccel, imuStats->lpfAccel, imuStats->accelMean, imuStats->accelVar);
    imuStats->bValid = bfLpfGyro.full && bfLpfAccel.full;

    // Detect static period using var calculated above.
    imuStats->bStaticIMU = DetectStaticIMU( imuStats->gyroVar,
                                            imuStats->gyroMean,
                                            imuStats->accelVar,
                                            &gAlgorithm.staticDetectParam);
}

BOOL DetectStaticIMU(real *gyroVar, real *gyroMean, real *accelVar, STATIC_DETECT_SETTING *threshold)
{
    BOOL bStatic = TRUE;
    int i;
    static float multiplier[3] = { 0.0, 0.0, 0.0 };
    static float gyroVarThreshold = 0.0;
    static float accelVarThreshold = 0.0;
    static float gyroBiasThreshold = 0.0;
    // Update threshold
    if (multiplier[0] != threshold->staticNoiseMultiplier[0])
    {
        multiplier[0] = threshold->staticNoiseMultiplier[0];
        gyroVarThreshold = multiplier[0] * threshold->staticVarGyro;
    }
    if (multiplier[1] != threshold->staticNoiseMultiplier[1])
    {
        multiplier[1] = threshold->staticNoiseMultiplier[1];
        accelVarThreshold = multiplier[1] * threshold->staticVarAccel;
    }
    if (multiplier[2] != threshold->staticNoiseMultiplier[2])
    {
        multiplier[2] = threshold->staticNoiseMultiplier[2];
        gyroBiasThreshold = multiplier[2] * threshold->maxGyroBias;
    }

    for (i = 0; i < 3; i++)
    {
        if (gyroVar[i] > gyroVarThreshold ||
            accelVar[i] > accelVarThreshold ||
            fabs(gyroMean[i]) > gyroBiasThreshold)
        {
            bStatic = FALSE;
            break;
        }
    }

    return bStatic;
}




void EstimateAccelError(real *accel, real *w, real dt, uint32_t staticDelay, ImuStatsStruct *imuStats)
{
    static BOOL bIni = false;               // indicate if the procedure is initialized
    static real lastAccel[3];               // accel input of last step
    static real lastGyro[3];                // gyro input of last step
    static float lastEstimatedAccel[3];     // propagated accel of last step
    static uint32_t counter = 0;            // propagation counter
    static uint32_t t[3];
    // initialize
    if (!bIni)
    {
        bIni = true;
        lastAccel[0] = accel[0];
        lastAccel[1] = accel[1];
        lastAccel[2] = accel[2];
        lastGyro[0] = w[0];
        lastGyro[1] = w[1];
        lastGyro[2] = w[2];
        t[0] = 0;
        t[1] = 0;
        t[2] = 0;
        imuStats->accelErr[0] = 0.0;
        imuStats->accelErr[1] = 0.0;
        imuStats->accelErr[2] = 0.0;
        return;
    }

    /* Using gyro to propagate accel and then to detect accel error can give valid result for a
     * short period of time because the inhere long-term drift of integrating gyro data.
     * So, after this period of time, a new accel input will be selected.
     * Beside, this method cannot detect long-time smooth linear acceleration. In this case, we
     * can only hope the linear acceleration is large enough to make an obvious diffeerence from
     * the Earth gravity 1g.
     */
    if (counter == 0)
    {
        lastEstimatedAccel[0] = lastAccel[0];
        lastEstimatedAccel[1] = lastAccel[1];
        lastEstimatedAccel[2] = lastAccel[2];
    }
    counter++;
    if (counter == staticDelay)
    {
        counter = 0;
    }

    // propagate accel using gyro
    //  a(k) = a(k-1) -w x a(k-1)*dt
    real ae[3];
    lastGyro[0] *= -dt;
    lastGyro[1] *= -dt;
    lastGyro[2] *= -dt;
    cross(lastGyro, lastEstimatedAccel, ae);
    ae[0] += lastEstimatedAccel[0];
    ae[1] += lastEstimatedAccel[1];
    ae[2] += lastEstimatedAccel[2];

    // save this estimated accel
    lastEstimatedAccel[0] = ae[0];
    lastEstimatedAccel[1] = ae[1];
    lastEstimatedAccel[2] = ae[2];

    // err = a(k) - am
    ae[0] -= accel[0];
    ae[1] -= accel[1];
    ae[2] -= accel[2];

    /* If the difference between the propagted accel and the input accel exceeds some threshold,
     * we assume there is linear acceleration and set .accelErr to be a large value (0.1g).
     * If the difference has been within the threshold for a period of time, we start to decrease
     * estimated accel error .accelErr.
     */
    int j;
    imuStats->accelErrLimit = false;
    for (j = 0; j < 3; j++)
    {
        if (fabs(ae[j]) > 0.0980665) // linear accel detected, 0.01g
        {
            t[j] = 0;
            imuStats->accelErr[j] = (real)0.980665;   // 0.1g
        }
        else    // no linear accel detected, start to decrease estimated accel error
        {
            if (t[j] > staticDelay) // decrease error  
            {
                imuStats->accelErr[j] *= 0.9f;
                imuStats->accelErr[j] += 0.1f * ae[j];
            }
            else    // keep previous error value
            {
                t[j]++;
                // imuStats->accelErr[j];
            }
        }
        // limit error, not taking effect here since the max accelErr should be 0.1g
        if (imuStats->accelErr[j] > 5.0)    // 0.5g
        {
            imuStats->accelErr[j] = 5.0;
            imuStats->accelErrLimit = true;
        }
        if (imuStats->accelErr[j] < -5.0)
        {
            imuStats->accelErr[j] = -5.0;
            imuStats->accelErrLimit = true;
        }
    }
    // record accel for next step
    lastAccel[0] = accel[0];
    lastAccel[1] = accel[1];
    lastAccel[2] = accel[2];
    lastGyro[0] = w[0];
    lastGyro[1] = w[1];
    lastGyro[2] = w[2];
}

BOOL DetectMotionFromAccel(real accelNorm, int iReset)
{
    if (iReset)
    {
        gAlgorithm.linAccelSwitch = false;
        gAlgorithm.linAccelSwitchCntr = 0;
    }
    /* Check for times when the acceleration is 'close' to 1 [g].  When this occurs,
     * increment a counter.  When it exceeds a threshold (indicating that the system
     * has been at rest for a given period) then decrease the R-values (in the
     * update stage of the EKF), effectively increasing the Kalman gain.
     */
    if (fabs(1.0 - accelNorm/GRAVITY) < gAlgorithm.Limit.accelSwitch)
    {
        gAlgorithm.linAccelSwitchCntr++;
        if (gAlgorithm.linAccelSwitchCntr >= gAlgorithm.Limit.linAccelSwitchDelay)
        {
            gAlgorithm.linAccelSwitch = TRUE;
        }
        else
        {
            gAlgorithm.linAccelSwitch = FALSE;
        }
    }
    else
    {
        gAlgorithm.linAccelSwitchCntr = 0;
        gAlgorithm.linAccelSwitch = FALSE;
    }

    return TRUE;
}

BOOL DetectStaticGnssVelocity(double *vNED, real threshold, BOOL gnssValid)
{
    static uint8_t cntr = 0;
    if (gnssValid)
    {
        if (fabs(vNED[0]) < threshold && fabs(vNED[1]) < threshold && fabs(vNED[2]) < threshold)
        {
            if (cntr < 3)
            {
                cntr++;
            }
        }
        else
        {
            cntr = 0;
        }
    }
    else
    {
        cntr = 0;
    }

    return cntr >= 3;
}

BOOL DetectStaticOdo(real odo)
{
    return FALSE;
}

void EKF_Algorithm(void)
{
    static uint16_t freeIntegrationCounter = 0;

    /* After STABILIZE_SYSTEM, the accel data will first pass a low-pass filter.
     * Stats of the filter accel will then be calculated.
     * According to gAlgorithm.useRawAccToDetectLinAccel,
     * raw or filtered accel is used to detect linear accel.
     */
    if ( gAlgorithm.state > STABILIZE_SYSTEM )
    {
        /* Compute IMU mean/var, filter IMU data (optional), detect static.
         * After STABILIZE_SYSTEM, each IMU sample is pushed into a buffer.
         * Before the buffer is full, results are not accurate should not be used.
         */
        MotionStatusImu(gEKFInput.angRate_B, gEKFInput.accel_B, &gImuStats, FALSE);

        // estimate accel error
        if (gAlgorithm.useRawAccToDetectLinAccel)
        {
            EstimateAccelError( gEKFInput.accel_B,
                                gEKFInput.angRate_B,
                                gAlgorithm.dt,
                                gAlgorithm.Limit.linAccelSwitchDelay,
                                &gImuStats);
        }
        else
        {
            EstimateAccelError( gImuStats.lpfAccel,
                                gEKFInput.angRate_B,
                                gAlgorithm.dt,
                                gAlgorithm.Limit.linAccelSwitchDelay,
                                &gImuStats);
        }
        
        // Detect if the unit is static or dynamic
        DetectMotionFromAccel(gImuStats.accelNorm, 0);
    }

    // Compute the EKF solution if past the stabilization and initialization stages
    if( gAlgorithm.state > INITIALIZE_ATTITUDE )
    {
        // Increment the algorithm itow
        gAlgorithm.itow = gAlgorithm.itow + gAlgorithm.dITOW;

        // Perform EKF Prediction
        EKF_PredictionStage(gImuStats.lpfAccel);

        /* Update the predicted states if not freely integrating
         * NOTE: free- integration is not applicable in HG AHRS mode.
         */
        if (gAlgorithm.Behavior.bit.freeIntegrate && (gAlgorithm.state > HIGH_GAIN_AHRS))
        {
            /* Limit the free-integration time before reverting to the complete
             * EKF solution (including updates).
             */
            freeIntegrationCounter = freeIntegrationCounter + 1;   // [cycles]
            if (freeIntegrationCounter >= gAlgorithm.Limit.Free_Integration_Cntr) 
            {
                freeIntegrationCounter = 0;
                enableFreeIntegration(FALSE);

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Display the time at the end of the free-integration period
                TimingVars_DiagnosticMsg("Free integration period ended");
#endif
            }
            // Restart the system in LG AHRS after free integration is complete
            gAlgorithm.insFirstTime = TRUE;
            gAlgorithm.state = LOW_GAIN_AHRS;
            gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;
        }
        else 
        {
            enableFreeIntegration(FALSE);
            freeIntegrationCounter = 0;

            // Perform EKF Update
            EKF_UpdateStage();
        }

        /* Save the past attitude quaternion before updating (for use in the
         * covariance estimation calculations).
         */
        gKalmanFilter.quaternion_Past[Q0] = gKalmanFilter.quaternion[Q0];
        gKalmanFilter.quaternion_Past[Q1] = gKalmanFilter.quaternion[Q1];
        gKalmanFilter.quaternion_Past[Q2] = gKalmanFilter.quaternion[Q2];
        gKalmanFilter.quaternion_Past[Q3] = gKalmanFilter.quaternion[Q3];

        /* Generate the transformation matrix (R_BinN) based on the past value of
         * the attitude quaternion (prior to prediction at the new time-step)
         */
        QuaternionToR321(gKalmanFilter.quaternion, &gKalmanFilter.R_BinN[0][0]);
        
        /* Euler angels are not calcualted here because it is used as a propagation resutls
         * to calculate system innovation. So, Euler angles are updated in the prediction
         * stage. In theory, Euler angles should be updated after each measurement update.
         * However, after EKF converges, it does not matter.
         */
        
        // Update LLA
        if ((gAlgorithm.insFirstTime == FALSE))
        {
            double r_E[NUM_AXIS];
            float pointOfInterestN[3];
            pointOfInterestN[0] = gKalmanFilter.R_BinN[0][0] * gAlgorithm.pointOfInterestB[0] +
                                  gKalmanFilter.R_BinN[0][1] * gAlgorithm.pointOfInterestB[1] +
                                  gKalmanFilter.R_BinN[0][2] * gAlgorithm.pointOfInterestB[2];
            pointOfInterestN[1] = gKalmanFilter.R_BinN[1][0] * gAlgorithm.pointOfInterestB[0] +
                                  gKalmanFilter.R_BinN[1][1] * gAlgorithm.pointOfInterestB[1] +
                                  gKalmanFilter.R_BinN[1][2] * gAlgorithm.pointOfInterestB[2];
            pointOfInterestN[2] = gKalmanFilter.R_BinN[2][0] * gAlgorithm.pointOfInterestB[0] +
                                  gKalmanFilter.R_BinN[2][1] * gAlgorithm.pointOfInterestB[1] +
                                  gKalmanFilter.R_BinN[2][2] * gAlgorithm.pointOfInterestB[2];
            pointOfInterestN[0] += gKalmanFilter.Position_N[0];
            pointOfInterestN[1] += gKalmanFilter.Position_N[1];
            pointOfInterestN[2] += gKalmanFilter.Position_N[2];
            PosNED_To_PosECEF(pointOfInterestN, &gKalmanFilter.rGPS0_E[0], &gKalmanFilter.Rn2e[0][0], &r_E[0]);
            //                 100 Hz           generated once          1 Hz                      100 Hz
            // output variable (ned used for anything else); this is in [ deg, deg, m ]
            ECEF_To_LLA(&gKalmanFilter.llaDeg[LAT], &r_E[X_AXIS]);
            //          100 Hz                    100 Hz
        }
    }

    /* Select the algorithm state based upon the present state as well as
     * operational conditions (time, sensor health, etc).  Note: This is called
     * after the the above code-block to prevent the transition from occuring
     * until the next time step.
     */
    switch( gAlgorithm.state ) 
    {
        case STABILIZE_SYSTEM:
            StabilizeSystem();
            break;
        case INITIALIZE_ATTITUDE:
            InitializeAttitude();
            break;
        case HIGH_GAIN_AHRS:
            HG_To_LG_Transition_Test();
            break;
        case LOW_GAIN_AHRS:
            LG_To_INS_Transition_Test();
            break;
        case INS_SOLUTION:
            INS_To_AHRS_Transition_Test();
            break;
        default:
            return;
    }

    // Dynamic motion logic (to revert back to HG AHRS)
    DynamicMotion();
}
