#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h> 

#define M_PI_FLOAT 3.14159265358979323846f
#define BIQUAD_Q 1/sqrtf(2.0f)
#define XYZ_AXIS_COUNT 3

struct filter_s;
typedef struct filter_s filter_t;

typedef float (*filterApplyPtr)(filter_t *filter, float input);

typedef struct stdev_s{
  float oldMean, newMean, oldSigma, newSigma; 
  int n;
} stdev_t;

typedef struct gyroCalibration_s {
    double sum[XYZ_AXIS_COUNT];
    stdev_t varianz[XYZ_AXIS_COUNT];
    uint32_t cyclesRemaining;
    uint16_t gyroMovementCalibrationThreshold;  
} gyroCalibration_t;

typedef struct gyro_s {
  uint32_t targetLoopTime; //expected cycleTime for gyro Update in us
  float gyroADCf[XYZ_AXIS_COUNT];
  float gyroZero[XYZ_AXIS_COUNT];
  gyroCalibration_t calibration;
  uint8_t gyroIsCalibrated;    
} gyro_t;
extern gyro_t gyro; //will be initialized in mainVariables.h

typedef struct gyroDev_s {
	//sensorGyroInitFuncPtr initFn;                             // initialize function
	//sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    	//sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    	//extiCallbackRec_t exti;
    	//busDevice_t bus;
	float scale;                                            // scalefactor
	float gyroZero[XYZ_AXIS_COUNT];
	float gyroADC[XYZ_AXIS_COUNT];                        // gyro data after calibration and alignment
	float gyroADCf[XYZ_AXIS_COUNT];
	int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
	int16_t gyroADCRaw[XYZ_AXIS_COUNT];
	int16_t temperature;
	//mpuConfiguration_t mpuConfiguration;
	//mpuDetectionResult_t mpuDetectionResult;
	//sensor_align_e gyroAlign;
	//gyroRateKHz_e gyroRateKHz;
	bool dataReady;
	bool gyro_high_fsr;
	uint8_t hardware_lpf;
	uint8_t hardware_32khz_lpf;
	uint8_t mpuDividerDrops;
	//ioTag_t mpuIntExtiTag;
	uint8_t gyroHasOverflowProtection;
	//gyroSensor_e gyroHardware;
} gyroDev_t;


typedef struct gyroConfig_s {
	//uint8_t  gyro_align;                       // gyro alignment
	//uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
	//uint8_t  gyro_sync_denom;                  // Gyro sample divider
	//uint8_t  gyro_hardware_lpf;                // gyro DLPF setting
	//uint8_t  gyro_32khz_hardware_lpf;          // gyro 32khz DLPF setting
	//uint8_t  gyro_high_fsr;
	//uint8_t  gyro_use_32khz;
	//uint8_t  gyro_to_use;
	uint16_t gyro_lowpass_Hz;
	uint16_t gyro_lowpass2_Hz;
	uint16_t gyro_soft_notch_hz_1;
	uint16_t gyro_soft_notch_cutoff_1;
	uint16_t gyro_soft_notch_hz_2;
	uint16_t gyro_soft_notch_cutoff_2;
	uint8_t DynNotchFilterActive;
	//int16_t  gyro_offset_yaw;
	//uint8_t  checkOverflow;

	// Lowpass primary/secondary
	uint8_t  gyro_lowpass_type;
	uint8_t  gyro_lowpass2_type;
	//uint8_t  yaw_spin_recovery;
	//int16_t  yaw_spin_threshold;
	uint16_t gyroCalibrationDuration;  // Gyro calibration duration in 1/100 second
	//uint8_t dyn_notch_quality; // bandpass quality factor, 100 for steep sided bandpass
	//uint8_t dyn_notch_width_percent;
} gyroConfig_t;
extern gyroConfig_t gyroConfig;

typedef struct dtermFilter_s{
	uint16_t dterm_notch_hz;
	uint16_t dterm_notch_cutoff_hz;
	uint16_t dterm_lowpass2_hz;
	uint8_t dterm_lowpass_type;
	uint16_t dterm_lowpass_hz;
} dtermFilter_t;
extern dtermFilter_t dtermFilterSettings;


typedef struct pt1Filter_s{
	float state;
	float k;
} pt1Filter_t;

typedef struct biquadFilter_s{
	float b0, b1, b2, a1, a2;
	float x1, x2, y1, y2;
} biquadFilter_t;

biquadFilter_t accelFilter[3];

typedef union gyroLowpassFilter_u{
	pt1Filter_t pt1FilterState;
	biquadFilter_t biquadFilterState;
} gyroLowpassFilter_t;

typedef enum{
	FILTER_LOWPASS = 0,
	FILTER_LOWPASS2
} filterTypes;

typedef enum {
	FILTER_PT1 = 0,
	FILTER_BIQUAD,
} lowpassFilterType_e;

typedef enum {
	FILTER_LPF,    // 2nd order Butterworth section
	FILTER_NOTCH,
	FILTER_BPF,
} biquadFilterType_e;

typedef struct gyroSensor_s {
	gyroDev_t gyroDev;
	//gyroCalibration_t calibration;

	// lowpass gyro soft filter
	filterApplyPtr lowpassFilterApplyFn;
	gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

	// lowpass2 gyro soft filter
	filterApplyPtr lowpass2FilterApplyFn;
	gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

	// notch filters
	filterApplyPtr notchFilter1ApplyFn;
	biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

	filterApplyPtr notchFilter2ApplyFn;
	biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];
	
  	filterApplyPtr notchFilterDynApplyFn;
	biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];
	
    // overflow and recovery needs to be implemented
} gyroSensor_t;
extern gyroSensor_t gyroSensor;

typedef union dtermLowpass{
	pt1Filter_t pt1Filter;
	biquadFilter_t biquadFilter;
} dtermLowpass_t;

filterApplyPtr dtermNotchFilterApply;
biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
filterApplyPtr dtermLowpassFilterApply;
dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
filterApplyPtr dtermLowpassFilter2Apply;
pt1Filter_t dtermPT1[XYZ_AXIS_COUNT];

void gyroInitFilters (gyroSensor_t *gyroSensor);
void gyroInitLowPassFilter(gyroSensor_t *gyroSensor, int slot, int type, uint16_t lpfHz);
float pt1FilterApply(pt1Filter_t *filter, float input);
float biquadFilterApply(biquadFilter_t *filter, float input);
float nullFilterApply(filter_t *filter, float input);
float pt1FilterGain(uint16_t f_cut, float dt);
void pt1FilterInit(pt1Filter_t *filter, float);
void biquadFilterInit(biquadFilter_t *filter, float freq, uint32_t refreshrate, float Q, biquadFilterType_e FilterType);	
uint16_t calculateNyquistAdjustNotchHz(uint16_t notchHz, uint16_t notchCutoffHz);
void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz);
void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz);
uint16_t calculateNyquistAdjustNotchHz(uint16_t notchHz, uint16_t notchCutoffHz);
float filterGetNotchQ(float centerFreq, float cutoffFreq); 
void accelInitLowPassFilter(biquadFilter_t *filter, float accLpfHz, uint16_t refreshrate, float Q, biquadFilterType_e type);
void gyroInitFilterDynNotch(gyroSensor_t* gyroSensor);
float biquadFilterApplyDyn(biquadFilter_t *filter, float input);
void initDtermNotchFilter(void);
void initDtermLowpassFilter(void);
void initDtermLpfFilter(uint8_t lpf_type);
	
	

