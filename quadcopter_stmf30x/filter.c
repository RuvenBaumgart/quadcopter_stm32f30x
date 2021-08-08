#include "filter.h"
#include "maths.h"
#include <math.h>

gyroSensor_t gyroSensor; //Initalizing the gyro with the variables that holds the filter functions and the corresponding values.

void gyroInitFilters (gyroSensor_t *gyroSensor){
	gyroInitLowPassFilter(gyroSensor, FILTER_LOWPASS, gyroConfig.gyro_lowpass_type, gyroConfig.gyro_lowpass_Hz);
	gyroInitLowPassFilter(gyroSensor, FILTER_LOWPASS2, gyroConfig.gyro_lowpass2_type, gyroConfig.gyro_lowpass2_Hz);
	gyroInitFilterNotch1(gyroSensor, gyroConfig.gyro_soft_notch_hz_1, gyroConfig.gyro_soft_notch_cutoff_1);	
	gyroInitFilterNotch2(gyroSensor, gyroConfig.gyro_soft_notch_hz_2, gyroConfig.gyro_soft_notch_cutoff_2);
	gyroInitFilterDynNotch(gyroSensor);
	
};

void initDtermNotchFilter(){	
	
	dtermNotchFilterApply = nullFilterApply;
	uint16_t notchHz;
	
	//Initalization of the dterm Notch filter
	if(dtermFilterSettings.dterm_notch_hz && dtermFilterSettings.dterm_notch_cutoff_hz){
		notchHz = calculateNyquistAdjustNotchHz(dtermFilterSettings.dterm_notch_hz, dtermFilterSettings.dterm_notch_cutoff_hz);
		dtermNotchFilterApply = (filterApplyPtr) biquadFilterApply;
		const float notchQ = filterGetNotchQ(notchHz, dtermFilterSettings.dterm_notch_cutoff_hz);
		for(int axis = 0; axis < 3; ++axis){
			biquadFilterInit(&dtermNotch[axis], notchHz, gyro.targetLoopTime, notchQ, FILTER_NOTCH);
		};
	};
};

void initDtermLowpassFilter(void){	

	dtermLowpassFilter2Apply = nullFilterApply;
	
	//Initalization of the 2nd LowpassFilter pt1Filter
	if(dtermFilterSettings.dterm_lowpass2_hz){
		dtermLowpassFilter2Apply = (filterApplyPtr) pt1FilterApply;
		for(int axis = 0; axis < 3; ++axis){
			pt1FilterInit(&dtermPT1[axis], pt1FilterGain(dtermFilterSettings.dterm_lowpass2_hz, gyro.targetLoopTime * 1e-6f));
		};	
	};
};

void initDtermLpfFilter(uint8_t lpf_type){
	
	dtermLowpassFilterApply = nullFilterApply;

	//Initalization of the 	1st LowPassFilter pt1Filter or BIQUAD
	if(lpf_type){
		switch(dtermFilterSettings.dterm_lowpass_type){
			case FILTER_PT1:
				dtermLowpassFilterApply = (filterApplyPtr) pt1FilterApply;
				for(int axis = 0; axis < 3; ++axis){
					pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(dtermFilterSettings.dterm_lowpass_hz, gyro.targetLoopTime * 1e-6f));
				};
			break;
			case FILTER_BIQUAD:
				dtermLowpassFilterApply = (filterApplyPtr) biquadFilterApply;
				for(int axis = 0; axis < 3; ++axis){
					biquadFilterInit(&dtermLowpass[axis].biquadFilter, dtermFilterSettings.dterm_lowpass_hz, gyro.targetLoopTime, BIQUAD_Q, FILTER_LPF);
				};
			break;
			default:
			break;
		};
	};
};


void accelInitLowPassFilter(biquadFilter_t *filter, float accLpfHz, uint16_t refreshrate, float Q, biquadFilterType_e type){
	for(int axis =0; axis < 3; ++axis){
		 biquadFilterInit(&filter[axis], accLpfHz, refreshrate, Q, type);
	}
};


void gyroInitLowPassFilter(gyroSensor_t *gyroSensor, int slot, int type, uint16_t lpfHz){
	filterApplyPtr* lowpassFilterApplyFn; //filterApplyPTr** because we point to a Pointer that points to the functionPtr
	gyroLowpassFilter_t* lowpassFilter = NULL;

	// Define the lowpassFilter function in the gyroSensor Structure
	switch(slot){
		case FILTER_LOWPASS: // PT1 Filter
			lowpassFilterApplyFn = &gyroSensor->lowpassFilterApplyFn;
			lowpassFilter = &gyroSensor->lowpassFilter[0]; //one possible way
		break;
		case FILTER_LOWPASS2: 
			lowpassFilterApplyFn = &gyroSensor->lowpass2FilterApplyFn;
			lowpassFilter = gyroSensor->lowpass2Filter; //second possbile way
		break;
		default:
		return;
	};
	
	// Calculation of constants that need to be used 
	const uint32_t gyroFrequencyNyquist = (1e6/2/gyro.targetLoopTime);
	const float gyroDt = gyro.targetLoopTime * 1.0e-6f;
	const float gain = pt1FilterGain(lpfHz, gyroDt);
	
	//Dereference the pointer lowpassFilerApplyFn because it´s a pointer pointer (filterApplyPtr) and assign the value of the 
	//gyroSensor->lowpassFilterApplyFn should be the address of the function nullFilerApply.
	// if the lpfHz is specified it must be lower then Nyquist frequenzy otherwise no filter will be applied as function
	*lowpassFilterApplyFn = nullFilterApply; //dereferenzing because lowpassFilterApplyFn is from Type **
	if(lpfHz && lpfHz <= gyroFrequencyNyquist){
		switch (type){
		case FILTER_PT1:
			*lowpassFilterApplyFn = (filterApplyPtr) pt1FilterApply;
			for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis){
				pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
			};
		break;
		case FILTER_BIQUAD:
			*lowpassFilterApplyFn = (filterApplyPtr) biquadFilterApply;
			for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis){
				biquadFilterInit(&lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLoopTime, BIQUAD_Q, FILTER_LPF);
			};
		break;
		};
	};
};


void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz){
	gyroSensor->notchFilter1ApplyFn = nullFilterApply;
	notchHz = calculateNyquistAdjustNotchHz(notchHz, notchCutoffHz);
	
	if (notchHz != 0 && notchCutoffHz != 0) {
		gyroSensor->notchFilter1ApplyFn = (filterApplyPtr) biquadFilterApply;
		const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
		for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
			biquadFilterInit(&gyroSensor->notchFilter1[axis], notchHz, gyro.targetLoopTime, notchQ, FILTER_NOTCH);
		};
	};	
};

void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz){
	gyroSensor->notchFilter2ApplyFn = nullFilterApply;
	notchHz = calculateNyquistAdjustNotchHz(notchHz, notchCutoffHz);
	
	if (notchHz != 0 && notchCutoffHz != 0) {
		gyroSensor->notchFilter2ApplyFn = (filterApplyPtr) biquadFilterApply;
		const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
		for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
			biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLoopTime, notchQ, FILTER_NOTCH);
		};
	};	
}

float pt1FilterApply(pt1Filter_t *filter, float input){
	filter->state = filter->state + filter->k * (input - filter->state);
	return filter->state;
};

float biquadFilterApply(biquadFilter_t *filter, float input){
	const float result = filter->b0 * input + filter->x1;
	filter->x1 = filter->b1 * input - filter->a1 * result  + filter->x2;
	filter->x2 = filter->b2 * input - filter->a2 * result;
	return result;
};

float biquadFilterApplyDyn(biquadFilter_t *filter, float input){
	/* compute result */
	const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

	/* shift x1 to x2, input to x1 */
	filter->x2 = filter->x1;
	filter->x1 = input;

	/* shift y1 to y2, result to y1 */
	filter->y2 = filter->y1;
	filter->y1 = result;

	return result;
};
	
float nullFilterApply(filter_t* filter, float input){
	(void)(filter);
	return input;
};

float pt1FilterGain(uint16_t f_cut, float dt){
	float RC = 1.0 / (2.0  * M_PI_FLOAT * f_cut);
	return dt / (RC + dt);
};

void pt1FilterInit(pt1Filter_t* filter, float k){
	filter->state = 0.0f;
	filter->k = k;
};

void gyroInitFilterDynNotch(gyroSensor_t *gyroSensor){
	gyroSensor->notchFilterDynApplyFn = nullFilterApply;	
	if (gyroConfig.DynNotchFilterActive){
		gyroSensor->notchFilterDynApplyFn = (filterApplyPtr)biquadFilterApplyDyn;
		const float notchQ = filterGetNotchQ(400, 390);
		for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){
			biquadFilterInit(&gyroSensor->notchFilterDyn[axis], 400, gyro.targetLoopTime, notchQ, FILTER_NOTCH);
		};
	};
};

void biquadFilterInit(biquadFilter_t *filter, float freq, uint32_t refreshrate, float Q, biquadFilterType_e FilterType){
	//setup the variable
	const float omega = 2.0f * M_PI_FLOAT * freq * refreshrate * 1.0e-6f;
	const float sn = sin_approx(omega);
	const float cs = cos_approx(omega);
	const float alpha = sn / (2.0f * Q);
	float b0 = 0.0, b1 = 0.0, b2 = 0.0, a0 = 0.0, a1 = 0.0, a2 = 0.0;
	switch (FilterType){
		case FILTER_LPF:
			b0 = (1.0f - cs) * 0.5f;
			b1 = 1.0f - cs;
			b2 = (1.0f - cs) * 0.5f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * cs;
			a2 = 1.0f - alpha;
		break;
		case FILTER_NOTCH:
			b0 =  1.0f;
			b1 = -2.0f * cs;
			b2 =  1.0f;
			a0 =  1.0f + alpha;
			a1 = -2.0f * cs;
			a2 =  1.0f - alpha;
		break;
		case FILTER_BPF:
			b0 = alpha;
			b1 = 0.0f;
			b2 = -alpha;
			a0 = 1.0f + alpha;
			a1 = -2.0f * cs;
			a2 = 1.0f - alpha;
		break;
	};

	// precompute the coefficients
	filter->b0 = b0 / a0;
	filter->b1 = b1 / a0;
	filter->b2 = b2 / a0;
	filter->a1 = a1 / a0;
	filter->a2 = a2 / a0;

	// zero initial samples
	filter->x1 = filter->x2 = 0;
	filter->y1 = filter->y2 = 0;

};


uint16_t calculateNyquistAdjustNotchHz(uint16_t notchHz, uint16_t notchCutoffHz){
	const uint32_t gyroFrequencyNyquist = 1e6 / 2 / gyro.targetLoopTime; //666 Hz in Version 12.7.2
	if (notchHz > gyroFrequencyNyquist){
		if(notchCutoffHz < gyroFrequencyNyquist){
			notchHz = gyroFrequencyNyquist;	
		} else {
			notchHz = 0;
		};
	};
	return notchHz;
};

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq) {
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
};
