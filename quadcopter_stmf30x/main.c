#include <stm32f30x_conf.h>
#include <stm32f30x.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "i2c.h"
#include "MPU6050.h"
#include <math.h>
#include <string.h>
#include "filter.h"
#include "maths.h"
#include "spRacingF3.h"
#include "mainVariables.h" 
#include "stm32f3x_system.h"
#include "buzzer.h"
#include "scheduler.h"
#include <stdbool.h>

#define DEBUG

int main(void){
  stm32f3x_init();	
  initSystemAndModules();
  I2C_DMACmd(I2C1, I2C_DMAReq_Rx, ENABLE); //Needs to be enabled here. ohterwise it will cause conflicts 
  SET_FLAG(f.DISARMED);
  runFlightControllerTasks();
  return 0;
};

void runFlightControllerTasks(void){ 
 
  while(true){
    getReceiverInputValues(&rxSetPoint[0]); 
    areRxSetPointsValid();
    if(f.VALID_RX_SIGNALS){ 	
      getFlightModeAndArmDisarmSignal();
      parseArmDisarmSignal(); 
      taskMainPidLoop();
	
      if(f.ARMED && !f.DISARMED){ 
        calculateEscTimingFlight();
      } else if (f.DISARMED && !f.ARMED) {
        calculateEscTimingGround(); 
      };
     
      taskWriteMotors();
 /**
 * If not valid signals and was not armed already then fc probably is turned on for debug
 */
    } else if(!f.VALID_RX_SIGNALS && !f.RX_SIGNALS_WAS_VALID) { 
      
      calculateEscTimingGround();
      taskWriteMotors();
      taskHandleSerialPortDebug(); 
 
      static uint16_t lastFlashingTime;
      if(millis() - lastFlashingTime > 1000){
        GPIOB->ODR ^= (0x01 << LED_R);
        lastFlashingTime = millis();  
      };  
      subtaskUpdateBuzzer(millis());
    } else if(!f.VALID_RX_SIGNALS && f.RX_SIGNALS_WAS_VALID && f.ARMED){
   /**
    * If the signal was valid, then the signal could be lost if we are in flight. 
    * if we are on ground and the Motors are stoped then we turned of the receiver.
    * Definition of the FAILSAIFE ROUTINE HERE
    */
    };
#ifdef DEBUG
   flushBufferOverUart();
#endif
   checkLoopTimingAndWait(targetLoopTime);
   };
};

//------------------------------------------------------------------
//------------------ FUNCTION IMPLEMENTATION -----------------------
//------------------------------------------------------------------

static inline void taskMainPidLoop(void){
  uint32_t currentTime;
  taskFetchGyroData(); 
  calculateRcRatesForPitchAndRoll();
  getThrottleSetPointAndCalculateExpo();
  get_tpa_factor();  
  processGyroRawData();
  currentTime = micros(); 
  filterGyro(&Gyro[0], &Gyro_f, currentTime);
  filterAccel(&Accel[0], &Acc_f);
  executeNextSubTask(currentTime);
  pid_calculation();
};

void setPidFrequency(uint16_t targetPidLoopTime)
{
  float dT = targetPidLoopTime * 1e-6f;
  pidFrequency = 1.0f / dT;
};

static inline void pid_calculation(void){
  static float gyroDtermPrevious[3];    
  static float gyroDterm[3];
  static float itermError;
        
  for(int axis = ROLL; axis <= YAW; ++axis){
    PID_ERR.temp = pidSetpoint[axis] - Gyro_f.input[axis];
        
    if((f.ANGLE_MODE) && (axis != YAW)){
      pidSetpoint[axis] = constrain_float(pidSetpoint[axis], MIN_ANGLE, MAX_ANGLE); //Setpoint: 500 is 50 degrees
      PID_ERR.temp = ((pidSetpoint[axis] - angle[axis] ) * gain_level) - Gyro_f.input[axis];
    };
    
    itermError = PID_ERR.temp;
    applyItermRelax(axis, PID_ERR.I[axis], &itermError, pidSetpoint[axis]);

    PID_ERR.I[axis] += (pidGain[axis].I * itermError);
    if(axis == YAW){
      if(pidSetpoint[axis] >= 120 || pidSetpoint[axis] <= -120){
        pidGain[axis].I = 0.0f;
      };
    }; 
 
    gyroDterm[axis] = dtermNotchFilterApply((filter_t*) &dtermNotch[axis], Gyro_f.input[axis]);
    gyroDterm[axis] = dtermLowpassFilterApply((filter_t*) &dtermLowpass[axis], gyroDterm[axis]);
    gyroDterm[axis] = dtermLowpassFilter2Apply((filter_t*) &dtermPT1[axis], gyroDterm[axis]);  
      
    PID_ERR.d_now[axis] = -(gyroDterm[axis] - gyroDtermPrevious[axis])  * pidFrequency; 
    gyroDtermPrevious[axis] = gyroDterm[axis];
    PID_ERR.D[axis] = pidGain[axis].D * throttlePIDAttenuation * PID_ERR.d_now[axis];
    PID_ERR.P[axis] = pidGain[axis].P * PID_ERR.temp * throttlePIDAttenuation;
   
    pidOutput[axis] = PID_ERR.P[axis] + PID_ERR.I[axis] + PID_ERR.D[axis];
    pidOutput[axis] = rintf(constrain_float(pidOutput[axis], (pidGain[axis].max * -1.0f), pidGain[axis].max));
  };
};

void applyItermRelax(int axis, float previousIterm, float* itermError, float currentSetPoint)
{
  if(axis < YAW){
    const float itermRelaxFactor = MAX(0.0f, 1.0f - fabs(currentSetPoint) / ITERM_RELAX_SETPOINT_THRESHOLD);
    const bool isIDecreasing = ((previousIterm < 0) && (*itermError > 0)) || ((previousIterm > 0) && (*itermError < 0));
    if(isIDecreasing){
    } else{
      *itermError *=itermRelaxFactor;
    };
  };
};

void calculateRcRatesForPitchAndRoll(void){ //deg/sec
  int16_t temp_rc[3],temp_rc2[3];
  int16_t rcCommand[3]; //Array to saw the RC VALUES [1000;2000] 

   for(uint8_t axis=0;axis<3;++axis){
    temp_rc[axis] = 0;
    if (rxSetPoint[axis] >= (1500 + DEADBAND_RCSETPOINT)){
       temp_rc[axis] = rxSetPoint[axis] - 1500;
    } else if (rxSetPoint[axis] <= (1500 - DEADBAND_RCSETPOINT)){
       temp_rc[axis] = rxSetPoint[axis] - 1500;
    };
    temp_rc[axis] = constrain_int(temp_rc[axis],-500,500);       
                
    if(axis!=2){ //If axis is 0 Roll or 1 Pitch
       if (temp_rc[axis] < 0){
 	 temp_rc[axis]= temp_rc[axis] * -1;
	 temp_rc2[axis] = temp_rc[axis] >> 7; //--> this will result in a range 0 to 3
         rcCommand[axis] = lookupRollPitch[temp_rc2[axis]] + ((temp_rc[axis] -(temp_rc2[axis] << 7)) * (lookupRollPitch[temp_rc2[axis] +1]-lookupRollPitch[temp_rc2[axis]])>>7); 
         rcCommand[axis] = rcCommand[axis]* -1;
       } else {
	 temp_rc2[axis] = temp_rc[axis] >> 7; //--> this will result in a range 0 to 3
	 rcCommand[axis] = lookupRollPitch[temp_rc2[axis]] + ((temp_rc[axis] -(temp_rc2[axis] << 7)) * (lookupRollPitch[temp_rc2[axis] +1]-lookupRollPitch[temp_rc2[axis]])>>7);         
       };
     pidSetpoint[axis] = rcCommand[axis]; 
   } else { //YAE AXIS is direct ouput
     rcCommand[axis] = temp_rc[axis];
     pidSetpoint[axis] =  (rcCommand[YAW] / yaw_rate) * yawSetPointInversion;
   }; 
  };	
};
  
void getThrottleSetPointAndCalculateExpo(void){
  uint16_t tempThrottle;
  uint8_t temp2Throttle;
  throttle = rxSetPoint[THROTTLE];
  tempThrottle = constrain_int(throttle, MINCHECK, 2000); //Minimum input resp. Dead band at low throttle position
  tempThrottle = (uint32_t)(tempThrottle - MINCHECK) * 2559 / (2000-MINCHECK);
  temp2Throttle = tempThrottle / 256; //Results in a range of 0 - 9;
  throttle = lookupThrottle[temp2Throttle] + (tempThrottle - temp2Throttle * 256) * (lookupThrottle[temp2Throttle + 1] - lookupThrottle[temp2Throttle])/256; 
};

void calculateEscTimingFlight(){
  esc1 = throttle * mixerQuadX[0].throttle + pidOutput[PITCH] * mixerQuadX[0].pitch + pidOutput[ROLL] * mixerQuadX[0].roll + pidOutput[YAW] * mixerQuadX[0].yaw ; // FRONT_L
  esc2 = throttle * mixerQuadX[1].throttle + pidOutput[PITCH] * mixerQuadX[1].pitch + pidOutput[ROLL] * mixerQuadX[1].roll + pidOutput[YAW] * mixerQuadX[1].yaw ; // FRONT_R
  esc3 = throttle * mixerQuadX[2].throttle + pidOutput[PITCH] * mixerQuadX[2].pitch + pidOutput[ROLL] * mixerQuadX[2].roll + pidOutput[YAW] * mixerQuadX[2].yaw ; // REAR_R
  esc4 = throttle * mixerQuadX[3].throttle + pidOutput[PITCH] * mixerQuadX[3].pitch + pidOutput[ROLL] * mixerQuadX[3].roll + pidOutput[YAW] * mixerQuadX[3].yaw ; // REAR_L
  /**
   * get the values mapped for oneshot 125us - 250us
   */  

  if(esc1 > MAX_RC) esc1 = MAX_RC;
  if(esc2 > MAX_RC) esc2 = MAX_RC;
  if(esc3 > MAX_RC) esc3 = MAX_RC;
  if(esc4 > MAX_RC) esc4 = MAX_RC;
        
  if(esc1 < MIN_RC) esc1 = MIN_RC;
  if(esc2 < MIN_RC) esc2 = MIN_RC;
  if(esc3 < MIN_RC) esc3 = MIN_RC;
  if(esc4 < MIN_RC) esc4 = MIN_RC;
};


void calculateEscTimingGround(void){
 esc1 = LOW_RC;
 esc2 = LOW_RC;
 esc3 = LOW_RC;
 esc4 = LOW_RC;   
};        


void updatePWMTimerRegistersForMotors(void){
  /**
   * Looking in Flightdirection the default control is clockwise from 1 to 4 starting front left
   */
#if defined QUAD_4I
  TIM3->CCR1 = esc1;
  TIM3->CCR2 = esc4;
  TIM4->CCR1 = esc2;
  TIM4->CCR2 = esc3;
#else
  TIM3->CCR1 = esc1; //FRONT LEFT
  TIM3->CCR2 = esc2; //FRRONT RIGHT
  TIM4->CCR1 = esc3; // REAR RIGHT
  TIM4->CCR2 = esc4; // REAR LEFT
#endif
};

void reinitializePWMTimerCounter(void){
 /**
  * Reinitialize the counter and generates an update of the registers. 
  * Note that the prescaler counter is cleared too (anyway the prescaler ratio is not affected). 
  */
 TIM4->EGR |= 0x01;
 TIM3->EGR |= 0x01;
};

void updatePWMTimerRegistersForGimbal(void){
 TIM16->CCR1 = rxSetPoint[6];
 TIM17->CCR1 = rxSetPoint[7]; 
};

static inline void checkLoopTimingAndWait(const uint32_t LoopTime){	
  static uint32_t lastProceedingTime;
  loop_duration = micros() - lastProceedingTime;
  if( (loop_duration) <= LoopTime) {
    GPIOB->ODR |= (0x01 << LED_R);
  } else {
    GPIOB->ODR &= (0x00 << LED_R); //Inverted_ Turns on the LED_R
  };
  while (TIM7->CNT <= LoopTime){};
  TIM7->EGR |= 0x01;
  loopDurationAfterWait = micros() - lastProceedingTime;
  lastProceedingTime = micros();
};

//------------------------- Interrupts -----------------------------

void DMA1_Channel4_IRQHandler(void){
  if(DMA_GetFlagStatus(DMA1_FLAG_TC4)){
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    *pbufferPosition = 0;
    usart1TransferComplete = 1;
  };	
};

void DMA1_Channel7_IRQHandler(void){
  if(DMA_GetFlagStatus(DMA1_FLAG_TC7)){
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
    SET_FLAG(f.GYRO_DATA_AVAILABLE);
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC7);
  };
};


/*
void TIM7_IRQHandler()
{
 SET_FLAG(f.LOOP_TIME_EXCEEDED);
 TIM7->SR |= 0x00 ; 
};
*/


void TIM2_IRQHandler(void){	
  register int32_t start_time;
	register int32_t measuredTime;
	static int32_t  previousStartTime;
	static uint8_t channel_select_counter; 
  
  start_time = TIM2->CCR4;
  measuredTime = start_time - previousStartTime;
  if (measuredTime < 0) measuredTime += 0xFFFF; // Correct the timer overflow by adding max number
  previousStartTime = start_time;
        	
	if (measuredTime > 2600){
		channel_select_counter = 0;
	} else {
		++channel_select_counter;
	};

	// QX7 TARANIS ARTE CHANNEL ORDER
	switch(channel_select_counter){
		case 1:
			rxInputData[PITCH] = measuredTime;
			break;
		case 2:
			rxInputData[YAW] = measuredTime;
			break;
		case 3:
			rxInputData[ROLL] = measuredTime;
			break;
		case 4:
			rxInputData[THROTTLE] = measuredTime;
			break;
		case 5:
			rxInputData[CH5] = measuredTime;
			break;
		case 6:
			rxInputData[CH6] = measuredTime;
			break;
		case 7:
			rxInputData[CH7] = measuredTime;
			break;
		case 8:
			rxInputData[CH8] = measuredTime;
			break;
		default:
			break;
	};
};

void SysTick_Handler(void){
	msTicks++;
};


uint32_t millis(void){
    return msTicks;
};

void delay_millis(uint32_t dlyticks){
    uint32_t curTicks = msTicks;
    while ((msTicks - curTicks) < dlyticks){};
};

uint32_t micros(){
  uint32_t sys_clock_val; 
  sys_clock_val = SysTick->VAL;
  usTicks = msTicks * 1000 + 1000 - sys_clock_val/72; //To calculate the micros, the value will be added to the millis counter
  return usTicks;
};

static void delay_micros(uint32_t dlyticks){
    uint32_t curTicks = micros();
    while ((micros() - curTicks)<dlyticks){};
};

void init_gyro(gyro_t *gyro, gyroConfig_t *gyroConfig){
  init_MPU6050();
  //Gyro settings
  gyro->targetLoopTime = targetLoopTime;
  gyro->gyroIsCalibrated = FALSE;
  gyroConfig->gyroCalibrationDuration = 1.25f;  //in seconds
  gyro->calibration.gyroMovementCalibrationThreshold = 48;
  setGyroCalibrationCycles();
  //Gyro Configuration
  gyroConfig->gyro_lowpass_type = GYRO_LOW_PASS_FILTER; // FILTER_PT1 FILTER_BIQUAD 
  gyroConfig->gyro_lowpass_Hz = GYRO_LOW_PASS_HZ; //Need to be within the nyquist frequenzy of 250 hz (500 Hz is the loop time)
  gyroConfig->gyro_lowpass2_type = GYRO_LOW_PASS_FILTER2;
  gyroConfig->gyro_lowpass2_Hz = GYRO_LOW_PASS_HZ_2;
  gyroConfig->gyro_soft_notch_hz_1 = GYRO_SOFT_NOTCH_HZ_1;
  gyroConfig->gyro_soft_notch_cutoff_1 = GYRO_SOFT_NOTCH_CUTOFF_1;
  gyroConfig->gyro_soft_notch_hz_2 = GYRO_SOFT_NOTCH_HZ_2;
  gyroConfig->gyro_soft_notch_cutoff_2 = GYRO_SOFT_NOTCH_CUTOFF_2;
  gyroConfig->DynNotchFilterActive = DYN_NOTCH_FILTER_ACTIVE; 	
};

void initDtermSettings(dtermFilter_t *dtermSettings){

  dtermSettings->dterm_notch_hz = DTERM_NOTCH_HZ;
  dtermSettings->dterm_notch_cutoff_hz = DTERM_NOTCH_CUTOFF_HZ;
  dtermSettings->dterm_lowpass2_hz = DTERM_LOWPASS2_HZ;
  dtermSettings->dterm_lowpass_type = DTERM_LOWPASS_TYPE;
  dtermSettings->dterm_lowpass_hz= DTERM_LOWPASS_HZ;
};

void init_MPU6050(){
	I2C_writeByte(I2C1, MPU6050, PWR_MGMT_1, 0x80); // Resets the MPU6050. To perform the setback properly. SIGNAL_PATH  Register should be Reset afterwards
	delay_millis(100);
	I2C_writeByte(I2C1, MPU6050, SIGNAL_PATH_RESET, 0x00); // Resets the gyro, accel and temp values;
	delay_millis(100);
	I2C_writeByte(I2C1, MPU6050, PWR_MGMT_1, 0x01); // Sets the Clock Source for the MPU6050 to the gyroscope 
	I2C_writeByte(I2C1, MPU6050, CONFIG, 0x00); // No DLPF on the Gyro
	I2C_writeByte(I2C1, MPU6050, SMPLRT_DIV, 0x00); //Samplerate divider is not set. This results in a sample rate of 8kHz. 
	delay_millis(10); 
	//Get the setting of the Gyro

	uint8_t c = I2C_readByte(I2C1, MPU6050, GYRO_CONFIG);    
	I2C_writeByte(I2C1, MPU6050, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	I2C_writeByte(I2C1, MPU6050, GYRO_CONFIG, c & ~0x18); // Clear FS_SELL bits [4:3]
	I2C_writeByte(I2C1, MPU6050, GYRO_CONFIG, c | (0x03 << 3)); // Set range for the gyro to +-2000°/sec Output 16,4 LSB deg/s for
	// Set accelerometer configuration
	
	c = I2C_readByte(I2C1, MPU6050, ACCEL_CONFIG);
	I2C_writeByte(I2C1, MPU6050, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	I2C_writeByte(I2C1, MPU6050, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	I2C_writeByte(I2C1, MPU6050, ACCEL_CONFIG, c | (0x02 << 3)); // Set range for the accelerometer to +- 8g
    
	I2C_writeByte(I2C1, MPU6050, INT_PIN_CFG, 0x02); //set interrupt pin active high
	I2C_writeByte(I2C1, MPU6050, INT_ENABLE, 0x01); //enable interrupt 
};

void taskFetchGyroData(){
 if(!f.GYRO_DATA_NOT_PROCESSED){
    I2C_DmaReadGyroValues(MPU6050, ACCEL_XOUT_H); //gyroRawData is defined in i2c.c
    SET_FLAG(f.GYRO_DATA_NOT_PROCESSED);
 };
};

static inline void processGyroRawData(){
  if(f.GYRO_DATA_AVAILABLE){
    imu.accel[X] = ((int16_t)gyroRawData[0]<<8)| gyroRawData[1]; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    imu.accel[Y] = ((int16_t)gyroRawData[2]<<8)| gyroRawData[3]; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    imu.accel[Z] = ((int16_t)gyroRawData[4]<<8)| gyroRawData[5]; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    imu.temp = ((int16_t)gyroRawData[6]<<8) | gyroRawData[7]; // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    imu.gyro[X] = ((int16_t)gyroRawData[8]<<8)| gyroRawData[9]; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    imu.gyro[Y] = ((int16_t)gyroRawData[10]<<8) | gyroRawData[11]; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    imu.gyro[Z] = ((int16_t)gyroRawData[12]<<8) | gyroRawData[13]; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
	
    // Based on the orientation of the MPU on the SP Board the orientation output needs to be adjusted.
    alignSensors(&imu.accel[0], ACC_MPU6050_ALIGN);
    alignSensors(&imu.gyro[0], GYRO_MPU6050_ALIGN);
    
   if(gyro.gyroIsCalibrated){
      Gyro[X] = (imu.gyro[X]) - gyro.gyroZero[X];
      Gyro[Y] = (imu.gyro[Y]) - gyro.gyroZero[Y];
      Gyro[Z] = (imu.gyro[Z]) - gyro.gyroZero[Z];
      Accel[X] = (imu.accel[X] );
      Accel[Y] = (imu.accel[Y] );
      Accel[Z] = (imu.accel[Z] );
   } else {
     performGyroCalibration();
   };
 UNSET_FLAG(f.GYRO_DATA_NOT_PROCESSED);
 UNSET_FLAG(f.GYRO_DATA_AVAILABLE);
 };
};

void performGyroCalibration(void)
{
  for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis){ 
    if(isOnFirstCalibrationCycle()){
      gyro.calibration.sum[axis] = 0.0;
      gyro.calibration.varianz[axis].n = 0.0f;
      gyro.gyroZero[axis] = 0.0f;
    };
  
    gyro.calibration.sum[axis] += imu.gyro[axis];
    devPush(&gyro.calibration.varianz[axis], imu.gyro[axis]);
   
    if(isOnFinalCalibrationCycle()){
        const float stddev = calculateStandardDeviation(&gyro.calibration.varianz[axis]);
         if(gyro.calibration.gyroMovementCalibrationThreshold && stddev > gyro.calibration.gyroMovementCalibrationThreshold){
          setGyroCalibrationCycles();
        return;
        };
        gyro.gyroZero[axis] = gyro.calibration.sum[axis] / gyroCalibrationCycles();
    };
  };
  if (isOnFinalCalibrationCycle()){
    activateBuzzer(&buzz_readyBeep[0]);
    gyro.gyroIsCalibrated = TRUE;
  }; 
  --gyro.calibration.cyclesRemaining; 
};

bool isOnFirstCalibrationCycle(void)
{
  return gyro.calibration.cyclesRemaining == gyroCalibrationCycles();
};

uint32_t gyroCalibrationCycles(void)
{
  return (gyroConfig.gyroCalibrationDuration * 1e6) / gyro.targetLoopTime; 
};

bool isOnFinalCalibrationCycle(void)
{
  return gyro.calibration.cyclesRemaining == 1;
};

void setGyroCalibrationCycles(void)
{
  gyro.calibration.cyclesRemaining = gyroCalibrationCycles();
};

void devPush(stdev_t *dev, float x)
{
    dev->n++;
    if (dev->n == 1) {
        dev->oldMean = dev->newMean = x;
        dev->oldSigma = 0.0f;
    } else {
        dev->newMean = dev->oldMean + (x - dev->oldMean) / dev->n;
        dev->newSigma = dev->oldSigma + (x - dev->oldMean) * (x - dev->newMean);
        dev->oldMean = dev->newMean;
        dev->oldSigma = dev->newSigma;
    };
};

float calculateStandardDeviation(stdev_t *dev)
{
  return sqrtf(devVarianz(dev));
};

float devVarianz(stdev_t *dev)
{
  return (dev->n > 1 ? (dev->newSigma / (dev->n - 1 )) : 0.0f);
};

int16_t constrain_int(int16_t value_to_constrain, int16_t min, int16_t max){
    int16_t result;
    if (value_to_constrain < min){
        result = min;
    } else if (value_to_constrain >max) {
        result = max;
    } else{
        result = value_to_constrain;
    };
return result;
};


float constrain_float(float value_to_constrain, float min, float max){
    float result;
    if(value_to_constrain < min){
        result = min;
    } else if (value_to_constrain > max){
        result = max;
    } else {
        result = value_to_constrain;
    };
return result;
}

void init_lookupRollPitch(){
    for (int i=0;i<5;i++){
        lookupRollPitch[i] = (1526 + EXPO * (i*i-15)) * i * (uint32_t)RC_RATE/1192;
    };
};

void init_lookupThrottle(){
	int8_t i;
	int8_t tmp,y;
	for(i=0;i<12;i++){
		tmp = 10 * i - ThrMidEx;
		y = ThrMidEx;
		lookupThrottle[i]= 100 * ThrMidEx + tmp * (10 - ExpoThrottle + ExpoThrottle * (tmp * tmp)/(y *y)+ ExpCorFactor - ThrMidEx);
		lookupThrottle[i]= MINTHROTTLE + ( MAXTHROTTLE - MINTHROTTLE) * lookupThrottle[i] / 10000;
	}; 
};

void getFlightModeAndArmDisarmSignal(void){

  if(rxSetPoint[ARM]> 992 && rxSetPoint[ARM] < 1020 ){
    SET_FLAG(f.DISARME_SIG);
  };
  if(rxSetPoint[ARM] > 1900 && rxSetPoint[ARM] < 2010){
    UNSET_FLAG(f.DISARME_SIG);
    UNSET_FLAG(f.OK_TO_DISARME);
    SET_FLAG(f.OK_TO_ARM);
  };
  if(rxSetPoint[CH6] > 992 && rxSetPoint[CH6] < 1020 ){
    UNSET_FLAG(f.ANGLE_MODE);
  };
  if (rxSetPoint[CH6] > 1480 && rxSetPoint[CH6] < 1520){
    SET_FLAG(f.ANGLE_MODE);
  };
};

void get_tpa_factor(){
 	static int32_t prop;
	if(rxSetPoint[THROTTLE] < TpaBreakPoint){
		prop = 100;
		throttlePIDAttenuation = 1.0f;
	} else{
		if(rxSetPoint[THROTTLE] < 2000){
			prop = 100 - dynThrPid * (rxSetPoint[THROTTLE] - TpaBreakPoint) / (2000 - TpaBreakPoint);
		} else {
			prop = 100 - dynThrPid;
		}; 
		throttlePIDAttenuation = prop / 100.0f;
	};
};

static void imuCalculationAttitude(uint32_t currentTimeUs){
	static uint32_t previousIMUUpdateTime;
	uint8_t USE_ACC = 0;
	uint8_t USE_MAG = 0;
	uint8_t USE_COG = 0;
	static const float dcmKpGainConf = 0.25f;	
	uint32_t deltaTime = currentTimeUs - previousIMUUpdateTime;
	previousIMUUpdateTime = currentTimeUs;
	// For 3-Axis;
	//gyroGetAccumulationAverage(gyroAverage);
	
	//deltaTime needs to be set to us	
	USE_ACC = imuIsAccelerometerHealthy(&Acc_f.input[0]);

	imuMahonyCalculation(deltaTime * 1e-6f, gyroAngleIntegrate[X] * DEG_TO_RAD, gyroAngleIntegrate[Y] * DEG_TO_RAD, gyroAngleIntegrate[Z]* DEG_TO_RAD, USE_ACC, Acc_f.input[X], Acc_f.input[Y], Acc_f.input[Z], dcmKpGainConf);
	
	imuUpdateEulerAngles();
};

static void imuMahonyCalculation(float deltaT, float gx, float gy, float gz, uint8_t useAcc, float ax, float ay, float az, float dcmKpGainConf){
	static float integralFBx = 0.0f;
	static float integralFBy = 0.0f;
	static float integralFBz = 0.0f;
	static float  dcmKpGain;
  float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
	//error
	float ex = 0, ey = 0, ez = 0;
	float recipAccNorm = sq(ax) + sq(ay) + sq(az);

	if(useAcc && recipAccNorm > 0.01f){
		//Normalise accelerometer measuremet
		recipAccNorm = invSqrt(recipAccNorm);
		ax *= recipAccNorm;
		ay *= recipAccNorm;
		az *= recipAccNorm;
	
		//error is the sum of the cross product between estimated direction and measured direction of gravity
		ex += (ay * rotMat[2][2] - az * rotMat[2][1]);
		ey += (az * rotMat[2][0] - ax * rotMat[2][2]);
		ez += (ax * rotMat[2][1] - ay * rotMat[2][0]);
	};

	//Compute and apply integral feedback if enabled 	
	
	if(spin_rate < DEG_TO_RAD * 20.0f){
		const float dcmKiGain = 0.01;
		integralFBx += dcmKiGain * ex * deltaT;
		integralFBy += dcmKiGain * ey * deltaT;
		integralFBz += dcmKiGain * ez * deltaT;
	} else {
		integralFBx = 0.0f;
		integralFBy = 0.0f;
		integralFBz = 0.0f;
	};
	
        dcmKpGain = dcmKpGainConf * getPGainScaleFactor();
	//Apply proportional and integral feedback
	gx += dcmKpGain * ex + integralFBx;
	gy += dcmKpGain * ey + integralFBy;
	gz += dcmKpGain * ez + integralFBz;

	//integrate the rate of change of quaternion
	gx *= (0.5f * deltaT);
	gy *= (0.5f * deltaT);
	gz *= (0.5f * deltaT);

	QUATERNION buffer;
	buffer.w = q.w;
	buffer.x = q.x;
	buffer.y = q.y;
	buffer.z = q.z;

	q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
	q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
	q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
	q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);
	
	// Normalise quaternion
	float recipNorm = invSqrt((sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z)));
	q.w *= recipNorm;
	q.x *= recipNorm;
	q.y *= recipNorm;
	q.z *= recipNorm;
	
  // Pre-compute rotation matrix from quaternion
  imuComputeRotationMatrix();	
}

static void imuComputeRotationMatrix(void){
	imuComputeQuaternionProducts(&q, &qP);
	
	rotMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
	rotMat[0][1] = 2.0f * (qP.xy + -qP.wz);
	rotMat[0][2] = 2.0f * (qP.xz - -qP.wy);

	rotMat[1][0] = 2.0f * (qP.xy - -qP.wz);
	rotMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
	rotMat[1][2] = 2.0f * (qP.yz + -qP.wx);

	rotMat[2][0] = 2.0f * (qP.xz + -qP.wy);
	rotMat[2][1] = 2.0f * (qP.yz - -qP.wx);
	rotMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
};

void imuComputeQuaternionProducts(QUATERNION* quat, QUATERNION_PRODUCTS* quatProd){
	quatProd->ww = quat->w * quat->w;
	quatProd->wx = quat->w * quat->x;
	quatProd->wy = quat->w * quat->y;
	quatProd->wz = quat->w * quat->z;
	quatProd->xx = quat->x * quat->x;
	quatProd->xy = quat->x * quat->y;
	quatProd->xz = quat->x * quat->z;
	quatProd->yy = quat->y * quat->y;
	quatProd->yz = quat->y * quat->z;
	quatProd->zz = quat->z * quat->z;
};

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static float invSqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
};

void imuUpdateEulerAngles(void){
	angle[ROLL] = lrintf(atan2_approx(rotMat[2][1], rotMat[2][2])*(1800.0f / M_PIf)) - (angleTrim.roll * 10.0f);
	angle[PITCH] =lrintf(((0.5f * M_PIf) - acos_approx(-rotMat[2][0])) * ( 1800.0f / M_PIf)) - (angleTrim.pitch * 10.0f);
	angle[YAW]= lrintf(-atan2_approx(rotMat[1][0], rotMat[0][0]) * (1800.0f / M_PIf));

	if(angle[YAW] < 0){
		angle[YAW] += 3600;
	};

};

static void alignSensors(int16_t *dest, uint8_t rotation){
   	
	const int16_t x = dest[X];
	const int16_t y = dest[Y];
	const int16_t z = dest[Z];

	switch (rotation) {
	case CW0_DEG:
	dest[X] = x;
	dest[Y] = y;
	dest[Z] = z;
	break;
	case CW90_DEG:
	dest[X] = y;
	dest[Y] = -x;
	dest[Z] = z;
	break;
	case CW180_DEG:
	dest[X] = -x;
	dest[Y] = -y;
	dest[Z] = z;
	break;
	case CW270_DEG:
	dest[X] = -y;
	dest[Y] = x;
	dest[Z] = z;
	break;
	case CW0_DEG_FLIP:
	dest[X] = -x;
	dest[Y] = y;
	dest[Z] = -z;
	 break;
	case CW90_DEG_FLIP:
	dest[X] = y;
	dest[Y] = x;
	dest[Z] = -z;
	break;
	case CW180_DEG_FLIP:
	dest[X] = x;
	dest[Y] = -y;
	dest[Z] = -z;
	break;
	case CW270_DEG_FLIP:
	dest[X] = -y;
	dest[Y] = -x;
	dest[Z] = -z;
	break;
	default:
	break;
	};
};

static void filterGyro (float* gyroRaw, Gyro_t* gyroValues, uint32_t currentTime){	
	static uint32_t previousTime;
	const uint32_t deltaT = currentTime - previousTime;
	previousTime = currentTime;

	static float gyroRaw_f[3];	
	
	for(uint8_t axis = 0; axis < 3; ++axis){
		gyroRaw_f[axis] = (float)(gyroRaw[axis] / 16.4f); // Scale the gyrovalue to deg per second
	};
	
	// apply static notch filters and software lowpass filters
	for(uint8_t axis = 0; axis < 3 ; ++axis){ 
		gyroValues->input[axis] = gyroSensor.notchFilter1ApplyFn((filter_t *)&gyroSensor.notchFilter1[axis], gyroRaw_f[axis]);
		gyroValues->input[axis] = gyroSensor.notchFilter2ApplyFn((filter_t *)&gyroSensor.notchFilter2[axis], gyroValues->input[axis]);
		gyroValues->input[axis] = gyroSensor.lowpassFilterApplyFn((filter_t *)&gyroSensor.lowpassFilter[axis], gyroValues->input[axis]);	
		gyroValues->input[axis] = gyroSensor.lowpass2FilterApplyFn((filter_t *)&gyroSensor.lowpass2Filter[axis], gyroValues->input[axis]);

        /*
		if(gyroConfig.DynNotchFilterActive){
			gyroValues->input[axis] = gyroSensor.notchFilterDynApplyFn((filter_t*)&gyroSensor.notchFilterDyn[axis], gyroValues->input[axis]);
		};
		*/
        
		/**
		* For the Angle Mode we need the Angle
		* Caculate the angle based on trapezium rule between to gyro readings.
		* Gyro integrated if only used once per cycle it´s giving the angle.
		* However, the input for the quaterions calc is deg. per sec
		* if gyroAngleIntegrate gets accumulated after the use it must be set to 0 again.
		* gyroAngleIntegrated[axis] += ... if the cyle for the reading is faster the overall loop it can be calucalted the average
		*/

		gyroAngleIntegrate[axis] = 0.5f * (gyroValues->last[axis] + gyroValues->input[axis]); // *(deltaT) and later / measurementTimesAccumulated to get average;
		gyroValues->last[axis] = gyroValues->input[axis];
	};
};

static void filterAccel(float *accelRaw, Acc_t *Acc_f){
	for (uint8_t axis = 0; axis < 3; axis++){
		Acc_f->input[axis]= biquadFilterApply(&accelFilter[axis], accelRaw[axis]);
	};
};

static bool imuIsAccelerometerHealthy(float *AccelReadings){
    float accMagnitude = 0;
    for (int axis = 0; axis < 3; axis++) {
        const float a = AccelReadings[axis];
        accMagnitude += a * a;
    }
   	accMagnitude = accMagnitude * sq((1.0f/4096)); //int32_t)acc.dev.acc_1G))	
	 
    // Accept accel readings only in range 0.90g - 1.10g
    return (0.81f < accMagnitude) && (accMagnitude < 1.21f);
};

//-------------------------------------------------------------------
//------------------------- Fast MATHS Functions --------------------
//-------------------------------------------------------------------

float atan2_approx(float y, float x)
{
   float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
};

float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
};

float cos_approx(float x){
	return sin_approx(x + (0.5f * M_PIf));
};

float sin_approx(float x){
	int32_t xint = x;
	if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
	while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    	while (x < -M_PIf) x += (2.0f * M_PIf);
	if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
	else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
	float x2 = x * x;
	return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
};

void queueFloatToBuf(float num){
	float_to_buffer(num, pbuffer, pbufferPosition);
};

void queueTextToBuf(char* text){
	text_to_buffer(text, pbuffer, pbufferPosition);
};

void queueIntToBuf(int32_t num){
	int_to_buffer(num, pbuffer, pbufferPosition);
};

void sendBufferOverUart1(void){
  DMA1_Channel4->CNDTR = buffPosition; //Setting the size of the buffer dynamically;
  DMA1_Channel4->CMAR = (uint32_t)&buffer;
  DMA_Cmd(DMA1_Channel4, ENABLE);
  usart1TransferComplete = 0;	
};

void areRxSetPointsValid(void){	
  static uint16_t invalidOccurences = 0;
  (rxSetPoint[0] < 2010 && rxSetPoint[0]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch1) : RC_CHANNEL_NOT_VALID(rxChannel.Ch1);
  (rxSetPoint[1] < 2010 && rxSetPoint[1]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch2) : RC_CHANNEL_NOT_VALID(rxChannel.Ch2);
  (rxSetPoint[2] < 2010 && rxSetPoint[2]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch3) : RC_CHANNEL_NOT_VALID(rxChannel.Ch3);
  (rxSetPoint[3] < 2010 && rxSetPoint[3]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch4) : RC_CHANNEL_NOT_VALID(rxChannel.Ch4);
  (rxSetPoint[4] < 2010 && rxSetPoint[4]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch5) : RC_CHANNEL_NOT_VALID(rxChannel.Ch5);
  (rxSetPoint[5] < 2010 && rxSetPoint[5]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch6) : RC_CHANNEL_NOT_VALID(rxChannel.Ch6);
  (rxSetPoint[6] < 2010 && rxSetPoint[6]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch7) : RC_CHANNEL_NOT_VALID(rxChannel.Ch7);
  (rxSetPoint[7] < 2010 && rxSetPoint[7]> 990) ? RC_CHANNEL_IS_VALID(rxChannel.Ch8) : RC_CHANNEL_NOT_VALID(rxChannel.Ch8);
    
  if(validRxChannels == 255){ //0b11111111 all signals are valid
    SET_FLAG(f.VALID_RX_SIGNALS);
    UNSET_FLAG(f.RX_SIGNALS_WAS_VALID);
    invalidOccurences = 0;
  } else if (invalidOccurences <= 500){
    ++invalidOccurences;
  };
  
  if(invalidOccurences >= 500 && f.VALID_RX_SIGNALS){
    UNSET_FLAG(f.VALID_RX_SIGNALS);
    SET_FLAG(f.RX_SIGNALS_WAS_VALID);
  };
  
};

void flashBoardLed(uint16_t flashTime){
  current_time = millis();
  while(millis() - current_time < flashTime){
    GPIOB->ODR ^= (0x1 << LED_R);
    delay_millis(100);
  };
  GPIOB->ODR |= (0x1 << LED_R); //Turns out the LED_R somehow inverted
};


void initSystemAndModules(void){	
  initBoardLed();
  init_Buzzer();
  systemBeepPtr = beeperSpRacing; 
  usart1_init();
  delay_millis(100);
  init_i2c1();
  delay_millis(100);
  init_timer_ppm();
  delay_millis(100);
  init_timer_pwm();	
  delay_millis(100);
  init_lookupRollPitch();	
  init_lookupThrottle();
  init_gyro(&gyro, &gyroConfig);
  gyroInitFilters(&gyroSensor);
  accelInitLowPassFilter(&accelFilter[0], accLpfCutHz, targetLoopTime, BIQUAD_Q, FILTER_LPF);
  initDtermSettings(&dtermFilterSettings);	
  initDtermNotchFilter();
  initDtermLowpassFilter();
  initDtermLpfFilter(dtermFilterSettings.dterm_lowpass_type);
  initDmaUsart1Tx();
  delay_millis(100);
  initDmaUsart1Rx();
  delay_millis(100);
  init_dmaUart1Rx_t();
  delay_millis(100);
  imuComputeRotationMatrix();
  delay_millis(100); 
  initI2C1DmaRx(); 
  delay_millis(100);
  assignPidValues();
  setPidFrequency(targetLoopTime);
  
 };

void getReceiverInputValues(int32_t *receiverInputValues){ 
  static const float alpha  = 0.2f;
  int16_t rxInputDataTemp[RC_CHANNELS];
  memcpy(rxInputDataTemp, rxInputData, sizeof(rxInputData));
  for (int i = 0; i < RC_CHANNELS; ++i){
    receiverInputValues[i] = lrintf((alpha * rxInputDataTemp[i] + ( 1.0f - alpha) * receiverInputValues[i]));
  };
};

void taskHandleSerialPortDebug(void){
  static uint8_t ch;
  while(uart1VTable.serialRxTotalBytesWaiting()){
    ch = uart1VTable.serialRead();  	
   };   
  if(ch == 'R'){
    systemResetToBootloader();
  };
  if(ch == 'C'){
   checkReceivedCommandAndPerformAction(); 
  };
  
};

#ifdef DEBUG
void flushBufferOverUart(void){
  //COMMUNICATION WITH USART	
  if(usart1TransferComplete){
    queueIntToBuf(loop_duration);
    queueIntToBuf(loopDurationAfterWait);
    /*
    queueIntToBuf(angle[PITCH]);
    queueIntToBuf(esc3);
    queueIntToBuf(esc4);
    queueIntToBuf(rxSetPoint[4]);
    queueIntToBuf(rxSetPoint[5]);
    queueIntToBuf(rxSetPoint[6]);
    queueIntToBuf(rxSetPoint[7]);
    queueIntToBuf(AttitudeCalled);
    queueIntToBuf(f.OK_TO_ARM);
*/
    queueTextToBuf("\n");	
    sendBufferOverUart1();
  };
};
#endif

void parseArmDisarmSignal(void){
//Check for disarm_signal - needs to be valid for 0,5 sec because we are using one channel.	
  static uint8_t disarmOccurences=0;

  if(f.DISARME_SIG && f.ARMED){
    disarmOccurences += 1;
  } else if(!f.DISARME_SIG) {
    disarmOccurences = 0;
  };
  
  if(disarmOccurences > 50 && f.ARMED){ //Disarming signal needs to be valid for 0,01 secs
    SET_FLAG(f.OK_TO_DISARME);
  };
   
 // ARME the QUAD and reset all the values for the PID Structre where the errors and mem are saved
 if (f.OK_TO_ARM && f.DISARMED && rxSetPoint[THROTTLE] < 1020){
   SET_FLAG(f.ARMED);
   UNSET_FLAG(f.DISARMED);
   UNSET_FLAG(f.OK_TO_ARM);
   UNSET_FLAG(f.WAS_ARMED);
   memset(&PID_ERR, 0, sizeof(PID_ERRORS)); //memset --> reset the structure mem for bumbless restart
 };
       
 //Turn Motors off
 if(f.OK_TO_DISARME && f.ARMED && rxSetPoint[THROTTLE] < 1050){
   UNSET_FLAG(f.ARMED);
   SET_FLAG(f.DISARMED);
   SET_FLAG(f.WAS_ARMED);
 };
};

void taskWriteMotors(void){
  updatePWMTimerRegistersForMotors();	
  updatePWMTimerRegistersForGimbal();
};  

void map(uint16_t* value){
  *value = ((*value - OLD_RANGE_MIN) * (NEW_RANGE_MAX - NEW_RANGE_MIN) / (OLD_RANGE_MAX - OLD_RANGE_MIN) + NEW_RANGE_MIN);
}; 

void subtaskUpdateAttitude(uint32_t callingTime){
   imuCalculationAttitude(callingTime);
};

float getPGainScaleFactor(void){
  static float pGainScale;
  if (!f.ARMED && (millis() < 20000)){
    pGainScale = 100.0f;
  } else {
    pGainScale = 1.0f;
  };
return pGainScale;
};

void assignPidValues(void){
  pidGain[ROLL].P = pid_p_gain_roll;
  pidGain[ROLL].I = pid_i_gain_roll;
  pidGain[ROLL].D = pid_d_gain_roll;
  pidGain[ROLL].max = pid_max_roll;

  pidGain[PITCH].P = pid_p_gain_pitch;
  pidGain[PITCH].I = pid_i_gain_pitch;
  pidGain[PITCH].D = pid_d_gain_pitch;
  pidGain[PITCH].max = pid_max_roll;

  pidGain[YAW].P = pid_p_gain_yaw;
  pidGain[YAW].I = pid_i_gain_yaw;
  pidGain[YAW].D = pid_d_gain_yaw;
  pidGain[YAW].max = pid_max_roll;
};

