#define MPU6050 ((MPU6050_ADDRESS &0x7f)<<1)
/**
 * Address needs to be shifted to the upper 7 bits
 */
#define LED_R 3
#define RC_CHANNELS 8
#define GYRO_LSB 16.4  //Full Scale of the gyro
#define ARM CH5
#define DEG_TO_RAD 0.0174532925
#define ACC_MPU6050_ALIGN CW270_DEG
#define GYRO_MPU6050_ALIGN CW270_DEG
/* 
 * Calculations for Motors are done based on standard PWM Signals
 * Values must be mapped for oneshot125
 */ 
#define OLD_RANGE_MIN 1000
#define OLD_RANGE_MAX 2000
#define NEW_RANGE_MIN 125
#define NEW_RANGE_MAX 250
#define DEADBAND_RCSETPOINT 10

//############  Failsafe settings ######
/* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC is avaliable),
PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */

#define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
#define FAILSAFE_TRESHOLD  985

#define QUAD_5I
//#define QUAD_4I
//#define QUAD_X
#if defined QUAD_4I
	#include "4Inch.h"
#elif defined QUAD_5I
	#include "5Inch.h"
#elif defined QUAD_X
    #include "XInch.h"
#endif

#define FALSE 0;
#define TRUE 1;

//-------------------------- VARIABELS -----------------------------
uint16_t failsafe_cnt = 0;
const uint16_t targetLoopTime = 500; //Should be multiple of OneShot125 e.g 250 us 
volatile uint32_t usTicks, msTicks;
float Gyro[3]; 
float Accel[3];
int32_t current_time;
volatile int16_t rxInputData[RC_CHANNELS];
int32_t rxSetPoint[RC_CHANNELS];
uint16_t esc1, esc2, esc3, esc4, throttle;
uint32_t lookupRollPitch[5]; //5 Values [0-507];
uint32_t lookupThrottle[12]; //12 Values for the Throttle Lookup 
float throttlePIDAttenuation;
float rotMat[3][3];
float gyroAngleIntegrate[3];
uint8_t buffer[250] = {0x00};
uint8_t* pbuffer= buffer;
uint16_t buffPosition;
uint16_t* pbufferPosition = &buffPosition;
volatile uint8_t usart1TransferComplete = 1;
uint16_t AttitudeUpdateFrequenzy;
int32_t loop_duration, loopDurationAfterWait;
//initialize extern variables here
//EXTERNAL variables from filter.h used in filter.c aswell
extern uint8_t gyroRawData[];
gyro_t gyro;
gyroConfig_t gyroConfig;
dtermFilter_t dtermFilterSettings;
static float pidFrequency;


//########################## ENUM ################################

#if RC_CHANNELS == 6
    enum RxChan{
        ROLL,
        PITCH,
        YAW,
        THROTTLE,
        CH5,
        CH6
    };
#elif RC_CHANNELS == 7 
	enum RxChan{
	ROLL,
	PITCH,
	YAW,
	THROTTLE,
	CH5,
	CH6,
	CH7
   	};
#elif RC_CHANNELS == 8 
	enum RxChan{
	ROLL,
	PITCH,
	YAW,
	THROTTLE,
	CH5,
	CH6,
	CH7,
	CH8
        };
#endif


enum axis{
	X,
	Y,
	Z,
};

typedef struct{
  uint8_t Ch1;
  uint8_t Ch2;
  uint8_t Ch3;
  uint8_t Ch4;
  uint8_t Ch5;
  uint8_t Ch6;
  uint8_t Ch7;
  uint8_t Ch8;
} rxChannel_s;
rxChannel_s rxChannel = {.Ch1=1<<0,.Ch2=1<<1,.Ch3=1<<2,.Ch4=1<<3,.Ch5=1<<4,.Ch6=1<<5,.Ch7=1<<6,.Ch8=1<<7};

uint8_t validRxChannels = 0;
#define RC_CHANNEL_IS_VALID(channel) (validRxChannels |= (channel))
#define RC_CHANNEL_NOT_VALID(channel) (validRxChannels &= ~(channel)) 

enum orientation{
	CW0_DEG = 1,
	CW90_DEG,
	CW180_DEG,
	CW270_DEG,
	CW0_DEG_FLIP,
	CW90_DEG_FLIP,
	CW180_DEG_FLIP,
	CW270_DEG_FLIP,
};

typedef struct motorMixer_s {
    float throttle;
    float pitch;
    float roll;
    float yaw;
} motorMixer_t;

int32_t angle[XYZ_AXIS_COUNT];

static const motorMixer_t mixerQuadX[] = {
#if defined MOTORSETUP_1
//THROTTLE - PITCH - ROLL - YAW
    { 1.0f, -1.0f, 1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f, 1.0f },          // FRONT_R
    { 1.0f, 1.0f, -1.0f, -1.0f },          // REAR_R
    { 1.0f, 1.0f, 1.0f, 1.0f },          // REAR_L

#elif defined MOTORSETUP_2

//THROTTLE - PITCH - ROLL - YAW
    { 1.0f, -1.0f, 1.0f, 1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f, -1.0f },          // FRONT_R
    { 1.0f, 1.0f, -1.0f, 1.0f },          // REAR_R
    { 1.0f, 1.0f, 1.0f,  -1.0f },          // REAR_L
#endif
};

typedef struct{
    	float temp;
	float P[XYZ_AXIS_COUNT];
	float last[XYZ_AXIS_COUNT];
	float I[XYZ_AXIS_COUNT];
	float d_now[XYZ_AXIS_COUNT];
       	float D[XYZ_AXIS_COUNT];
} PID_ERRORS;
PID_ERRORS PID_ERR = {0};

float pidOutput[XYZ_AXIS_COUNT];
float pidSetpoint[XYZ_AXIS_COUNT];

typedef struct pid_t{
  float P;
  float D;
  float I;
  uint16_t max;
}pid_s;
pid_s pidGain[3];


//Create a own struct for the level mode values that can be erased in the mode is not active

typedef struct{
	float w,x,y,z;
} QUATERNION;
static QUATERNION q = {.w=1.0, .x=0.0, .y=0.0, .z = 0.0};

typedef struct{
	float ww, wx, wy, wz, xx, xy, xz, yy, yz, zz;
} QUATERNION_PRODUCTS;
static QUATERNION_PRODUCTS qP = {.ww = 1.0, .wx=0.0, .wy=0.0, .wz=0.0, .xx=0.0, .xy=0.0, .xz=0.0, .yy=0.0, .yz=0.0, .zz=0.0};

void imuComputeQuaternionProducts(QUATERNION* quat, QUATERNION_PRODUCTS* quatProd);

/**
 * Bitfeld zur Nutzung von Flags. Es könnte auch eine enumration verwendet werden in Kombination
 * mit einer Variablen und Bit-Operatoren
 */
typedef struct {   // Create a bitfield according to a structure that is saving some flags
  uint8_t ARMED :1;
  uint8_t OK_TO_ARM :1;
  uint8_t WAS_ARMED :1;
  uint8_t VALID_RX_SIGNALS :1;
  uint8_t RX_SIGNALS_WAS_VALID :1;
  uint8_t DISARMED :1;
  uint8_t ACC_CALIBRATED :1;
  uint8_t ANGLE_MODE :1;
  uint8_t HORIZON_MODE :1;
  uint8_t MAG_MODE :1;
  uint8_t BARO_MODE :1;
  uint8_t OK_TO_DISARME :1;
  uint8_t DISARME_SIG :1;
  uint8_t GYRO_CALIBRATED :1;
  uint8_t GYRO_DATA_AVAILABLE :1;
  uint8_t GYRO_DATA_NOT_PROCESSED :1;
} FLAGS;
FLAGS f = {0}; //Initialize the structure with Zero

#define SET_FLAG(flag) (flag = 1)
#define UNSET_FLAG(flag) (flag = 0)

typedef struct{
	int16_t gyro[3];
	int16_t accel[3];
	int16_t temp;
} SENSORS;
SENSORS imu = {0};

float gyro_roll_input = 0.0;
float gyro_pitch_input = 0.0;
float gyro_yaw_input = 0.0;

typedef struct{
	float temp[3];
	float temp2[3];
	float input[3];
	float last[3];
} Gyro_t;
Gyro_t Gyro_f = {0}; //structure was needed for the running average an will yield the filter (f) gyro values.

typedef struct{
	float input[3];
} Acc_t;
Acc_t Acc_f = {0};

/**
 * ---------------------------- FUNCTIONS ---------------------------
 */
void SysTick_Handler(void);
uint32_t millis();
void delay_millis(uint32_t dlyticks);
uint32_t micros();
static void delay_micros(uint32_t dlyticks);
void initSystemAndModules(void);
void runFlightControllerTasks(void);
void updatePWMTimerRegistersForMotors(void);
void updatePWMTimerRegistersForGimbal(void);
void init_MPU6050(void);
void init_gyro(gyro_t *gyro, gyroConfig_t *gyroConfig);
static inline void processGyroRawData(void);
void areRxSetPointsValid(void);
int16_t constrain_int(int16_t value_to_constrain, int16_t min, int16_t max);
float constrain_float(float value_to_constrain, float min, float max);
void init_lookupRollPitch(void);
void init_lookupThrottle(void);
static inline void pid_calculation(void);
void getFlightModeAndArmDisarmSignal(void);
void get_tpa_factor(void);
void calculateRcRatesForPitchAndRoll(void);
void getThrottleSetPointAndCalculateExpo(void);
void calculateEscTimingFlight(void);
void calculateEscTimingGround(void);
static inline void checkLoopTimingAndWait(const uint32_t LoopTime);
static void imuCalculationAttitude(uint32_t currentTimeUs);
static void imuMahonyCalculation(float deltaT, float gx, float gy, float gz, uint8_t useAcc, float ax, float ay, float az, float dcmKpGain);	
static void imuComputeRotationMatrix(void);
static float invSqrt(float x);
void imuUpdateEulerAngles(void);
static void alignSensors(int16_t *dest, uint8_t rotation);
static void filterGyro (float* gyroRaw, Gyro_t* gyroValues, uint32_t currentTime);
static void filterAccel(float *accelRaw, Acc_t* Acc_f);
static bool imuIsAccelerometerHealthy(float *AccelReadings);
void initDtermSettings(dtermFilter_t *dtermSettings);
void queueFloatToBuf(float num);
void queueTextToBuf(char* text);
void queueIntToBuf(int32_t num);
void printIntro(void);
void i2cScanner(void);
void sendBufferOverUart1(void);
extern void checkReceivedCommandAndPerformAction(void);
void flashBoardLed(uint16_t flashTime);
void initBoardLed(void);
void getReceiverInputValues(int32_t *receiverInputValues);
void writePWMSignalsForGimbal(void);
void reinitializePWMTimerCounter(void);
static inline void taskMainPidLoop(void);
void taskHandleSerialPortDebug(void);
void flushBufferOverUart(void);
void parseArmDisarmSignal(void);
void init_Buzzer(void);
void taskWriteMotors(void);
void map(uint16_t* value);
void initI2CDma(void);
void taskFetchGyroData(void);
float getPGainScaleFactor(void);
void performGyroCalibration(void);
bool isOnFirstCalibrationCycle(void);
uint32_t gyroCalibrationCycles(void);
bool isOnFinalCalibrationCycle(void);
void setGyroCalibrationCycles(void);
void devPush(stdev_t *dev, float x);
float calculateStandardDeviation(stdev_t *dev);
float devVarianz(stdev_t *dev);
void applyItermRelax(int axis, float previousIterm, float* itermError, float currentSetPoint);
void assignPidValues(void);
void setPidFrequenzcy(uint16_t pidTargetLoopTime);
