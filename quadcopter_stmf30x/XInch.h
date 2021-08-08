//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//############### PID gain and limit settings / auto_level ##########################
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const float pid_p_gain_roll = 1.5;            // +- 0.01 Gain setting for the roll P-controller.
const float pid_i_gain_roll = 0.0065;          // +- 0.0001 Gain setting for the roll I-controller. Last value 0.013
const float pid_d_gain_roll = 15.25;             //+- 0.05 Gain setting for the roll D-controller. Last value 12.05
const uint16_t pid_max_roll = 490;              //Maximum output of the PID-controller (+/-)

const float pid_p_gain_pitch = 1.85;             //Gain setting for the pitch P-controller. 1.22
const float pid_i_gain_pitch = 0.00075;             //Gain setting for the pitch I-controller 0.012
const float pid_d_gain_pitch = 16.65;            //Gain setting for the pitch D-controller. 13.15
const uint16_t pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)

const float pid_p_gain_yaw  = 2.0;                //Gain setting for the pitch P-controller 3.1
const float pid_i_gain_yaw = 0.00085;               //Gain setting for the pitch I-controller 0.015
const float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller 0.0
const uint16_t pid_max_yaw = 430;                 //Maximum output of the PID-controller (+/-)

//Roll Rate only needed in the old setpoint calculation
const float yaw_rate = 1.22;          
const float yawSetPointInversion = -1.0f;
// Portion to calculate the max roll rate ( +-500/x) 
#define MOTORSETUP_1

const uint16_t TpaBreakPoint = 1350;		// TPA Breakpoint when the dampening of the pids will start  
const uint16_t dynThrPid = 30;			// dampaning value 

const uint16_t accLpfCutHz = 30;
//Level Values

float gain_level = 0.50f;

typedef struct{
	float roll;
	float pitch;
} TRIMS;
TRIMS angleTrim = { .roll = -0.5, .pitch = -1.1};

// Values for THrottle Expo
uint8_t ThrMidEx = 50;
uint8_t ExpoThrottle = 85; // Values need to be between 
uint8_t ExpCorFactor = 140;

#define MINCHECK 1010 // Flat curve until 1050 at MINTHROTTLE 
#define MINTHROTTLE 1080	//1100
#define MAXTHROTTLE 1885	//1850
#define MID_RC  190
#define MAX_RC 250
#define MIN_RC 125
#define LOW_RC 122

//value for the expo curve for roll and pitch

#define EXPO 65 // determing the expofunction for ROLL and PITCH
#define RC_RATE 98 // 95 will result at a Rande of [-498 - +498]

#define MAX_ANGLE 180 //will result in 18 degree
#define MIN_ANGLE (MAX_ANGLE * -1)

///////////////////////////////////////////////////////
//------------------ FILTER SETTINGS ------------------
///////////////////////////////////////////////////////

#define GYRO_LOW_PASS_FILTER FILTER_BIQUAD //2nd order Butterworth if set to biquad
#define GYRO_LOW_PASS_HZ 90
#define GYRO_LOW_PASS_FILTER2 FILTER_PT1
#define GYRO_LOW_PASS_HZ_2 180
#define GYRO_SOFT_NOTCH_HZ_1 220
#define GYRO_SOFT_NOTCH_CUTOFF_1 170
#define GYRO_SOFT_NOTCH_HZ_2 170
#define GYRO_SOFT_NOTCH_CUTOFF_2 100 
#define DYN_NOTCH_FILTER_ACTIVE 0 //Not working yet
////////////////////////////////////////////////////////////////////////////////////////////
#define DTERM_NOTCH_HZ 220; //250 notch needs to be established 
#define DTERM_NOTCH_CUTOFF_HZ 140;//160;
#define DTERM_LOWPASS2_HZ 200 ;//200;
#define DTERM_LOWPASS_TYPE FILTER_PT1;
#define DTERM_LOWPASS_HZ 100 ;//100;
