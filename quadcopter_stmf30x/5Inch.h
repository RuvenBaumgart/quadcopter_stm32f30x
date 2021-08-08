//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//############### PID gain and limit settings / auto_level ##########################
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const float pid_p_gain_roll = 1.28;            // +- 0.03 Gain setting for the roll P-controller. Previous Value 1.77
const float pid_i_gain_roll = 4.7 * 1e-3f;          // +- 0.00001 Gain setting for the roll I-controller. 4.7
const float pid_d_gain_roll = 14.15 * 1e-3f;             //+- 0.05 Gain setting for the roll D-controller.  15.75
const uint16_t pid_max_roll = 490;              //Maximum output of the PID-controller (+/-)

const float pid_p_gain_pitch = 1.68;             //Gain setting for the pitch P-controller. 2.45
const float pid_i_gain_pitch = 5.3 * 1e-3f;             //Gain setting for the pitch I-controller 5.3
const float pid_d_gain_pitch = 16.25 * 1e-3f;            //Gain setting for the pitch D-controller. 18.75
const uint16_t pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)

const float pid_p_gain_yaw  = 4.55;                //Gain setting for the pitch P-controller 1.4
const float pid_i_gain_yaw = 7.5 * 1e-3f;               //Gain setting for the pitch I-controller 0.00125
const float pid_d_gain_yaw = 0.0 * 1e-3f;                //Gain setting for the pitch D-controller 0.0
const uint16_t pid_max_yaw = 430;                 //Maximum output of the PID-controller (+/-)

//Roll Rate only needed in the old setpoint calculation
const float yaw_rate = 1.0;          
const float yawSetPointInversion = -1.0f;
// Portion to calculate the max roll rate ( +-500/x) 
#define MOTORSETUP_1

const uint16_t TpaBreakPoint = 1350;		// TPA Breakpoint when the dampening of the pids will start  
const uint16_t dynThrPid = 86;			// dampaning value 

const uint16_t accLpfCutHz = 30;
//Level Values

const float gain_level = 1.0f;

typedef struct{
	float roll;
	float pitch;
} TRIMS;
TRIMS angleTrim = { .roll = 0.0, .pitch = -1.0};

// Values for THrottle Expo
uint8_t ThrMidEx = 50;
uint8_t ExpoThrottle = 45; // Values need to be between 
uint8_t ExpCorFactor = 140;

#define MINCHECK 1050 // Flat curve until 1050 at MINTHROTTLE 
#define MINTHROTTLE 1100	//1180
#define MAXTHROTTLE 1851	//1850
#define MID_RC 1500
#define MAX_RC 1990
#define MIN_RC 1000
#define LOW_RC 985

//value for the expo curve for roll and pitch

#define EXPO 65 // determing the expofunction for ROLL and PITCH
#define RC_RATE 130 // 95 will result at a Rande of [-498 - +498]
#define MAX_ANGLE 180 //will result in 18 degree
#define MIN_ANGLE (MAX_ANGLE * -1)
#define ITERM_RELAX_SETPOINT_THRESHOLD 40.0f

///////////////////////////////////////////////////////
//------------------ FILTER SETTINGS ------------------
///////////////////////////////////////////////////////

#define GYRO_LOW_PASS_FILTER FILTER_PT1  // FILTER_BIQUAD 2nd order Butterworth if set to biquad
#define GYRO_LOW_PASS_HZ 100
#define GYRO_LOW_PASS_FILTER2 FILTER_BIQUAD
#define GYRO_LOW_PASS_HZ_2 170
#define GYRO_SOFT_NOTCH_HZ_1 400
#define GYRO_SOFT_NOTCH_CUTOFF_1 300
#define GYRO_SOFT_NOTCH_HZ_2 200
#define GYRO_SOFT_NOTCH_CUTOFF_2 100 
#define DYN_NOTCH_FILTER_ACTIVE 0 //Not working yet
////////////////////////////////////////////////////////////////////////////////////////////
#define DTERM_NOTCH_HZ 260 //notch needs to be established 
#define DTERM_NOTCH_CUTOFF_HZ 160//160;
#define DTERM_LOWPASS2_HZ 200 //200 /PT1 Filter;
#define DTERM_LOWPASS_TYPE FILTER_PT1
#define DTERM_LOWPASS_HZ 100 //100;
