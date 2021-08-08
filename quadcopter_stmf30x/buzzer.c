#include "buzzer.h"


buzzer_t buzzer;

const uint8_t buzz_shortBeep[] = {
  10, 10, 0xFF
};
const uint8_t buzz_gyroCalibrated[] = {
  5, 15, 5, 15, 5, 15, 20, 15, 0xFF
};
const uint8_t buzz_disArmBeep[] = {
 5, 5, 15, 5, 5, 15, 30, 0xFF 
};
const uint8_t buzz_readyBeep[] = {
  4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, 0xFF
};

void activateBuzzer(const uint8_t* Sequence){
  buzzer.SequencePtr = Sequence;
  buzzer.Mode = ACTIVE;
  buzzer.Pos = 0;
};

void subtaskUpdateBuzzer(uint32_t currentBuzzerUpdate){
  //If the buzzer has nothing to do the return
  if(buzzer.Mode == INACTIVE || buzzer.SequencePtr == NULL) return;
  
  //Otherwise check the buzzer sequence 
  if(!buzzer.IsOn && buzzer.NextToggleTime <= currentBuzzerUpdate){
    buzzer.IsOn = ON;
    if(buzzer.SequencePtr[buzzer.Pos] != 0) toggleBuzzer(ON);
    calculateBuzzer(currentBuzzerUpdate);
  };

  if(buzzer.IsOn && buzzer.NextToggleTime <= currentBuzzerUpdate){
    buzzer.IsOn = OFF;
    if(buzzer.SequencePtr[buzzer.Pos] != 0) toggleBuzzer(OFF);
    calculateBuzzer(currentBuzzerUpdate);
  };

};

void calculateBuzzer(uint32_t callingTime){
  if(buzzer.SequencePtr[buzzer.Pos] == 0xFF)
  { //turn off 
    buzzer.IsOn = OFF;
    buzzer.Mode = INACTIVE;
    buzzer.SequencePtr = NULL;
    toggleBuzzer(OFF);
  } else 
    {
    buzzer.NextToggleTime = callingTime + 10 * 1000 * buzzer.SequencePtr[buzzer.Pos];
    buzzer.Pos ++;
    };
};

void toggleBuzzer(bool onoff){
  systemBeepPtr(onoff); //->beeperSpRaching 
};

void beeperSpRacing(bool onoff){
  if(onoff){
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET); 
  } else {
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);  
  };
};
