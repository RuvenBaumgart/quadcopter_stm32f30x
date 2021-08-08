#include <stm32f30x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define ACTIVE 1
#define INACTIVE 0
#define ON 1
#define OFF 0

extern const uint8_t buzz_shortBeep[];
extern const uint8_t buzz_gyroCalibrated[];
extern const uint8_t buzz_readyBeep[];
extern const uint8_t buzz_disArmBeep[];

typedef struct buzzer_s{
  const uint8_t *SequencePtr;
  uint8_t Mode;
  uint8_t Pos;
  uint8_t IsOn;
  uint32_t NextToggleTime;
} buzzer_t;
extern buzzer_t buzzer;

void activateBuzzer(const uint8_t *sequence);
void subtaskUpdateBuzzer(uint32_t currentBuzzerUpdate);
void (*systemBeepPtr)(bool onoff);
void calculateBuzzer(uint32_t callingTime);
void toggleBuzzer(bool onoff);
void beeperSpRacing(bool onoff);
extern uint32_t millis(void);
