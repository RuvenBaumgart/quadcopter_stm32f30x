#include <stm32f30x.h>
#include <stdlib.h>
#include <stdio.h>
  
enum subtasks_e{
  ATTITUDE_UPDATE = 0,
  BUZZER_UPDATE
};

typedef struct {
  uint8_t taskName;
  void (*subtaskFunction)(uint32_t time); 
  uint32_t taskUpdateFrq;
  //uint32_t taskLastCall;
  uint32_t nextCallingTime;  
} task_t;

extern void subtaskUpdateBuzzer(uint32_t currentBuzzerUpdate);
extern void subtaskUpdateAttitude(uint32_t currentAttitudeUpdate);
void executeNextSubTask(uint32_t currentCallingTime);
