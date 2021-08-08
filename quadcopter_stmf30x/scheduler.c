#include "scheduler.h" 
#define FRQ_TO_MISEK(frq) ((1/frq)*1e6)

task_t taskList[2] = {
  {ATTITUDE_UPDATE, subtaskUpdateAttitude, 10000, 1},
  {BUZZER_UPDATE, subtaskUpdateBuzzer, 10000, 1}
};

void executeNextSubTask(uint32_t currentCallingTime){ 
  static int subTask = 0;
  switch(subTask){
    case ATTITUDE_UPDATE:
      if(taskList[subTask].nextCallingTime <= currentCallingTime){
        taskList[subTask].nextCallingTime = currentCallingTime + taskList[subTask].taskUpdateFrq;
        taskList[subTask].subtaskFunction(currentCallingTime); //call the subTaskFunction
        subTask = BUZZER_UPDATE; //set next subTask
      };
      break;

    case BUZZER_UPDATE:
      if(taskList[subTask].nextCallingTime <= currentCallingTime){
        taskList[subTask].nextCallingTime = currentCallingTime + taskList[subTask].taskUpdateFrq;
        taskList[subTask].subtaskFunction(currentCallingTime); //call the subTaskFunction
        subTask = ATTITUDE_UPDATE; //set next subTask
      };
      break;
    default:
      break;
  };
};
