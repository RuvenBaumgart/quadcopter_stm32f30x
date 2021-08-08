#include "fc_config.h"

void printIntro(void){
  uint16_t lines; 
  uint16_t lengthOfLine;  
  
  lines = sizeof(introBuffer)/sizeof(introBuffer[0]);
  for( int i = 0; i < lines; i++){
    while(!usart1TransferComplete);
    lengthOfLine = strlen(introBuffer[i].text);
    DMA1_Channel4->CNDTR = lengthOfLine; //Setting the size of the buffer dynamically;
    DMA1_Channel4->CMAR = (uint32_t)introBuffer[i].text;
    DMA_Cmd(DMA1_Channel4, ENABLE);
    usart1TransferComplete = 0;
  };  
};

void i2cScanner(void){
	uint8_t address, returnValue;
	uint16_t deviceFound;
	
	while(!usart1TransferComplete);	
	  queueTextToBuf("Scanning address spectrum 1 - 127 for i2c device \n");
	  sendBufferOverUart1();

	for(address = 1; address < 127; address++){
	  returnValue = I2C_getStatus(I2C1, (address & 0x7f)<<1 );
	  
	  switch(returnValue){
	    case 1:
	      while(!usart1TransferComplete);	
	        queueTextToBuf("Device found at: ");
	        queueIntToBuf(address);
	        queueTextToBuf("\n");	
	        sendBufferOverUart1();
	        deviceFound++;
	      break;
	    default:
	      break;
	  };   
	};
};
	
void checkReceivedCommandAndPerformAction(){
  uint8_t command;
  uint8_t exitFlag;
  exitFlag = 0;
  command = 0;

  while(!exitFlag){
    while(uart1VTable.serialRxTotalBytesWaiting()){ 
    command = uart1VTable.serialRead();  
	    delay_millis(10);
	    /**
	    * Uart request needs some time, otherwise the 
	    * system will hang up
	    */
    };
    switch(command) {
      case 97://a
        command = 0;
	/**
	 * print pwm receiversignals
	 */		
	break;
	case 98: //
	  i2cScanner();
	  command = 0;
	  break;
	case 112: //p
	  queueTextToBuf("Please increase the throttle to max - wait - and decrease it to min in the next 10 sec \n");
	  sendBufferOverUart1();
	  escCalibration();
	  command = 0;
	  break;
	case 113: //q
	  exitFlag = 1;
	default:
	  break;
    };
   flashBoardLed(2000);		
  }; 
};

void escCalibration(){
  int32_t receiverInput[8];
  uint32_t timeout;  
  timeout = millis();
  esc1 = 1;
  esc2 = 1;
  esc3 = 1;
  esc4 = 1;
  updatePWMTimerRegistersForMotors();  
  delay_millis(1000);
  while(millis() - timeout < 5000 || receiverInput[3] < 1900){
    flashBoardLed(1000);
    getReceiverInputValues(receiverInput); 
    queueIntToBuf(receiverInput[3]);
    sendBufferOverUart1(); 
  };
  
  if(receiverInput[3] > 1900){
    timeout = millis();
    while(millis() - timeout < 10000 ){ 
     getReceiverInputValues(receiverInput);
     esc1 = receiverInput[3];
     esc2 = receiverInput[3];
     esc3 = receiverInput[3];
     esc4 = receiverInput[3];
     queueIntToBuf(receiverInput[3]);
     queueTextToBuf(" \n");
     sendBufferOverUart1(); 
     updatePWMTimerRegistersForMotors();
    };
  };
  esc1 = 985;
  esc2 = 985;
  esc3 = 985;
  esc4 = 985;
};

void calibrate_esc(void){
	esc1=2000;
	esc2=2000;
	esc3=2000;
	esc4=2000;
	updatePWMTimerRegistersForMotors();
	delay_millis(8000);
	esc1=1000;
	esc2=1000;
	esc3=1000;
	esc3=1000;
	updatePWMTimerRegistersForMotors();
	delay_millis(8000);
};
