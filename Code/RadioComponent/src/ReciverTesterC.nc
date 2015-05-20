#include "Timer.h"
#define Size 35
module ReciverTesterC @safe() {
  uses {
    interface Boot;
	interface RadioTransfereReciverI as Reciver;
	interface Leds;
  }
  
}
implementation {

  bool isSending = FALSE; 
  uint8_t data[] = {1, 2, 3, 4, 5,6, 7,8,9,10,11,12,13,14,15,16,17,18, 19 };
  uint8_t Rx[255];
  
  event void Boot.booted() {
     call Reciver.Start();
  }
  
  event uint8_t * Reciver.GetBuffer(uint16_t length){
  		
  		return Rx;
  }
  


   event void Reciver.PacketRecived(uint16_t length, error_t error){
		if(error == SUCCESS)
		{
			call Leds.led0Toggle();
		}
		else
		{
			call Leds.led1Toggle();
		}
   }
}




