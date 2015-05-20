#include "Timer.h"
#include "Huffman.h"

#define Size 35
module SenderTesterC @safe() {
  uses {
    interface Boot;
    interface Timer<TMilli> as MilliTimer;
	interface RadioTransfereSenderI as Sender;
	interface Leds;
  }
  
}
implementation {

  bool isSending = FALSE; 
  uint8_t in[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  uint8_t out[250];
  
  event void Boot.booted() {
  	 bool hasThing = TRUE; 
  	 uint8_t si = 0;
  	 call Leds.led2On();
  	 
  	 si = Huffman_Compress(in , out, sizeof(in));
  	 Huffman_Uncompress(out, in, si, sizeof(in));
  	 
  	 for(si = 0; si<sizeof(in); si++)
  	 {
  	 	if(in[si] != 1)
  	 	{
  	 		call Leds.led0On();
  	 		hasThing = FALSE;
  	 	}
  	 }
  	 
  	 if(hasThing == TRUE)
  	 	call Leds.led1On();
  	 	
     call MilliTimer.startPeriodic(5000);
     
  }
  
  event void MilliTimer.fired() {
    	if(isSending == FALSE)
    	{
    		isSending = TRUE;
    		
    		call Sender.Send(in, sizeof(in));
    	}
  }

  


	event void Sender.SendDone(){
			
    		isSending = FALSE;
	}
}




