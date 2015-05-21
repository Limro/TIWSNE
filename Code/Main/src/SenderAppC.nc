#include <UserButton.h>
#include "printf.h"

module SenderAppC{
	uses
	{
		interface FlashManager as Flash; 
		interface Notify<button_state_t> as Notify; 
		interface Boot;
		interface Leds;
		
		interface RadioTransfereSenderI as Radio; 
	}
	
}
implementation{
	
	uint8_t normal[1024]; 
	uint8_t compressed[2000];  
	
	uint8_t blockCount = 0; 
	bool isSending = FALSE; 
	
	
	void RequestSend()
	{
		if(blockCount<=64)
		{
			blockCount++; 
			call Flash.GetData(normal, (blockCount-1)*16, 16); 
		}
		else
		{
			isSending = FALSE; 
			call Leds.led2On();
		}
	}
	
	void StartSend()
	{
		if(isSending == FALSE)
		{
			isSending = TRUE; 
			blockCount = 0; 
			RequestSend();
		}
	}
	
	event void Flash.SetDone(){
		// TODO Auto-generated method stub
	}

	event void Flash.GetDone(uint8_t *ptr, uint16_t length){
		call Radio.Send(normal, length);
	}

	event void Notify.notify(button_state_t state){
		if ( state == BUTTON_PRESSED)
		{
			call Flash.GetData(normal, 0, 16);
		}
	}

	event void Boot.booted(){
		call Leds.led2Off();
		StartSend();
	}
	

	event void Radio.SendDone(){
		call Leds.led1Toggle();
		RequestSend();
	}
}