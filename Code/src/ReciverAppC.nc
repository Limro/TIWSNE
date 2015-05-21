#include <UserButton.h>
#include "printf.h"

module ReciverAppC{
	uses
	{
		interface FlashManager as Flash; 
		interface Boot;
		interface Leds;
		
		interface RadioTransfereReciverI as Radio; 
	}
	
}
implementation{
	
	uint8_t normal[1024]; 
	uint8_t compressed[1024];  
	
	uint8_t blockCount = 0; 
	
	
	
	event void Flash.SetDone(){
		// TODO Auto-generated method stub
	}

	event void Flash.GetDone(uint8_t *ptr, uint16_t length){
		
	}

	event void Boot.booted(){
		call Radio.Start(); 
	}
	

	event uint8_t * Radio.GetBuffer(uint16_t length){
		return normal; 
	}

	event void Radio.PacketRecived(uint16_t length, error_t error){
		if(error == SUCCESS)
		{
			call Leds.led1Toggle();
			blockCount = (++blockCount)%65;
			call Flash.SetData(normal,(blockCount-1)*16 , 16);
			
			if(blockCount== 0)
				call Leds.led2Toggle();
		}
		else
		{
			call Leds.led0Toggle();
		}
	}
}