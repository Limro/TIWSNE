#include <UserButton.h>
#include "printf.h"

module ReciverAppC
{
	uses
	{
		interface FlashManager as Flash; 
		interface Boot;
		interface Leds;
		
		interface RadioTransfereReciverI as Radio; 
		interface Compression as comp; 
	}
}
implementation
{
	uint8_t normal[1024]; 
	uint8_t compressed[1024];  
	uint16_t blockCount = 0; 
	
	event void Flash.SetDone()
	{
	}

	event void Flash.GetDone(uint8_t *ptr, uint16_t length)
	{	
	}

	event void Boot.booted()
	{
		call Flash.erase();
	}

	event uint8_t * Radio.GetBuffer(uint16_t length)
	{
		return normal; 
	}

	event void Radio.PacketRecived(uint16_t length, error_t error)
	{
		if(error == SUCCESS)
		{
			int i; 	
			
			blockCount = (++blockCount)%65;
			
			//call comp.Decompress(compressed, normal);
			
			call Flash.SetData(normal,(blockCount-1)*16 , 16);
			
			if(blockCount== 0)
				call Leds.led2Toggle();
		}
		else
		{
			call Leds.led0Toggle();
		}
	}

	event void Flash.eraseDone()
	{
		call Radio.Start(); 
	}
}