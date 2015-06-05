#include <UserButton.h>
#include "printf.h"

module SenderAppC
{
	uses
	{
		interface FlashManager as Flash; 
		interface Notify<button_state_t> as Notify; 
		interface Boot;
		interface Leds;
		interface RadioTransfereSenderI as Radio; 
		interface Compression as comp;
	}
}
implementation
{
	uint8_t normal[1024]; 
	uint8_t compressed[2000];  
	
	uint16_t blockCount = 0; 
	bool isSending = FALSE; 
	
	void SetDummyData()
	{
		uint16_t i;
		for(i =0; i<sizeof(normal); i++)
		{
			normal[i] = (uint8_t)(i % 256);
		}
	}
	
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
	
	event void Flash.SetDone()
	{
		RequestSend();
	}

	event void Flash.GetDone(uint8_t *ptr, uint16_t length)
	{
		//uint16_t size = call comp.Compress(ptr, compressed);
		call Radio.Send(ptr, length);
	}

	event void Notify.notify(button_state_t state)
	{
		if ( state == BUTTON_PRESSED)
		{
			call Flash.GetData(normal, 0, 16);
		}
	}

	event void Boot.booted()
	{
		call Leds.led2Off();
		StartSend();
	}
	
	event void Radio.SendDone()
	{
		RequestSend();
	}

	event void Flash.eraseDone()
	{
		StartSend();
	}
}