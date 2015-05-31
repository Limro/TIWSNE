#include "TestSerial.h"
#include "Timer.h"
#include <UserButton.h>
#include "Flash.h"
#include "StorageVolumes.h"

configuration MainAppC {}
implementation {
	components TestSerialC as Serial, MainC;
	components SerialActiveMessageC as AM;
	components FlashC;
	components FlashC as Flash2;
	components new BlockStorageC(BLOCK_VOLUME);
	components SenderAppC;
	components FlashManagerC;
	components UserButtonC;
	components LedsC;
	
	components RadioSenderC as Sender;
  	components new AMSenderC(6);
  	components ActiveMessageC;
  	
  	components FourBitCompression as comp; 
  	
  	
  	Sender.AMSend -> AMSenderC;
  	Sender.AMControl -> ActiveMessageC;
  	Sender.Packet -> AMSenderC;

	Serial.Boot -> MainC.Boot;
	Serial.Control -> AM;
	Serial.ReceiveData -> AM.Receive[AM_CHUNK_MSG_T];
	Serial.SendData -> AM.AMSend[AM_CHUNK_MSG_T];
	Serial.ReceiveStatus -> AM.Receive[AM_STATUS_MSG_T];
	Serial.SendStatus -> AM.AMSend[AM_STATUS_MSG_T];
	Serial.PacketAck -> AM.PacketAcknowledgements;
	Serial.Packet -> AM;
	Serial.Flash -> FlashC;
	
	FlashC.BlockRead -> BlockStorageC.BlockRead;
	FlashC.BlockWrite -> BlockStorageC.BlockWrite;
	
	FlashManagerC.Flash ->Flash2;
	
	SenderAppC.comp -> comp;
	SenderAppC.Flash ->FlashManagerC;
	SenderAppC.Notify -> UserButtonC;
	SenderAppC.Leds -> LedsC;
	SenderAppC.Boot -> MainC;
	SenderAppC.Radio -> Sender;
}


