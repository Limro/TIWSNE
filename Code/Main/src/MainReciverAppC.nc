#include "TestSerial.h"
#include "Timer.h"
#include <UserButton.h>
#include "Flash.h"
#include "StorageVolumes.h"

configuration MainReciverAppC {}
implementation {
	components TestSerialC as Serial, MainC;
	components SerialActiveMessageC as AM;
	components FlashC;
	components FlashC as FlashC2;
	components new BlockStorageC(BLOCK_VOLUME);
	components ReciverAppC;
	components FlashManagerC;
	components UserButtonC;
	components LedsC;
	
	components RadioReciverC as Reciver;
  	components new AMReceiverC(6);
  	components ActiveMessageC;
  	
  	components FourBitCompression as comp; 
  	
  	
  	Reciver.AMControl -> ActiveMessageC;
  	Reciver.Packet ->AMReceiverC;
  	Reciver.Receive ->AMReceiverC;

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
	
	FlashManagerC.Flash ->FlashC2;
	
	ReciverAppC.comp -> comp;
	ReciverAppC.Flash ->FlashManagerC;
	ReciverAppC.Leds -> LedsC;
	ReciverAppC.Boot -> MainC;
	ReciverAppC.Radio -> Reciver;
}


