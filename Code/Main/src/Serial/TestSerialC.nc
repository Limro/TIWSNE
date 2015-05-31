#include "Timer.h"
#include "TestSerial.h"
#include "Flash.h"
#include <UserButton.h>

module TestSerialC {
	uses {
		interface SplitControl as Control;
		interface Boot;
	
		interface Receive as ReceiveData;
		interface AMSend as SendData;
		interface Receive as ReceiveStatus; 
		interface AMSend as SendStatus;
		interface Packet;
		interface PacketAcknowledgements as PacketAck;
	
		interface Flash;
	}
}
implementation {

	message_t packet;
	message_t chunk_pkt;
	message_t status_pkt;

	uint16_t counter = 0;
	int maxChunks = 1024;
 
	int sendIndex = 0;
	char sendArray[64];
	
	bool isSerial = FALSE;
  
	event void Boot.booted() {
		call Control.start();
	}
	
	void sendStatusMessage( uint8_t status )
	{
		status_msg_t* statusMsg = (status_msg_t*) call Packet.getPayload(&status_pkt, sizeof(status_msg_t));
		statusMsg->status = status;
		statusMsg->chunkNum = sendIndex;
	
		call PacketAck.noAck(&status_pkt);
	
		if (call SendStatus.send(AM_BROADCAST_ADDR, &status_pkt, sizeof(status_msg_t) ) == SUCCESS)
		{
			// Do nothing
		}
	
	}
	
	void sendChunkMessage(uint8_t* source )
	{
		chunk_msg_t* chunkMsg = (chunk_msg_t*) call Packet.getPayload(&chunk_pkt, sizeof(chunk_msg_t));
		memcpy(chunkMsg->chunk, source, 64);
	
		chunkMsg->chunkNum = sendIndex;
		call PacketAck.noAck(&chunk_pkt);
	
		if (call SendData.send(AM_BROADCAST_ADDR, &chunk_pkt, sizeof(chunk_msg_t)) == SUCCESS)
		{
			// Do nothing	
		}
	}
	
	event message_t* ReceiveStatus.receive(message_t* bufPtr, 
			void* payload, uint8_t len) 
	{
		if (len != sizeof(status_msg_t)) 
		{
			return bufPtr;
		}
		else
		{
			status_msg_t* statusMsg = (status_msg_t*)payload;
	
			if(statusMsg->status == TRANSFER_TO_TELOS)
			{
				// getting Image from PC
				// Delete Flash content
				isSerial = TRUE;
				sendIndex = 0;
				call Flash.erase();
			}
			else if (statusMsg->status == TRANSFER_OK)
			{
				// transfer was okay
			}
			else if (statusMsg->status == TRANSFER_DONE)
			{
				// full transfer completed
				isSerial = FALSE;
			}
			else if(statusMsg->status == TRANSFER_FAIL)
			{
				// TODO
			}
			else if(statusMsg->status == TRANSFER_READY)
			{
	
			}
			else if(statusMsg->status == TRANSFER_FROM_TELOS)
			{
				//Start remote transfer (to PC)
				isSerial = TRUE;
				sendIndex = 0;
				call Flash.read(sendArray, sendIndex);	
			}
			else
			{
				// nothing
			}
			return bufPtr;
		}
	}

	event void SendStatus.sendDone(message_t* bufPtr, error_t error) {
			// TODO
	}

	event message_t* ReceiveData.receive(message_t* bufPtr, 
				void* payload, uint8_t len) {
		if (len != sizeof(chunk_msg_t)) 
		{
			return bufPtr;
		}
		else
		{
			chunk_msg_t* data = (chunk_msg_t*)payload;
	
			call Flash.write(data->chunk, data->chunkNum);
			sendIndex++;
				
			return bufPtr;
		}
	}

	event void SendData.sendDone(message_t* bufPtr, error_t error) {
			if (&chunk_pkt == bufPtr)
		{
			sendIndex++;
	
			if(sendIndex < maxChunks)
			{
				call Flash.read(sendArray, sendIndex);
			}
			else
			{
				isSerial = FALSE;
				sendStatusMessage(TRANSFER_DONE);
			}
		}
	
	}

	event void Control.startDone(error_t err) {}
		event void Control.stopDone(error_t err) {}
	
	// Flash Events

	event void Flash.writeDone(error_t result){
		if(isSerial)
		{
			sendStatusMessage(TRANSFER_OK);
			}
	}

	event void Flash.readDone(error_t result)
	{
		if(isSerial)
		{
		sendChunkMessage(sendArray);
		}
	}

	event void Flash.eraseDone(error_t result)
	{
		sendStatusMessage(TRANSFER_READY);
	}

}


