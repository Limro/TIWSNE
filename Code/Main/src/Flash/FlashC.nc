#include "Flash.h"

module FlashC
{
	uses
	{
		interface BlockRead;
		interface BlockWrite;
	}	
	provides
	{
		interface Flash;
	}
}
implementation
{		
	command error_t Flash.erase()
	{
		return call BlockWrite.erase();
	}
	
	command error_t Flash.write(uint8_t* uint8Array, uint32_t chunkNum)
	{
		error_t Status = call BlockWrite.write(chunkNum*BLOCKPART_VOLUME,uint8Array,BLOCKPART_VOLUME);
		return Status;
	}
	
	command error_t Flash.read(uint8_t* uint8Array, uint32_t chunkNum)
	{
		error_t Status = call BlockRead.read(chunkNum*BLOCKPART_VOLUME,uint8Array,BLOCKPART_VOLUME);
		return Status;
	}
	
	event void BlockWrite.eraseDone(error_t result) 
	{
		signal Flash.eraseDone(result);
	}
	
	event void BlockWrite.writeDone(storage_addr_t x, void* buf, storage_len_t y, error_t result) 
	{
		call BlockWrite.sync();
	}

	event void BlockWrite.syncDone(error_t result) 
	{
		signal Flash.writeDone(result);
	}

	event void BlockRead.readDone(storage_addr_t x, void* buf, storage_len_t rlen, error_t result) __attribute__((noinline))
	{
		signal Flash.readDone(result);
	}

	event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result) 
	{
	}
}
