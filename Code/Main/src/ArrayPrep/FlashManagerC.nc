#include "Flash.h"

module FlashManagerC{
	provides interface FlashManager;
	uses interface Flash;
}
implementation{
	
	uint8_t *ptr_;
	uint16_t blockindex_;
	uint8_t blocklength_; 
	uint8_t blockCount_;
	
	bool isWorking = FALSE; 
	
	void RequestRead()
	{
		uint8_t * pointer = ptr_ + blockCount_*BLOCKPART_VOLUME;
		call Flash.read(pointer, blockindex_+blockCount_);			
	}
	
	void RequestWrite()
	{
		uint8_t*  pointer = ptr_ + (blockCount_*BLOCKPART_VOLUME);
		call Flash.write(pointer, blockindex_+blockCount_);			
	}
	
	
	event void Flash.eraseDone(error_t result){
		if(isWorking == FALSE) return;
		isWorking = FALSE;
		signal FlashManager.eraseDone();
	}

	event void Flash.writeDone(error_t result){
		if(isWorking == FALSE) return;
		blockCount_++;
		
		if(blockCount_ == blocklength_)
		{
			isWorking = FALSE;
			signal FlashManager.SetDone();
		}
		else
			RequestWrite();
	}

	event void Flash.readDone(error_t result){
		if(isWorking == FALSE) return;
		blockCount_++;
		
		if(blockCount_ == blocklength_)
		{
			isWorking = FALSE; 
			signal FlashManager.GetDone(ptr_, blocklength_*BLOCKPART_VOLUME);	
		}
		else
			RequestRead();
	}

	command void FlashManager.SetData(uint8_t *ptr, uint16_t blockindex, uint8_t blocklength){
		if(isWorking == FALSE)
		{
			ptr_ = ptr; 
			blockindex_ = blockindex; 
			blocklength_ = blocklength; 
			blockCount_ = 0; 
			isWorking = TRUE;
			RequestWrite();
		}
	}

	command void FlashManager.GetData(uint8_t *ptr, uint16_t blockindex, uint8_t blocklength){
		if(isWorking == FALSE)
		{
			ptr_ = ptr; 
			blockindex_ = blockindex; 
			blocklength_ = blocklength; 
			blockCount_ = 0; 
			isWorking = TRUE;
			RequestRead();
		}
	}
	
	command void FlashManager.erase()
	{
		isWorking = TRUE;
		call Flash.erase();
	}
	
}