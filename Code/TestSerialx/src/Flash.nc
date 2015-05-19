interface Flash
{
	command error_t erase();
	event void eraseDone(error_t result);
		
	command error_t write(uint8_t* uint8Array, uint32_t chunkNum);
	event void writeDone(error_t result);

	command error_t read(uint8_t *uint8Array, uint32_t chunkNum);
	event void readDone(error_t result);
}