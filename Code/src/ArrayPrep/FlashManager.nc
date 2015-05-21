interface FlashManager{
	command void GetData(uint8_t *ptr, uint16_t blockindex, uint8_t blocklength);
	
	command void SetData(uint8_t *ptr, uint16_t blockindex, uint8_t blocklength);
	
	event void GetDone(uint8_t *ptr, uint16_t length);
	event void SetDone();
}