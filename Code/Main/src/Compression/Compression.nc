interface Compression{
	command uint16_t Compress(uint8_t* in, uint8_t* out, uint16_t insize);
	
	command void Decompress(uint8_t *in, uint8_t *out, uint16_t insize,  uint16_t outsize);
}