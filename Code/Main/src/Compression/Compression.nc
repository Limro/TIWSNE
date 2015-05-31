interface Compression{
	command uint16_t Compress(uint8_t* in, uint8_t* out);
	
	command void Decompress(uint8_t *in, uint8_t *out);
}