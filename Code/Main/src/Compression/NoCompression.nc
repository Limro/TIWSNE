#include "printf.h"

#define BLOCK_LENGTH 1024

module NoCompression
{
	provides interface Compression;
}
implementation
{		
	command uint16_t Compression.Compress(uint8_t* src, uint8_t* dest) {
		memcpy(dest,src,BLOCK_LENGTH);
		return BLOCK_LENGTH;
	}

	command void Compression.Decompress(uint8_t* src, uint8_t* dest) {
		memcpy(dest,src,BLOCK_LENGTH);
	}
}