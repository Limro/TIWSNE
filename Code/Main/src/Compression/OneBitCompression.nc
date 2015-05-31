#include "printf.h"

#define BLOCK_LENGTH 1024

module OneBitCompression
{
	provides interface Compression;
}
implementation
{		
	command uint16_t Compression.Compress(uint8_t* src, uint8_t* dest) 
	{
		uint16_t i, j;
		
		for (i = 0; i < BLOCK_LENGTH/8; i++) 
		{
			uint16_t si = i*8, di = i*7;
				
			for (j = 0; j < 7; j++)
				dest[di+j] = (src[si+j] & 0xFE) | ((src[si+7] >> (7-j)) & 1);
		}
		
		return (BLOCK_LENGTH/8)*7;
	}

	command void Compression.Decompress(uint8_t* src, uint8_t* dest) 
	{
		uint16_t i, j;
		
		for (i = 0; i < BLOCK_LENGTH/8; i++) 
		{
			uint16_t si = i*7, di = i*8;

			dest[di+7] = 0;

			for (j = 0; j < 7; j++) 
			{
				dest[di+j] = src[si+j] & 0xFE;
				dest[di+7] += (src[si+j] << (7-j)) & (128 >> j);
			}
		}
	}
}