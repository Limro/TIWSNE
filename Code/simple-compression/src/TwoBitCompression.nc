#include "printf.h"

#define BLOCK_LENGTH 1024

module TwoBitCompression
{
	uses interface Boot;
}
implementation
{		
	void compress(unsigned char* src, unsigned char* dest) {
		uint16_t i, j;
		
		for (i = 0; i < BLOCK_LENGTH/4; i++) {
		    uint16_t si = i*4, di = i*3;
		    
			for (j = 0; j < 3; j++)
			    dest[di+j] = (src[si+j] & 0xFC) | ((src[si+3] >> (6-j*2)) & 3);
		}
	}

	void decompress(unsigned char* src, unsigned char* dest) {
		uint16_t i, j;
		
		for (i = 0; i < BLOCK_LENGTH/4; i++) {
		    uint16_t si = i*3, di = i*4;

			dest[di+3] = 0;

			for (j = 0; j < 3; j++) {
				dest[di+j] = src[si+j] & 0xFC;
				dest[di+3] += (src[si+j] << (6-j*2)) & (192 >> j*2);
			}
		}
	}

	event void Boot.booted() {
		unsigned char compressed[BLOCK_LENGTH*3/4];
		unsigned char decompressed[BLOCK_LENGTH];
		uint16_t i;

		for(i = 0; i < BLOCK_LENGTH; i++)
			decompressed[i] = i;
		
		compress(decompressed, compressed);
		decompress(compressed, decompressed);

		for(i = 0; i < BLOCK_LENGTH; i++)
			printf("\n%i:%u", i, compressed[i]);	

		printfflush();
	}
}