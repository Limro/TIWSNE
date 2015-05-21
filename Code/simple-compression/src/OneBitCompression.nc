#include "printf.h"

#define PACKETLENGTH 1024

module OneBitCompression
{
	uses interface Boot;
}
implementation
{		
	void compress(unsigned char* src, unsigned char* dest) {
		int i, j;
		
		for (i = 0; i < PACKETLENGTH/8; i++) {
		    int si = i*8, di = i*7;
		    
			for (j = 0; j < 7; j++)
			    dest[di+j] = (src[si+j] & 0xFE) | ((src[si+7] >> (7-j)) & 1);
		}
	}

	void decompress(unsigned char* src, unsigned char* dest) {
		int i, j;
		
		for (i = 0; i < PACKETLENGTH/8; i++) {
		    int si = i*7, di = i*8;

			dest[di+7] = 0;

			for (j = 0; j < 7; j++) {
				dest[di+j] = src[si+j] & 0xFE;
				dest[di+7] += (src[si+j] << (7-j)) & (128 >> j);
			}
		}
	}
		
	event void Boot.booted() {
		unsigned char compressed[PACKETLENGTH*7/8];
		unsigned char decompressed[PACKETLENGTH];
		uint8_t i;

		for(i = 0; i < PACKETLENGTH; i++)
			decompressed[i] = i;
		
		compress(decompressed, compressed);
		decompress(compressed, decompressed);

		for(i = 0; i < PACKETLENGTH; i++)
			printf("\n%i:%u", i, compressed[i]);	

		printfflush();
	}
}