#include "printf.h"
#define LENGTH2 16

module OneBitCompression
{
  uses interface Boot;
}
implementation
{		
	void compress(unsigned char* src, unsigned char* dest) {
		uint16_t i, j;
		
		for (i = 0; i < LENGTH2; i += 8) {
	
			for (j = i; j < i+7; j++)
				dest[j-i/8] = src[j] | ((src[i+7] >> (7-i-j)) & 1);
		}
	}
	
	void decompress(unsigned char* src, unsigned char* dest) {
		uint16_t i, j;
		
		for (i = 0; i < LENGTH2; i += 8) {
			dest[i+7] = 0;
	
			for (j = i; j < i+7; j++) {
				dest[j] = src[j] & 0xFE;
				dest[i+7] += (src[j] << (7-i-j)) & (128 >> (7-i-j));
			}
		}
	}
		
	event void Boot.booted() {
		unsigned char compressed[14];
		unsigned char decompressed[LENGTH2];
		uint8_t i;

		for(i = 0 ; i < LENGTH2; i++)
			decompressed[i] = 2;
		
		compress(decompressed, compressed);
		
		for(i = 0; i < LENGTH2; i++)
		{
			printf("\n%i:%u", i, compressed[i]);	
		}
		printfflush();
		
		decompress(compressed, decompressed);

//		for(i = 6; i < LENGTH2; i++)
//		{
//			printf("\n%i:%u", i, decompressed[i]);	
//		}
//		printf("\n");
//		printfflush();
	}
}