#include <stdio.h>
#include <stdlib.h>
#include "printf.h"

#define LENGTH 1024

module FourBitCompression
{
	uses interface Boot;
}
implementation
{
	void compress(unsigned char* src, unsigned char* dest) {
    uint16_t i;
    
    for(i = 0; i < LENGTH/2; i++) {
	        dest[i] = src[2*i] & 0xF0;
	        dest[i] += src[2*i+1] >> 4;
	    }
	} 
	
	void decompress(unsigned char* src, unsigned char* dest) {
	    uint16_t i;
	    
	    for(i = 0; i < LENGTH/2; i++) {
	        dest[2*i] = src[i] & 0xF0;
	        dest[2*i+1] = src[i] << 4;
	    }
	}
	
	event void Boot.booted() { 
		unsigned char compressed[LENGTH/2];
		unsigned char decompressed[LENGTH];
		uint16_t i;
		
		for(i = 0 ; i < LENGTH; i++)
			decompressed[i] = i%255;
		
		compress(decompressed, compressed);
		
		for(i = 0; i < 0; i++)
			printf("\n1 bit: %u", compressed[i]);

		printfflush();
		
		decompress(compressed, decompressed);

		for(i = 0; i < LENGTH; i+= 16)
		{
			printf("\n%i:%u", i, decompressed[i]);	
		}
		
		printf("\nV3");
		printfflush();
	}
}

