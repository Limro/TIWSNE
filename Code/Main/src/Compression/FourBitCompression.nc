#include <stdio.h>
#include <stdlib.h>

#define LENGTH 1024

module FourBitCompression
{
	provides interface Compression;
}
implementation
{
	command uint16_t Compression.Compress(uint8_t* src, uint8_t* dest) {
    uint16_t i;
    
    for(i = 0; i < LENGTH/2; i++) {
	        dest[i] = src[2*i] & 0xF0;
	        dest[i] += src[2*i+1] >> 4;
	    }
	    
	  return LENGTH/2;
	} 
	
	command void Compression.Decompress(uint8_t* src, uint8_t* dest) {
	    uint16_t i;
	    
	    for(i = 0; i < LENGTH/2; i++) {
	        dest[2*i] = src[i] & 0xF0;
	        dest[2*i+1] = src[i] << 4;
	    }
	}
	
}

