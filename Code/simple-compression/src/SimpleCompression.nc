#include <stdio.h>
#include <stdlib.h>

#define LENGTH 65536

module SimpleCompression
{
	uses interface Boot;
}
implementation
{
	void compress(unsigned char* src, unsigned char* dest) {
    int i;
    
    for(i = 0; i < LENGTH/2; i++) {
	        dest[i] = src[2*i] & 0xF0;
	        dest[i] += src[2*i+1] >> 4;
	    }
	} 
	
	void decompress(unsigned char* src, unsigned char* dest) {
	    int i;
	    
	    for(i = 0; i < LENGTH/2; i++) {
	        dest[2*i] = src[i] & 0xF0;
	        dest[2*i+1] = src[i] << 4;
	    }
	}
	
	event void Boot.booted() {
	    
	}
}

