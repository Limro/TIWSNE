#include "printf.h"

module OneBitCompression
{
  uses interface Boot;
}
implementation
{
	void SliceBits(unsigned char* data, unsigned char* output) 
	{
		uint8_t i;
		unsigned char tmp[8];

		//Remove unwanted bits
		//xxxx xxxx -> xxxx xxx0
		for(i = 0; i < 8; i++)
		{
			tmp[i] = data[i] & 0xFE;
		}
		
		//Insert missing bits
		for(i = 0; i < 6; i++) 
		{
			output[i] = (tmp[i] << i) + (tmp[i+1] >> 7);
		}

		output[6] = (tmp[6] << 6) | (tmp[7] >> 1);
	}
	
	void AddBits(unsigned char* data, unsigned char* output)
	{
		uint8_t i;
		uint8_t tmp;
		
		//xxxx xxxy -> xxxx xxx0
		output[0] = (data[0] >> 1) << 1;
		
		for(i = 1; i < 7; i++)
		{
			tmp = data[i-1] << (8-i);
			output[i] = tmp | ((data[i] >> (i+1)) << 1);
		}

		output[7] = data[6] << 1;
	}
	
	void compress(unsigned char* src, unsigned char* dest) 
	{
		uint16_t destLength = 896;
		uint8_t inputLength = 8;
		uint8_t outputLength = 7;

		uint16_t index;
		for(index = 0; index < 1024; index += inputLength) 
		{
			uint8_t i;
			uint8_t input[inputLength];
			uint8_t output[outputLength];

			for(i = 0; i < inputLength; i++) 
				input[i] = src[index + i];

			SliceBits(input, output);

			for(i = 0; i < outputLength; i++) 
				dest[index - index/inputLength + i] = output[i];
		}
	}

	void decompress(unsigned char* src, unsigned char* dest) 
	{
		uint16_t srcLength = 896;
		uint8_t inputLength = 7;
		uint8_t outputLength = 8;

		uint16_t index;
		for(index = 0; index < srcLength; index += inputLength) 
		{
			uint8_t i;
			unsigned char input[inputLength];
			unsigned char output[outputLength];

			for(i = 0; i < inputLength; i++) 
				input[i] = src[index + i];

			AddBits(input, output);

			for(i = 0; i < outputLength; i++) 
				dest[index + index/inputLength + i] = output[i];
		}
	}
		
	event void Boot.booted() {
		unsigned char data[8] = {2,2,2,2,2,2,2,2}; 
		unsigned char data2[8] = {2,4,8,16,32,64,129};
		unsigned char output[7];
		unsigned char output2[8];
		uint8_t i;
		
		SliceBits(data, output);
		
		for(i = 0; i < 7; i++)
			printf("\n1 bit: %u", output[i]);

		printfflush();
		
		AddBits(output, output2);

		for(i = 0; i < 8; i++)
			printf("\nAdded bit: %u", output2[i]);

		printfflush();
	}
}