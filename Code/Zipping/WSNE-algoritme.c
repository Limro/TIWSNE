#include "printf.h"
module NullC @safe()
{
  uses interface Boot;
}
implementation
{
	uint8_t* SliceBits(uint8_t* data, uint8_t dataSize, uint8_t compressBits) 
	{
		uint8_t i;
		uint8_t* output = malloc(dataSize - compressBits); 
//		char output[64 - (compressBits*8)] = {'x'};
		uint8_t tmp[8];

		//Remove unwanted bits
		//xxxx xxxx -> xxxx xxx0
		for(i = 0; i < 8; i++) 
		{
			tmp[i] = ((data[i] >> compressBits) << compressBits);
		}
		
		//Insert missing bits
		for(i = 0; i < 7 - compressBits; i++) 
		{
			output[i] = (tmp[i] << i*compressBits) + (tmp[i+1] >> (8-compressBits));
		}
		
		output[6] = (tmp[6] << 6) | (tmp[7] >> 1);
		
		return output;
	}
	
	//Input size 7
	uint8_t* AddBits(uint8_t* input)
	{
		uint8_t i;
		uint8_t* output = malloc(8);
		uint8_t tmp;
		
		//xxxx xxxy -> xxxx xxx0
		output[0] = (input[0] >> 1) << 1;
		
		for(i = 1; i < 7; i++)
		{
			tmp = input[i-1] << (8-i);
//			printf("tmp[%i]: %u\n", i, tmp);
			output[i] = tmp | ((input[i] >> (i+1)) << 1);
		}
		output[7] = input[6] << 1;
		
		return output;
	}
	
	// Do not set compressBits to zero!
	void *zip(uint8_t* data, uint16_t dataSize, uint8_t compressBits) 
	{
		uint8_t *workedData = malloc((dataSize / 8)*(8-compressBits));
		uint8_t index;
		uint8_t* output;

		for(index = 0; index < dataSize; index = index + 8) 
		{
			uint8_t i;
			uint8_t tmpData[8];

			//Extract 8 bytes of data
			for(i = 0; i < 8; i++) 
			{
				tmpData[i] = data[index + i];
			}

			//Take x bytes, push them to function
			output = SliceBits(tmpData, sizeof(tmpData), compressBits);

			//Store the workedData in the compressedData variable
			for(i = 0; i < (8 - compressBits); i++) 
			{
				workedData[i + index] = output[i];
			}
			
			free(output);
		}
		return workedData;
	}
		
	event void Boot.booted() {
		uint8_t data[8] = {2,2,2,2,2,2,2,2}; 
		uint8_t data2[8] = {2,4,8,16,32,64,129};
		uint8_t *output;
		uint8_t i;
		output = SliceBits(data, 8, 1);
		
		printf("\n");
		for(i = 0; i < 7; i++ )
		{
			printf("1 bit: %u\n", output[i]);
		}
		printfflush();
		
		output = AddBits(output);
		for(i=0; i < 8; i++)
		{
			printf("Added bit: %u\n", output[i]);
		}
		printfflush();
		free(output);
		
//		output = SliceBits(data, 8, 2);
//		
//		for(i=0; i < 6 ; i++ )
//		{
//			printf("\n2 bit: %u", output[i]);
//		}
//		printfflush();
//		free(output);
		
//		output = AddBits(data2, 8, 1);
//		for(i=0; i < 8; i++)
//		{
//			printf("Added bit: %u\n", output[i]);
//		}
//		
//		free(output);
	}

	//1 xxxx xxxx
	//2 yyyy yyyy
	//3 zzzz zzzz
	//4 xxxx xxxx
	//5 yyyy yyyy
	//6 zzzz zzzz
	//7 xxxx xxxx
	//8 yyyy yyyy 

	// Becomes 

	//1 xxxx xxxy
	//2 yyyy yyzz
	//3 zzzz zxxx
	//4 xxxx yyyy
	//5 yyyz zzzz
	//6 zzxx xxxx
	//7 xyyy yyyy
	
//		0000 0010
//		0000 0010
//		0000 0010
//		0000 0010
//		0000 0010
//		0000 0010
//		0000 0010
//		0000 0010
	
	//1 0000 0010 2
	//2 0000 0100 4
	//3 0000 1000 8
	//4 0001 0000 16
	//5 0010 0100 32
	//6 0100 0000 64
	//7 1000 0001 129
	
}