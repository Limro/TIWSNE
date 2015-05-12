uint16_t zip(uint16_t[] data, uint8_t compressBits)
{
	uint16_t[] compressedData = {};
	uint8_t index;

	for(index = 0 ; index < sizeof(data) ; index = index+8)
	{
		uint8_t i;
		uint8_t[8] tmpData = {};

		//Extract 8 bytes of data
		for(i = 0 ; i < 8 ; i++)
		{
			tmpData[i] = data[index+i];
		}

		//Take x bytes, push them to function
		uint8_t[7] workedData = SliceBits(tmpData, compressBits);

		//Store the workedData in the compressedData variable
		for(i = 0; i<7 ; i++)
		{
			compressedData[index] = workedData[i];
		}
	}
	return compressedData;
}

uint8_t[] SliceBits(uint8_t[] data, uint8_t compressBits)
{
	uint8_t i;
	uint8_t[sizeof(data)-1] workedData = {};
	uint8_t tmp;
	
	workedData[0] = (( data[0] >> 1 ) << 1); // xxxx xxx0
	workedData[0] |= (data[1] >> 7);		 // xxxx xxxy
	
	for(i = 1; i < sizeof(data)-1; i++)
	{
		workedData[i] = (( data[i] >> i+1 ) << i+2);
		workedData[i] |= (data[i+1] >> 7-i);
	}
	return workedData;
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