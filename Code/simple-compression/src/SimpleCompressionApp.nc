#include "printf.h"

configuration SimpleCompressionApp {}
implementation {
  components MainC, OneBitCompression, FourBitCompression;

  MainC.Boot <- OneBitCompression;
//	MainC.Boot <- FourBitCompression;
}