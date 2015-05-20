#include "printf.h"

configuration SimpleCompressionApp {}
implementation {
  components MainC, OneBitCompression;

  MainC.Boot <- OneBitCompression;
}