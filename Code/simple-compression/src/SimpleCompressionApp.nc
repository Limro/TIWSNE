
configuration SimpleCompressionApp {}
implementation {
  components MainC, SimpleCompression;

  MainC.Boot <- SimpleCompression;
}