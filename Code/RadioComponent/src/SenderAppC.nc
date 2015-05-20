#include "RadioInfo.h"


configuration SenderAppC {}
implementation {
  components MainC as App; 
  components RadioSenderC as Sender;
  components SenderTesterC as MApp;
  components new AMSenderC(6);
  components new TimerMilliC();
  components ActiveMessageC;
  components LedsC;
  
  MApp.Boot -> App.Boot;
  MApp.Sender -> Sender;
  MApp.MilliTimer -> TimerMilliC;
  
  Sender.AMSend -> AMSenderC;
  Sender.AMControl -> ActiveMessageC;
  Sender.Packet -> AMSenderC;
  MApp.Leds -> LedsC;
}


