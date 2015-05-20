#include "RadioInfo.h"


configuration ReciverAppC {}
implementation {
  components MainC as App; 
  components RadioReciverC as Reciver;
  components ReciverTesterC as MApp;
  components ActiveMessageC;
  components new AMReceiverC(6);
  components LedsC;
  
  MApp.Boot -> App.Boot;
  MApp.Reciver -> Reciver;
  MApp.Leds -> LedsC;
  
  Reciver.AMControl ->ActiveMessageC;
  Reciver.Packet ->AMReceiverC;
  Reciver.Receive ->AMReceiverC;
}


