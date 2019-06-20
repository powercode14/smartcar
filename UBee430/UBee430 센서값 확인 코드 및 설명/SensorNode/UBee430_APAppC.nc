/*									
 * Copyright (c) 2007 Huins
 * All rights reserved.
 * 
 */


configuration UBee430_APAppC{
}
implementation 
{
  components UBee430_APC as App, LedsC, MainC;
  components new TimerMilliC() as Timer0;
  components new Sht11C();
  components new VoltageC();
  components new TemperatureC();
  components new LightToVoltageC();
  components Ds28dg02C;
  components new AdcZeroC();

	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;

	App.Temperature->Sht11C.Temperature;
	App.TemperatureMetadata->Sht11C.TemperatureMetadata;
	App.Humidity -> Sht11C.Humidity;
	App.HumidityMetadata->Sht11C.HumidityMetadata;
	App.InternalVolt->VoltageC.Read;
	App.InternalTemp->TemperatureC.Read;
	App.Photo->LightToVoltageC.Read;
	App.DS28->Ds28dg02C.Ds28dg02s;
	App.AdcZero->AdcZeroC;
	

	components ActiveMessageC as AM;
	//components SerialActiveMessageC as AM;
	App.SplitControl -> AM;
	App.Receive -> AM.Receive[AM_MSG];
	App.AMSend -> AM.AMSend[AM_MSG];
	App.Packet -> AM;
	
	App.Timer0 -> Timer0;
}


