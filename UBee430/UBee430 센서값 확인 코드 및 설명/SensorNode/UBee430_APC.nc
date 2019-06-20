/*									
 * Copyright (c) 2007 Huins
 * All rights reserved.
 * 
 */


#include <stdio.h>
#include "UBee430_AP_MSG.h"

#define SENSORBOARD TOS_NODE_ID

module UBee430_APC{
	uses{
		interface Leds;
		interface Boot;

		interface Read<uint16_t> as Temperature;
		interface Read<uint16_t> as Humidity;
		interface DeviceMetadata as TemperatureMetadata;
		interface DeviceMetadata as HumidityMetadata;

		interface Read<uint16_t> as InternalVolt;
		interface Read<uint16_t> as InternalTemp;

		interface Read<uint16_t> as Photo;

		interface SplitControl;
		interface Receive;
		interface AMSend;
		interface Packet;
		interface Ds28dg02s as DS28;

		interface Read<uint16_t> as AdcZero;
		interface Timer<TMilli> as Timer0;
	}
}

implementation{
	message_t packet;
	bool locked = FALSE;
	uint8_t count=0, old_sec=0;
	msg_t message;
	uint8_t rt_data ;
	task void Send();
	message_t tpacket;
	return_msg rtm;
	task void rt_Send();
	bool boot_tx_flag  = 0;

	void getRTC(){
		message.RTC[5] = call DS28.ReadRTC_seconds();
		message.RTC[4] = call DS28.ReadRTC_minutes();
		message.RTC[3] = call DS28.ReadRTC_hours();
		message.RTC[2] = call DS28.ReadRTC_date();
		message.RTC[1] = call DS28.ReadRTC_months();
		message.RTC[0] = call DS28.ReadRTC_years();
	}

	void getID(){
		message.ID[0] = call DS28.ReadID_number0();
		message.ID[1] = call DS28.ReadID_number1();
		message.ID[2] = call DS28.ReadID_number2();
		message.ID[3] = call DS28.ReadID_number3();
		message.ID[4] = call DS28.ReadID_number4();
		message.ID[5] = call DS28.ReadID_number5();
	}

	task void next(){
		count = 0;
		call InternalVolt.read();
		call InternalTemp.read();
		call Temperature.read();
		call Humidity.read();
		call Photo.read();
		call AdcZero.read();
		post Send();
	}

	task void Send(){
		if(count < 6)
			post Send();
		else{
			//msg_t *rcm = ( msg_t * ) call Packet.getPayload( &packet, NULL );
			message.start = 0x02;
			message.etx = 0x03;
			message.msg_type='S';
			message.node_id = TOS_NODE_ID;
			message.device_type = SENSORBOARD;
			//message.rssi = 0;
			//message.lqi = 0;
			getRTC();
			getID();
			memcpy(call AMSend.getPayload(&packet), &message, sizeof message);
			if(call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof message) == SUCCESS){}
			locked =TRUE;
		}
	}

	event void Boot.booted(){
		call SplitControl.start();
		call Timer0.startPeriodic(100);
		TOSH_MAKE_GIO57_OUTPUT();
		rtm.start = 0x02;
		//rtm.rssi = 0;
		//rtm.lqi = 0;
		rtm.msg_type = 'R';
		rtm.device_type = SENSORBOARD;
		rtm.node_id = TOS_NODE_ID;
		rtm.d0 = '0';
		rtm.d1 = '0';
		rtm.d2 = '0';
		rtm.d3 = '0';
		rtm.etx = 0x03;
	}

	void rtc_init(){
		call DS28.init();
		call DS28.WriteRTC_seconds(0x00);
		call DS28.WriteRTC_minutes(0x00);
		call DS28.WriteRTC_hours(0x23);
		call DS28.WriteRTC_dayofweek(0x02);
		call DS28.WriteRTC_date(0x1c);
		call DS28.WriteRTC_months(0x0C);
		call DS28.WriteRTC_years(0x0A);
	}

	event void Photo.readDone(error_t result, uint16_t data){
		if(result == SUCCESS)
			message.photo = (uint16_t) data;
		else
			message.photo = 0xFFFF;
		count++;
	}

	event void InternalTemp.readDone(error_t result, uint16_t data)
	{
		if(result == SUCCESS)
			message.internaltemp = (uint8_t) ((data/4096.0*1.5 - 0.986 ) / 0.00355 );
		else
			message.internaltemp = 0xFF;
		count++;
	}

	event void InternalVolt.readDone(error_t result, uint16_t data){
		if(result == SUCCESS)
			message.internalvolt = (uint8_t) ( (data / 4096.0) * 3.0 *10.0 );
		else
			message.internalvolt = 0xFF;
		count++;
	}

	event void Humidity.readDone(error_t result, uint16_t data){
		uint8_t sig;
		uint16_t t, sorh, i;
		if(result == SUCCESS){
			sig = (uint8_t)call HumidityMetadata.getSignificantBits();
			t = (1<<sig) - 1;
			sorh = (u_int16_t)(t&data);
			message.humid = (uint8_t) (-4 + 0.0405 * sorh - 2.8 * 0.000001 * sorh * sorh);
		}
		else
			message.humid = 0xFF;
		count++;
	}

	event void Temperature.readDone(error_t result, uint16_t data){
		uint8_t sig;
		uint16_t t;
		if(result == SUCCESS){
			sig = (uint8_t)call TemperatureMetadata.getSignificantBits();
			t = (1<<sig) - 1;
			message.temp = (uint8_t) (-39.55 + 0.01 * (t&data));
		}
		else
			message.temp = 0xFF;
		count++;
	}

	event void AdcZero.readDone(error_t result, uint16_t data){
		//message.adc0 = (uint16_t)data;
		if(data < 300)
			data == 0;
		else
			message.adc0 = ((((float)data-300)/1365)*10);
		count++;
	}

	event void DS28.EEPROM_WriteDone(){}

	task void rt_Send(){
		memcpy(call AMSend.getPayload(&packet), &rtm, sizeof rtm);
		if(call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof rtm) == SUCCESS){}
	}

	event message_t* Receive.receive( message_t *buf, void *payload, uint8_t len){
		return buf;
	}

	event void AMSend.sendDone(message_t *buf, error_t error){
		call Leds.led1Toggle();
		//boot_tx_flag = 0;
	}

	event void SplitControl.startDone(error_t err){
		if(err == SUCCESS){
			rtc_init();
			post next();
		}
		else call SplitControl.start();
	}

	event void SplitControl.stopDone( error_t err ){}
	
	event void Timer0.fired(){
		call Leds.led0Toggle();
		post next();
		if(!boot_tx_flag){
			boot_tx_flag = 1;
			post rt_Send();
		}
	}
}/*implementation*/
