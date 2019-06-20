#ifndef UBee430_AP_MSG_H
#define UBee430_AP_MSG_H

typedef nx_struct msg{

	nx_uint8_t start;
	//nx_int8_t rssi;
	//nx_uint8_t lqi;
	nx_uint8_t msg_type; /* 메시지 타입 -  data송신 S*/
	nx_uint8_t device_type;
	nx_uint16_t node_id;
	nx_uint8_t ID[6];
	nx_uint8_t RTC[6];
	nx_uint8_t internalvolt;
	nx_uint8_t internaltemp;
	nx_uint8_t temp;
	nx_uint8_t humid;
	nx_uint16_t photo;
	nx_uint16_t adc0;
	nx_uint8_t etx;
}msg_t;


typedef nx_struct _recv_msg{
	nx_uint8_t start;         // 0x02
	nx_uint8_t msg_type;      // COMMAND : 'C' 
	nx_uint8_t device_type;   // RFID:1,GAS:10,AC:20,DC:30,DOOR:40,BIO:50 (DEC)
	nx_uint16_t node_id; 
	nx_uint8_t d0;           // ON:0x31 , 0FF:0x30
	nx_uint8_t d1;
	nx_uint8_t d2;
	nx_uint8_t d3;
	nx_uint8_t d4;
	nx_uint8_t d5;
	nx_uint8_t etx;           // 0x03
}recv_msg;

typedef nx_struct rt_msg{
  
	nx_uint8_t start;         // 0x02
	//nx_int8_t rssi;
	//nx_uint8_t lqi;
	nx_uint8_t msg_type;        // RETURN : 'R' 
	nx_uint8_t device_type;       // RFID:1,GAS:10,AC:20,DC:30,DOOR:40,BIO:50 (DEC)   
	nx_uint16_t node_id; 
	nx_uint8_t d0;      
	nx_uint8_t d1;    
	nx_uint8_t d2;
	nx_uint8_t d3;   
	nx_uint8_t etx;           // 0x03
  
}return_msg;
  
enum{
	AM_MSG = 9,
};

#endif
