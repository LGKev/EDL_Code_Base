/*
	code for the adc on the attiny84
	Author: Kevin Kuwata
	https://github.com/LGKev/EDL_Code_Base
	Created: 4/16/18 10:42pm
	
	uses ATTinyCore by Spence Konde
	https://github.com/SpenceKonde/ATTinyCore
	
	
	This MCU will act as a slave. It has a default address of 0x28 and will respond accordinginly. 
	Plan is to have the commands to read an ADC and then report back the value over i2c when the MASTER 
	asks for a given amount of data. I envision the following commands:
		
		Pin				pin number sent from MASTER
		ReadADC			MASTER asks for 1 byte and the slave returns the value
		AnalogWrite 	Takes a pin and sets the value
		
		MCU_Report		the Slave will respond with data from the line follower and a byte on what the final command is. 
						ie turn left, turn right, straight
						
						
	FOR now. to prove the ADC work and the pwm works we are going to simply read the ADC of the first few and then write the values 
	to pwm to show that the duty cycles are indead different!
*/

#include <Wire.h>


#define DEFAULT_ADDRESS_ATTINY84 0X28
/*
	PIN MAP FOR ATTINY84
		** note there is an alternate pinout lots of flexibility!
	digital pins == Atin84 pin == physical package pins == ANALOG PINS
	10 = PB0 = 11
	9 	= PB1	= 12
	11	= PB3	=	13
	8 	= PB2	=	14
	7 	= PA7	=	15	= A7
	6	= PA6 	=	16	= A6
	5	= PA5	= 	20	= A5
	4 	= PA4	= 1	=	A4
	3	= PA3	= 2	=	A3
	2	= PA2	= 3	=	A2
	1	= PA1	= 4	= 	A1
	0	= PA0	= 5	= 	A0
		


*/
void setup(){
	for(int i=0; i < 3; i++){
		pinMode(i, OUTPUT);
	}
	
}

int analogA0 = 0;
int analogA1 = 0;
void loop(){
	for(int i =0; i< 3; i++){
	digitalWrite(i, HIGH);
	}
	delay(100);
	for(int i =0; i< 3; i++){
	digitalWrite(i, LOW);
	}
	delay(100);
	
	for(int i=3; i<=7; i+=2){
		analogWrite(i, 100);
	}
	delay(100);
}	



void determineNotificationPinState(byte notificationPin, unsigned int measuredDistance){
	if(measuredDistance > DISTANCE_A){
		// consider far away don't worry
		analogWrite(notificationPin, 255);
	}
	
	int count = 0;
	while(measuredDistance < DISTANCE_F){
		count++;
		
		if(count == 300){
		analogWrite(notificationPin, 0);
		break;
		}
	}
}