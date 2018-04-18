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


