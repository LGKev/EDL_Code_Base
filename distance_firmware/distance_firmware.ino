/*
	Modified: Kevin Kuwata
	Date: 4/12/18
	
	@brief: this program will monitor disance. A hard limit will be set and if an object is detected within that range
	pin A0 will be used for PWM so that a range can be sentout to the master. This will allow for easy communication of 
	distance ranges.
	
	first just implemented a distance. If within a given range stop that robot. 
	


  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  
  So I am adding to this but base code is from Nathan. 

*/

#include "vl53l1_register_map.h"

#include <Wire.h>

#define PRINTOUT
const byte notificationPinOUT = A0;
const byte rxFromMaster	 = A1; //so there can be analog reads. 

void setup(void)
{
  Wire.begin();

	//set up the notification pin
	pinMode(notificationPinOUT, OUTPUT);
	pinMode(rxFromMaster, INPUT); 
	
		pinMode(A1 	, OUTPUT);

	
	analogWrite(notificationPinOUT, 255); //set low. voltage will be proportional to the distance. high voltage is far, 0 is low. 
  
  
  Serial.begin(9600);
  Serial.println(" Distance Module ready!");

}


/*
	@name: determineNotificationPinState
	@brief: Will set a PWM value that the master arduino. Range of output is 0 to 3.3V because of the pro mini. 
	@input: notificationPin, allows the user to change which analog pin to be used. generally will be A0. but can be chagned at top
			of the file. 
			pass the measuredDistance calculated by Nates library
	@output: [hardware] the notificationPin passed will set a PWM that the master will read. 
*/
#define DISTANCE_A 	2000
#define	DISTANCE_B 	1500
#define	DISTANCE_C 	1000
#define	DISTANCE_D 	500
#define	DISTANCE_E 	250
#define	DISTANCE_F 	50

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

void loop(void)
{
	
	writeConfiguration(); //Write configuration block of 135 bytes to setup a measurement

  //Poll for completion of measurement. Takes 40-50ms.
  while (newDataReady() == false)
    delay(5);

  int distance = getDistance(); //Get the result of the measurement from the sensor

  
  determineNotificationPinState(notificationPinOUT, distance);
 
 

 #ifdef PRINTOUT
  Serial.print("Distance(mm): ");
  Serial.println(distance);
#endif
}


