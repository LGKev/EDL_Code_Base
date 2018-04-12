/*
	Author: Kevin Kuwata
		Zach Butler
	Kevinkuwata.com
	Github: LGKev
	Created: 3/31/18
	
	Last modified: 4/2/18
	
	EDL_Code.ino
	Arduino 1.8.5
	
	This is the code used in the Electronics Design Lab (EDL) ECEN2270 at the University of Colorado @ Boulder
	
	MIT License

Copyright (c) [2018] [Kevin Kuwata]


Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
*/

#include "I2Cdev.h" //https://github.com/jrowberg/i2cdevlib
#include "MPU6050.h" //https://github.com/jrowberg/i2cdevlib
#include "Wire.h" //accl and line follower and coms between ATTINy84 and 328p

#include <SoftwareSerial.h>
//#include "EDL_Code_Base.h"

#define CLOCKWISE_R   	11
#define C_CLOCKWISE_R 	12
#define V_REF_R			10
#define V_REF_L			9

#define C_CLOCKWISE_L	8
#define CLOCKWISE_L		7
#define ON_OFF_SWITCH	6

#define ENCODER_R		3
#define ENCODER_L		2

#define LED				13


#define BLE_TX			5 //so opposite of the arduino pins so pin 5 (TX of arduio) to the RX pin of the module.
#define BLE_RX			4

#define ENCODER_PULSE_PER_SINGLE_ROTATION		2304 // 12*64 // where did 3 come from? pi? //arbitrarily chosen, change. and calculate value, verify and tune experimentally.
#define ENCODER_L_COUNT_2_FEET_DISTANCE			1052 //Experimentally tested
#define ENCODER_R_COUNT_2_FEET_DISTANCE			1256	//Experimentally tested, note they are different
#define ENCODER_L_COUNT_180_TURN		730 //Experimentally tested, note they are different
#define ENCODER_R_COUNT_180_TURN		750	//Experimentally tested

#define ENCODER_L_COUNT_90_TURN		530// Experimentally found
#define ENCODER_R_COUNT_90_TURN		520// Experimentally found

/*  Set up globals for Accelerometer and the BLE UART  */
SoftwareSerial BLE_UART(BLE_RX, BLE_TX); // RX | TX
MPU6050 ACCL;
int16_t ax, ay, az; //accelerometer globals
int16_t gx, gy, gz; //need both even if not using them, because of function call that updates values.

volatile int encoder_count_left = 0;
volatile int encoder_count_right = 0;

volatile int encoder_Left_Manual_reset = 0;
volatile int encoder_Right_Manual_reset = 0;

volatile byte keyboardSpeed = 75; 

bool demo_4_flag	= false; 	// because I want the robot to rotate around. in infinite loop
bool displayFlag = true; //used for printout in the KEYBOARD_INPUT test.
int incomingByte = 0;   // for incoming serial data


/* ====================================================================================  */
/*
	These are defines used for testing so you don't have to keep 
	re-writing code. Only one test_define should be uncommented at a time.
	
	Debugging 101: If something stops working break the problem into smaller logical blocks.
		These defines help you do that by using small bite size code examples. 
		Don't be a hero. Break it down into smaller parts.
*/
/* ====================================================================================  */

//#define ACCL_TEST		// get accl data and send to serial monitor not bluetooth, only for testing

//#define TEST_LAB4_DEMO			//demo for lab 4, read function for details.
//#define KEYBOARD_INPUT				//purely for printf debgging. 
#define TEST_BLE_ACCL_DATA			// print out accelerometer data, tell us moving forward or backward. give data to bluetooth UART
//#define TEST_BLE_UART_ONLY

//#define TEST_FINAL			// runs the official main code used for final.
//#define TEST_STRAIGHT			// make robot go straight and show value in serial monitor.
//#define TEST_STOP				// literally stops the motors, independent of encoder
//#define TEST_MANUAL_CHANGE_DIRECTIONS_RIGHT 	//manually tests the right motor, independent of encoder
//#define TEST_MANUAL_CHANGE_DIRECTIONS_LEFT	//manually tests the left motor, independent of encoder
//#define TEST_SPIN_CLOCKWISE			// makes motors spin opposite directions to spin robot clockwise, independent of encoder
//#define TEST_SPIN_COUNTER_CLOCKWISE	// makes motors spin opposite directions to spin robot counter clockwise, independent of encoder
//#define TEST_ENCODER_LEFT				// test the encoder printout values to serial.
//#define TEST_ENCODER_RIGHT			// test the encoder printout values to serial.
/* ====================================================================================  */
/* ====================================================================================  */
/* ====================================================================================  */
/* ====================================================================================  */
/* ====================================================================================  */
/*
		Set up initialization
*/
/* ====================================================================================  */

#ifdef ACCL_TEST
void loop(){
	    ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		Serial.print("a_x: \t");Serial.print(ax); Serial.print("\t"); 
       Serial.print("a_y: \t"); Serial.print(ay); Serial.print("\t");
        Serial.print("a_z: \t");Serial.print(az); Serial.print("\t");
		Serial.println();
		
		
		
		delay(100);
}
#endif

#ifdef KEYBOARD_INPUT
volatile int LATEST_ADDRESS = 0x18;     //global so address can be changed by user.
byte x = 0;
#endif

#ifdef TEST_ENCODER_LEFT 			// Used for TEST_ENCODER_LEFT. Do not remove.
byte flag =1; //just for a test delete later. 
#endif
#ifdef TEST_ENCODER_RIGHT			// Used for TEST_ENCODER_RIGHT. Do not remove.
byte flag =1; //just for a test delete later. 
#endif
/*
	@name: setup()
	@brief: runs once. configure GPIO and ISRs for motor control. Set up Serial port for
		debugging and for possible control in the future. 
	@input: check the pin defines above
	@output: check the pin defines above
	@return: none

		TODO: add UART polling to the main loop for keyboard or UART input. This will
			aid in debugging and quicken development time.
*/
void setup() { 
  pinMode(V_REF_L, OUTPUT);
  pinMode(V_REF_R, OUTPUT);
  
  pinMode(CLOCKWISE_R, OUTPUT);
  pinMode(C_CLOCKWISE_R, OUTPUT);
  
  pinMode(CLOCKWISE_L, OUTPUT);
  pinMode(C_CLOCKWISE_L, OUTPUT);
  
  pinMode(ENCODER_L, INPUT);
  pinMode(ENCODER_R, INPUT);
  
  pinMode(ON_OFF_SWITCH, INPUT);
  
  pinMode(LED, OUTPUT);
  
  Serial.begin(9600); //gotta go fast.
  Serial.println("start");
  
  //register ISR 
  attachInterrupt(0, count_Left, RISING);
  attachInterrupt(1, count_Right, RISING);
  
  //HC-08 setup
  BLE_UART.begin(9600);  //Default Baud for comm, it may be different for your Module. 
  BLE_UART.write("we are live");
  
  
  //set up the MPU-6050
     ACCL.initialize();
	 Serial.println(ACCL.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

/* ====================================================================================  */
/*
		Main Loop
*/
/* ====================================================================================  */
#ifdef TEST_BLE_UART_ONLY

int number = 0;
	void loop(){
	BLE_UART.println("number: ");
	BLE_UART.write(number); //purely a number
	BLE_UART.print("p num: "); //works good for UART serial com, not ascii values (ints)
	BLE_UART.println(number);
	
	if (BLE_UART.available() > 0) {
    // read the incoming byte:
    incomingByte = BLE_UART.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
	
	if(incomingByte == 'r'){
		incomingByte = 0;
		number = 0;
	}

	}
	
	delay(1000);
	number++;
	
	}
#endif

#ifdef TEST_BLE_ACCL_DATA
/*
	Re arrange the keys for the gaiming pattern
	
			   'w' 
		'a'	   's'		'd'

		w = forward
		a = left turn 90 
		d = right turn 90
		s = backward
		
		r = reset
		
		space = stop
		
		distance is the incremental distance. 
		*/
byte byteCountBle =0;
void loop(){
	while (BLE_UART.available() > 0) {
    // read the incoming byte only first one:
	if(byteCountBle ==0){
    incomingByte = BLE_UART.read();
	Serial.print("I received:");
    Serial.print(incomingByte, DEC);
	// send to ble terminal
	BLE_UART.print("UART_RX:");
	BLE_UART.print(incomingByte);
	}
	else{
	BLE_UART.read(); //don't collect
	}
	byteCountBle++;
	}
	byteCountBle = 0; // reset for next time we get something.
    // say what you got:
   
  
  switch (incomingByte) {
    case ' ': //space stop and reset
	BLE_UART.println("(space) stop and reset speed");
	incomingByte = 0; // reset, or else infinite loop.
	encoder_count_right = 0;
	encoder_count_left = 0;
	encoder_Right_Manual_reset = 0;
	encoder_Left_Manual_reset = 0;
	keyboardSpeed = 100;
	stop();
	displayFlag = true;
      break;

    case 'w': 
	BLE_UART.println("forward");
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_right <  ENCODER_R_COUNT_2_FEET_DISTANCE){
		printBLE_accl_data();
	straight(keyboardSpeed,keyboardSpeed,1);
		}
	stop();
	encoder_count_right = 0;
	encoder_count_left = 0;
		displayFlag = true;
      break;

    case 's': 
	incomingByte = 0; // reset, or else infinite loop.
	BLE_UART.println("backward");
	while(encoder_count_right <  ENCODER_R_COUNT_2_FEET_DISTANCE){
				printBLE_accl_data();
	straight(keyboardSpeed,keyboardSpeed, -1);
		}
	stop();
	encoder_count_right = 0;
	encoder_count_left = 0;
		displayFlag = true;
      break;
  
      case 'a': 
	BLE_UART.println("90 left turn");
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
  	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
			printBLE_accl_data();
	}
	stop();
	encoder_count_right = 0;
	encoder_count_left = 0;
	displayFlag = true;
	break;
	
	case 'd': //f 
	 BLE_UART.println("90 right turn");
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_left < ENCODER_L_COUNT_90_TURN){
				printBLE_accl_data();
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
	encoder_count_right = 0;
	encoder_count_left = 0;
	displayFlag = true;
    break;
	
	case '+':
	keyboardSpeed +=10;
	BLE_UART.print("+ speed: ");
	BLE_UART.println(keyboardSpeed);
	incomingByte = 0; // reset, or else infinite loop.
	break;		
	
	case '-':
	keyboardSpeed -=10;
	BLE_UART.print("+ speed: ");
	BLE_UART.println(keyboardSpeed);
	incomingByte = 0; // reset, or else infinite loop.
	break;		
  
	default:
	incomingByte = 0;	
	break;

 }
 
 if(displayFlag == true){
	BLE_UART.print("Left encoder manual: ");
	BLE_UART.println(encoder_Left_Manual_reset);
	BLE_UART.print("Right encoder manual: ");
	BLE_UART.println(encoder_Right_Manual_reset);
	
	//Try ble_uart.print to do numbers in ascii. 

	 
	 
    Serial.print("Left encoder manual: ");
	Serial.print(encoder_Left_Manual_reset);
	Serial.print("     Right encoder manual:  ");
	Serial.println(encoder_Right_Manual_reset);
	displayFlag = false;
 }
   ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  BLE_UART.print("a_x   "); BLE_UART.print(ax);  BLE_UART.print("\t a_y "); BLE_UART.print(ay);  BLE_UART.print("\t a_z  "); BLE_UART.println(az);
	delay(200);


	}
#endif
/*
	Prints accel data to uart. 
*/
void printBLE_accl_data(){
	 ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  BLE_UART.print("a_x   "); BLE_UART.print(ax);  BLE_UART.print("\t a_y "); BLE_UART.print(ay);  BLE_UART.print("\t a_z  "); BLE_UART.println(az);
	delay(200);
}

/*
	@name: void loop()
	@brief: Function will allow for user input from the computer and perform different functions. 
			Excessive use of serial print lines for debugging. This should b ea good way to get 
			data from the encoders and setting speed on the fly. 
	@input: [hardware] keyboard
	@output: serial prints
	@global: incomingByte used for taking into serial data, used in SWITCH STATEMENT  statement. 
			encoder_Left_Manual_reset is used to read out the value from the encoder it should follow the
					encoder_count_left, but instead of being reset automatically, it won't reset after each command, its a debug tool
			encoder_Right_Manual_reset (DITO ABOVE).
*/
#ifdef KEYBOARD_INPUT
void loop(){
	
	// Collect Keyboard Input
	if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }
  
  switch (incomingByte) {
    case 97: //a
	incomingByte = 0; // reset, or else infinite loop.
	straight(keyboardSpeed,keyboardSpeed, 1); //go straight
	delay(750);
	stop();
	displayFlag = true;
      break;

    case 98: //b
	incomingByte = 0; // reset, or else infinite loop.
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	delay(750);
	stop();
		displayFlag = true;
      break;

    case 99: //c 
	incomingByte = 0; // reset, or else infinite loop.
	
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	delay(750);
	stop();
		displayFlag = true;
      break;
  
      case 100: //d 
	incomingByte = 0; // reset, or else infinite loop.
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	delay(750);
	stop();
		displayFlag = true;

    break;
	
	 case 102: //f 
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_left < ENCODER_L_COUNT_90_TURN){
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
		displayFlag = true;
    break;
	
	case 103: //g
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_left < ENCODER_L_COUNT_90_TURN){
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
		displayFlag = true;
    break;
	
	case 104: //h
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_left < ENCODER_L_COUNT_180_TURN){
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
		displayFlag = true;
    break;
	
	case 105: //i
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_left < ENCODER_L_COUNT_180_TURN){
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
		displayFlag = true;
    break;
	  
	  
	  
	case 115: //s
	incomingByte = 0; // reset, or else infinite loop.
	delay(750);
	stop();
	displayFlag = true;
    break;
	  
	case 114: //r
	incomingByte = 0; // reset, or else infinite loop.
	encoder_Left_Manual_reset = 0;
	encoder_Right_Manual_reset = 0;
	stop();
	displayFlag = true;
	break;
  
  
	default:
	incomingByte = 0;
	break;

 }
 
 if(displayFlag == true){
    Serial.print("Left encoder manual: ");
	Serial.print(encoder_Left_Manual_reset);
	Serial.print("     Right encoder manual:  ");
	Serial.println(encoder_Right_Manual_reset);
	delay(500);
		displayFlag = false;
 }
  
}
#endif



#ifdef TEST_FINAL
void loop(){
	byte speed = 100;			//@Kevin speed different for each motor, no longer needed
	byte small_delay = 200;
	byte V_REF_L_VALUE = 52; 	// experimentally tested value (double value works also but more prone to slipping at startup)
	byte V_REF_R_VALUE = 50;	// experimentally tested value (double value works also but more prone to slipping at startup)
	
	do {} while (digitalRead(ON_OFF_SWITCH) == LOW);  // wait for ON switch
	
	delay(1000); // delay before start
	
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0; 
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 3 feet
	//@Kevin
	/*
		Change encoder to right side to avoid motor with catch
	*/
	while(encoder_count_right <  ENCODER_R_COUNT_3_FEET_DISTANCE){		// 3 feet encoder check
	//@Kevin
	/*
		during while loop we can test for line following
	*/
		int x = encoder_count_right - encoder_count_left;				// calculate difference
		Serial.print("differences CCW; ");								// print difference
		Serial.println(x);
		delay(20);		
	}
	stop();
	delay(small_delay);
		
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);	// Rotate CCW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
    int z = encoder_count_right - encoder_count_left;					
		Serial.print("difference CCW: ");								
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);
  
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 6 feet
	while(encoder_count_right < ENCODER_R_COUNT_6_FEET_DISTANCE){		// 6 feet encoder check
		int z = encoder_count_right - encoder_count_left;			
		Serial.print("difference CCW: ");							
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;

	Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);			// Rotate CW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);		
	}
	stop();
 	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;

	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 3 feet
	while(encoder_count_right < ENCODER_R_COUNT_3_FEET_DISTANCE){		// 3 feet encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);			// rotate CW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);    
	}
	stop();
 	delay(small_delay);
  
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 3 feet
	while(encoder_count_right < ENCODER_R_COUNT_3_FEET_DISTANCE){		// 3 feet encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);

    encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);			// rotate CW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);    
	}
	stop();
	delay(small_delay);
  
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 6 feet
	while(encoder_count_right < ENCODER_R_COUNT_6_FEET_DISTANCE){		// 6 feet encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);	// rotate CCW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);
  
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 3 feet
	while(encoder_count_right < ENCODER_R_COUNT_3_FEET_DISTANCE){		// 3 feet encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);	// rotate CCW
	while(encoder_count_right <  ENCODER_R_COUNT_90_TURN){				// 90 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
  	delay(10000);  // giving time to flip On Off switch before next loop
}
#endif

/* ====================================================================================  */

/*=======================================================
=========================================================

	Interrupt Handlers
	These are gonna count the encoder pulses 
	
=========================================================
=========================================================
*/

/*
	@name: count_Right
	@brief: This is an ISR. counts encoder pulses on GPIO 2
	@input: [hardware] input from encoder
	@output: none
	@global: encoder_count_left
	@return: none
*/
void count_Left(){
	encoder_count_left++;
	encoder_Left_Manual_reset++;
}

/*
	@name: count_Right
	@brief: This is an ISR. counts encoder pulses on GPIO 3
	@input: [hardware] input from encoder
	@output: none
	@globals: encoder_count_right
*/
void count_Right(){
	encoder_count_right++;
	encoder_Right_Manual_reset++;
}

/*
	@name: Rotate_Robot_ClockWise360
	@brief: Rotate robot 360 degrees independent of encoder
	@inputs: V_REF_L_VALUE, V_REF_R_VALUE between 0 and 255
	@output: none
	@return: none
	
	IMPORTANT: independent of encoder.
*/
void Rotate_Robot_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, HIGH);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, HIGH);
}

/*
	@name: Rotate_Robot_Counter_ClockWise360
	@brief: Rotate robot 360 degrees independent of encoder
	@inputs: V_REF_L_VALUE, V_REF_R_VALUE between 0 and 255
	@output: none
	@return: none
	IMPORTANT: independent of encoder.
*/
void Rotate_Robot_Counter_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, HIGH);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, HIGH);
	digitalWrite(C_CLOCKWISE_L, LOW);
}


/*
	@name: void straight(byte V_REF_L_VALUE, byte V_REF_R_VALUE, int direction)
	@brief: makes the robot go straight if the input values are equal. 
	@inputs: V_REF_L_VALUe, V_REF_R_VALUE between 0 and 255
			direction +1 for forward
					- for backward
	@outputs: none
	
	Note: it may be the case that the motors require different ref voltages, hence the
	two inputs. If motor and encoders were identical we could just use 1 value.
*/
void straight(byte V_REF_L_VALUE, byte V_REF_R_VALUE, int direction){
	if(direction == 1){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, HIGH);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, HIGH);
	digitalWrite(C_CLOCKWISE_L, LOW);
	}
	else if(direction == -1){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, HIGH);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, HIGH);
	}
	//should never reach here
}	

/*
	@name: stop()
	@brief: stops the motors by setting CLOCKWISE_L, CLOCKWISE_R, C_CLOCKWISE_L, C_CLOCKWISE_R to LOW
	@input: none
	@output: [hardware] sets the pins low, check #defines at top of file for pinout.
	@return: none
*/
void stop(){
	//stop
	// @Kevin 
	/*
	This function stops both motors even if both encoders do not reach desired values.
	Do we want seperate functions to stop each motor seperately so they both reach the desired encoder values,
	or just modify this function with inputs of Left, Right, Both. Also we could slow one motors voltage to slow how
	quickly it reaches its encoder value so that they reach them at similar times (a few options are available)
	*/
	
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, LOW);
}

#ifdef TEST_LAB4_DEMO
/*
	@name: loop
	@brief: This is the demo loop for lab 4. The robot will:
		1) wait for input from user to start (GPIO: button)
		2) Wait 1 second
		3) move forward 2 feet
		4) 180* clockwise rotation
		5) move forward 2 feet
		6) 180 counter clockwise rotation
	@input: [hardware] GPIO input
	@output: control the pins for motor movement, check defines above. 
	@return: none
	@define: ENCODER_L_COUNT_2_FEET_DISTANCE - used to determine distance via encoder
*/
void loop(){
	byte speed = 100;			//@Kevin speed different for each motor
	byte small_delay = 200;
	byte V_REF_L_VALUE = 52; 	// experimentally tested value (double value works also but more prone to slipping at startup)
	byte V_REF_R_VALUE = 50;	// experimentally tested value (double value works also but more prone to slipping at startup)
	
	do {} while (digitalRead(ON_OFF_SWITCH) == LOW);  // wait for ON switch
	
	delay(1000); // delay before start
	
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0; 
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 2 feet
	while(encoder_count_right <  ENCODER_R_COUNT_2_FEET_DISTANCE){		// 2 feet encoder check
		int x = encoder_count_right - encoder_count_left;
		Serial.print("differences CCW; ");
		Serial.println(x);
		delay(20);		
	}
	stop();
	delay(small_delay);

    encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);			// rotate CW
	while(encoder_count_right <  ENCODER_R_COUNT_180_TURN){				// 180 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);    
	}
	stop();
 	delay(small_delay);
	
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0; 
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 2 feet
	while(encoder_count_right <  ENCODER_R_COUNT_2_FEET_DISTANCE){		// 2 feet encoder check
		int x = encoder_count_right - encoder_count_left;
		Serial.print("differences CCW; ");
		Serial.println(x);
		delay(20);		
	}
	stop();
	delay(small_delay);

	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0;
	
	Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);	// rotate CCW
	while(encoder_count_right <  ENCODER_R_COUNT_180_TURN){				// 180 degree encoder check
		int z = encoder_count_right - encoder_count_left;
		Serial.print("difference CCW: ");
		Serial.println(z);
		delay(20);
	}
	stop();
  	delay(10000);  // giving time to flip On Off switch before next loop
}
#endif

#ifdef TEST_STRAIGHT
/*
	@name: loop()
	@brief: simple test to get motors to go straight, reports the encoder values
	@input: [hardware] encoder pins check pin defines at top of file
	@output: Serial prints encoder value 
	@return: none
	@global: encoder_count_left
*/
void loop(){
	straight(100,100,1);
	delay(500);
	stop();
	Serial.print("Left encoder:  ");
	Serial.print(encoder_count_left);
	Serial.print("Right encoder: 	");
	Serial.println(encoder_count_right);
	delay(3000);

}
#endif

#ifdef TEST_MANUAL_CHANGE_DIRECTIONS_RIGHT
/*
	@name: loop()
	@brief: Tests the direction of the motors. Good as a first test to see which side is defective
			Does not use encoder!
	@input: none
	@output: [hardware] output check pin defines at top of file.
	@return: none
*/
void loop() {
  // TODO: define which direction this set up is: ________________
  	analogWrite(V_REF_R, 229); //set full speed, 90 % duty cycle
	analogWrite(V_REF_L, 229);
	digitalWrite(CLOCKWISE_R, HIGH);
	digitalWrite(C_CLOCKWISE_R, LOW);
	delay(2000);
	
	
	
	//stop
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, LOW);
	delay(2000);
 // go the other direction  TODO: define which direction this set up is: ________________
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, HIGH);
	delay(2000);
	//stop
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, LOW);
	delay(2000);
}
#endif

#ifdef TEST_MANUAL_CHANGE_DIRECTIONS_LEFT
void loop() {
  // TODO: define which direction this set up is: ________________
  	analogWrite(V_REF_R, 229); //set full speed, 90 % duty cycle
	analogWrite(V_REF_L, 229);
	digitalWrite(CLOCKWISE_L, HIGH);
	digitalWrite(C_CLOCKWISE_L, LOW);
	delay(2000);
	
	
	
	//stop
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, LOW);
	delay(2000);
 // go the other direction  TODO: define which direction this set up is: ________________
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, HIGH);
	delay(2000);
	//stop
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, LOW);
	delay(2000);
}
#endif

#ifdef TEST_SPIN_CLOCKWISE
/*
	@name: loop()
	@brief: Spins the motors so that robot would spin clockwise. Tests the FETS.
	@input: none
	@output: [hardware] output check pin defines at top of file.
			Serial output the encoder counts so we can test if encoder is working and also quickly find values we need. ie 180, 360, 270
	@return: none
*/
void loop() {
	Rotate_Robot_ClockWise360( 75, 75);
	Serial.print("Left encoder:  ");
	Serial.print(encoder_count_left);
	Serial.print("Right encoder: 	");
	Serial.println(encoder_count_right);}
#endif

#ifdef TEST_SPIN_COUNTER_CLOCKWISE
/*
	@name: loop()
	@brief: Tests that the FET are correctly working by making motors spin opposite direction.
			Does not use encoder!
	@input: none
	@output: [hardware] output check pin defines at top of file.
	@return: none
*/
void loop() {
	Rotate_Robot_Counter_ClockWise360( 200, 200);
}
#endif


#ifdef TEST_ENCODER_LEFT
/*
	@name: loop()
	@brief: Tests the inputs and ISR for the left encoder.
	@input: [hardware] pin 2 for encoder ISR
	@output: [hardware] output check pin defines at top of file.
	@global: encoder_count_left
	@return: none
*/
void loop(){
	
	if(flag == 1){
		straight(50,50,1); //make it go
		encoder_count_left = 0; //reset the count
		flag = 0;
	}
  Serial.print("left count: ");
  Serial.println(encoder_count_left);
		delay(300);

}
#endif

#ifdef TEST_ENCODER_RIGHT //encoder works.
/*
	@name: loop()
	@brief: Tests the inputs and ISR for the right encoder.
	@input: [hardware] pin 3 for encoder ISR
	@output: [hardware] output check pin defines at top of file.
	@global: encoder_count_right
	@return: none
*/
void loop(){
	if(flag == 1){
		straight(50,50,1); //make it go
		encoder_count_right = 0; //reset the count
		flag = 0;
	}
  Serial.print("right count: ");
  Serial.println(encoder_count_right);
		delay(300);

}
#endif

#ifdef TEST_STOP
/*
	@name: loop()
	@brief: Forces the motors to stop.
	@input: none
	@output: [hardware] output check pin defines at top of file.
	@global: none
	@return: none
*/
void loop(){
	stop();
}
#endif
