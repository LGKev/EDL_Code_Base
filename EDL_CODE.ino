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
#include "sensorbar.h"	// line follower library
#include <SoftwareSerial.h>
//#include "EDL_Code.h"

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

MPU6050 ACCL;
int16_t ax = 4;
int16_t ay =5;
int16_t az =7; //accelerometer globals
int16_t gx, gy, gz; //need both even if not using them, because of function call that updates values.

// Line follow variables
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
unsigned long previous_ms = 0;
unsigned long current_ms = 0;
long interval = 0;
char bot_position;
char led_count;
char mode_select = 0;
uint8_t var;
SensorBar mySensorBar(SX1509_ADDRESS);

volatile int encoder_count_left = 0;
volatile int encoder_count_right = 0;

volatile int encoder_Left_Manual_reset = 0;
volatile int encoder_Right_Manual_reset = 0;

byte V_REF_L_VALUE = 0;
byte V_REF_R_VALUE = 0;

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

//#define TEST_ACCL		// get accl data and send to serial monitor not bluetooth, only for testing

//#define TEST_LAB4_DEMO			//demo for lab 4, read function for details.
#define TEST_KEYBOARD_INPUT				//purely for printf debgging. 
//#define TEST_BLE_ACCL_DATA			// print out accelerometer data, tell us moving forward or backward. give data to bluetooth UART
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

  Wire.begin();

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
  Serial.begin(9600);  //Default Baud for comm, it may be different for your Module. 
  Serial.write("we are live");
  
  
  //set up the MPU-6050
  ACCL.initialize();
  Serial.println(ACCL.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // Line Follow Sensor Setup
  
  //mySensorBar.setBarStrobe();		// strobe
  mySensorBar.clearBarStrobe(); 	// more power no strobe

  //mySensorBar.clearInvertBits();	// dark line
  mySensorBar.setInvertBits(); 	// light line

  // Check I2C for line follow sensor
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)  {
    Serial.println("Line follower IC communication OK");
  }
  else  {
    Serial.println("Line follower IC communication FAILED");
    while(1);
  }
  
}

/* ====================================================================================  */
/*
		Main Loop
*/
/* ====================================================================================  */

#ifdef TEST_ACCL
void loop(){
	    ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		Serial.print("a_x: \t");Serial.print(ax); Serial.print("\t"); 
		Serial.print("a_y: \t"); Serial.print(ay); Serial.print("\t");
        Serial.print("a_z: \t");Serial.print(az); Serial.print("\t");
		Serial.println();
		
		
		
		delay(1200);
}
#endif

#ifdef TEST_BLE_UART_ONLY

int number = 0;
	void loop(){
	Serial.println("number: ");
	Serial.write(number); //purely a number
	Serial.print("p num: "); //works good for UART serial com, not ascii values (ints)
	Serial.println(number);
	
	if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

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
		f = line follow
		
		r = reset
		
		space = stop
		
		distance is the incremental distance. 
		*/
byte byteCountBle =0;
void loop(){
	while (Serial.available() > 0) {
    // read the incoming byte only first one:
	if(byteCountBle ==0){
    incomingByte = Serial.read();
	Serial.print("I received:");
    Serial.print(incomingByte, DEC);
	// send to ble terminal
	Serial.print("UART_RX:");
	Serial.print(incomingByte);
	}
	else{
	Serial.read(); //don't collect
	}
	byteCountBle++;
	}
		byteCountBle = 0; // reset for next time we get something.
    // say what you got:
   
  
  switch (incomingByte) {
    case ' ': //space stop and reset
	Serial.println("(space) stop and reset speed");
	incomingByte = 0; // reset, or else infinite loop.
	encoder_count_right = 0;
	encoder_count_left = 0;
	encoder_Right_Manual_reset = 0;
	encoder_Left_Manual_reset = 0;
	keyboardSpeed = 100;
	keyboardSpeedright = 100;	// added variables to allow seperate Vref for each wheel
	keyboardSpeedleft = 104;	
	stop();
	displayFlag = true;
      break;

    case 'w': 
	Serial.println("forward");
	incomingByte = 0; // reset, or else infinite loop.
	while(encoder_count_right <  ENCODER_R_COUNT_2_FEET_DISTANCE){
	printBLE_accl_data();
	straight(keyboardSpeed,keyboardSpeed, 1);
		}
	stop();
	encoder_count_right = 0;
	encoder_count_left = 0;
		displayFlag = true;
      break;

    case 's': 
	incomingByte = 0; // reset, or else infinite loop.
	Serial.println("backward");
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
	Serial.println("90 left turn");
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
	 Serial.println("90 right turn");
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
	Serial.print("+ speed: ");
	Serial.println(keyboardSpeed);
	incomingByte = 0; // reset, or else infinite loop.
	break;		
	
	case '-':
	keyboardSpeed -=10;
	Serial.print("+ speed: ");
	Serial.println(keyboardSpeed);
	incomingByte = 0; // reset, or else infinite loop.
	break;

  case 'l': //line follow
  incomingByte = 0; // reset, or else infinite loop.
  encoder_count_right = 0;
  encoder_count_left = 0;
  encoder_Right_Manual_reset = 0;
  encoder_Left_Manual_reset = 0;
  keyboardSpeed = 0;
  Serial.println("Line Following");
  mode_select = 1;
  line_follow();
  stop();
  displayFlag = true;
      break;
     
  
	default:
	incomingByte = 0;	
	break;

 }
 
 if(displayFlag == true){
	Serial.print("Left encoder manual: ");
	Serial.println(encoder_Left_Manual_reset);
	Serial.print("Right encoder manual: ");
	Serial.println(encoder_Right_Manual_reset);
	
	//Try Serial.print to do numbers in ascii. 

	 
	 
    Serial.print("Left encoder manual: ");
	Serial.print(encoder_Left_Manual_reset);
	Serial.print("     Right encoder manual:  ");
	Serial.println(encoder_Right_Manual_reset);
	displayFlag = false;
 }
 /*  ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a_x   "); Serial.print(ax);  Serial.print("\t a_y "); Serial.print(ay);  Serial.print("\t a_z  "); Serial.println(az);
	delay(200);
*/

	}
#endif
/*
	Prints accel data to uart. 
*/
void printBLE_accl_data(){
	 ACCL.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a_x   "); Serial.print(ax);  Serial.print("\t a_y "); Serial.print(ay);  Serial.print("\t a_z  "); Serial.println(az);
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
#ifdef TEST_KEYBOARD_INPUT
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
    case 119: //w
	incomingByte = 0; // reset, or else infinite loop.
	straight(keyboardSpeed,keyboardSpeed, 1); //go straight
	delay(750);
	stop();
	displayFlag = true;
      break;

    case 115: //s
	incomingByte = 0; // reset, or else infinite loop.
	straight(keyboardSpeed,keyboardSpeed, -1);
	delay(750);
	stop();
		displayFlag = true;
      break;

    case 100: //d 
	incomingByte = 0; // reset, or else infinite loop.
	
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	delay(500);
	stop();
		displayFlag = true;
      break;
  
      case 97: //a 
	incomingByte = 0; // reset, or else infinite loop.
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	delay(500);
	stop();
		displayFlag = true;

    break;
	
	case 102: //f 
	  incomingByte = 0; // reset, or else infinite loop.
	  encoder_count_left = 0;
    encoder_count_right = 0;

	  while(encoder_count_left < ENCODER_L_COUNT_90_TURN){
	    Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	  }
	  stop();
	  displayFlag = true;
    break;
	
	case 103: //g
	incomingByte = 0; // reset, or else infinite loop.
  encoder_count_left = 0;
  encoder_count_right = 0;

	while(encoder_count_left < ENCODER_L_COUNT_90_TURN){
	Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
  displayFlag = true;
  break;
	
	case 104: //h
	incomingByte = 0; // reset, or else infinite loop.
  encoder_count_left = 0;
  encoder_count_right = 0;
	while(encoder_count_left < ENCODER_L_COUNT_180_TURN){
	  Rotate_Robot_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
  displayFlag = true;
  break;
	
	case 73: //i
	incomingByte = 0; // reset, or else infinite loop.
  encoder_count_left = 0;
  encoder_count_right = 0;
	while(encoder_count_left < ENCODER_L_COUNT_180_TURN){
	Rotate_Robot_Counter_ClockWise360(keyboardSpeed,keyboardSpeed);
	}
	stop();
		displayFlag = true;
    break;
		  
	case 32: //space
	incomingByte = 0; // reset, or else infinite loop.
	delay(750);
	stop();
	displayFlag = true;
    break;
	  
	case 114: //r
	incomingByte = 0; // reset, or else infinite loop.
	encoder_Left_Manual_reset = 0;
	encoder_Right_Manual_reset = 0;
  encoder_count_left = 0;
  encoder_count_right = 0;
	stop();
	displayFlag = true;
	break;

	case 108: //l
	incomingByte = 0; // reset, or else infinite loop.
	encoder_Left_Manual_reset = 0;
	encoder_Right_Manual_reset = 0;
	mode_select = 1;
	line_follow();
	Serial.println("Returned from line following");
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
	
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, LOW);
}

/*
	@name: loop
	@brief: This is the line following code. The robot will:
		1) Check line sensors to see how many detect lines
		2) Check line sensors and return value from -127 to 127 depending on position of line
		3) Move forward if line detected and near center
		4) Rotate clockwise if value is positive and depending on which sensors detect line
		5) Rotate counter clockwise if value is negative and depending on which sensors detect line
		6) Stop if zero or more than 3 sensors detect line
	@input: [hardware] I2C sensor, ON_OFF_SWITCH
	@output: control the pins for motor movement, check defines above. 
	@return: none
*/

void line_follow(){
  delay(25);

  if (digitalRead(ON_OFF_SWITCH) == LOW){ // Check for On/Off switch
	stop();
	mode_select = 0;
	Serial.println("ON_OFF_SWITCH set to low");
	return;
  }

  while((mode_select == 1) && (digitalRead(ON_OFF_SWITCH) == HIGH)){
    
  led_count = mySensorBar.getDensity(); 			  // initial line check
	bot_position = mySensorBar.getPosition(); 		// initial position of line check
  
	while(led_count < 4 && led_count > 0){
	  led_count = mySensorBar.getDensity(); 			// line check
	  bot_position = mySensorBar.getPosition(); 		// check position of line
		
	  if (bot_position > -32 && bot_position < 32){		 // check if line is near center
		var = 1;
		break;
	  }
	  else if (bot_position > 46 && bot_position < 64){	 // checks slightly off center (positive)
		var = 2;
		break;
	  }
	  else if (bot_position > -64 && bot_position < -46){  // checks slightly off center (negative)
		var = 3;
		break;
	  }
	  else if (bot_position > 78 && bot_position < 96){	 // checks off center (positive)
		var = 4;
		break;
	  }
	  else if (bot_position > -96 && bot_position < -78){	 // checks off center (negative)
		var = 5;
		break;
	  }
	  else if (bot_position > 110 && bot_position <= 127){ // check edge of sensor (positive)
		var = 6;
		break;
	  }
	  else if (bot_position >= -127 && bot_position < 110){// checks edge of sensor (negative)
		var = 7;
		break;
	  }
	  else {
		delay(25);
	  }
	}
	if(led_count == 0 || led_count > 3){	// set default case if no line or more than 3 sensors pick up line
	  mode_select = 0;
	  var = 0;
    stop();
	  Serial.println("lost line detection");
	  return;	  
	}
	
    delay(25);  
  
    switch (var) {
	  case 1:
        //Straight
		V_REF_L_VALUE = 50;
		V_REF_R_VALUE = 50;
		straight(V_REF_L_VALUE, V_REF_R_VALUE, 1);	// forward
		break;
	  case 2:
		//Turn right
		V_REF_L_VALUE = 25;   // higher V_REF_L if straight function used
		V_REF_R_VALUE = 25;
		// Slower right turn, rotation can use straight function with variable voltages
		Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
	  case 3:
		//Turn left
		V_REF_L_VALUE = 25;
		V_REF_R_VALUE = 25;   // higher V_REF_R if straight function used
		Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
      case 4:
		//Turn right more
		V_REF_L_VALUE = 50;
		V_REF_R_VALUE = 50;
		Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
      case 5:
		//Turn left more
		V_REF_L_VALUE = 50;
		V_REF_R_VALUE = 50;
		Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
      case 6:
		//Turn right faster
		V_REF_L_VALUE = 75;
		V_REF_R_VALUE = 75;
		Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
      case 7:
		//Turn left faster
		V_REF_L_VALUE = 75;
		V_REF_R_VALUE = 75;
		Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
		break;
      default:
		V_REF_L_VALUE = 0;
		V_REF_R_VALUE = 0;
		stop();
	}
	delay(50);
  }
  mode_select = 0;
  Serial.println("Done Following");
}

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

#ifdef LINE_FOLLOW_TEST
/*
	@name: loop
	@brief: This is the line following code. The robot will:
		1) Check line sensors to see how many detect line
		2) Check line sensors and return value from -127 to 127 depending on position of line
		3) Move forward if line detected and near center
		4) Rotate clockwise if value is positive and depending on which sensors detect line
		5) Rotate counter clockwise if value is negative and depending on which sensors detect line
		6) Stop if zero or more than 3 sensors detect line
	@input: [hardware] I2C sensor, ON_OFF_SWITCH
	@output: control the pins for motor movement, check defines above. 
	@return: none
*/
void loop() {
  delay(25);

  do {} while (digitalRead(ON_OFF_SWITCH) == LOW);  // wait for ON switch

  led_count = mySensorBar.getDensity(); 			// line check
  bot_position = mySensorBar.getPosition(); 		// check position of line
  
  while(led_count < 4 && led_count > 0){
    
    if (bot_position > -32 && bot_position < 32){		 // check if line is near center
      var = 1;
      break;
    }
    else if (bot_position > 46 && bot_position < 64){	 // checks slightly off center (positive)
      var = 2;
      break;
    }
    else if (bot_position > -64 && bot_position < -46){  // checks slightly off center (negative)
      var = 3;
      break;
    }
    else if (bot_position > 78 && bot_position < 96){	 // checks off center (positive)
      var = 4;
      break;
    }
    else if (bot_position > -96 && bot_position < -78){	 // checks off center (negative)
      var = 5;
      break;
    }
    else if (bot_position > 110 && bot_position <= 127){ // check edge of sensor (positive)
      var = 6;
      break;
    }
    else if (bot_position >= -127 && bot_position < 110){// checks edge of sensor (negative)
      var = 7;
      break;
    }
    else {
      delay(25);
    }
  }
  if(led_count == 0 || led_count > 3){	// set default case if no line or more than 3 sensors pick up line
    var = 0;
  }
  delay(25);  
  
  switch (var) {
    case 1:
      //Straight
      V_REF_L_VALUE = 50;
      V_REF_R_VALUE = 50;
      straight(V_REF_L_VALUE, V_REF_R_VALUE, 1);	// forward
  	  break;
    case 2:
      //Turn right
  	  V_REF_L_VALUE = 50;   // higher V_REF_L if straight function used
      V_REF_R_VALUE = 50;
      stop();
      // Slower right turn, rotation can use straight function with variable voltages
      Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    case 3:
      //Turn left
  	  V_REF_L_VALUE = 50;
      V_REF_R_VALUE = 50;   // higher V_REF_R if straight function used
      stop();
      Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    case 4:
      //Turn right more
  	  V_REF_L_VALUE = 50;
      V_REF_R_VALUE = 50;
      stop();
      Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    case 5:
      //Turn left more
      V_REF_L_VALUE = 50;
      V_REF_R_VALUE = 50;
      stop();
      Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    case 6:
      //Turn right faster
    	V_REF_L_VALUE = 100;
      V_REF_R_VALUE = 100;
      stop();
      Rotate_Robot_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    case 7:
      //Turn left faster
  	  V_REF_L_VALUE = 100;
      V_REF_R_VALUE = 100;
      stop();
      Rotate_Robot_Counter_ClockWise360(V_REF_L_VALUE, V_REF_R_VALUE);
      break;
    default:
      V_REF_L_VALUE = 0;
      V_REF_R_VALUE = 0;
      stop();
  }
  delay(50);
}
#endif

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

#ifdef TEST_FINAL
void loop(){
	byte small_delay = 200;
	byte V_REF_L_VALUE = 52; 	// experimentally tested value (double value works also but more prone to slipping at startup)
	byte V_REF_R_VALUE = 50;	// experimentally tested value (double value works also but more prone to slipping at startup)
	
	do {} while (digitalRead(ON_OFF_SWITCH) == LOW);  // wait for ON switch
	
	delay(1000); // delay before start
	
	encoder_count_right = 0;	//reset the counts
	encoder_count_left = 0; 
	
	straight(V_REF_L_VALUE, V_REF_R_VALUE);								// straight 3 feet
	while(encoder_count_right <  ENCODER_R_COUNT_3_FEET_DISTANCE){		// 3 feet encoder check
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
