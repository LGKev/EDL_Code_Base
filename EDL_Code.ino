
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

#define ENCODER_PULSE_PER_SINGLE_ROTATION		2304 // 12*64 // where did 3 come from? pi?

volatile int encoder_count_left = 0;
volatile int encoder_count_right = 0;


/* ====================================================================================  */
/*
	These are defines used for testing so you don't have to keep 
	re-writing code. Only one test_define should be uncommented at a time.
	
	Debugging 101: If something stops working break the problem into smaller logical blocks.
		These defines help you do that by using small bite size code examples. 
		Don't be a hero. Break it down into smaller parts.
*/
/* ====================================================================================  */

//#define TEST_NORMAL			// runs the official main code

#define TEST_LAB4_DEMO			//demo for lab 4, read function for details.


//#define TEST_STOP						// literally stops the motors, independent of encoder
//#define TEST_MANUAL_CHANGE_DIRECTIONS_RIGHT 	//manually tests the right motor, independent of encoder
//#define TEST_MANUAL_CHANGE_DIRECTIONS_LEFT		//manually tests the left motor, independent of encoder

//#define TEST_SPIN_CLOCKWISE			// makes motors spin opposite directions to spin robot clockwise, independent of encoder
//#define TEST_SPIN_COUNTER_CLOCKWISE		// makes motors spin opposite directions to spin robot counter clockwise, independent of encoder

//#define TEST_ENCODER_LEFT			// test the encoder printout values to serial.

//#define TEST_ENCODER_RIGHT	// test the encoder printout values to serial.


/* ====================================================================================  */
/* ====================================================================================  */
/* ====================================================================================  */
/* ====================================================================================  */







/* ====================================================================================  */
/*
		Set up initialization
*/
/* ====================================================================================  */
#ifdef TEST_ENCODER_LEFT //for testing because do while wasn't exactly what I wanted.
byte flag =1; //just for a test delete later. 
#endif
#ifdef TEST_ENCODER_RIGHT
byte flag =1; //just for a test delete later. 
#endif



void setup() { 
  pinMode(V_REF_L, OUTPUT);
  pinMode(V_REF_R, OUTPUT);

  
  pinMode(CLOCKWISE_R, OUTPUT);
  pinMode(C_CLOCKWISE_R, OUTPUT);
  
  pinMode(CLOCKWISE_L, OUTPUT);
  pinMode(C_CLOCKWISE_L, OUTPUT);
  
  pinMode(LED, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("start");
  
  //register ISR with vector table
  attachInterrupt(0, count_Left, RISING);
  attachInterrupt(1, count_Right, RISING);
  
}


/* ====================================================================================  */
/*
		Main Loop
*/
/* ====================================================================================  */





#ifdef TEST_NORMAL
void loop(){
		encoder_count_left = 0; //reset the count.
	while(encoder_count_left <  ENCODER_PULSE_PER_SINGLE_ROTATION){
		Rotate_Robot_ClockWise360(200,200);
	}
	encoder_count_left = 0; //reset the count.
	
	stop();
	
	delay(1000);
	encoder_count_left = 0; //reset the count.
	while(encoder_count_left <  ENCODER_PULSE_PER_SINGLE_ROTATION){
		Rotate_Robot_Counter_ClockWise360(200, 200);
	}
	encoder_count_left = 0; //reset the count.
	delay(1000);
	
	encoder_count_left = 0; //reset the count.
	while(encoder_count_left <  ENCODER_PULSE_PER_SINGLE_ROTATION){
		Rotate_Robot_ClockWise360(100,100);
	}
	encoder_count_left = 0; //reset the count.
	
	stop();
	
	delay(1000);
	encoder_count_left = 0; //reset the count.
	while(encoder_count_left <  ENCODER_PULSE_PER_SINGLE_ROTATION){
		Rotate_Robot_Counter_ClockWise360(100, 100);
	}
	encoder_count_left = 0; //reset the count.
	delay(1000);
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

//encoder_count_left is global
//when does is get reset? in here or the user in main?
//connect to pin 2 on arduino
void count_Left(){
	encoder_count_left++;
}

//encoder_count_right is global
// connect to pin 3 on arduino
void count_Right(){
	encoder_count_right++;
}

/*
	This function will rotate the robot clockwise, takes in 2 inputs.
	Vref_L and Vref_R values. You set the duration under. This might
	not be the best implementation because of how the robot spins and how we might 
	want to do a rotation that is partial. 
	
	this is for the 360 turn.
*/
void Rotate_Robot_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, HIGH);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, HIGH);
}

void Rotate_Robot_Counter_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, HIGH);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, HIGH);
	digitalWrite(C_CLOCKWISE_L, LOW);
}


void straight(byte V_REF_L_VALUE, byte V_REF_R_VALUE){
	analogWrite(V_REF_R, V_REF_R_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_R, HIGH);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	analogWrite(V_REF_L, V_REF_L_VALUE); // FORCE one direction for right wheel
	digitalWrite(CLOCKWISE_L, HIGH);
	digitalWrite(C_CLOCKWISE_L, LOW);
}	


void stop(){
	//stop
	digitalWrite(CLOCKWISE_R, LOW);
	digitalWrite(C_CLOCKWISE_R, LOW);
	
	digitalWrite(CLOCKWISE_L, LOW);
	digitalWrite(C_CLOCKWISE_L, LOW);
}

DEMO CODE:
voild loop(){
	Rotate_Robot_ClockWise360(100, 100);
	delay(1000);
	Rotate_Robot_ClockWise360(200, 200);
	delay(1000);
	
	stop();
	delay(2000);
	straight(200, 200);
	delay(2000);
	stop();
	delay(2000);
}


#ifdef TEST_MANUAL_CHANGE_DIRECTIONS_RIGHT
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
void loop() {
	Rotate_Robot_ClockWise360( 200, 200);
}
#endif

#ifdef TEST_SPIN_COUNTER_CLOCKWISE
void loop() {
	Rotate_Robot_Counter_ClockWise360( 200, 200);
}
#endif


#ifdef TEST_ENCODER_LEFT //possibly broken.
void loop(){
	
	if(flag == 1){
		straight(50,50); //make it go
		encoder_count_left = 0; //reset the count
		flag = 0;
	}
  Serial.print("left count: ");
  Serial.println(encoder_count_left);
		delay(300);

}
#endif

#ifdef TEST_ENCODER_RIGHT //encoder works.
void loop(){
	if(flag == 1){
		straight(50,50); //make it go
		encoder_count_right = 0; //reset the count
		flag = 0;
	}
  Serial.print("right count: ");
  Serial.println(encoder_count_right);
		delay(300);

}
#endif

#ifdef TEST_STOP
void loop(){
	stop();
}
#endif