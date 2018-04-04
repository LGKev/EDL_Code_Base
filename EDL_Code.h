/*
	Author: Kevin Kuwata
	Kevinkuwata.come
	Github: LGKev
	Created: 4/3/18
	
	Last modified: 4/3/18
	
	EDL_Code.h
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


/*
	@name: count_Right
	@brief: This is an ISR. counts encoder pulses on GPIO 2
	@input: [hardware] input from encoder
	@output: none
	@global: encoder_count_left
	@return: none
*/
void count_Left();

/*
	@name: count_Right
	@brief: This is an ISR. counts encoder pulses on GPIO 3
	@input: [hardware] input from encoder
	@output: none
	@globals: encoder_count_right
*/
void count_Right();





/*
	@name: Rotate_Robot_ClockWise360
	@brief: Rotate robot 360 degrees independent of encoder
	@inputs: V_REF_L_VALUE, V_REF_R_VALUE between 0 and 255
	@output: none
	@return: none
	
	IMPORTANT: independent of encoder.
*/
void Rotate_Robot_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE);
/*
	@name: Rotate_Robot_Counter_ClockWise360
	@brief: Rotate robot 360 degrees independent of encoder
	@inputs: V_REF_L_VALUE, V_REF_R_VALUE between 0 and 255
	@output: none
	@return: none
	IMPORTANT: independent of encoder.
*/
void Rotate_Robot_Counter_ClockWise360( byte V_REF_L_VALUE, byte V_REF_R_VALUE);

/*
	@name: void straight(byte V_REF_L_VALUE, byte V_REF_R_VALUE)
	@brief: makes the robot go straight if the input values are equal. 
	@inputs: V_REF_L_VALUe, V_REF_R_VALUE between 0 and 255
	@outputs: none
	
	Note: it may be the case that the motors require different ref voltages, hence the
	two inputs. If motor and encoders were identical we could just use 1 value.
*/
void straight(byte V_REF_L_VALUE, byte V_REF_R_VALUE);

/*
	@name: stop()
	@brief: stops the motors by setting CLOCKWISE_L, CLOCKWISE_R, C_CLOCKWISE_L, C_CLOCKWISE_R to LOW
	@input: none
	@output: [hardware] sets the pins low, check #defines at top of file for pinout.
	@return: none
*/
void stop();