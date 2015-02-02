//********************************************************************************
// * File Name          : walk.ino
// * Author             : RadioShack Corporation
// * Version            : V1.0
// * Date               : 2014/02/28
// * Description        : Optical sensors on each leg send feedback to core PCB to enable walking function
// ********************************************************************************
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
// ********************************************************************************

#include<MakeItRobotics.h> //include library
#define IDLE            0
#define STARTING        1
#define MOVING          2
#define DEFAULT_SPEED	150
#define IR_HIGH	        1
#define IR_LOW	        0
#define CYCLE  	        400
#define CYCLE_UPPER		401
#define CYCLE_LOWER		400
#define SPEED_LOWER_LIMIT	150
#define SPEED_UPPER_LIMIT	220
#define SLIDE_DURATION	5 

// --------------------------------------------------------------------------------------------
// ------ Fine-tune optical sensor stopping position (0-70)  																				-----
// ------ Default value is 10; larger value creates a larger offset from sensor
// --------------------------------------------------------------------------------------------

#define	left_stop_position	10	
#define right_stop_position	10	

MakeItRobotics Walkingrobot;//declare object
void setup()
{
  Serial.begin(10420);                 //tell the Arduino to communicate with Make: it PCB
  Walkingrobot.walking_robot_setup();  //initialize the status of the walking robot
  Walkingrobot.all_stop();             //all motors stop
  delay(500);                          //delay 500ms
}

void loop()
{
  static int sensor_in;                 //variable to store the value of read_optical function feedback   
  static int sensorCnt = 0;             //variable to count for trigger which optical
  static long sensorTimer1 = millis();  //last triggered time
  static long sensorTimer2 = millis();  //now time
  int leftIR_current;
  int left_status;
  long left_start_time;
  int left_speed;
  int left_duration;

  int rightIR_current;
  int right_status;
  long right_start_time;
  int right_speed;
  int right_duration;
  // ---------------------------------
  // ---------- Init. Motor ----------
  // ---------------------------------
  left_speed=DEFAULT_SPEED;
  right_speed=DEFAULT_SPEED;

  left_speed=150;  // default left leg on
  Walkingrobot.walking_robot_left_leg_forward(left_speed);
  left_status=STARTING;
  left_start_time = millis();

  right_speed=150;  // default right leg off
  Walkingrobot.walking_robot_right_leg_forward(0);
  right_status=IDLE;
	
  Walkingrobot.trigger_optical1();  // read optical sensor Rx1 first
  // ------------------------------------------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------  Main Loop ------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------------------------
  // - Left and right leg alternate movements
  // - Moving leg stops when the IR sensor detects the tape. Then the other leg starts moving.
  // - When sensor times out, assume the leg is not moving; increase driving force and retry the same leg.
  // - When cycle time is too long, assume speed is slow, increase driving force (PWM)
  // - When cycle time is too short, assuming speed is fast, decrease driving force (PWM)
  // ************************************************
  // ******** Read Optical Sensor from OpAmp ********
  // ************************************************
  while (1)
  {
    sensor_in=Walkingrobot.read_optical();
    
    if ((sensor_in & 0xf00)==0)
    {
      rightIR_current=sensor_in & 0xff;
      if (rightIR_current == 0x0)
        rightIR_current =IR_HIGH;
      else
        rightIR_current =IR_LOW;      
    }  
    else if((sensor_in & 0xf00)>>8==1) 
    {
      leftIR_current=sensor_in & 0xff;
      if (leftIR_current == 0x0)
        leftIR_current =IR_LOW;
      else
        leftIR_current =IR_HIGH;      
    }  
    sensorTimer2 = millis();                 //read now time
    if (sensorTimer2 - sensorTimer1 > 15)    //if now time minus last triggered time is greater than 15ms, then trigger the other optical sensor
    {
      sensorTimer1 = sensorTimer2;           //last triggered time = now time
      /***********************************************************************
         -> trigger optical -> greater than 15ms -> trigger optical2 -> greater than 15ms ->|
         |----------------------------------------------------------------------------------|
      ***********************************************************************/
      if (sensorCnt == 0)
        Walkingrobot.trigger_optical1();
      else if (sensorCnt == 1)
        Walkingrobot.trigger_optical2();
      sensorCnt++;
      if (sensorCnt == 2)
        sensorCnt = 0;
    }
    // ---------------------------------------------------------------------------------------------------
    // --------------------------------------------- Left Leg -------------------------------------------
    // ---------------------------------------------------------------------------------------------------
    // ***********************************
    // ********** Check Duration *********
    // ***********************************		
    left_duration= abs( millis() - left_start_time);// if duration too long without trigger => overshoot => stop and reduce speed

    if ( (left_duration > CYCLE*2) && (left_status !=IDLE) )
    {
      if (left_speed<(SPEED_UPPER_LIMIT+30))
      {
        left_speed=left_speed+30;	
	Walkingrobot.walking_robot_left_leg_forward(left_speed);
	left_status=STARTING;
	left_start_time = millis();				
       }
    }
    // ************************************
    // ********** Left IDLE State *********
    // ************************************
    if (left_status == IDLE)
    {
      // waiting for right leg
    }
    // ****************************************
    // ********** Left STARTING State ********
    // ****************************************
    else if (left_status == STARTING)
    {
      if (leftIR_current==IR_HIGH)// wait until left leg moves away from sensor
      { 
        left_status = MOVING;
      }
    }
    // **************************************
    // ********** Left MOVING State *********
    // **************************************
    else 																						// Moving now
    {
      if ((leftIR_current == IR_LOW) && (left_duration>200))// Optical Sensor Trigger
      {
        delay(SLIDE_DURATION);
        delay(left_stop_position);// trim stop position. fine-tune the left_stop_position on line 41
        Walkingrobot.walking_robot_left_leg_backward(150);// BRAKE 
	delay (10);// right leg				
        Walkingrobot.walking_robot_left_leg_backward(0);// Stop left leg
	left_status = IDLE;
	Walkingrobot.walking_robot_right_leg_forward(right_speed);// Start right leg
	right_status=STARTING;
	right_start_time = millis();
	if (left_duration > CYCLE_UPPER)// if duration too long but trigger => speed slow => increase PWM
	{
          if (left_speed<(SPEED_UPPER_LIMIT+20))
	    left_speed=left_speed+20;				
	}
	else if (left_duration < CYCLE_LOWER)// if duration too short but trigger => speed high => reduce PWM
	{
          if (left_speed>(SPEED_LOWER_LIMIT-10))
	    left_speed=left_speed-10;
	}
      }			
    }
    // ----------------------------------------------------------------------------------------------------
    // --------------------------------------------- Right Leg --------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    // ***********************************
    // ********** Check Duration *********
    // ***********************************
    right_duration= abs( millis() - right_start_time);// if duration too long without trigger => overshoot => stop and reduce speed
    if ( (right_duration > CYCLE*2) && (right_status !=IDLE) )
    {
      if (right_speed<(SPEED_UPPER_LIMIT+30))
      {
        right_speed=right_speed+30;
	Walkingrobot.walking_robot_right_leg_forward(right_speed);
	right_status=STARTING;
	right_start_time = millis();				
      }		
    }
    // *************************************
    // ********** Right IDLE State **********
    // *************************************
    if (right_status == IDLE)
    {
      // waiting for left leg
    }
    // *****************************************
    // ********** RIGHT STARTING State *********
    // *****************************************
    else if (right_status == STARTING)
    {
      if (rightIR_current == IR_HIGH)// wait until right leg moves away from sensor
      { 
        right_status = MOVING;
      }
    }
    else 
    {
      if ((rightIR_current == IR_LOW) && (right_duration>200))// Optical Sensor Trigger
      {
        delay(SLIDE_DURATION);
	delay (right_stop_position);// trim stop position. fine-tune the right_stop_position on line 42
	Walkingrobot.walking_robot_right_leg_backward(150);													// BRAKE
	delay (10);	
	Walkingrobot.walking_robot_right_leg_backward(0);// Stop right leg
	right_status = IDLE;				
	Walkingrobot.walking_robot_left_leg_forward(left_speed);// Start left leg
	left_status=STARTING;
	left_start_time = millis();
	if (right_duration > CYCLE_UPPER)// if duration too long but trigger => speed slow => increase PWM
	{
	  if (right_speed<(SPEED_UPPER_LIMIT+20))
	    right_speed=right_speed+20;
        }
	else if (right_duration < CYCLE_LOWER)// if duration too short but trigger => speed high => reduce PWM
	{
	  if (right_speed>(SPEED_LOWER_LIMIT-10))
	    right_speed=right_speed-10;
	}
      }
    }
  }  
}
