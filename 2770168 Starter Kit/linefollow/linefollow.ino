//********************************************************************************
// * File Name          : linefollow
// * Author             : RadioShack Corporation
// * Version            : V1.0
// * Date               : 2014/01/16
// * Description        : Optical sensors send feedback to PCB 
// *                      to make the robot follow a black line on a white background.
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

#include <MakeItRobotics.h>//include library
MakeItRobotics line_following;//declare object
// **************************************************************************
// *                            Power Up Initial
// **************************************************************************
void setup() 
{ 
  Serial.begin(10420);                     //tell the Arduino to communicate with Make: it PCB
  delay(500);                              //delay 500ms
  line_following.line_following_setup();   //initialize the status of line following robot
  line_following.all_stop();               //all motors stop
}
// **************************************************************************
// *                            Main Loop 
// **************************************************************************
void loop() 
{  
  static int sensor_in;                  //variable to store the value of read_optical function feedback 
  static int sensorValue1 = 0;           //variable to store optical1 status
  static int sensorValue2 = 0;           //variable to store optical2 status
  static int sensorCnt = 0;              //variable to count for trigger which optical
  static long sensorTimer1 = millis();   //last triggered time
  static long sensorTimer2 = millis();   //now time
  static int action1 = 0;                //now action
  static int action2 = 0;                //last action
  //************************************************************************
  //  Trigger Left/Right optical every 15 milliseconds
  //************************************************************************ 
  sensorTimer2 = millis();                 //read now time
  if (sensorTimer2 - sensorTimer1 > 15)    //if now time minus last triggered time is greater than 15ms, then trigger another optical
  {
    sensorTimer1 = sensorTimer2;           //last triggered time = now time
    /***********************************************************************
       -> trigger optical1 -> greater than 15ms -> trigger optical2 -> greater than 15ms ->|
       |-----------------------------------------------------------------------------------|
    ***********************************************************************/
    if (sensorCnt == 0)
      line_following.trigger_optical1();
    else if (sensorCnt == 1)
      line_following.trigger_optical2();
    sensorCnt++;
    if (sensorCnt == 2)
      sensorCnt = 0;
  }            
  //***********************************************************************
  //  Read Left/Right optical status
  //***********************************************************************
  sensor_in=line_following.read_optical();  
  /************************************************************************
    read_optical()
    Description
      Reads the value from optical1(Right side) or optical2(Left side)
    Syntax
      read_optical()
    Parameters
      none
    Returns
      0x000  optical1 black
      0x0ff  optical1 white
      0x100  optical1 white
      0x1ff  optical1 black
      0x2XX  not ready; don't use this value      
  *************************************************************************/
  if((sensor_in & 0xf00)==0)
    sensorValue1=sensor_in & 0xff;
  else if((sensor_in & 0xf00)>>8==1)  
    sensorValue2=sensor_in & 0xff;
  if (sensorValue1 == 0x00)
    action1 =action1 & 0xfe;
  if (sensorValue1 == 0xFF)
    action1 = action1 | 0x01;
  if (sensorValue2 == 0x00)
    action1 = action1 | 0x02;
  if (sensorValue2 == 0xFF)
    action1 = action1 & 0xfd;  
  /************************************************************************
    action1
             left        right
    0x00    black        black
    0x01    black        white
    0x02    white        black
    0x03    white        white
  *************************************************************************/  
  /************************************************************************
     Make Robot Move
     if action1 is not equal to action2, then change motor status
     if action1 is equal to action2, then do nothing
  *************************************************************************/
  if(action1 != action2)
  {
    if (action1 == 3 )
      line_following.go_forward(50);
    if (action1 == 1)
      line_following.line_following_turn_left(50);    
    if (action1 == 2)
      line_following.line_following_turn_right(50);  
    if (action1 == 0)
      line_following.go_forward(50);  
  }
  action2=action1;  
}

