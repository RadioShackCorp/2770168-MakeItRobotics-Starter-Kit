// * Author             : RadioShack Corporation
// * Version            : V1.0
// * Date               : 2014/01/16 modified
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

#include "Arduino.h"
#include "MakeItRobotics.h"

#define IR_0_LOWER_LIMIT	250
#define IR_0_UPPER_LIMIT	550
#define IR_1_LOWER_LIMIT	900
#define IR_1_UPPER_LIMIT	1400
unsigned long current_IR_time;
unsigned long diff_IR_time;
byte incount=0;  
unsigned long last_IR_time=0;
unsigned char OirDataHigh, OirDataLow; 
unsigned int CirDataHigh, CirDataLow; 
unsigned char even=0; 
long repeatTimer1 = millis();
long repeatTimer2 = millis();  


byte incomingByte = 0;
int incomingCnt = 0;
boolean msgStart = false;
boolean msgEnd   = false;
int msgHeader = 0;
int msgId     = 0;
int msgValue  = 0;

void MakeItRobotics::line_following_setup(void)
{
  dc_write(DC_CMD_IR_TX1, SW_ON);
  dc_write(DC_CMD_IR_TX2, SW_ON);  
}

void MakeItRobotics::walking_robot_setup(void)
{
  dc_write(DC_CMD_IR_TX1, SW_ON);
  dc_write(DC_CMD_IR_TX2, SW_ON);  
}

void MakeItRobotics::sensor_kit_optical_setup(void)
{
  dc_write(DC_CMD_IR_TX3, SW_ON);
}

void MakeItRobotics::trigger_optical1(void)
{
  dc_write(DC_CMD_IR_RX1, 0x00);
}

void MakeItRobotics::trigger_optical2(void)
{
  dc_write(DC_CMD_IR_RX2, 0x00);
}

void MakeItRobotics::trigger_optical3(void)
{
  dc_write(DC_CMD_IR_RX3, 0x00);
}

int MakeItRobotics::read_optical(void)
{
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (msgStart && incomingCnt == 2)
    {
      msgValue = incomingByte;
      incomingCnt = 3;
      msgEnd = true;
      break;
    }
    if (msgStart && incomingCnt == 1)
    {
      msgId = incomingByte;
      incomingCnt = 2;
    }  
    if (incomingByte == DC_RECV_HEADER && !msgStart)
    {
      msgHeader = incomingByte;
      incomingCnt = 1;
      msgStart = true;
    }
  }
  if (msgStart && msgEnd)
  {
    msgStart = false;
    msgEnd = false;
    incomingCnt = 0;  
    if (msgId == DC_CMD_IR_RX1)
      msgValue &= 0xff;
    if (msgId == DC_CMD_IR_RX2)
      msgValue |= 0x100;
    return msgValue;
  }
  else
  {
    return 0x200;
  }
}

int MakeItRobotics::read_optical3(void)
{
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (msgStart && incomingCnt == 2)
    {
      msgValue = incomingByte;
      incomingCnt = 3;
      msgEnd = true;
      break;
    }
    if (msgStart && incomingCnt == 1)
    {
      msgId = incomingByte;
      incomingCnt = 2;
    }  
    if (incomingByte == DC_RECV_HEADER && !msgStart)
    {
      msgHeader = incomingByte;
      incomingCnt = 1;
      msgStart = true;
    }
  }
  if (msgStart && msgEnd)
  {
    msgStart = false;
    msgEnd = false;
    incomingCnt = 0;  
    if (msgId == DC_CMD_IR_RX3)
      msgValue &= 0xff;
    return msgValue;
  }
  else
  {
    return 0x200;
  }
}

void MakeItRobotics::line_following_turn_left(int speed)
{
  int duty = 0;
  int half = 0;
  if(speed == 0)
  {
    duty = 0;
    half = 0;
  }
  else if(speed >= 255)
  {
    duty = 1;
    half = 128;
  }
  else
  {
    duty = 257 - speed;
    half = 257 - speed*3/4;
  }
  dc_write(DC_CMD_DIRA, BW);
  dc_write(DC_CMD_DIRB, FW);
  dc_write(DC_CMD_PWMA, half);
  dc_write(DC_CMD_PWMB, duty);
}

void MakeItRobotics::line_following_turn_right(int speed)
{
  int duty = 0;
  int half = 0;
  if(speed == 0)
  {
    duty = 0;
    half = 0;
  }
  else if(speed >= 255)
  {
    duty = 1;
    half = 128;
  }
  else
  {
    duty = 257 - speed;
    half = 257 - speed*3/4;
  }
  dc_write(DC_CMD_DIRA, FW);
  dc_write(DC_CMD_DIRB, BW);
  dc_write(DC_CMD_PWMA, duty);
  dc_write(DC_CMD_PWMB, half);
}

void MakeItRobotics::dc_write(int type, int value)
{
  Serial.write(DC_SEND_HEADER);
  Serial.write(type);
  Serial.write(value);
  delay(20);
}

void MakeItRobotics::walking_robot_right_leg_forward(int speed)
{
  m1_action(BW, speed);		
}

void MakeItRobotics::walking_robot_left_leg_forward(int speed)
{	
  m2_action(BW, speed);
}

void MakeItRobotics::walking_robot_right_leg_backward(int speed)
{
  m1_action(FW, speed);		
}

void MakeItRobotics::walking_robot_left_leg_backward(int speed)
{		
  m2_action(FW, speed);
}

void MakeItRobotics::m1_action(int dir, int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRA, dir ? FW : BW);
  dc_write(DC_CMD_PWMA, speed);
}

void MakeItRobotics::m2_action(int dir, int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRB, dir ? FW : BW);
  dc_write(DC_CMD_PWMB, speed);
}

void MakeItRobotics::go_forward(int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRA, FW);
  dc_write(DC_CMD_DIRB, FW);
  dc_write(DC_CMD_PWMA, speed);
  dc_write(DC_CMD_PWMB, speed);
}

void MakeItRobotics::go_backward(int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRA, BW);
  dc_write(DC_CMD_DIRB, BW);
  dc_write(DC_CMD_PWMA, speed);
  dc_write(DC_CMD_PWMB, speed);
}

void MakeItRobotics::turn_left(int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRA, BW);
  dc_write(DC_CMD_DIRB, FW);
  dc_write(DC_CMD_PWMA, speed);
  dc_write(DC_CMD_PWMB, speed);
}

void MakeItRobotics::turn_right(int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRA, FW);
  dc_write(DC_CMD_DIRB, BW);
  dc_write(DC_CMD_PWMA, speed);
  dc_write(DC_CMD_PWMB, speed);
}

void MakeItRobotics::turn_front_left(int speed)
{
  int duty = 0;
  int half = 0;
  if(speed == 0)
  {
    duty = 0;
    half = 0;
  }
  else if(speed >= 255)
  {
    duty = 1;
    half = 128;
  }
  else
  {
    duty = 257 - speed;
    half = 257 - speed / 2;
  }
  dc_write(DC_CMD_DIRA, FW);
  dc_write(DC_CMD_DIRB, FW);
  dc_write(DC_CMD_PWMA, half);
  dc_write(DC_CMD_PWMB, duty);
}	

void MakeItRobotics::turn_front_right(int speed)   
{
  int duty = 0;
  int half = 0;
  if(speed == 0)
  {
    duty = 0;
    half = 0;
  }
  else if(speed >= 255)
  {
    duty = 1;
    half = 128;
  }
  else
  {
    duty = 257 - speed;
    half = 257 - speed / 2;
  }  dc_write(DC_CMD_DIRA, FW);
  dc_write(DC_CMD_DIRB, FW);
  dc_write(DC_CMD_PWMA, duty);
  dc_write(DC_CMD_PWMB, half);
}

void MakeItRobotics::move_stop()
{
  dc_write(DC_CMD_PWMA, 0);
  dc_write(DC_CMD_PWMB, 0);
}

void MakeItRobotics::m3_action(int dir, int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRC, dir ? FW : BW);
  dc_write(DC_CMD_PWMC, speed);
}

void MakeItRobotics::m4_action(int dir, int speed)
{
  if (speed >= 255)
    speed = 1;
  else if(speed!=0)
    speed = 256 - speed;
  dc_write(DC_CMD_DIRD, dir ? FW : BW);
  dc_write(DC_CMD_PWMD, speed);
}

void MakeItRobotics::bulldozer_head_up(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::bulldozer_head_down(int speed)
{
  m3_action(BW,speed);  
}

void MakeItRobotics::drilling_head_clockwise(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::drilling_head_counterclockwise(int speed)
{
  m3_action(BW,speed); 
}

void MakeItRobotics::surveillance_head_clockwise(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::surveillance_head_counterclockwise(int speed)
{
  m3_action(BW,speed); 
}

void  MakeItRobotics::boxer_left_hand_up(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::boxer_left_hand_down(int speed)
{
  m3_action(BW,speed);
}

void MakeItRobotics::boxer_right_hand_up(int speed)
{
  m4_action(FW,speed);
}

void MakeItRobotics::boxer_right_hand_down(int speed)
{
  m4_action(BW,speed);
}

void MakeItRobotics::street_sweeper_inward(int speed)
{
  m3_action(BW,speed);
  m4_action(BW,speed);  
}

void MakeItRobotics::street_sweeper_outward(int speed)
{
  m3_action(FW,speed);
  m4_action(FW,speed); 
}

void MakeItRobotics::clampingarm_up(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::clampingarm_down(int speed)
{
  m3_action(BW,speed);
}

void MakeItRobotics::clampingarm_clamp(int speed)
{
  m4_action(FW,speed);
}

void MakeItRobotics::clampingarm_release(int speed)
{
  m4_action(BW,speed);
}

void MakeItRobotics::catapult_head_clockwise(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::catapult_head_counterclockwise(int speed)
{
  m3_action(BW,speed);
}

void MakeItRobotics::catapult_head_pull(int speed)
{
  m4_action(BW,speed);
}

void MakeItRobotics::catapult_head_throw(int speed)
{
  m4_action(FW,speed);
}

void MakeItRobotics::spotlight_head_clockwise(int speed)
{
  m3_action(FW,speed);
}

void MakeItRobotics::spotlight_head_counterclockwise(int speed)
{
  m3_action(BW,speed);
}

void MakeItRobotics::spotlight_head_up(int speed)
{
  m4_action(BW,speed);
}

void MakeItRobotics::spotlight_head_down(int speed)
{
  m4_action(FW,speed);
}

void MakeItRobotics::spotlight_setup(void)
{
  pinMode(PIN_LED1, OUTPUT);    
  pinMode(PIN_LED2, OUTPUT); 
  pinMode(PIN_LED3, OUTPUT); 
  pinMode(PIN_LED4, OUTPUT);     
}

void MakeItRobotics::spotlight_on(void)
{
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  digitalWrite(PIN_LED3, HIGH);
  digitalWrite(PIN_LED4, HIGH);
}

void MakeItRobotics::spotlight_off(void)
{
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  digitalWrite(PIN_LED3, LOW);
  digitalWrite(PIN_LED4, LOW);
}

void MakeItRobotics::all_stop()
{
  dc_write(DC_CMD_PWMA, 0);
  dc_write(DC_CMD_PWMB, 0);
  dc_write(DC_CMD_PWMC, 0);
  dc_write(DC_CMD_PWMD, 0);
}

void MakeItRobotics::remote_setup()
{
  pinMode(10, INPUT);
  digitalWrite(10,HIGH);  
  PCICR|=(1<<PCIE0);
  PCMSK0|=(1<<PCINT2);
  MCUCR=(1<<ISC01)|(1<<ISC00);  
}

unsigned int MakeItRobotics::remote_value_read(void)
{
  repeatTimer2 = millis();
  if(repeatTimer2-repeatTimer1<300)
  {
      if(irRxFlag==true)
      {
        irRxFlag=false;
        irBits=0; 
      }
  }
  else
  {
     CirDataHigh=0;  
  }
  return CirDataHigh;
}

void MakeItRobotics::remote_scan(void)
{

  current_IR_time=micros();
  diff_IR_time=current_IR_time-last_IR_time;
  last_IR_time=current_IR_time;
  if(diff_IR_time>1500)
  {
    irBits=0;
    irDataLow=0;          
    irDataHigh=0;    
  }  
  if(irBits<24)
  {
    if(irBits%2)
    {   
      if (diff_IR_time>IR_0_LOWER_LIMIT && diff_IR_time<IR_0_UPPER_LIMIT)        
      {
        if(irBits<12)
        {
          irDataLow<<=1;
	  irDataLow&=0xFE;          
        }
        else
        {
          irDataHigh<<=1;
	  irDataHigh&=0xFE;          
        }        
      }
      else if (diff_IR_time>IR_1_LOWER_LIMIT && diff_IR_time<IR_1_UPPER_LIMIT)
      {
        if(irBits<12)
        {
          irDataLow<<=1;
	  irDataLow|=0x01;          
        }
        else
        {
          irDataHigh<<=1;
	  irDataHigh|=0x01;          
        }        
      }               
    }  
    irBits++;
    if(irBits==24)
    { 
      if(irDataLow==50)  //--------->ONES發生
      {
        CirDataHigh=irDataHigh;
        CirDataLow=irDataLow;
        CirDataHigh=CirDataHigh*256+CirDataLow;
        repeatTimer1 = millis();  
        irRxFlag=true;    
      }
      else
      {     
        if(even==0)
        {
          OirDataLow=irDataLow;          
          OirDataHigh=irDataHigh;     
          irBits=0;
          irDataLow=0;          
          irDataHigh=0;
          even=1;
        }
        else if(even==1)  
        {
          even=0;
          if(OirDataLow==irDataLow &&  OirDataHigh==irDataHigh )//--------->CONTINUES發生
          {
            CirDataHigh=irDataHigh;
            CirDataLow=irDataLow;
            CirDataHigh=CirDataHigh*256+CirDataLow;
            repeatTimer1 = millis();  
            irRxFlag=true;
          }
        }     
      }
    }  
  } 
}
void MakeItRobotics::sensor_kit_mic_setup(void)
{
  pinMode(10, INPUT);
}

int MakeItRobotics::read_mic(void)
{
  return digitalRead(10);
}

void MakeItRobotics::sensor_kit_infrared_setup(void)
{
  pinMode(10, INPUT);
}

int MakeItRobotics::read_infrared(int sec)
{
  int dly=0;
  if(sec>2000)
    sec=2000;
  dly=0;
  while(digitalRead(10)==0 && dly<sec)
  {
    dly++;
    delay(1);
  }  
  if(dly<sec)    
    return 1;
  else
    return 0;
}
