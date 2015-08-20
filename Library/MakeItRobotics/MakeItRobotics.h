// * Author             : RadioShack Corporation
// * Version            : V1.0
// * Date               : 2014/01/16 modified
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
#define DC_CMD_IR_RX1    0x60
#define DC_CMD_IR_RX2    0x61
#define DC_CMD_IR_RX3    0x62
#define DC_CMD_IR_TX1    0x70
#define DC_CMD_IR_TX2    0x71
#define DC_CMD_IR_TX3    0x72
#define SW_ON        0xff

#define DC_SEND_HEADER   0x56
#define DC_RECV_HEADER   0x76
#define DC_CMD_DIRA      0x73
#define DC_CMD_DIRB      0x74
#define DC_CMD_DIRC      0x75
#define DC_CMD_DIRD      0x76
#define DC_CMD_PWMA      0x80
#define DC_CMD_PWMB      0x81
#define DC_CMD_PWMC      0x82
#define DC_CMD_PWMD      0x83
#define FW               0xff
#define BW               0x00
#define PIN_LED1    8     //LED control
#define PIN_LED2    11    //LED control
#define PIN_LED3    12    //LED control
#define PIN_LED4    13    //LED control

#define SW1    0x2034
#define SW2    0x1034
#define SW3    0x0834
#define SW4    0x0434
#define SW5    0x0234
#define SW6    0x0134
#define SW7    0x2032
#define SW8    0x1032
#define SW51   0x2234
#define SW61   0x2134
#define SW53   0x0A34
#define SW63   0x0934
#define CONT   0x34
#define ONES   0x32

class MakeItRobotics
{
  public:
    //Function of send command
    void dc_write(int type, int value);  //send command to CORE PCBA

    //Functions of bsaic move
    //The left motor is M1
    //The right motor is M2
    void m1_action(int dir,int speed);	//M1 ACTION
    void m2_action(int dir,int speed);  //M2 ACTION
    void go_forward(int speed);		//left and right motors (M1 and M2) forward
    void go_backward(int speed);	//left and right motors (M1 and M2) forward
    void turn_left(int speed);		//left motor (M1) backward and right motor (M2) forward
    void turn_right(int speed);		//left motor (M1) forward and right motor (M2) backward
    void turn_front_left(int speed);	//left and right motors (M1 and M2) forward with right motor (M2) at twice the speed of left motor (M1)
    void turn_front_right(int speed);   //left and right motors (M1 and M2) forward with left motor (M1) at twice the speed of right motor (M2)
    void move_stop();			////left and right motors (M1 and M2) stop

    //Functios of M3/M4 behaviors
    void m3_action(int dir,int speed);	//M3 ACTION
    void m4_action(int dir,int speed);  //M4 ACTION

    //Bulldozer
    void bulldozer_head_up(int speed);
    void bulldozer_head_down(int speed);

    //Drilling
    void drilling_head_clockwise(int speed);
    void drilling_head_counterclockwise(int speed);

    //Surveillance
    void surveillance_head_clockwise(int speed);
    void surveillance_head_counterclockwise(int speed);

    //Boxer
    void boxer_left_hand_up(int speed);
    void boxer_left_hand_down(int speed);
    void boxer_right_hand_up(int speed);
    void boxer_right_hand_down(int speed);

    //Street_sweeper
    void street_sweeper_inward(int speed);
    void street_sweeper_outward(int speed);

    //Clampingarm
    void clampingarm_up(int speed);
    void clampingarm_down(int speed);
    void clampingarm_clamp(int speed);
    void clampingarm_release(int speed);

    //Catapult
    void catapult_head_clockwise(int speed);
    void catapult_head_counterclockwise(int speed);
    void catapult_head_pull(int speed);
    void catapult_head_throw(int speed);

    //Spotlight
    void spotlight_head_clockwise(int speed);
    void spotlight_head_counterclockwise(int speed);
    void spotlight_head_up(int speed);
    void spotlight_head_down(int speed);
    void spotlight_setup(void);
    void spotlight_on(void);
    void spotlight_off(void);

    //line_following
    void line_following_setup(void);
    void trigger_optical1(void);
    void trigger_optical2(void);
    int read_optical(void);
    void line_following_turn_left(int speed);
    void line_following_turn_right(int speed);

    //walking_robot
    void walking_robot_right_leg_forward(int speed);
    void walking_robot_left_leg_forward(int speed);
    void walking_robot_right_leg_backward(int speed);
    void walking_robot_left_leg_backward(int speed);
    void walking_robot_setup(void);

    void all_stop();	

    //Remote 2770173
    void remote_setup();
    void remote_scan(void);
    unsigned int remote_value_read(void);
    byte irDataHigh;
    byte irDataLow;
    byte irBits;
    boolean irRxFlag; 

    //2770172
    void sensor_kit_optical_setup(void);
    void trigger_optical3(void);
    int read_optical3(void);
  
    void sensor_kit_mic_setup(void);
    int read_mic(void);

    void sensor_kit_infrared_setup(void);
    int read_infrared(int sec);
};
