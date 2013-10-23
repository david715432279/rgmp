#include "ros/ros.h"
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include "rgmp/Arm_joint.h"
#include "rgmp/Robotcontrol.h"
#include "rgmp/robot_control_cmd.h"
#include "unistd.h"
#include "sensor_msgs/JointState.h"
#include "boost/algorithm/string.hpp"
#include "sensor_msgs/Joy.h"
#include "boost/thread.hpp"

using namespace boost;

//now we use ROBOT DOUBLE ARM so free state is 2
#define ROBOT_DOF 6

#define ROBOT_LEFT_ARM_DOF 	6
#define ROBOT_RIGHT_ARM_DOF 6

#define POSBUFLENGTH 		20

#define CLEAN_TARGET 		0
#define ROBOT_HEAD 			1
#define ROBOT_LEFT_ARM 		2
#define ROBOT_RIGHT_ARM 	3
#define ROBOT_MOVE_BUTTON 	4

#define ROBOT_FREE_STATE 	1
#define ROBOT_TEACH_STATE 	2
#define ROBOT_RUN_STATE 	3
#define ROBOT_WRBUSY_STATE 	4

#define ROBOT_HEAD_1_DATA   	0
#define ROBOT_HEAD_2_DATA   	1
#define ROBOT_LEFT_CLAW_DATA   	2
#define ROBOT_RIGHT_CLAW_DATA   3

#define LEFT_ARM 	1
#define RIGHT_ARM 	2

#define ROBOT_RESET 			0x01
#define ROBOT_HEAD_1_MSG 		0x02
#define ROBOT_HEAD_2_MSG 		0x03
#define ROBOT_LEFT_ARM_MSG  	0x04
#define ROBOT_RIGHT_ARM_MSG  	0x05
#define ROBOT_LEFT_CLAW_MSG  	0x06
#define ROBOT_RIGHT_CLAW_MSG  	0x07

#define ROBOT_CONTROL_SPEED     5

#define ROBOT_LEFT_ARM_POS_MAX 		660
#define ROBOT_LEFT_ARM_POS_MIN 		360
#define ROBOT_RIGHT_ARM_POS_MAX 	660
#define ROBOT_RIGHT_ARM_POS_MIN 	360
#define ROBOT_HEAD_1_POS_MAX 		800
#define ROBOT_HEAD_1_POS_MIN 		200
#define ROBOT_HEAD_2_POS_MAX 	    750	
#define ROBOT_HEAD_2_POS_MIN 		200

#define ROBOT_HEAD_SERVO1_NAME      "head_1_servo"
#define ROBOT_HEAD_SERVO2_NAME      "head_2_servo"
#define ROBOT_LEFT_CLAW_NAME        "left_claw"
#define ROBOT_RIGHT_CLAW_NAME       "right_claw"
#define ROBOT_LEFT_ARM_NAME         "left_joint"
#define ROBOT_RIGHT_ARM_NAME        "right_joint"

rgmp::Arm_joint left_msg_pos;
rgmp::Arm_joint right_msg_pos;
rgmp::Arm_joint left_pos_buf[POSBUFLENGTH];
rgmp::Arm_joint right_pos_buf[POSBUFLENGTH];
rgmp::Robotcontrol robot_control_msg; rgmp::Robotcontrol robot_can_msg; 
rgmp::Robotcontrol robot_teach_msg[ROBOT_DOF]; //robot_teach_msg 0 is the left_ARM 1 is the right ARM 

ros::Publisher robot_can_msg_pub;

int count=0; //ARM_joint buffer count
int addpointID = 0; //teach point ID

int robot_state;   //robot buttons state now

//robot now control object
int robot_control_obj = 0;         
//robot head and claw data;
float robot_servo_data[4];

//set robot msg flag
bool robot_control_msg_flag = false;
bool robot_teach_msg_flag = false;
//press_button record the button we enter 0 is no press
int  press_button;
float claw_pos;
float head_speed[2];

std::string teachfile = "robot_teachtest.txt";

bool check_pos(int arm);
void fill_robot_msg(int msg);
void send_robot_pos();     //send robot pos to the robot teach node
void servo_control_function();
bool check_msg_equ(const rgmp::Robotcontrol& a, const rgmp::Robotcontrol& b);

bool check_msg_equ(const rgmp::Robotcontrol& a, const rgmp::Robotcontrol& b){
	int i;
	for(i=0; i<a.length; i++){
		if(a.data[i]!=b.data[i])		
			return false;
	}
	return true;
}

void fill_robot_msg(int msg){
	int i;

	switch(msg){
		case ROBOT_HEAD_1_MSG:	
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_HEAD_SERVO2_NAME;
			robot_can_msg.length = 1;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			robot_can_msg.data[0] = robot_servo_data[ROBOT_HEAD_1_DATA];
			break;
		case ROBOT_HEAD_2_MSG:	
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_HEAD_SERVO2_NAME;
			robot_can_msg.length = 1;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			robot_can_msg.data[0] = robot_servo_data[ROBOT_HEAD_2_DATA];
			break;
		case ROBOT_LEFT_CLAW_MSG:	
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_LEFT_CLAW_NAME;
			robot_can_msg.length = 1;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			robot_can_msg.data[0] = robot_servo_data[ROBOT_LEFT_CLAW_DATA];
			break;
		case ROBOT_RIGHT_CLAW_MSG:	
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_RIGHT_CLAW_NAME;
			robot_can_msg.length = 1;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			robot_can_msg.data[0] = robot_servo_data[ROBOT_RIGHT_CLAW_DATA];
			break;
		case ROBOT_LEFT_ARM_MSG:
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_LEFT_ARM_NAME;
			robot_can_msg.length = ROBOT_LEFT_ARM_DOF;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			for(i=0; i<ROBOT_LEFT_ARM_DOF; i++)
				robot_can_msg.data[i] = left_msg_pos.joint_data[i];
			break;
		case ROBOT_RIGHT_ARM_MSG:
			robot_can_msg.cmd = msg;
			robot_can_msg.name = ROBOT_RIGHT_ARM_NAME;
			robot_can_msg.length = ROBOT_RIGHT_ARM_DOF;
			robot_can_msg.data.resize(robot_can_msg.length);
			robot_can_msg.header.stamp = ros::Time().now();
			for(i=0; i<ROBOT_RIGHT_ARM_DOF; i++)
				robot_can_msg.data[i] = right_msg_pos.joint_data[i];
			break;
		default: 
			return;
	}
	return;
}


void teachCallback(const rgmp::Robotcontrol::ConstPtr& teachmsg){
	int i;
	switch(teachmsg->cmd)
	{
		case RPY_ROBOT_TEACH_BEGIN:
			robot_state = ROBOT_TEACH_STATE;
			ROS_INFO("rgmp node enter the robot teach state!");
			break;
		case RPY_ROBOT_ADD_POINT:
			robot_state = ROBOT_TEACH_STATE;
			ROS_INFO("add one point over!");
			break;
		case RPY_ROBOT_TEACH_END:
			robot_state = ROBOT_FREE_STATE;
			ROS_INFO("rgmp node end teach and enter the free state!");
			break;
		case RPY_ROBOT_RUN_BEGIN:
			robot_state = ROBOT_RUN_STATE;
			ROS_INFO("rgmp node enter the robot run state!");
			break;
		case RPY_ROBOT_READ_POINT:
				robot_can_msg.name = teachmsg->name;
				robot_can_msg.length = teachmsg->length;
				robot_can_msg.data.resize(robot_can_msg.length);
				robot_can_msg.header.stamp = ros::Time().now();
				robot_can_msg.header.seq = teachmsg->header.seq;
				for(i=0 ; i<teachmsg->length; i++)
					robot_can_msg.data[i] = teachmsg->data[i];
			if(teachmsg->name == ROBOT_LEFT_ARM_NAME){
				robot_can_msg.cmd = ROBOT_LEFT_ARM_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the left arm message");
			}else if(teachmsg->name == ROBOT_RIGHT_ARM_NAME){
				robot_can_msg.cmd = ROBOT_RIGHT_ARM_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the right arm message");
			}else if(teachmsg->name == ROBOT_HEAD_SERVO1_NAME){
				robot_can_msg.cmd = ROBOT_HEAD_1_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the robot head servo 1 msg");
			}else if(teachmsg->name == ROBOT_HEAD_SERVO2_NAME){
				robot_can_msg.cmd = ROBOT_HEAD_2_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the robot head servo 2 msg");
			}else if(teachmsg->name == ROBOT_LEFT_CLAW_NAME){
				robot_can_msg.cmd = ROBOT_LEFT_CLAW_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the robot left claw msg");
			}else if(teachmsg->name == ROBOT_RIGHT_CLAW_NAME){
				robot_can_msg.cmd = ROBOT_RIGHT_CLAW_MSG;
				robot_can_msg_pub.publish(robot_can_msg);
				ROS_INFO("send the robot right claw msg");
			}
			//TODO: send the mssage to the real robot
			break;
		case RPY_ROBOT_RUN_END:
			robot_state = ROBOT_FREE_STATE;
			ROS_INFO("rgmp node end run mode and enter the free state!");
			break;

		default:
			ROS_INFO("Receive the unused msg");
	}
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joymsg){
	//select the control object
	if(joymsg->axes[4]==1)
		robot_control_obj = ROBOT_LEFT_ARM; 
	else if(joymsg->axes[4]==-1)
		robot_control_obj = ROBOT_RIGHT_ARM; 
	else if(joymsg->axes[5]==1)
		robot_control_obj = ROBOT_HEAD; 
	else if(joymsg->axes[5]==-1)
		robot_control_obj = ROBOT_MOVE_BUTTON;
	else if(joymsg->buttons[4]==1) //the button 5 on the joystick clean the target 
		robot_control_obj = CLEAN_TARGET;

	//function button
	if(joymsg->buttons[7]==1 && robot_state == ROBOT_FREE_STATE){
		//button 8 enter the teach mode
		//send the teach begin message
		robot_control_msg.cmd = CMD_ROBOT_TEACH_BEGIN;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}else if(joymsg->buttons[6]==1 && (robot_state == ROBOT_TEACH_STATE || robot_state == ROBOT_RUN_STATE)){
		//send the teach end message
		//button 7 end the teach and run mode and enter the free mode
		if(robot_state == ROBOT_TEACH_STATE){
			robot_control_msg.cmd = CMD_ROBOT_TEACH_END;
		}else{
			robot_control_msg.cmd = CMD_ROBOT_RUN_END;
		}
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}
	else if(joymsg->buttons[9]==1 && robot_state == ROBOT_TEACH_STATE){
		//button 10 on joystick add the point 
		 send_robot_pos();
		//robot_control_msg.cmd = CMD_ROBOT_ADD_POINT;
		//robot_control_msg.name = "robot_teachtest.txt";
		//add the robot state point	
	}else if(joymsg->buttons[8]==1 && robot_state == ROBOT_FREE_STATE){
		//button 8 on joystick start the run mode
		// enter the robot run mode	
		robot_control_msg.cmd = CMD_ROBOT_RUN_BEGIN;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}else if(joymsg->buttons[10]==1 && robot_state == ROBOT_RUN_STATE){
		//button 11 on joystick reduction the positon about the claw
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}else if(joymsg->buttons[11]==1){
		//button 12 on joystick increase the positon about the claw
		press_button = 12;
		claw_pos = joymsg->axes[3]+1;
	}else if(joymsg->buttons[10]==1){
		//button 11 on joystick reduction the positon about the claw
		press_button = 11;
		claw_pos = joymsg->axes[3]+1;
	}else if(robot_control_obj==ROBOT_HEAD){
	    head_speed[ROBOT_HEAD_1_DATA] = joymsg->axes[2]; 
	    head_speed[ROBOT_HEAD_2_DATA] = -joymsg->axes[1]; 
	}else if(joymsg->buttons[10] == 0 && joymsg->buttons[11] == 0){
		press_button = 0;
	}
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	int i=0;

	//get the group name of the arm
    left_pos_buf[count].name = trim_right_copy_if(msg->name[1],is_digit());	
    right_pos_buf[count].name = trim_right_copy_if(msg->name[7],is_digit());	
    //set the joint data size
    //set the ROS_INFO
	//ROS_INFO("I hread: left pos buf[%s]",left_pos_buf[count].name.c_str());
	//ROS_INFO("I hread: right pos buf[%s]",right_pos_buf[count].name.c_str());

	//put the leftjoint message in to the pos buffer
	for(i=0;i<6;i++){
       left_pos_buf[count].joint_data[i] = msg->position[i+1];
	  // ROS_INFO("joint %d is [%f]",i,left_pos_buf[count].joint_data[i]);
	}
	//put the rightjoint message in to the pos buffer
	for(i=6;i<12;i++){
       right_pos_buf[count].joint_data[i-6] = msg->position[i+1];
	  // ROS_INFO("joint %d is [%f]",i,right_pos_buf[count].joint_data[i-6]);
	}

	if(count == POSBUFLENGTH){
		count = 0;	
	}else{
        count++;	
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "rgmp");

	boost::thread servo_control_thread(boost::bind(&servo_control_function));
	//boost::thread head_control_thread(boost::bind(&head_control_function));

	ros::NodeHandle n;
  //  ros::Publisher chatter_pub = n.advertise<rgmp::Arm_joint>("arm_control",1);
    ros::Publisher robot_control_pub = n.advertise<rgmp::Robotcontrol>("robot_control_msg",1);
    robot_can_msg_pub = n.advertise<rgmp::Robotcontrol>("robot_can_msg",10);
	ros::Subscriber sub = n.subscribe("joint_states",1, jointCallback);
	ros::Subscriber sub_joy_dev = n.subscribe("joy",1,joyCallback);
	ros::Subscriber sub_teach_node = n.subscribe("robot_teach_rpy",1000,teachCallback);

//init the left and rigth arm joints pos buf	
	int i;

	for(i=0;i<POSBUFLENGTH;i++){
		left_pos_buf[i].joint_data.resize(ROBOT_LEFT_ARM_DOF);
		right_pos_buf[i].joint_data.resize(ROBOT_RIGHT_ARM_DOF);
	}
    left_msg_pos.joint_data.resize(ROBOT_LEFT_ARM_DOF);
    right_msg_pos.joint_data.resize(ROBOT_RIGHT_ARM_DOF);

	robot_state = ROBOT_FREE_STATE;

	//init the left and right claw and head 
	robot_servo_data[ROBOT_HEAD_1_DATA] = 511;   //the init stat about the robot_servo
	robot_servo_data[ROBOT_HEAD_2_DATA] = 511;   //the init stat about the robot_servo
	robot_servo_data[ROBOT_LEFT_CLAW_DATA] = 511;   //the init stat about the robot_servo
	robot_servo_data[ROBOT_RIGHT_CLAW_DATA] = 511;   //the init stat about the robot_servo

	while(ros::ok())
	{
		if(robot_control_msg_flag){
			robot_control_msg.header.stamp = ros::Time().now();
			robot_control_pub.publish(robot_control_msg);
			ros::spinOnce();
			robot_control_msg.header.seq++;
			robot_control_msg_flag = false;
		}
		if(robot_teach_msg_flag){
			for(i=0;i<ROBOT_DOF; i++){
			robot_control_pub.publish(robot_teach_msg[i]);
			usleep(50000);
			}
			robot_teach_msg_flag=false; }
		ros::spinOnce();
	//	usleep(5000);
	}
	return 0;
}

bool check_pos(int arm){
	int i,j;
	rgmp::Arm_joint pos_temp;
    
	if(arm == LEFT_ARM){
		pos_temp = left_pos_buf[0];
		for(i=1;i<POSBUFLENGTH;i++){
			for(j=0;j<6;j++){
				if(pos_temp.joint_data[j] != left_pos_buf[i].joint_data[j]){
					return false;
				}
			}
		}
		left_msg_pos = pos_temp;
		return true;	
	}
	else if(arm == RIGHT_ARM){
		pos_temp = right_pos_buf[0];
		for(i=1;i<POSBUFLENGTH;i++){
			for(j=0;j<6;j++){
				if(pos_temp.joint_data[j] != right_pos_buf[i].joint_data[j]){
					return false;
				}
			}
		}
		right_msg_pos = pos_temp;
		return true;	
	}

	return false;
}

//TODO: the robot state must rewrite becauser the robot state must read from the
//real robot
void send_robot_pos(){
	int i;

   /*if(check_pos(LEFT_ARM)==false || check_pos(RIGHT_ARM)==false){
		ROS_INFO("Error point can't be added!");
		return;
	}*/
	ROS_INFO("I hread: left arm[%s]",left_msg_pos.name.c_str());
	ROS_INFO("I hread: right arm[%s]",right_msg_pos.name.c_str());
		 
	//step1 set the robot state to wrbusy
	robot_state = ROBOT_WRBUSY_STATE;
    //step2 fill the left_msg_pos
	robot_teach_msg[0].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[0].name = ROBOT_LEFT_ARM_NAME;
	robot_teach_msg[0].header.seq = addpointID;
    robot_teach_msg[0].header.stamp = ros::Time().now();
	robot_teach_msg[0].length = ROBOT_LEFT_ARM_DOF;
	robot_teach_msg[0].data.resize(ROBOT_LEFT_ARM_DOF);
	for(i=0;i<ROBOT_LEFT_ARM_DOF;i++)
		robot_teach_msg[0].data[i] = left_msg_pos.joint_data[i];
	
	//step3 fill the right_msg_pos
	robot_teach_msg[1].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[1].name = ROBOT_RIGHT_ARM_NAME;
	robot_teach_msg[1].header.seq = addpointID;
    robot_teach_msg[1].header.stamp = ros::Time().now();
	robot_teach_msg[1].length = ROBOT_RIGHT_ARM_DOF;
	robot_teach_msg[1].data.resize(ROBOT_RIGHT_ARM_DOF);
	for(i=0;i<ROBOT_RIGHT_ARM_DOF;i++)
		robot_teach_msg[1].data[i] = right_msg_pos.joint_data[i];
	
	//step4 fill the head 1_pos
	robot_teach_msg[2].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[2].name = ROBOT_HEAD_SERVO1_NAME;
	robot_teach_msg[2].header.seq = addpointID;
    robot_teach_msg[2].header.stamp = ros::Time().now();
	robot_teach_msg[2].length = 1;
	robot_teach_msg[2].data.resize(1);
	robot_teach_msg[2].data[0] = robot_servo_data[ROBOT_HEAD_1_DATA];

	//step5 fill the head 2_pos
	robot_teach_msg[3].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[3].name = ROBOT_HEAD_SERVO2_NAME;
	robot_teach_msg[3].header.seq = addpointID;
    robot_teach_msg[3].header.stamp = ros::Time().now();
	robot_teach_msg[3].length = 1;
	robot_teach_msg[3].data.resize(1);
	robot_teach_msg[3].data[0] = robot_servo_data[ROBOT_HEAD_2_DATA];

	//step6 fill the left claw data
	robot_teach_msg[4].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[4].name = ROBOT_LEFT_CLAW_NAME;
	robot_teach_msg[4].header.seq = addpointID;
    robot_teach_msg[4].header.stamp = ros::Time().now();
	robot_teach_msg[4].length = 1;
	robot_teach_msg[4].data.resize(1);
	robot_teach_msg[4].data[0] = robot_servo_data[ROBOT_LEFT_CLAW_DATA];

	//step7 fill the right claw data
	robot_teach_msg[5].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[5].name = ROBOT_RIGHT_CLAW_NAME;
	robot_teach_msg[5].header.seq = addpointID;
    robot_teach_msg[5].header.stamp = ros::Time().now();
	robot_teach_msg[5].length = 1;
	robot_teach_msg[5].data.resize(1);
	robot_teach_msg[5].data[0] = robot_servo_data[ROBOT_RIGHT_CLAW_DATA];

	//step8 add the addpointID for next pos
	robot_teach_msg_flag = true;
	addpointID++;
}

void servo_control_function()
{
	float head_pos_last_data[2];

	head_pos_last_data[ROBOT_HEAD_1_DATA] = -1;
	head_pos_last_data[ROBOT_HEAD_2_DATA] = -1;

	while(ros::ok()){
		if(robot_state == ROBOT_RUN_STATE){
			usleep(500000);
			continue;
		}
		if(robot_control_obj == ROBOT_LEFT_ARM && press_button == 12){
			robot_servo_data[ROBOT_LEFT_CLAW_DATA] += claw_pos*ROBOT_CONTROL_SPEED;   //add the stat about the robot_servo
			//check the pos
			if(robot_servo_data[ROBOT_LEFT_CLAW_DATA] >= ROBOT_LEFT_ARM_POS_MAX){
				robot_servo_data[ROBOT_LEFT_CLAW_DATA] = ROBOT_LEFT_ARM_POS_MAX;
			}
		//	ROS_INFO("the left arm position is %f",robot_servo_data[ROBOT_LEFT_CLAW_DATA]);
			fill_robot_msg(ROBOT_LEFT_CLAW_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
			//ros::spinOnce();
			//continue;
			//reduce the claw position	
		}else if(robot_control_obj == ROBOT_LEFT_ARM && press_button == 11){
			robot_servo_data[ROBOT_LEFT_CLAW_DATA] -= claw_pos*ROBOT_CONTROL_SPEED;   //reduce the stat about the robot_servo
			//check the pos
			if(robot_servo_data[ROBOT_LEFT_CLAW_DATA] <= ROBOT_LEFT_ARM_POS_MIN){
				robot_servo_data[ROBOT_LEFT_CLAW_DATA] = ROBOT_LEFT_ARM_POS_MIN;
			}
		//	ROS_INFO("the left arm position is %f",robot_servo_data[ROBOT_LEFT_CLAW_DATA]);
			fill_robot_msg(ROBOT_LEFT_CLAW_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
			//increase the claw position	
		}else if(robot_control_obj == ROBOT_RIGHT_ARM && press_button == 12){
			robot_servo_data[ROBOT_RIGHT_CLAW_DATA] += claw_pos*ROBOT_CONTROL_SPEED;   //add the stat about the robot_servo
			//check the pos
			if(robot_servo_data[ROBOT_RIGHT_CLAW_DATA] >= ROBOT_RIGHT_ARM_POS_MAX){
				robot_servo_data[ROBOT_RIGHT_CLAW_DATA] = ROBOT_RIGHT_ARM_POS_MAX;
			}
		//	ROS_INFO("the right arm position is %f",robot_servo_data[ROBOT_RIGHT_CLAW_DATA]);
			fill_robot_msg(ROBOT_RIGHT_CLAW_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
			
			//reduce the claw position	
		}else if(robot_control_obj == ROBOT_RIGHT_ARM && press_button == 11){
			robot_servo_data[ROBOT_RIGHT_CLAW_DATA] -= claw_pos*ROBOT_CONTROL_SPEED;   //reduce the stat about the robot_servo
			//check the pos
			if(robot_servo_data[ROBOT_RIGHT_CLAW_DATA] <= ROBOT_RIGHT_ARM_POS_MIN){
				robot_servo_data[ROBOT_RIGHT_CLAW_DATA] = ROBOT_RIGHT_ARM_POS_MIN;
			}
		//	ROS_INFO("the right arm position is %f",robot_servo_data[ROBOT_RIGHT_CLAW_DATA]);
			fill_robot_msg(ROBOT_RIGHT_CLAW_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
			//increase the claw position	
		}

		if(robot_control_obj == ROBOT_HEAD){
			robot_servo_data[ROBOT_HEAD_1_DATA] += head_speed[ROBOT_HEAD_1_DATA]*(ROBOT_CONTROL_SPEED*2);
			robot_servo_data[ROBOT_HEAD_2_DATA] += head_speed[ROBOT_HEAD_2_DATA]*(ROBOT_CONTROL_SPEED);
			//step 1 check the pos
			if(robot_servo_data[ROBOT_HEAD_1_DATA] >= ROBOT_HEAD_1_POS_MAX){
				robot_servo_data[ROBOT_HEAD_1_DATA] = ROBOT_HEAD_1_POS_MAX;
			}else if(robot_servo_data[ROBOT_HEAD_1_DATA] <= ROBOT_HEAD_1_POS_MIN){
				robot_servo_data[ROBOT_HEAD_1_DATA] = ROBOT_HEAD_1_POS_MIN;
			}

			if(robot_servo_data[ROBOT_HEAD_2_DATA] >= ROBOT_HEAD_2_POS_MAX){
				robot_servo_data[ROBOT_HEAD_2_DATA] = ROBOT_HEAD_2_POS_MAX;
			}else if(robot_servo_data[ROBOT_HEAD_2_DATA] <= ROBOT_HEAD_2_POS_MIN){
				robot_servo_data[ROBOT_HEAD_2_DATA] = ROBOT_HEAD_2_POS_MIN;
			}
			//step 2 reflash the data about the last data push the new position
			if(robot_servo_data[ROBOT_HEAD_2_DATA] != head_pos_last_data[ROBOT_HEAD_2_DATA]){
				head_pos_last_data[ROBOT_HEAD_2_DATA] = robot_servo_data[ROBOT_HEAD_2_DATA];
		//		ROS_INFO("the head 2 position is %f",robot_servo_data[ROBOT_HEAD_2_DATA]);
				fill_robot_msg(ROBOT_HEAD_2_MSG);
				robot_can_msg_pub.publish(robot_can_msg);
			}else if(robot_servo_data[ROBOT_HEAD_1_DATA] != head_pos_last_data[ROBOT_HEAD_1_DATA]){
				head_pos_last_data[ROBOT_HEAD_1_DATA] = robot_servo_data[ROBOT_HEAD_1_DATA];
			//	ROS_INFO("the head 1 position is %f",robot_servo_data[ROBOT_HEAD_1_DATA]);
				fill_robot_msg(ROBOT_HEAD_1_MSG);
				robot_can_msg_pub.publish(robot_can_msg);
			}
		}

		if(robot_control_obj == ROBOT_LEFT_ARM && check_pos(LEFT_ARM)){
			fill_robot_msg(ROBOT_LEFT_ARM_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
		}else if(robot_control_obj == ROBOT_RIGHT_ARM && check_pos(RIGHT_ARM)){
			fill_robot_msg(ROBOT_RIGHT_ARM_MSG);
			robot_can_msg_pub.publish(robot_can_msg);
		}

		ros::spinOnce();
		usleep(50000);
	}
}
