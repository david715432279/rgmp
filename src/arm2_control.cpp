#include "ros/ros.h"
#include "rgmp/Arm_joint.h"
#include "rgmp/Robotcontrol.h"
#include "rgmp/robot_control_cmd.h"
#include "unistd.h"
#include "sensor_msgs/JointState.h"
#include "boost/algorithm/string.hpp"
#include "sensor_msgs/Joy.h"

using namespace boost;

//now we use ROBOT DOUBLE ARM so free state is 2
#define ROBOT_DOF 2

#define ROBOT_LEFT_ARM_DOF 6
#define ROBOT_RIGHT_ARM_DOF 6

#define POSBUFLENGTH 20

#define CLEAN_TARGET 0
#define ROBOT_HEAD 1
#define ROBOT_LEFT_ARM 2
#define ROBOT_RIGHT_ARM 3
#define ROBOT_MOVE_BUTTON 4

#define ROBOT_FREE_STATE 1
#define ROBOT_TEACH_STATE 2
#define ROBOT_RUN_STATE 3
#define ROBOT_WRBUSY_STATE 4

#define LEFT_ARM 1
#define RIGHT_ARM 2

rgmp::Arm_joint left_msg_pos;
rgmp::Arm_joint right_msg_pos;
rgmp::Arm_joint left_pos_buf[POSBUFLENGTH];
rgmp::Arm_joint right_pos_buf[POSBUFLENGTH];
rgmp::Robotcontrol robot_control_msg;
rgmp::Robotcontrol robot_teach_msg[ROBOT_DOF];
//robot_teach_msg 0 is the left_ARM 1 is the right ARM

ros::Publisher robot_control_pub;

int count=0; //ARM_joint buffer count
int addpointID = 0; //teach point ID

int robot_state;   //robot buttons state now

//robot now control object
int robot_control_obj = 0;         
//set robot msg flag
bool robot_control_msg_flag = false;
bool robot_teach_msg_flag = false;

std::string teachfile = "robot_teachtest.txt";

bool check_pos(int arm);
void send_robot_pos();

void send_robot_pos(){
	int i;

    if(check_pos(LEFT_ARM)==false && check_pos(RIGHT_ARM)==false){
		ROS_INFO("Error point can't be added!");
		return;
	}
	ROS_INFO("I hread: left arm[%s]",left_msg_pos.name.c_str());
	ROS_INFO("I hread: right arm[%s]",right_msg_pos.name.c_str());
		 
	//step1 set the robot state to wrbusy
	robot_state = ROBOT_WRBUSY_STATE;
    //step2 fill the left_msg_pos
	robot_teach_msg[0].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[0].name = left_msg_pos.name;
	robot_teach_msg[0].header.seq = addpointID;
    robot_teach_msg[0].header.stamp = ros::Time().now();
	robot_teach_msg[0].length = ROBOT_LEFT_ARM_DOF;
	robot_teach_msg[0].data.resize(ROBOT_LEFT_ARM_DOF);
	for(i=0;i<ROBOT_LEFT_ARM_DOF;i++)
		robot_teach_msg[0].data[i] = left_msg_pos.joint_data[i];
	
	//step3 fill the right_msg_pos
	robot_teach_msg[1].cmd = CMD_ROBOT_ADD_POINT;
	robot_teach_msg[1].name = right_msg_pos.name;
	robot_teach_msg[1].header.seq = addpointID;
    robot_teach_msg[1].header.stamp = ros::Time().now();
	robot_teach_msg[1].length = ROBOT_RIGHT_ARM_DOF;
	robot_teach_msg[1].data.resize(ROBOT_RIGHT_ARM_DOF);
	for(i=0;i<ROBOT_RIGHT_ARM_DOF;i++)
		robot_teach_msg[1].data[i] = right_msg_pos.joint_data[i];
	
	//TODO:add the other msg about the robot_state
	//step4 add the addpointID for next pos
	robot_teach_msg_flag = true;
	addpointID++;
}

void teachCallback(const rgmp::Robotcontrol::ConstPtr& teachmsg){
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
			//TODO: send the mssage to the real robot
			ROS_INFO("send the message");
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
	if(joymsg->axes[4]==1)
		robot_control_obj = ROBOT_LEFT_ARM; 
	else if(joymsg->axes[4]==-1)
		robot_control_obj = ROBOT_RIGHT_ARM; 
	else if(joymsg->axes[5]==1)
		robot_control_obj = ROBOT_HEAD; 
	else if(joymsg->axes[5]==-1)
		robot_control_obj = ROBOT_MOVE_BUTTON;
	else if(joymsg->buttons[4]==1)
		robot_control_obj = CLEAN_TARGET;

	if(joymsg->buttons[7]==1 && robot_state == ROBOT_FREE_STATE){
		//send the teach begin message
		robot_control_msg.cmd = CMD_ROBOT_TEACH_BEGIN;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}
	else if(joymsg->buttons[6]==1 && robot_state == ROBOT_TEACH_STATE){
		//send the teach end message
		robot_control_msg.cmd = CMD_ROBOT_TEACH_END;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}
	else if(joymsg->buttons[9]==1 && robot_state == ROBOT_TEACH_STATE){
		 send_robot_pos();
		//robot_control_msg.cmd = CMD_ROBOT_ADD_POINT;
		//robot_control_msg.name = "robot_teachtest.txt";
		//add the robot state point	
	}
	else if(joymsg->buttons[11]==1 && robot_state == ROBOT_FREE_STATE){
		// enter the robot run mode	
		robot_control_msg.cmd = CMD_ROBOT_RUN_BEGIN;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
	}
	else if(joymsg->buttons[10]==1 && robot_state == ROBOT_RUN_STATE){
		// end the run mode
		// go to the free state
		robot_control_msg.cmd = CMD_ROBOT_RUN_END;
		robot_control_msg.name = teachfile; 
		robot_control_msg_flag = true;
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
	//ROS_INFO("I hread: [%s]",left_pos_buf[count].name.c_str());
	//ROS_INFO("I hread: [%s]",right_pos_buf[count].name.c_str());

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
	ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<rgmp::Arm_joint>("arm_control",1);
    ros::Publisher robot_control_pub = n.advertise<rgmp::Robotcontrol>("robot_control_msg",1);
	ros::Subscriber sub = n.subscribe("joint_states",1, jointCallback);
	ros::Subscriber sub_joy_dev = n.subscribe("joy",1,joyCallback);
	ros::Subscriber sub_teach_node = n.subscribe("robot_teach_rpy",1,teachCallback);

//init the left and rigth pos buf	
	int i;

	for(i=0;i<POSBUFLENGTH;i++){
		left_pos_buf[i].joint_data.resize(ROBOT_LEFT_ARM_DOF);
		right_pos_buf[i].joint_data.resize(ROBOT_RIGHT_ARM_DOF);
	}
    left_msg_pos.joint_data.resize(ROBOT_LEFT_ARM_DOF);
    right_msg_pos.joint_data.resize(ROBOT_RIGHT_ARM_DOF);

	robot_state = ROBOT_FREE_STATE;

	while(ros::ok())
	{
		if(robot_control_msg_flag){
			robot_control_msg.header.stamp = ros::Time().now();
			robot_control_pub.publish(robot_control_msg);
			ros::spinOnce();
			robot_control_msg.header.seq++;
			robot_control_msg_flag = false;
		}
//		ROS_INFO("good");
        if(check_pos(LEFT_ARM)){
			chatter_pub.publish(left_msg_pos);
			ros::spinOnce();
		}
        if(check_pos(RIGHT_ARM)){
			chatter_pub.publish(right_msg_pos);
			ros::spinOnce();
		}
		if(robot_teach_msg_flag){
			robot_control_pub.publish(robot_teach_msg[0]);
//			ros::spinOnce();
			robot_control_pub.publish(robot_teach_msg[1]);
			ros::spinOnce();
			robot_teach_msg_flag=false;
		}
		ros::spinOnce();
	//	usleep(5000);
	}
	return 0;
}

bool check_pos(int arm){
	int i,j;
    
	if(arm == LEFT_ARM){
		rgmp::Arm_joint pos_temp = left_pos_buf[0];
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
	if(arm == RIGHT_ARM){
		rgmp::Arm_joint pos_temp = right_pos_buf[0];
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
