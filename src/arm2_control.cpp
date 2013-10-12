#include "ros/ros.h"
#include "rgmp/Arm_joint.h"
#include "unistd.h"
#include "sensor_msgs/JointState.h"
#include "boost/algorithm/string.hpp"

using namespace boost;

#define POSBUFLENGTH 20
#define LEFT_ARM 1
#define RIGHT_ARM 2

rgmp::Arm_joint left_msg_pos;
rgmp::Arm_joint right_msg_pos;
rgmp::Arm_joint left_pos_buf[POSBUFLENGTH];
rgmp::Arm_joint right_pos_buf[POSBUFLENGTH];
int count=0;

bool check_pos(int arm);

bool check_pos(int arm){
	int i,j;
	int flag = 1;
	int result = 0;
    
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

}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	int i=0;

	//get the group name of the arm
    left_pos_buf[count].name = trim_right_copy_if(msg->name[1],is_digit());	
    right_pos_buf[count].name = trim_right_copy_if(msg->name[7],is_digit());	
//	left_pos_buf[count].name = left_msg_pos.name;
//	right_pos_buf[count].name = right_msg_pos.name;
    //set the joint data size
    //set the ROS_INFO
	ROS_INFO("I hread: [%s]",left_pos_buf[count].name.c_str());
	ROS_INFO("I hread: [%s]",right_pos_buf[count].name.c_str());

	for(i=0;i<6;i++){
       left_pos_buf[count].joint_data[i] = msg->position[i+1];
	   ROS_INFO("joint %d is [%f]",i,left_pos_buf[count].joint_data[i]);
	}
	for(i=6;i<12;i++){
       right_pos_buf[count].joint_data[i-6] = msg->position[i+1];
	   ROS_INFO("joint %d is [%f]",i,right_pos_buf[count].joint_data[i-6]);
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
	//ros::Rate loop_rate(1);
    ros::Publisher chatter_pub = n.advertise<rgmp::Arm_joint>("chatter",4);
	ros::Subscriber sub = n.subscribe("joint_states",1, jointCallback);

	
	int i;
	for(i=0;i<POSBUFLENGTH;i++){
		left_pos_buf[i].joint_data.resize(6);
		right_pos_buf[i].joint_data.resize(6);
	}
    left_msg_pos.joint_data.resize(6);
    right_msg_pos.joint_data.resize(6);

	while(ros::ok())
	{
        if(check_pos(LEFT_ARM))
			chatter_pub.publish(left_msg_pos);
        if(check_pos(RIGHT_ARM))
			chatter_pub.publish(right_msg_pos);
		ros::spinOnce();
		usleep(5000);
	//	++count;
	}
	return 0;
}


