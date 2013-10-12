#include "ros/ros.h"
#include "rgmp/Pos.h"
#include "unistd.h"
#include "sensor_msgs/JointState.h"

#define POSBUFLENGTH 20

rgmp::Pos msg_pos;
rgmp::Pos pos_buf[POSBUFLENGTH];
int count=0;

bool check_pos();

bool check_pos(){
	int i;
	rgmp::Pos pos_temp;

	pos_temp.joint1 = pos_buf[0].joint1;
	pos_temp.joint2 = pos_buf[0].joint2;
	pos_temp.joint3 = pos_buf[0].joint3;
	pos_temp.joint4 = pos_buf[0].joint4;
	pos_temp.joint5 = pos_buf[0].joint5;
	pos_temp.joint6 = pos_buf[0].joint6;
	
	for(i=0;i<POSBUFLENGTH;i++){
		if(pos_temp.joint1 != pos_buf[i].joint1)
			return false;
		if(pos_temp.joint2 != pos_buf[i].joint2)
			return false;
		if(pos_temp.joint3 != pos_buf[i].joint3)
			return false;
		if(pos_temp.joint4 != pos_buf[i].joint4)
			return false;
		if(pos_temp.joint5 != pos_buf[i].joint5)
			return false;
		if(pos_temp.joint6 != pos_buf[i].joint6)
			return false;
	}

	    msg_pos.joint1 = pos_temp.joint1;	
	    msg_pos.joint2 = pos_temp.joint2;	
	    msg_pos.joint3 = pos_temp.joint3;	
	    msg_pos.joint4 = pos_temp.joint4;	
	    msg_pos.joint5 = pos_temp.joint5;	
	    msg_pos.joint6 = pos_temp.joint6;	
	return true;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	ROS_INFO("I hread: [%f]",msg->position[4]);
	pos_buf[count].joint1 = msg->position[1];
	pos_buf[count].joint2 = msg->position[2];
	pos_buf[count].joint3 = msg->position[3];
	pos_buf[count].joint4 = msg->position[4];
	pos_buf[count].joint5 = msg->position[5];
	pos_buf[count].joint6 = msg->position[6];
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
    ros::Publisher chatter_pub = n.advertise<rgmp::Pos>("chatter",4);
	ros::Subscriber sub = n.subscribe("joint_states",4, jointCallback);

	int count = 1;

	while(ros::ok())
	{
        if(check_pos())
			chatter_pub.publish(msg_pos);
		ros::spinOnce();
		usleep(5000);
		++count;
	}
	return 0;
}


