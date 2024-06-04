
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "DecodeFrame.hpp"

#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 1024
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

serial::Serial ser;
DecodeFrame tmp_frame();


typedef union
{
	float data;
	unsigned char data8[4];
} data_u;

data_u vel_1;
data_u vel_2;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"serial_node");
	ros::NodeHandle nh;
	try
	{
		{/* code */
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to=serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();}
	}
	catch(serial::IOException &e)
	{
		ROS_INFO_STREAM("Failed to open port ");
		return -1;
	}
	if (ser.isOpen()){
		ROS_INFO_STREAM("Succeed to open port ");
		}
	else{
		return -1;
	}
		int cnt = 0;
		ros::Rate loop_rate(100);
		//ros::Subscriber vel_pub = nh.
	
	while(ros::ok()){
		unsigned char senddata[5]={0x11,0x02,0x03,0x04,0x05};
		ser.write(senddata,5);
		loop_rate.sleep();
	}
	ser.close();
	return 0;

}


