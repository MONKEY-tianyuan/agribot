#include "subnpub.h"
#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc,argv,"node");
    subscriberandpublisher sp;
    ROS_INFO("main done! ");
    ros::spin();
}