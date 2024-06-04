#ifndef __SUBNPUB_H__
#define __SUBNPUB_H__


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h> 

#include "serial_node/stm2ros_time.h"
class subscriberandpublisher{
    public:
        subscriberandpublisher();
        void callback(const sensor_msgs::ImuConstPtr &imu, const geometry_msgs::TransformStampedConstPtr &vicon);
    private:
        ros::NodeHandle nh;
        ros::Publisher imu_pub;
        ros::Publisher vicon_pub;
        message_filters::Subscriber<sensor_msgs::Imu>  imu_sub; //subscribe imu message
        message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_sub; //subscribe vicon message
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> syncpolicy;
        typedef message_filters::Synchronizer<syncpolicy> Sync;
        boost::shared_ptr<Sync> sync_;
};


#endif

