#ifndef __SUBNPUB_H__
#define __SUBNPUB_H__


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h> 
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "time_synchronization/sync_gather.h"
#include "serial_node/stm2ros_tao.h"
class subscriberandpublisher{
    public:
        subscriberandpublisher();
        void callback(const sensor_msgs::ImuConstPtr &imu, const geometry_msgs::Vector3StampedConstPtr &vicon,const geometry_msgs::QuaternionStampedConstPtr &quanternion,const serial_node::stm2ros_taoConstPtr &encoder);
    private:
        ros::NodeHandle nh;
        ros::Publisher imu_pub;
        ros::Publisher vicon_pub;
        ros::Publisher encoder_pub;
        ros::Publisher quanternion_pub;
        ros::Publisher gather_pub;
        message_filters::Subscriber<sensor_msgs::Imu>  imu_sub; //subscribe imu message
        message_filters::Subscriber<geometry_msgs::Vector3Stamped> vicon_sub; //subscribe vicon message
        message_filters::Subscriber<serial_node::stm2ros_tao> encoder_sub;
        message_filters::Subscriber<geometry_msgs::QuaternionStamped> quanternion_sub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped,geometry_msgs::QuaternionStamped, serial_node::stm2ros_tao> syncpolicy;
        typedef message_filters::Synchronizer<syncpolicy> Sync;
        boost::shared_ptr<Sync> sync_;
};


#endif

