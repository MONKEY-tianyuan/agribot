#include "subnpub.h"


subscriberandpublisher::subscriberandpublisher(){

    imu_sub.subscribe(nh,"/imu/data",1);
    vicon_sub.subscribe(nh,"/filter/positionlla",1);
    //vicon_sub.subscribe(nh,"/vicon/Agribot/Agribot",1);
	encoder_sub.subscribe(nh,"/encoder",1);
    quanternion_sub.subscribe(nh,"/filter/quaternion",1);
    
    imu_pub = nh.advertise<sensor_msgs::Imu>("sync/imu",1000);
    vicon_pub = nh.advertise<geometry_msgs::Vector3Stamped>("sync/vicon",1000);
    encoder_pub = nh.advertise<serial_node::stm2ros_tao>("sync/encoder",1000);
    quanternion_pub = nh.advertise<geometry_msgs::QuaternionStamped>("sync/quaternion",1000);
    gather_pub = nh.advertise<time_synchronization::sync_gather>("/sync/gather",1000);
    sync_.reset(new Sync(syncpolicy(30),imu_sub,vicon_sub,quanternion_sub,encoder_sub));
    sync_->registerCallback(boost::bind(&subscriberandpublisher::callback, this, _1,_2,_3,_4));

}

void subscriberandpublisher::callback(const sensor_msgs::ImuConstPtr &imu, const geometry_msgs::Vector3StampedConstPtr &vicon, const geometry_msgs::QuaternionStampedConstPtr &quanternion,const serial_node::stm2ros_taoConstPtr &encoder){
    ROS_INFO("I heard...");
    imu_pub.publish(imu);
    vicon_pub.publish(vicon);
    encoder_pub.publish(encoder);
    quanternion_pub.publish(quanternion);


    time_synchronization::sync_gather msg;
    msg.header.stamp.sec = vicon->header.stamp.sec;
    msg.header.stamp.nsec = vicon->header.stamp.nsec;

    msg.transform.translation.x = vicon->vector.x;
    msg.transform.translation.y = vicon->vector.y;
    msg.transform.translation.z = vicon->vector.z;

    msg.transform.rotation.x = quanternion->quaternion.x;
    msg.transform.rotation.y = quanternion->quaternion.y;
    msg.transform.rotation.z = quanternion->quaternion.z;
    msg.transform.rotation.w = quanternion->quaternion.w;

    
    msg.angular_velocity.x = imu->angular_velocity.x;
    msg.angular_velocity.y = imu->angular_velocity.y;
    msg.angular_velocity.z = imu->angular_velocity.z;
    
    msg.linear_acceleration.x = imu->linear_acceleration.x;
    msg.linear_acceleration.y = imu->linear_acceleration.y;
    msg.linear_acceleration.z  =imu->linear_acceleration.z;
    msg.encoder3 = encoder->encoder3;
    msg.encoder1 = encoder->encoder4;

    // memcpy(&msg.header, &vicon->header,sizeof(&vicon->header));
    // memcpy(&msg.transform, &vicon->transform, sizeof(&vicon->transform));
    // memcpy(&msg.angular_velocity,&imu->angular_velocity, sizeof(&imu->angular_velocity));
    // memcpy(&msg.linear_acceleration,&imu->linear_acceleration,sizeof(&imu->linear_acceleration));
    gather_pub.publish(msg);
    //ROS_INFO("imu: %f,%f",msg.linear_acceleration.x,msg.linear_acceleration.y);
}
