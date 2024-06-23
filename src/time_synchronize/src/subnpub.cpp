#include "subnpub.h"


subscriberandpublisher::subscriberandpublisher(){

    imu_sub.subscribe(nh,"/os1_cloud_node/imu",1);
    vicon_sub.subscribe(nh,"/vicon/Agribot/Agribot",1);

    imu_pub = nh.advertise<sensor_msgs::Imu>("sync/imu",1000);
    vicon_pub = nh.advertise<geometry_msgs::TransformStamped>("sync/vicon",1000);

    sync_.reset(new Sync(syncpolicy(10),imu_sub,vicon_sub));
    sync_->registerCallback(boost::bind(&subscriberandpublisher::callback, this, _1,_2));

}

void subscriberandpublisher::callback(const sensor_msgs::ImuConstPtr &imu, const geometry_msgs::TransformStampedConstPtr &vicon){
    ROS_INFO("done! ");
    imu_pub.publish(imu);
    vicon_pub.publish(vicon);
}