#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <swex_android_odom/Odometer.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "android_test");

    Odometer od;

    ros::spin();
    return 0;
}
