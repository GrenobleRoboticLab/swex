#ifndef ODOMETER_H
#define ODOMETER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <swex_msgs/MapperOdom.h>

#define BASE_SIZE 25

template<typename T>
bool inRange(T val, T min, T max) { return (min <= val && val <= max); }

class Odometer {
public:
    Odometer();
    ~Odometer();

private:
    ros::NodeHandle                     m_NodeHandle;
    ros::Subscriber                     m_Sub;
    ros::Publisher                      m_Pub;
    ros::Time                           m_LastTime;

    std::vector<geometry_msgs::Vector3> m_vBaseAcc;
    geometry_msgs::Vector3              m_MinLAcc;
    geometry_msgs::Vector3              m_MaxLAcc;

    swex_msgs::MapperOdom             m_Odom;

    bool                                m_bReady;

    void                                imu_cb(const sensor_msgs::Imu::ConstPtr & msg);
    void                                genMinMaxAcc();
    geometry_msgs::Vector3              linearAccFilter(const geometry_msgs::Vector3 & linear_acceleration);
    void                                computeOrientation(const geometry_msgs::Quaternion &quat);
    void                                computeDeltaPose(const geometry_msgs::Vector3 & linear_acceleration, const ros::Time & time);
    void                                computeSpeed(const geometry_msgs::Vector3 & linear_acceleration, const ros::Time & time);

};

#endif // ODOMETER_H
