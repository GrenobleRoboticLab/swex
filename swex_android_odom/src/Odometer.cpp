#include "swex_android_odom/Odometer.h"


Odometer::Odometer()
{
    m_Sub       = m_NodeHandle.subscribe("android/imu", 1, &Odometer::imu_cb, this);
    m_Pub       = m_NodeHandle.advertise<swex_msgs::MapperOdom>("car_odom", 1);
    m_bReady    = false;

    // extrem value compute in genMinMaxAcc();
    m_MinLAcc.x = 100;
    m_MinLAcc.y = 100;
    m_MinLAcc.z = 100;

    m_MaxLAcc.x = -100;
    m_MaxLAcc.y = -100;
    m_MaxLAcc.z = -100;
}

Odometer::~Odometer() { ; }

void Odometer::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!m_bReady)
    {
        m_vBaseAcc.push_back(msg->linear_acceleration);
        genMinMaxAcc();
    }

    if (( msg->header.stamp - m_LastTime).toSec() > 0.1)
    {
        geometry_msgs::Vector3 v3 = linearAccFilter(msg->linear_acceleration);
        computeOrientation(msg->orientation);
        computeDeltaPose(v3, msg->header.stamp);
        computeSpeed(v3, msg->header.stamp);

        m_LastTime = msg->header.stamp;

        if (m_bReady)
            m_Pub.publish(m_Odom);
    }
}

void Odometer::genMinMaxAcc()
{
    for (uint i = 0; i < m_vBaseAcc.size(); i++)
    {
        if (m_vBaseAcc[i].x < m_MinLAcc.x)
            m_MinLAcc.x = m_vBaseAcc[i].x;
        if (m_vBaseAcc[i].y < m_MinLAcc.y)
            m_MinLAcc.y = m_vBaseAcc[i].y;
        if (m_vBaseAcc[i].z < m_MinLAcc.z)
            m_MinLAcc.z = m_vBaseAcc[i].z;

        if (m_vBaseAcc[i].x > m_MaxLAcc.x)
            m_MaxLAcc.x = m_vBaseAcc[i].x;
        if (m_vBaseAcc[i].y > m_MaxLAcc.y)
            m_MaxLAcc.y = m_vBaseAcc[i].y;
        if (m_vBaseAcc[i].z > m_MaxLAcc.z)
            m_MaxLAcc.z = m_vBaseAcc[i].z;
    }

    if (m_vBaseAcc.size() == BASE_SIZE)
    {
        m_bReady = true;
        std::cout << "Min Linear Acceleration : X = " << m_MinLAcc.x << " Y = " << m_MinLAcc.y << " Z = " << m_MinLAcc.z << std::endl;
        std::cout << "Max Linear Acceleration : X = " << m_MaxLAcc.x << " Y = " << m_MaxLAcc.y << " Z = " << m_MaxLAcc.z << std::endl;
    }
}

geometry_msgs::Vector3 Odometer::linearAccFilter(const geometry_msgs::Vector3 &linear_acceleration)
{
    geometry_msgs::Vector3 ret;

    ret.x = (inRange(linear_acceleration.x, m_MinLAcc.x, m_MaxLAcc.x)) ? 0 : linear_acceleration.x;
    ret.y = (inRange(linear_acceleration.y, m_MinLAcc.y, m_MaxLAcc.y)) ? 0 : linear_acceleration.y;
    ret.z = (inRange(linear_acceleration.z, m_MinLAcc.z, m_MaxLAcc.z)) ? 0 : linear_acceleration.z;

    return ret;
}

void Odometer::computeOrientation(const geometry_msgs::Quaternion & quat)
{
    double dAlphaFXYZ = 2. * asin(sqrt(pow(quat.x, 2) + pow(quat.y, 2) + pow(quat.z, 2))),
           dAlphaFW   = 2. * acos(quat.w);

    if (quat.z < 0)
    {
        dAlphaFXYZ *= -1;
        dAlphaFW   *= -1;
    }

    m_Odom.orientation = (dAlphaFXYZ + dAlphaFW) / 2.;
}

void Odometer::computeDeltaPose(const geometry_msgs::Vector3 &linear_acceleration, const ros::Time &time)
{
    ros::Duration deltaT = time - m_LastTime;
    // deltaPose = (Acc * deltaT²) / 2; => resultat en m => * 100 pour un resultat en cm
    m_Odom.pose.x = ((linear_acceleration.x * pow(deltaT.toSec(), 2)) / 2.) * 100;
    m_Odom.pose.y = ((linear_acceleration.y * pow(deltaT.toSec(), 2)) / 2.) * 100;
    m_Odom.pose.z = ((linear_acceleration.z * pow(deltaT.toSec(), 2)) / 2.) * 100;
}

void Odometer::computeSpeed(const geometry_msgs::Vector3 &linear_acceleration, const ros::Time &time)
{
    ros::Duration deltaT = time - m_LastTime;
    if ((linear_acceleration.x == 0)
     && (linear_acceleration.y == 0)
     && (linear_acceleration.z == 0))
        m_Odom.speed = 0.;
    m_Odom.speed += (linear_acceleration.x * deltaT.toSec()) + (linear_acceleration.y * deltaT.toSec()) + (linear_acceleration.z * deltaT.toSec());
}
