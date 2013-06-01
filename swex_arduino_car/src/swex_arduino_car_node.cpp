#include <ros/ros.h>
#include <swex_arduino_car/Order.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

class CarNode {
public:
    CarNode()
    {
        orderPub   = nh.advertise<swex_arduino_car::Order>("car_order", 1);
        statePub   = nh.advertise<std_msgs::String>("arduino_state", 1);
	driverSub  = nh.subscribe("driver_order", 1, &CarNode::driver_callback, this);
	carSub     = nh.subscribe("car_ready", 1, &CarNode::car_callback, this);
    }

private:
    ros::NodeHandle  nh;

    ros::Subscriber  carSub;
    ros::Subscriber  driverSub;
    ros::Publisher   orderPub;
    ros::Publisher   statePub;
    bool             order_transmitted;
    
    void             car_callback(const std_msgs::Empty::ConstPtr& msg)
    {
        order_transmitted = true;
    }

    void driver_callback(const swex_arduino_car::Order::ConstPtr& msg)
    {
        if (!order_transmitted)
        {
            std_msgs::String str_msg;
	    str_msg.data = "Car didn't received last order... Aborting.";
	    statePub.publish(str_msg);
        }
        else
        {
            orderPub.publish(*msg);
	    order_transmitted = false;
	}
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "arduino_car");
    CarNode node;
    ros::spin();

    return 0;
}
