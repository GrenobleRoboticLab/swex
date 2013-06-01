#include <ros.h>
#include <swex_arduino_car/Order.h>
#include <std_msgs/Empty.h>

#define TM_LEFT  10
#define TM_RIGHT 11
#define SM_AHEAD 12
#define SM_BACK  13

#define POWER_TIME 10

void order_callback(const swex_arduino_car::Order & order);

void apply_order();

void rotate_turn_motor();
void rotate_speed_motor();

void stop_turn_motor();
void stop_speed_motor();

ros::NodeHandle          nh;
std_msgs::Empty          empty_msg;
swex_arduino_car::Order  current_order;

ros::Publisher pub("car_ready", &empty_msg);
ros::Subscriber<swex_arduino_car::Order>  sub("car_order", &order_callback);

void setup()
{
  pinMode(TM_LEFT, OUTPUT); 
  pinMode(TM_RIGHT, OUTPUT); 
  pinMode(SM_AHEAD, OUTPUT); 
  pinMode(SM_BACK, OUTPUT); 

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pub.publish(&empty_msg);

  current_order.speedState = current_order.directionState = current_order.speedRate = 0;

  stop_turn_motor();
  stop_speed_motor();
}

void loop()
{
  nh.spinOnce();
  apply_order();
  delay(1);
}

void order_callback(const swex_arduino_car::Order & order)
{
  current_order = order;
  pub.publish(&empty_msg);
}

void apply_order()
{
  if (current_order.speedRate != 0)
  {
    int wait_time   = (1000 / current_order.speedRate) - POWER_TIME;
    
    rotate_turn_motor();
    rotate_speed_motor();
    
    delay(wait_time);
    
    stop_speed_motor();
  }
  else
  {
    stop_speed_motor();
    stop_turn_motor();
  }
}

void rotate_turn_motor()
{
  if (current_order.directionState > 0)
  {
    digitalWrite(TM_RIGHT, LOW);
    digitalWrite(TM_LEFT, HIGH);
  }
  else if (current_order.directionState < 0)
  {
    digitalWrite(TM_LEFT, LOW);
    digitalWrite(TM_RIGHT, HIGH);
  }
  else
    stop_turn_motor();
}

void rotate_speed_motor()
{
  if (current_order.speedState > 0)
  {
    digitalWrite(SM_BACK, LOW);
    digitalWrite(SM_AHEAD, HIGH);
  }
  else if (current_order.speedState < 0)
  {
    digitalWrite(SM_AHEAD, LOW);
    digitalWrite(SM_BACK, HIGH);
  }
  else
    stop_speed_motor();
}

void stop_turn_motor()
{
  digitalWrite(TM_LEFT, LOW);
  digitalWrite(TM_RIGHT, LOW);
}

void stop_speed_motor()
{
  digitalWrite(SM_AHEAD, LOW);
  digitalWrite(SM_BACK, LOW);
}
