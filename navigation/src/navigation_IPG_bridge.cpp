#include "ros/ros.h"
#include "vehiclecontrol_msgs/VehicleControl_msgs.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "sensor_msgs/Imu.h"
#include "PID.h"
#include "nav_msgs/Odometry.h"

#define PID_KP 0.5f
#define PID_KI 0.05f
#define PID_KD 0.015f
#define PID_TAU 0.02f
#define PID_LIM_MIN -1.0f
#define PID_LIM_MAX 1.0f
#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f
#define SAMPLE_TIME_S 0.1f


  PIDController pid = {PID_KP, PID_KI, PID_KD,
                       PID_TAU,
                       PID_LIM_MIN, PID_LIM_MAX,
                       PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                       SAMPLE_TIME_S};

float Throttle = 0;
float Brake = 0;
float  velocity_setpoint = 0.0f;
float desired_steering_angle = 0;
float current_velocity = 0.0f;
vehiclecontrol_msgs::VehicleControl_msgs Control_msg;

void Bridge_CB(const nav_msgs::Odometry &msg){

velocity_setpoint = msg.twist.twist.linear.x;
desired_steering_angle = msg.twist.twist.linear.z;
current_velocity = msg.twist.twist.linear.y;

}

void Bridge_Callback_state_estimation(const sensor_msgs::Imu::ConstPtr &msg)
{
 current_velocity = sqrt(pow(msg->linear_acceleration.x, 2) + pow(msg->linear_acceleration.y, 2));
 
}




int main(int argc, char **argv)
{
ros::init(argc, argv, "IPG_Bridge");
ros::NodeHandle n;
ros::Publisher Bridge_pub;
ros::Subscriber Bridge_sub_control;
ros::Subscriber Bridge_sub_velocity;

  ROS_INFO("Running");
  Bridge_pub = n.advertise<vehiclecontrol_msgs::VehicleControl_msgs>("/carmaker/VehicleControl", 1000);
  Bridge_sub_control = n.subscribe("sub_control_actions", 1000, Bridge_CB);
  //Bridge_sub_velocity = n.subscribe("/IMU", 1000, Bridge_Callback_state_estimation);
  ros::Rate loop_rate(10);
  Control_msg.use_vc = 1;
  Bridge_pub.publish(Control_msg);
  while (ros::ok())
  { 

  PIDController_Update(&pid, velocity_setpoint, current_velocity);
    if (pid.out > 0)
    {
      Brake = 0;
      Throttle = pid.out;
      Control_msg.use_vc = 1;
      Control_msg.gas = Throttle;
      Control_msg.brake = Brake;
    }
    else
    {
      Throttle = 0;
      Brake = -1 * pid.out;
      Control_msg.use_vc = 1;
      Control_msg.gas = Throttle;
      Control_msg.brake = Brake;
    }
    Control_msg.steer_ang = desired_steering_angle * 6.66666666;
    Bridge_pub.publish(Control_msg);
    ROS_INFO("Publishing Vehicle control");
    loop_rate.sleep();
    ros::spinOnce();

  }
}
