#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include<iostream>

#include "string.h"
#include <std_msgs/Header.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int8.h>

#include "vehiclecontrol_msgs/VehicleControl_msgs.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "navigation/Control_actions.h"
#include "navigation/state_control.h"


enum move_command {stop=0, easy_stop=1, move=2};
enum states { null=3, move_pp=4, move_mpc=5, move_pp_mpc=6 };    //the genral states of navigation
enum plot { no_plotting=7, plotting=8};
enum map {no_map=9, not_complete=10, complete=11};
enum state_estimation { not_ready=12, ready=13};
enum system_diagnostic {not_finished=14, finished=15}; 


move_command move_state = stop;
states car_state = null ;            // init the car to stop state for safety
plot plot_state = no_plotting;
map map_state = not_complete;
state_estimation state_estimation_state = not_ready;
system_diagnostic system_diagnostic_state = not_finished;



class navigation_handler
{


public:
    bool first_check();
    void control_act_sub_cb(const navigation::Control_actions& contorl_act_msg);
    void SP_sub_cb(const navigation::state_control& SP_msg);
    

    void run();
    //void status_sub_cb();
    void spinOnce(){ros::spinOnce();}
    void spin(){ros::spin();}

    navigation_handler(/* args */);
    ~navigation_handler();
    navigation::Control_actions control_actions;

private:


    int dump_var ;    //for converting int to enums
    //std_msgs::Int8 subcontrol_msg; //msg between navigation_handler and (mpc, pure_pursuit, plotter)
    navigation::state_control subcontrol_msg;   //msg between navigation_handler and (mpc, pure_pursuit, plotter)
    vehiclecontrol_msgs::VehicleControl_msgs ipg_Control_msg_pub;


    ros::NodeHandle nh,nh_private;
    //ros::Subscriber status_substatus_sub ;
    ros::Publisher ipg_control_act_pub ;
    ros::Publisher can_control_act_pub;
    ros::Publisher model_control_act_pub;
    ros::Subscriber status_substatus_sub;
    ros::Subscriber control_actions_controllers_sub;
    ros::Subscriber SP_sub;
    ros::Publisher subcontrol_msg_pub ;
    ros::Publisher subcontrol_plot_msg_pub ;
    ros::Publisher control_actions_arduino_steering; 
    ros::Rate loop_rate;
    //ros::Rate errors_loop_rate(1);

};

navigation_handler::navigation_handler(/* args */): loop_rate(20)
{
    dump_var =0;
    //ros::Subscriber status_substatus_sub = nh.subscribe("status_msg", 1, &receivePath);
    ipg_control_act_pub = nh.advertise<vehiclecontrol_msgs::VehicleControl_msgs>("/VehicleControl", 1000, true);
    can_control_act_pub = nh.advertise<vehiclecontrol_msgs::VehicleControl_msgs>("/VehicleControl", 1000, true); // to be edited to fit car can
    //status_substatus_sub = nh.subscribe("cmd_acker", 10, &navigation_handler::control_act_sub_cb, this);
    SP_sub = nh.subscribe("SP_state_control", 1000, &navigation_handler::SP_sub_cb, this);     //deal with ganzo about topic name
    control_actions_controllers_sub= nh.subscribe("sub_control_actions", 1000, &navigation_handler::control_act_sub_cb, this);
    //subcontrol_msg_pub = nh.advertise<std_msgs::Int8>("subcontrol_msg", 100);
    subcontrol_msg_pub = nh.advertise<navigation::state_control>("state_control_msg", 1000, true);
    model_control_act_pub = nh.advertise<navigation::Control_actions>("control_actions", 1000, true);

    subcontrol_plot_msg_pub = nh.advertise<std_msgs::Int8>("subcontrol_plot_msg", 1000), true;
    control_actions_arduino_steering = nh.advertise<std_msgs::Float32>("/angle", 1000), true;
    //ros::Rate loop_rate(1);
    //ros::Rate errors_loop_rate(1);

    //nh_private_.param<double>("wheelbase", L_, 1.0);
    // setting genral parameters for enums
    nh_private.getParam("move_command",dump_var);
    subcontrol_msg.move_command = static_cast<move_command>(dump_var);
    
    nh_private.getParam("states",dump_var);
    subcontrol_msg.states = static_cast<states>(dump_var);

    nh_private.getParam("plot",dump_var);
    subcontrol_msg.plot = static_cast<plot>(dump_var);

    nh_private.getParam("map",dump_var);
    subcontrol_msg.map = static_cast<map>(dump_var);

    nh_private.getParam("state_estimation",dump_var);
    subcontrol_msg.state_estimation = static_cast<state_estimation>(dump_var);

    nh_private.getParam("system_diagnostic",dump_var);
    subcontrol_msg.system_diagnostic = static_cast<system_diagnostic>(dump_var); 

}

navigation_handler::~navigation_handler()
{
}

bool navigation_handler::first_check()
{
    //ros::spinOnce();
    if ((subcontrol_msg.map == no_map) || (subcontrol_msg.state_estimation == not_ready) || (subcontrol_msg.system_diagnostic == not_finished) )
    {   
        //errors_loop_rate->sleep();   
        if (subcontrol_msg.map == no_map) {ROS_WARN("there is no map");}
        if (subcontrol_msg.state_estimation == not_ready) {ROS_WARN("state estimation not ready");}
        if (subcontrol_msg.system_diagnostic == not_finished) {ROS_WARN("system_diagnostic not finished");}
        subcontrol_msg.move_command = stop;
        return false;
    }
    else
    {
        return true;
    }
}

void navigation_handler::SP_sub_cb(const navigation::state_control& SP_msg)
{
    //move_state = static_cast<move_command>(SP_msg.move_command);           //move or stop or easy_stop
    //map_state = static_cast<map>(SP_msg.map);
    subcontrol_msg = SP_msg;
    //state_estimation_state = static_cast<state_estimation>(SP_msg.state_estimation);
    //system_diagnostic_state = static_cast<system_diagnostic>(SP_msg.system_diagnostic);
    //car_state = static_cast<states>(SP_msg.states);
}

void navigation_handler::control_act_sub_cb(const navigation::Control_actions& contorl_act_msg)
{
    ipg_Control_msg_pub.gas = contorl_act_msg.throttle ;   
    ipg_Control_msg_pub.brake = contorl_act_msg.brake ;   
    ipg_Control_msg_pub.steer_ang = contorl_act_msg.steering_angel*6.66666666 ;
    ipg_Control_msg_pub.steer_ang_vel = 0.0;
    ipg_Control_msg_pub.steer_ang_acc = 0.0;
    ipg_Control_msg_pub.use_vc = 1;
    //ipg_Control_msg_pub.selector_ctrl=0.0;
    //ipg_Control_msg_pub.steer_ang_vel = contorl_act_msg.steering_angel_velocity ;
    //ipg_Control_msg_pub.steer_ang_acc = contorl_act_msg.steering_angel_acceleration ;
    //ipg_Control_msg_pub.selector_ctrl;            //????????????????
    //ipg_Control_msg_pub.use_vc;                   //????????????????      
    //ipg_control_act_pub.publish(ipg_Control_msg_pub);
    //model_control_act_pub.publish(ipg_contorl_act_msg);
                ROS_WARN("ipg_Control_msg_pub.steer_ang[%f]",ipg_Control_msg_pub.steer_ang);
                ROS_WARN("ipg_Control_msg_pub.gas[%f]",ipg_Control_msg_pub.gas);
                ROS_WARN("ipg_Control_msg_pub.brake[%f]",ipg_Control_msg_pub.brake);
            

                        
    std_msgs::Float32 steering_angle_arduino;
    steering_angle_arduino.data = contorl_act_msg.steering_angel * -(180/3.14);
    
    if(steering_angle_arduino.data >= 15.0){
    
     steering_angle_arduino.data = 15.0; 
    }
    
    if(steering_angle_arduino.data <= -15.0){
    
     steering_angle_arduino.data = -15.0; 
    }
    

    
    ipg_control_act_pub.publish(ipg_Control_msg_pub);
    control_actions_arduino_steering.publish(steering_angle_arduino);

    control_actions = contorl_act_msg;
    //model_control_act_pub.publish(control_actions);

}

void navigation_handler::run()
{
    
    if (  first_check() && subcontrol_msg.move_command == move)
    {   
        if (!(ros::ok()))
        {   
            ROS_WARN("navigation handler node shutted down");
            ros::shutdown();
        }
        
        if ((subcontrol_msg.map == complete) && (subcontrol_msg.states == move_mpc))
        {
            ROS_INFO("car on mpc mode");
            subcontrol_msg.states = move_mpc;   // m:mpc
            subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node

            
            // publish command to move mpc mode 
        }
        
        else if (subcontrol_msg.map == complete && subcontrol_msg.states == move_pp_mpc)
        {
            ROS_INFO("car on mpc and pure pursuit mode");
            //subcontrol_msg.states = move_pp_mpc;           // c:compaiend
            subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node
            // publish command to move pp and mpc mode
        }
        else if (subcontrol_msg.map == complete && subcontrol_msg.states == move_pp)
        {
            ROS_INFO("car on pure pursuit mode with complete map");
            //subcontrol_msg.states = move_pp;           // p:pure pursuit with complete map
            subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node
            // publish command to move pp and mpc mode
        }
        else if (subcontrol_msg.map == not_complete || subcontrol_msg.states == move_pp)
        {
            ROS_INFO("car on pure_pursuit mode with not_complete map");
            //subcontrol_msg.states = move_pp;           // r:pure pursuit with not_complete map
            subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node
            // publish command to move pp mode
        }

        if (subcontrol_msg.plot == plotting)
        {
            ROS_INFO("plotting");
            //subcontrol_msg.plot = plotting;
            subcontrol_plot_msg_pub.publish(subcontrol_msg);    //publish to plot node
            //publish for plotting
        }
        else 
        {
            ROS_INFO("no_plotting");
            //subcontrol_msg.plot = no_plotting;
            subcontrol_plot_msg_pub.publish(subcontrol_msg);    //publish to plot node
            //publish for no_ploting
        }
        
        //loop_rate.sleep();
        //ros::spinOnce();

    }
    
    
    if (subcontrol_msg.move_command == stop)
    {
        ROS_INFO("stop car");
        //subcontrol_msg.move_command = stop;           // p:pure pursuit with complete map
        subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node
        ipg_Control_msg_pub.brake=1.0;              // todo : think of way to relese th brake 
        ipg_Control_msg_pub.gas=0.0;
        ipg_Control_msg_pub.steer_ang=0.0;
        ipg_control_act_pub.publish(ipg_Control_msg_pub);
        //can_control_act_pub.publish();      //publish to car can
        
    }

    if (subcontrol_msg.move_command == easy_stop)
    {
        ROS_INFO("easy stop car");
        //subcontrol_msg.move_command = easy_stop;           // p:pure pursuit with complete map
        subcontrol_msg_pub.publish(subcontrol_msg); // publish to pp node and mpc node    
        // publish command to easy stop car
        control_actions.v_des = 0.0;
    }

    loop_rate.sleep();
    ros::spinOnce();
    //can_control_act_pub.publish();      //publish to car can
    //ipg_control_act_pub.publish(ipg_Control_msg_pub);

}


int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "nav_handler");

    navigation_handler handelr;

    handelr.spinOnce();

    while (ros::ok())
        {
            handelr.run();
        }

}
