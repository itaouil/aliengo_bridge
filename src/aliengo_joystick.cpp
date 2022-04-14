/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/


// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// C++
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

// Unitree
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"

using namespace UNITREE_LEGGED_SDK;

constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8081;
constexpr char TARGET_IP[] = "192.168.123.220";   // target IP address

class Custom
{
public:
    Custom(uint8_t level, ros::NodeHandle nh): safe(LeggedType::Aliengo), nh(nh),
        udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    UDP udp;
    Safety safe;
    float dt = 0.002;      
    HighCmd cmd = {0};
    int motiontime = 0;
    HighState state = {0};
    xRockerBtnDataStruct _keyData;

    ros::NodeHandle nh;
    sensor_msgs::Joy joy_msg;
    ros::Time last_update = ros::Time::now();
    ros::Publisher joy_pub = nh.advertise< sensor_msgs::Joy >( "joy", 1 );
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    motiontime += 2;
    udp.GetRecv(state);
	
    memcpy(&_keyData, state.wirelessRemote, 40);

    joy_msg.header.stamp = ros::Time::now();
    joy_msg.axes = {_keyData.lx, _keyData.ly, _keyData.rx, _keyData.ry};
    if ((ros::Time::now() - last_update).toSec() > 1) 
    {
        joy_msg.buttons = {((int)_keyData.btn.components.up == 1),
                        ((int)_keyData.btn.components.down == 1),
                        ((int)_keyData.btn.components.select == 1)};
        last_update = ros::Time::now();
    }
    else {
        joy_msg.buttons = {0, 0, 0};
    }

    joy_pub.publish(joy_msg);
}

int main(int argc, char** argv)
{
    ros::init( argc, argv, "aliengo_joystick" );
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    ros::NodeHandle nh( "~" );

    Custom custom(HIGHLEVEL, nh);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0;
}
