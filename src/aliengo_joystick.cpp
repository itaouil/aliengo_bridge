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
    Custom(uint8_t level): safe(LeggedType::Aliengo),
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
    ros::Publisher joy_pub = ph.advertise< sensor_msgs::Joy >( "joy", 1 );
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

    joy_msg.heade.stamp = ros::Time::now();
    joy_msg.axes = [_keyData.lx, _keyData.ly, _keyData.rx, _keyData.ry];
    joy_msg.buttons = [(int)_keyData.btn.components.up,
                       (int)_keyData.btn.components.down];

    auto axes_on = std::count_if(joy_msg.axes.begin(), joy_msg.axes.end(),[&](auto const& val){ return val >= 0.7; });
    auto btns_on = std::count_if(jjoy_msg.buttons.begin(), joy_msg.buttons.end(),[&](auto const& val){ return val >= 0.5; });

    if (btns_on || axes_on)
        joy_pub.publish(joy_msg);
}

int main(void)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
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
