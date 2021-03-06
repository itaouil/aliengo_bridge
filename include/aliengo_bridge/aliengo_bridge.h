#ifndef ALIEN_GO_BRIDGE_H_
#define ALIEN_GO_BRIDGE_H_

// C++
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// Unitree
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighStateStamped.h>
#include "unitree_legged_sdk/unitree_joystick.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

namespace aliengo_bridge
{
class AlienGoBridge
{
public:
    AlienGoBridge(ros::NodeHandle ph);
    
private:
    void UDPRecv();
    void UDPSend();
    void control();
    void resetCmd();
    void publishHighState();
    void publishJointState();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void cmdCallback(const unitree_legged_msgs::HighCmd& cmd);

    ros::NodeHandle m_ph;
    
    UDP m_udp;
    Safety m_safe;
    HighCmd m_cmd = {0};
    HighState m_state = {0};

    float m_dt = 0.001;

    sensor_msgs::JointState m_jointStateMsg;
    unitree_legged_msgs::HighStateStamped m_stateMsg;

    LoopFunc m_loop_control;
    LoopFunc m_loop_udpSend;
    LoopFunc m_loop_udpRecv;
    
    ros::Subscriber m_cmd_sub;
    ros::Publisher m_state_pub;
    ros::Publisher m_joints_pub;
    
    ros::Time m_last_cmd_time;
    float m_cmd_timeout = 0.1f;
    bool m_received_cmd = false;
    
    boost::mutex m_cmd_mutex;
    boost::mutex m_state_mutex;
};

}
#endif
