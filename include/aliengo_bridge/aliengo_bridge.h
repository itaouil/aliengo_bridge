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
#include <config_server/parameter.h>

// Unitree
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
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
    void RobotControl();
    void publishState();
    void cmdCallback(const unitree_legged_msgs::HighCmd& cmd);

    ros::NodeHandle m_ph;
    
    UDP m_udp;
    Safety m_safe;
    HighCmd m_cmd = {0};
    HighState m_state = {0};
    xRockerBtnDataStruct _keyData;

    float m_dt = 0.002;     // 0.001~0.01
    int m_motiontime = 0;
    int m_motion_timestep;
    float m_lower_velocity = 0.0;
    float m_higher_velocity = 0.1;
    
    LoopFunc m_loop_control;
    LoopFunc m_loop_udpSend;
    LoopFunc m_loop_udpRecv;
    
    ros::Subscriber m_cmd_sub;
    ros::Publisher m_state_pub;
    
    ros::Time m_last_cmd_time;
    float m_cmd_timeout = 1.;
    bool m_received_cmd = false;
    
    boost::mutex m_cmd_mutex;
    boost::mutex m_state_mutex;
    
};

}
#endif

