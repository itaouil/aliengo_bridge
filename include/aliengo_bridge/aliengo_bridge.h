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
#include <unitree_legged_msgs/HighCmdStamped.h>
#include <unitree_legged_msgs/HighStateStamped.h>
#include "unitree_legged_sdk/unitree_joystick.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;
using namespace unitree_legged_msgs;

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
    void setCmd();
    void resetCmd();
    void publishCmd();
    void publishState();
    void joystickUpdate();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    ros::NodeHandle m_ph;
    
    UDP m_udp;
    Safety m_safe;
    HighCmd m_cmdUDP;
    HighCmdStamped m_cmd;
    HighStateStamped m_state;

    float m_dt = 0.002;
    int m_motiontime = 0;
    int m_motion_timestep;

    xRockerBtnDataStruct m_joy;
    uint8_t m_cmd_speed_level = 0;
    float m_cmd_min_velocity = 0.0;
    float m_cmd_max_velocity = 0.0;
    ros::Time m_last_velocity_update;
    
    LoopFunc m_loop_control;
    LoopFunc m_loop_udpSend;
    LoopFunc m_loop_udpRecv;
    
    ros::Publisher m_cmd_pub;
    ros::Publisher m_state_pub;
    
    ros::Time m_last_cmd_time;
    float m_cmd_timeout = 1.;
    bool m_received_cmd = false;
    
    boost::mutex m_cmd_mutex;
    boost::mutex m_state_mutex;
};

}
#endif
