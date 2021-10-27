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
#include <config_server/parameter.h>

// Unitree
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

namespace aliengo_bridge
{
struct GamepadConfig {
    // Axis
    config_server::Parameter<int> gamepadLeftHorizontalAxis{"/gamepad/left_horizontal", 0, 1, 7, 0};
    config_server::Parameter<int> gamepadLeftVerticalAxis{"/gamepad/left_vertical", 0, 1, 7, 1};
    config_server::Parameter<int> gamepadRightHorizontalAxis{"/gamepad/right_horizontal", 0, 1, 7, 3};
    config_server::Parameter<int> gamepadRightVerticalAxis{"/gamepad/to_odom", 0, 1, 7, 4};
    config_server::Parameter<int> gamepadButtonStart{"/gamepad/start", 0, 1, 7, 7};
    
    config_server::Parameter<float> velLimitForward{"/vel_limits/forward", 0, 0.1, 1, 0.5};
    config_server::Parameter<float> velLimitSide{"/vel_limits/side", 0, 0.1, 1, 0.5};
    config_server::Parameter<float> velLimitRotation{"/vel_limits/rotation", 0, 0.1, 1, 0.5};
};

class AlienGoBridge
{
public:
    AlienGoBridge(ros::NodeHandle ph);
    
private:
    ros::NodeHandle m_ph;
    
    GamepadConfig m_config;
    
    Safety m_safe;
    UDP m_udp;
    HighCmd m_cmd = {0};
    HighState m_state = {0};
    int m_motiontime = 0;
    float m_dt = 0.002;     // 0.001~0.01
    int m_motion_timestep;
    
    LoopFunc m_loop_control;
    LoopFunc m_loop_udpSend;
    LoopFunc m_loop_udpRecv;
    
    ros::Publisher m_imu_pub;
    ros::Subscriber m_cmd_sub;
    ros::Subscriber m_joy_sub;
    ros::Publisher m_state_pub;
    
    ros::Time m_last_cmd_time;
    float m_cmd_timeout = 1.;
    bool m_received_cmd = false;
    
    boost::mutex m_cmd_mutex;
    boost::mutex m_state_mutex;
    
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    
    void cmdCallback( const unitree_legged_msgs::HighCmd& cmd );
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void publishState();
};

}
#endif

