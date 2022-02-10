#include <aliengo_bridge/aliengo_bridge.h>

using namespace UNITREE_LEGGED_SDK;

namespace aliengo_bridge
{

AlienGoBridge::AlienGoBridge(ros::NodeHandle ph)
: m_ph( ph )
, m_safe(LeggedType::Aliengo)
, m_udp(8090, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))^
, m_loop_control("control_loop", m_dt, boost::bind(&AlienGoBridge::control, this))
, m_loop_udpSend("udp_send", m_dt, 3, boost::bind(&AlienGoBridge::UDPSend, this))
, m_loop_udpRecv("udp_recv", m_dt, 3, boost::bind(&AlienGoBridge::UDPRecv, this))
{
    // ROS
    m_cmd_sub = ph.subscribe( "cmd", 1, &AlienGoBridge::cmdCallback, this );
    m_state_pub = ph.advertise< unitree_legged_msgs::HighState >( "high_state", 1 );
    
    // SDK
    m_udp.InitCmdData(m_cmd);
    m_motion_timestep = static_cast<int>( 1000 * m_dt );
    
    // Leave it commented
    //InitEnvironment();
    
    m_loop_udpSend.start();
    m_loop_udpRecv.start();
    m_loop_control.start();
}

void AlienGoBridge::cmdCallback( const unitree_legged_msgs::HighCmd& cmd )
{
    // boost::mutex::scoped_lock lock(m_cmd_mutex);
    // m_cmd.mode = cmd.mode;      // 0:idle, default stand      1:forced stand     2:walk continuously
    
    // m_cmd.forwardSpeed = cmd.forwardSpeed;
    // m_cmd.sideSpeed = cmd.sideSpeed;
    // m_cmd.rotateSpeed = cmd.rotateSpeed;
    // m_cmd.bodyHeight = cmd.bodyHeight;
    
    // m_cmd.yaw = cmd.yaw;
    // m_cmd.roll  = cmd.roll;
    // m_cmd.pitch = cmd.pitch;
    
    // m_received_cmd = true;
    // m_last_cmd_time = ros::Time::now();
}

void AlienGoBridge::UDPRecv()
{
    m_udp.Recv();
}

void AlienGoBridge::UDPSend()
{  
    m_udp.Send();
}

void AlienGoBridge::resetCmd()
{
    cmd.mode = 1;               // 0. idle, default stand
                                // 1. force stand (controlled by dBodyHeight + ypr)
                                // 2. target velocity walking (controlled by velocity + yawSpeed)
                                // 3. target position walking (controlled by position + ypr[0])
                                // 4. path mode walking (reserve for future release)
                                // 5. position stand down. 
                                // 6. position stand up 
                                // 7. damping mode
                                // 8. recovery stand
                                // 9. backflip
                                // 10. jumpYaw
                                // 11. straightHand
                                // 12. dance1
                                // 13. dance2
    cmd.gaitType = 0;           // 0.idle  1.trot  2.trot running  3.climb stair
    cmd.euler[0] = 0;           // (unit: rad), roll in stand mode
    cmd.euler[1] = 0;           // (unit: rad), pitch in stand mode
    cmd.euler[2] = 0;           // (unit: rad), yaw in stand mode
    cmd.speedLevel = 0;         // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
    cmd.bodyHeight = 0;         // (unit: m, default: 0.28m)
    cmd.yawSpeed = 0.0f;        // (unit: rad/s), rotateSpeed in body frame
    cmd.velocity[0] = 0.0f;     // (unit: m/s), forwardSpeed in body frame
    cmd.velocity[1] = 0.0f;     // (unit: m/s), sideSpeed in body frame
    cmd.footRaiseHeight = 0;    // (unit: m, default: 0.08m), foot up height while walking
}

void AlienGoBridge::control() 
{
    m_state_mutex.lock();
    m_udp.GetRecv( m_state );
    m_state_mutex.unlock();

    if ((ros::Time::now() - m_last_joy_time).toSec() > 1) 
    {
        memcpy(&_keyData, m_state.wirelessRemote, 40);

        if((int)_keyData.btn.components.select == 1)
        {
            m_cmd_max_velocity = 0.1;
            m_cmd_min_velocity = 0.0; 

            m_last_joy_time = ros::Time::now();
            std::cout << "Reset max and min velocities to: " << 0.1 << " and " << 0.0 << std::endl;
        }
        else if((int)_keyData.btn.components.Y == 1)
        {
            if (m_cmd_max_velocity < 0.9)
                m_cmd_max_velocity += 0.1;
            
            m_last_joy_time = ros::Time::now();
            std::cout << "Current max velocity is: " << m_cmd_max_velocity << std::endl;
        }
        else if((int)_keyData.btn.components.A == 1)
        {
            if (m_cmd_max_velocity > 0.0)
                m_cmd_max_velocity -= 0.1;

            m_last_joy_time = ros::Time::now();
            std::cout << "Current max velocity is: " << m_cmd_max_velocity << std::endl;
        }
        else if((int)_keyData.btn.components.B == 1)
        {
            if (m_cmd_min_velocity < 0.8)
                m_cmd_min_velocity += 0.1;
            
            m_last_joy_time = ros::Time::now();
            std::cout << "Current min velocity is: " << m_cmd_min_velocity << std::endl;
        }
        else if((int)_keyData.btn.components.X == 1)
        {
            if (m_cmd_min_velocity > 0.0)
                m_cmd_min_velocity -= 0.1;
            
            m_last_joy_time = ros::Time::now();
            std::cout << "Current min velocity is: " << m_cmd_min_velocity << std::endl;
        }
    }

    if (_keyData.ly > 0.95)
    {
        std::cout << "Sending fwd max velocity command: " << m_cmd_max_velocity << std::endl;
    }
    else if (_keyData.ly < -0.95)
    {
        std::cout << "Sending bwd max velocity command: " << -m_cmd_min_velocity << std::endl;
    }
    else if (_keyData.lx > 0.95)
    {
        std::cout << "Sending clockwise velocity command: " << m_cmd_max_velocity << std::endl;
    }
    else if (_keyData.lx < -0.95)
    {
        std::cout << "Sending counter clockwise velocity command: " << -m_cmd_max_velocity << std::endl;
    }
    else
    {
        std::cout << "Sending fwd min velocity command: " << m_cmd_min_velocity << std::endl;
    }
    
    m_cmd_mutex.lock();
    if (!m_received_cmd)
    {
        ROS_WARN_STREAM_THROTTLE( 5, "No cmd received." );
        resetCmd();
    }
    // reset cmd after timeout
    else if ((ros::Time::now() - m_last_cmd_time).toSec() > m_cmd_timeout)
    {
        ROS_WARN_STREAM( "Cmd timed out." );
        resetCmd();
    }
    
    m_udp.SetSend(m_cmd);
    m_cmd_mutex.unlock();
    
    publishState();
}

void AlienGoBridge::publishState()
{
    boost::mutex::scoped_lock lock(m_state_mutex);
    
    // Messages to be published
    sensor_msgs::Imu l_imuMsg;
    unitree_legged_msgs::HighState msg;
    sensor_msgs::JointState l_jointStateMsg;
    
    msg.levelFlag = m_state.levelFlag;
    msg.commVersion = m_state.commVersion;
    msg.robotID = m_state.robotID;
    msg.SN = m_state.SN;
    msg.bandWidth = m_state.bandWidth;
    msg.mode = m_state.mode;
    msg.progress = m_state.progress;
    
    msg.imu.quaternion[0] = m_state.imu.quaternion[0];
    msg.imu.quaternion[1] = m_state.imu.quaternion[1];
    msg.imu.quaternion[2] = m_state.imu.quaternion[2];
    msg.imu.quaternion[3] = m_state.imu.quaternion[3];
    msg.imu.gyroscope[0] = m_state.imu.gyroscope[0];
    msg.imu.gyroscope[1] = m_state.imu.gyroscope[1];
    msg.imu.gyroscope[2] = m_state.imu.gyroscope[2];
    msg.imu.accelerometer[0] = m_state.imu.accelerometer[0];
    msg.imu.accelerometer[1] = m_state.imu.accelerometer[1];
    msg.imu.accelerometer[2] = m_state.imu.accelerometer[2];
    msg.imu.rpy[0] = m_state.imu.rpy[0];
    msg.imu.rpy[1] = m_state.imu.rpy[1];
    msg.imu.rpy[2] = m_state.imu.rpy[2];
    msg.imu.temperature = m_state.imu.temperature;
    
    msg.gaitType = m_state.gaitType;
    msg.footRaiseHeight = m_state.footRaiseHeight;
    msg.position = m_state.position;
    msg.bodyHeight = m_state.bodyHeight;
    msg.velocity = m_state.velocity;
    msg.yawSpeed = m_state.yawSpeed;
    
    for ( int i = 0; i < 4; ++i )
    {
        msg.footPosition2Body[i].x = m_state.footPosition2Body[i].x;
        msg.footPosition2Body[i].y = m_state.footPosition2Body[i].y;
        msg.footPosition2Body[i].z = m_state.footPosition2Body[i].z;
        
        msg.footSpeed2Body[i].x = m_state.footSpeed2Body[i].x;
        msg.footSpeed2Body[i].y = m_state.footSpeed2Body[i].y;
        msg.footSpeed2Body[i].z = m_state.footSpeed2Body[i].z;
        
        msg.footForce[i] = m_state.footForce[i];
        msg.footForceEst[i] = m_state.footForceEst[i];
    }
    
    for ( int i = 0; i < 40; ++i )
        msg.wirelessRemote[i] = m_state.wirelessRemote[i];
    msg.reserve = m_state.reserve;
    msg.crc = m_state.crc;
    
    m_state_pub.publish( msg );
}

}
