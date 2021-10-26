#include <aliengo_bridge/aliengo_bridge.h>

using namespace UNITREE_LEGGED_SDK;

namespace aliengo_bridge
{

AlienGoBridge::AlienGoBridge(ros::NodeHandle ph)
: m_ph( ph )
, m_safe(LeggedType::Aliengo)
, m_udp(HIGHLEVEL)
, m_loop_control("control_loop", m_dt, boost::bind(&AlienGoBridge::RobotControl, this))
, m_loop_udpSend("udp_send", m_dt, 3, boost::bind(&AlienGoBridge::UDPSend, this))
, m_loop_udpRecv("udp_recv", m_dt, 3, boost::bind(&AlienGoBridge::UDPRecv, this))
{
    // ROS
    m_imu_pub = ph.advertise< sensor_msgs::Imu >( "imu", 1 );
    m_state_pub = ph.advertise< unitree_legged_msgs::HighState >( "robot_state", 1 );
    m_cmd_sub = ph.subscribe( "cmd", 1, &AlienGoBridge::cmdCallback, this );
    m_joy_sub = ph.subscribe( "joy", 1, &AlienGoBridge::joyCallback, this );
    
    // SDK
    m_motion_timestep = static_cast<int>( 1000 * m_dt );
    m_udp.InitCmdData(m_cmd);
    
    InitEnvironment();
    
    m_loop_udpSend.start();
    m_loop_udpRecv.start();
    m_loop_control.start();
}

void AlienGoBridge::cmdCallback( const unitree_legged_msgs::HighCmd& cmd )
{
    boost::mutex::scoped_lock lock(m_cmd_mutex);
    m_cmd.mode = cmd.mode;      // 0:idle, default stand      1:forced stand     2:walk continuously
    
    m_cmd.forwardSpeed = cmd.forwardSpeed;
    m_cmd.sideSpeed = cmd.sideSpeed;
    m_cmd.rotateSpeed = cmd.rotateSpeed;
    m_cmd.bodyHeight = cmd.bodyHeight;
    
//     m_cmd.footRaiseHeight = cmd.footRaiseHeight;
    
    m_cmd.yaw = cmd.yaw;
    m_cmd.roll  = cmd.roll;
    m_cmd.pitch = cmd.pitch;
    
    m_last_cmd_time = ros::Time::now();
    m_received_cmd = true;
}

void AlienGoBridge::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(m_cmd_mutex);
    m_last_cmd_time = msg->header.stamp;
    
    m_cmd.forwardSpeed = m_config.velLimitForward() * msg->axes[ m_config.gamepadRightVerticalAxis() ];
    m_cmd.sideSpeed = m_config.velLimitSide() * msg->axes[ m_config.gamepadRightHorizontalAxis() ];
    m_cmd.rotateSpeed = m_config.velLimitRotation() * msg->axes[ m_config.gamepadLeftHorizontalAxis() ];
    
    if ( m_config.gamepadButtonStart() )
    {
        if ( m_cmd.mode == 2 )
            m_cmd.mode = 1;
        else
            m_cmd.mode = 2;
    }
    
    m_received_cmd = true;
}

void AlienGoBridge::UDPRecv()
{
    m_udp.Recv();
}

void AlienGoBridge::UDPSend()
{  
    m_udp.Send();
}

void AlienGoBridge::RobotControl() 
{
//     m_motiontime += m_motion_timestep;
    
    m_state_mutex.lock();
    m_udp.GetRecv( m_state );
    m_state_mutex.unlock();
    
    m_cmd_mutex.lock();
    if ( !m_received_cmd )
    {
        ROS_WARN_STREAM_THROTTLE( 5, "No cmd received." );
        
        m_cmd.forwardSpeed = 0.0f;
        m_cmd.sideSpeed = 0.0f;
        m_cmd.rotateSpeed = 0.0f;
        m_cmd.bodyHeight = 0.0f;
        
        m_cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
        m_cmd.roll  = 0;
        m_cmd.pitch = 0;
        m_cmd.yaw = 0;
    }
    // reset cmd after timeout
    else if ( (ros::Time::now() - m_last_cmd_time).toSec() > m_cmd_timeout )
    {
        ROS_WARN_STREAM( "Cmd timed out." );
        m_received_cmd = false;
        m_cmd.forwardSpeed = 0.0f;
        m_cmd.sideSpeed = 0.0f;
        m_cmd.rotateSpeed = 0.0f;
        m_cmd.bodyHeight = 0.0f;
        
        m_cmd.mode = 1;      // 0:idle, default stand      1:forced stand     2:walk continuously
        m_cmd.roll  = 0;
        m_cmd.pitch = 0;
        m_cmd.yaw = 0;
    }
    
    m_udp.SetSend(m_cmd);
    m_cmd_mutex.unlock();
    
    publishState();
}

void AlienGoBridge::publishState()
{
    boost::mutex::scoped_lock lock(m_state_mutex);
    
    sensor_msgs::Imu imu_msg;
    unitree_legged_msgs::HighState msg;
    
    msg.levelFlag = m_state.levelFlag;
    msg.commVersion = m_state.commVersion;
    msg.robotID = m_state.robotID;
    msg.SN = m_state.SN;
    msg.bandWidth = m_state.bandWidth;
    msg.mode = m_state.mode;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation.x = m_state.imu.quaternion[0];
    imu_msg.orientation.y = m_state.imu.quaternion[1];
    imu_msg.orientation.z = m_state.imu.quaternion[2];
    imu_msg.orientation.w = m_state.imu.quaternion[3];

    imu_msg.angular_velocity.x = m_state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = m_state.imu.gyroscope[1];
    imu_msg.angular_velocity.z = m_state.imu.gyroscope[2];

    imu_msg.linear_acceleration.x = m_state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = m_state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = m_state.imu.accelerometer[2];
    
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
    
    msg.imu.temperature = m_state.imu.temperature;
    
    msg.forwardSpeed = m_state.forwardSpeed;
    msg.sideSpeed = m_state.sideSpeed;
    msg.rotateSpeed = m_state.rotateSpeed;
    msg.bodyHeight = m_state.bodyHeight;
    msg.updownSpeed = m_state.updownSpeed;
    msg.forwardPosition = m_state.forwardPosition;
    msg.sidePosition = m_state.sidePosition;
    
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
    
    msg.tick = m_state.tick;               
    for ( int i = 0; i < 40; ++i )
        msg.wirelessRemote[i] = m_state.wirelessRemote[i];
    msg.reserve = m_state.reserve;
    msg.crc = m_state.crc;
    
    m_imu_pub.publish( imu_msg );
    m_state_pub.publish( msg );
}

}
