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
    m_cmd_sub = ph.subscribe( "aliengo/command", 1, &AlienGoBridge::cmdCallback, this );
    m_state_pub = ph.advertise< unitree_legged_msgs::HighState >( "aliengo/high_state", 1 );
    
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
    boost::mutex::scoped_lock lock(m_cmd_mutex);
    m_cmd.mode = cmd.mode;      // 0:idle, default stand      1:forced stand     2:walk continuously
    
    m_cmd.forwardSpeed = cmd.forwardSpeed;
    m_cmd.sideSpeed = cmd.sideSpeed;
    m_cmd.rotateSpeed = cmd.rotateSpeed;
    m_cmd.bodyHeight = cmd.bodyHeight;
    
    m_cmd.yaw = cmd.yaw;
    m_cmd.roll  = cmd.roll;
    m_cmd.pitch = cmd.pitch;
    
    m_received_cmd = true;
    m_last_cmd_time = ros::Time::now();
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
    m_state_mutex.lock();
    m_udp.GetRecv( m_state );
    m_state_mutex.unlock();

    memcpy(&_keyData, m_state.wirelessRemote, 40);
    if((int)_keyData.btn.components.A == 1){
        std::cout << "The key A is pressed, and the value of lx is " << _keyData.lx << std::endl;
    }
    
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
    
    m_state_pub.publish( msg );
}

}
