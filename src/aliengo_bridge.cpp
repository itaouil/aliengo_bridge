#include <aliengo_bridge/aliengo_bridge.h>

using namespace UNITREE_LEGGED_SDK;

namespace aliengo_bridge
{
    AlienGoBridge::AlienGoBridge(ros::NodeHandle ph)
    : m_ph( ph )
    , m_safe(LeggedType::Aliengo)
    , m_udp(8081, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
    , m_loop_control("control_loop", m_dt, boost::bind(&AlienGoBridge::control, this))
    , m_loop_udpSend("udp_send", m_dt, 3, boost::bind(&AlienGoBridge::UDPSend, this))
    , m_loop_udpRecv("udp_recv", m_dt, 3, boost::bind(&AlienGoBridge::UDPRecv, this))
    {
        // Flag
        m_received_cmd = false;

        // Timers
        m_last_cmd_time = ros::Time::now();
        m_last_velocity_update = ros::Time::now();

        // ROS
        m_cmd_sub = ph.subscribe( "cmd", 1, &AlienGoBridge::cmdCallback, this );
        m_state_pub = ph.advertise< unitree_legged_msgs::HighState >( "high_state", 1 );
        
        // SDK
        m_udp.InitCmdData(m_cmd);
        m_motion_timestep = static_cast<int>( 1000 * m_dt );
        
        m_loop_udpSend.start();
        m_loop_udpRecv.start();
        m_loop_control.start();
    }

    void AlienGoBridge::cmdCallback( const unitree_legged_msgs::HighCmd& cmd )
    {
        boost::mutex::scoped_lock lock(m_cmd_mutex);
        m_cmd.mode = cmd.mode;  // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
                                // 2.target velocity walking (controlled by velocity + yawSpeed)
                                // 3.target position walking (controlled by position + rpy[2])
                                // 4. path mode walking (reserve for future release)
                                // 5. position stand down. |6. position stand up |7. damping mode | 8. recovery mode
        
        m_cmd.gaitType = cmd.gaitType; // 0.trot | 1. trot running  | 2.climb stair

        m_cmd.speedLevel = cmd.speedLevel; // 0. default low speed. 1. medium speed 2. high speed. during walking
        
        m_cmd.dFootRaiseHeight = cmd.dFootRaiseHeight; // (unit: m), swing foot height adjustment from default swing height.

        m_cmd.dBodyHeight = cmd.dBodyHeight; // (unit: m), body height adjustment from default body height.

        m_cmd.position[0] = cmd.position[0]; // (unit: m), desired x in inertial frame.
        m_cmd.position[1] = cmd.position[1]; // (unit: m), y position in inertial frame.

        m_cmd.rpy[0] = cmd.rpy[0]; // (unit: rad), desired roll euler angle
        m_cmd.rpy[1] = cmd.rpy[1]; // (unit: rad), desired pitch euler angle
        m_cmd.rpy[2] = cmd.rpy[2]; // (unit: rad), desired yaw euler angle

        m_cmd.velocity[0] = cmd.velocity[0]; // (unit: m/s), forwardSpeed in body frame.
        m_cmd.velocity[1] = cmd.velocity[1]; // (unit: m/s), sideSpeed in body frame.
        m_cmd.yawSpeed = cmd.yawSpeed;    // (unit: rad/s), rotateSpeed in body frame.

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

    void AlienGoBridge::resetCmd()
    {
        m_cmd.mode = 0; // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
                        // 2.target velocity walking (controlled by velocity + yawSpeed)
                        // 3.target position walking (controlled by position + rpy[2])
                        // 4. path mode walking (reserve for future release)
                        // 5. position stand down. 
                        // 6. position stand up 
                        // 7. damping mode 
                        // 8. recovery mode
        
        m_cmd.gaitType = 0; // 0.trot | 1. trot running  | 2.climb stair

        m_cmd.speedLevel = 0; // 0. default low speed. 1. medium speed 2. high speed. during walking
        
        m_cmd.dFootRaiseHeight = 0.0f; // (unit: m), swing foot height adjustment from default swing height.

        m_cmd.dBodyHeight = 0.0f; // (unit: m), body height adjustment from default body height.

        m_cmd.position[0] = 0.0f; // (unit: m), desired x in inertial frame.
        m_cmd.position[1] = 0.0f; // (unit: m), y position in inertial frame.

        m_cmd.rpy[0] = 0.0f; // (unit: rad), desired roll euler angle
        m_cmd.rpy[1] = 0.0f; // (unit: rad), desired pitch euler angle
        m_cmd.rpy[2] = 0.0f; // (unit: rad), desired yaw euler angle

        m_cmd.velocity[0] = 0.0f; // (unit: m/s), forwardSpeed in body frame.
        m_cmd.velocity[1] = 0.0f; // (unit: m/s), sideSpeed in body frame.
        m_cmd.yawSpeed = 0.0f; // (unit: rad/s), rotateSpeed in body frame.
    }

    void AlienGoBridge::setCmd()
    {
        m_cmd.mode = 2; // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
                        // 2.target velocity walking (controlled by velocity + yawSpeed)
                        // 3.target position walking (controlled by position + rpy[2])
                        // 4. path mode walking (reserve for future release)
                        // 5. position stand down. 
                        // 6. position stand up 
                        // 7. damping mode 
                        // 8. recovery mode
        
        m_cmd.gaitType = 0; // 0.trot | 1. trot running  | 2.climb stair

        m_cmd.speedLevel = 0; // 0. default low speed. 1. medium speed 2. high speed. during walking
        
        m_cmd.dFootRaiseHeight = 0.0f; // (unit: m), swing foot height adjustment from default swing height.

        m_cmd.dBodyHeight = 0.0f; // (unit: m), body height adjustment from default body height.

        m_cmd.position[0] = 0.0f; // (unit: m), desired x in inertial frame.
        m_cmd.position[1] = 0.0f; // (unit: m), y position in inertial frame.

        m_cmd.rpy[0] = 0.0f; // (unit: rad), desired roll euler angle
        m_cmd.rpy[1] = 0.0f; // (unit: rad), desired pitch euler angle
        m_cmd.rpy[2] = 0.0f; // (unit: rad), desired yaw euler angle

        m_cmd.velocity[0] = 0.1f; // (unit: m/s), forwardSpeed in body frame.
        m_cmd.velocity[1] = 0.0f; // (unit: m/s), sideSpeed in body frame.
        m_cmd.yawSpeed = 0.0f; // (unit: rad/s), rotateSpeed in body frame.
    }

    void AlienGoBridge::control() 
    {
        m_state_mutex.lock();
        m_udp.GetRecv( m_state );
        m_state_mutex.unlock();
        
        m_cmd_mutex.lock();
        if (!m_received_cmd)
        {
            ROS_WARN_STREAM_THROTTLE( 5, "No cmd received." );
            resetCmd();
        }
        else
        {
            setCmd();
        }
        
        m_udp.SetSend(m_cmd);
        m_cmd_mutex.unlock();
        
        publishState();
        joystickUpdate();
    }

    void AlienGoBridge::publishState()
    {
        boost::mutex::scoped_lock lock(m_state_mutex);
        
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
        msg.imu.rpy[0] = m_state.imu.rpy[0];
        msg.imu.rpy[1] = m_state.imu.rpy[1];
        msg.imu.rpy[2] = m_state.imu.rpy[2];
        msg.imu.temperature = m_state.imu.temperature;
        
        msg.gaitType = m_state.gaitType;
        msg.footRaiseHeight = m_state.footRaiseHeight;
        msg.bodyHeight = m_state.bodyHeight;

        msg.position[0] = m_state.position[0];
        msg.position[1] = m_state.position[1];
        msg.position[2] = m_state.position[2];
        
        msg.velocity[0] = m_state.velocity[0];
        msg.velocity[1] = m_state.velocity[1];
        msg.velocity[2] = m_state.velocity[2];
        
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
        }
        
        for ( int i = 0; i < 40; ++i )
            msg.wirelessRemote[i] = m_state.wirelessRemote[i];

        msg.reserve = m_state.reserve;
        msg.crc = m_state.crc;
        
        m_state_pub.publish( msg );
    }

    void AlienGoBridge::joystickUpdate()
    {
        boost::mutex::scoped_lock cmd_lock(m_cmd_mutex);
        boost::mutex::scoped_lock state_lock(m_state_mutex);
        
        memcpy(&m_joy, m_state.wirelessRemote, 40);

        // Update velocities every second otherwise values increase too quick
        if ((ros::Time::now() - m_last_velocity_update).toSec() > 1.)
        {
            if (((int)m_joy.btn.components.up == 1) && m_cmd_max_velocity < 1.0) 
            {
                m_cmd_max_velocity += 0.1;
                ROS_INFO_STREAM("Increased max velocity by 0.1: " << m_cmd_max_velocity);
                m_last_velocity_update = ros::Time::now();
            }
            else if (((int)m_joy.btn.components.down == 1) && m_cmd_max_velocity > 0.1)
            {
                m_cmd_max_velocity -= 0.1;
                ROS_INFO_STREAM("Decreased max velocity by 0.1: " << m_cmd_max_velocity);
                m_last_velocity_update = ros::Time::now();
            }
            else if (((int)m_joy.btn.components.F1 == 1))
            {
                m_cmd_max_velocity = 0.0f;
                ROS_INFO_STREAM("Reset max velocity to 0.0: " << m_cmd_max_velocity);
                m_last_velocity_update = ros::Time::now();
            }
            else if (((int)m_joy.btn.components.select == 1))
            {
                m_received_cmd = !m_received_cmd;
                ROS_INFO_STREAM("Received command flag toggled " << m_received_cmd);
                m_last_velocity_update = ros::Time::now();
            }
        }
    }
}