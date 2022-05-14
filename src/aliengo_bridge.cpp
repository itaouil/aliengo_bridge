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
        // ROS
        m_cmd_sub = ph.subscribe( "cmd", 1, &AlienGoBridge::cmdCallback, this );
        m_joints_pub = ph.advertise< sensor_msgs::JointState >( "joint_states", 1 );
        m_state_pub = ph.advertise< unitree_legged_msgs::HighStateStamped >( "high_state", 1 );

        // Init joint state msg size
        m_jointStateMsg.name.resize(12);
        m_jointStateMsg.position.resize(12);
        m_jointStateMsg.velocity.resize(12);
        m_jointStateMsg.effort.resize(12);
        m_jointStateMsg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
        
        // SDK
        m_udp.InitCmdData(m_cmd);
        
        m_loop_udpSend.start();
        m_loop_udpRecv.start();
        m_loop_control.start();
    }

    void AlienGoBridge::cmdCallback( const unitree_legged_msgs::HighCmd& cmd )
    {
        boost::mutex::scoped_lock lock(m_cmd_mutex);
        m_cmd.mode = cmd.mode;    // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
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
        m_cmd.mode = 0;         // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
                                // 2.target velocity walking (controlled by velocity + yawSpeed)
                                // 3.target position walking (controlled by position + rpy[2])
                                // 4. path mode walking (reserve for future release)
                                // 5. position stand down. |6. position stand up |7. damping mode | 8. recovery mode
        
        m_cmd.gaitType = 2; // 0.trot | 1. trot running  | 2.climb stair

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
        m_cmd.yawSpeed = 0.0f;    // (unit: rad/s), rotateSpeed in body frame.
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
        // reset cmd after timeout
        else if ((ros::Time::now() - m_last_cmd_time).toSec() > m_cmd_timeout)
        {
            ROS_WARN_STREAM( "Cmd timed out." );
            resetCmd();
        }
        
        m_udp.SetSend(m_cmd);
        m_cmd_mutex.unlock();
        
        publishHighState();
        publishJointState();
    }

    void AlienGoBridge::publishJointState()
    {
        boost::mutex::scoped_lock lock(m_state_mutex);
        
        m_jointStateMsg.header.stamp = ros::Time::now();
        for ( int i = 0; i < 12; ++i )
        {
            m_jointStateMsg.position[i] = m_state.motorState[i].q;
            m_jointStateMsg.velocity[i] = m_state.motorState[i].dq;
            m_jointStateMsg.effort[i] = m_state.motorState[i].tauEst;
        }

        m_joints_pub.publish(m_jointStateMsg);
    }

    void AlienGoBridge::publishHighState()
    {
        boost::mutex::scoped_lock lock(m_state_mutex);

        m_stateMsg.header.stamp = ros::Time::now();
        
        m_stateMsg.levelFlag = m_state.levelFlag;
        m_stateMsg.commVersion = m_state.commVersion;
        m_stateMsg.robotID = m_state.robotID;
        m_stateMsg.SN = m_state.SN;
        m_stateMsg.bandWidth = m_state.bandWidth;
        m_stateMsg.mode = m_state.mode;
        
        m_stateMsg.imu.quaternion[0] = m_state.imu.quaternion[0];
        m_stateMsg.imu.quaternion[1] = m_state.imu.quaternion[1];
        m_stateMsg.imu.quaternion[2] = m_state.imu.quaternion[2];
        m_stateMsg.imu.quaternion[3] = m_state.imu.quaternion[3];
        m_stateMsg.imu.gyroscope[0] = m_state.imu.gyroscope[0];
        m_stateMsg.imu.gyroscope[1] = m_state.imu.gyroscope[1];
        m_stateMsg.imu.gyroscope[2] = m_state.imu.gyroscope[2];
        m_stateMsg.imu.accelerometer[0] = m_state.imu.accelerometer[0];
        m_stateMsg.imu.accelerometer[1] = m_state.imu.accelerometer[1];
        m_stateMsg.imu.accelerometer[2] = m_state.imu.accelerometer[2];
        m_stateMsg.imu.rpy[0] = m_state.imu.rpy[0];
        m_stateMsg.imu.rpy[1] = m_state.imu.rpy[1];
        m_stateMsg.imu.rpy[2] = m_state.imu.rpy[2];
        m_stateMsg.imu.temperature = m_state.imu.temperature;

        for ( int i = 0; i < 20; ++i )
        {
            m_stateMsg.motorState[i].q = m_state.motorState[i].q;
            m_stateMsg.motorState[i].dq = m_state.motorState[i].dq;
            m_stateMsg.motorState[i].tauEst = m_state.motorState[i].tauEst;
        }
        
        m_stateMsg.gaitType = m_state.gaitType;
        m_stateMsg.footRaiseHeight = m_state.footRaiseHeight;
        m_stateMsg.bodyHeight = m_state.bodyHeight;

        m_stateMsg.position[0] = m_state.position[0];
        m_stateMsg.position[1] = m_state.position[1];
        m_stateMsg.position[2] = m_state.position[2];
        
        m_stateMsg.velocity[0] = m_state.velocity[0];
        m_stateMsg.velocity[1] = m_state.velocity[1];
        m_stateMsg.velocity[2] = m_state.velocity[2];
        
        m_stateMsg.yawSpeed = m_state.yawSpeed;
        
        for ( int i = 0; i < 4; ++i )
        {
            m_stateMsg.footPosition2Body[i].x = m_state.footPosition2Body[i].x;
            m_stateMsg.footPosition2Body[i].y = m_state.footPosition2Body[i].y;
            m_stateMsg.footPosition2Body[i].z = m_state.footPosition2Body[i].z;
            
            m_stateMsg.footSpeed2Body[i].x = m_state.footSpeed2Body[i].x;
            m_stateMsg.footSpeed2Body[i].y = m_state.footSpeed2Body[i].y;
            m_stateMsg.footSpeed2Body[i].z = m_state.footSpeed2Body[i].z;
            
            m_stateMsg.footForce[i] = m_state.footForce[i];
        }
        
        for ( int i = 0; i < 40; ++i )
            m_stateMsg.wirelessRemote[i] = m_state.wirelessRemote[i];

        m_stateMsg.reserve = m_state.reserve;
        m_stateMsg.crc = m_state.crc;
        
        m_state_pub.publish(m_stateMsg);
    }
}