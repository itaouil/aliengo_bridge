#include <ros/ros.h>
#include <aliengo_bridge/aliengo_bridge.h>

int main( int argc, char** argv )
{
    ros::init( argc, argv, "aliengo_bridge" );
    std::cout << "Starting AlienGo bridge. Communication level is set to HIGH-level." << std::endl;
    
    ros::NodeHandle ph( "~" );
    aliengo_bridge::AlienGoBridge bridge( ph );
    
    ros::spin();
    return 0;
}
