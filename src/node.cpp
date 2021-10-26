#include <ros/ros.h>
#include <aliengo_bridge/aliengo_bridge.h>

int main( int argc, char** argv )
{
    ros::init( argc, argv, "aliengo_bridge" );
    std::cout << "Communication level is set to HIGH-level." << std::endl
    << "WARNING: Make sure the robot is standing on the ground." << std::endl
    << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    ros::NodeHandle ph( "~" );
    aliengo_bridge::AlienGoBridge bridge( ph );
    
    ros::spin();
    return 0;
}
