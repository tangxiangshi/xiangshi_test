#include <ros/ros.h>
#include "../include/vision_test/point_manager.h"
int main(int argc, char** argv)
{   
    ros::init(argc, argv, "vision_test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);


    return 0;
}
