#include <ros/ros.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <mutex>
#define TRIGGER_PERIOD 0.05
#define TOLERANCE 0.008
using namespace std;

mutex m_buf;
queue<mavros_msgs::CamIMUStamp> trigger_buf;
ros::Publisher correct_img_pub;
sensor_msgs::Image image_correct;
void triggerCallback(const mavros_msgs::CamIMUStamp::ConstPtr &msg)
{
    m_buf.lock();
    mavros_msgs::CamIMUStamp trigger_msg = *msg;
    trigger_buf.push(trigger_msg);
    m_buf.unlock();
}
void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    m_buf.lock();
    while (1)
    {
        if (!trigger_buf.empty())
        {
            // trigger time should happened before image time
            // int delta_sec = (msg->header.stamp.sec - trigger_buf.front().frame_stamp.sec) * 1000000000;
            // int delta_nsec = msg->header.stamp.nsec - trigger_buf.front().frame_stamp.nsec;
            // int time_difference_msec = (delta_sec + delta_nsec) / 1000000;
            // float time_difference = time_difference_msec; // change type!!
            // time_difference = time_difference / 1000;
            double time_difference = msg->header.stamp.toSec() - trigger_buf.front().frame_stamp.toSec();
            if (time_difference > 0)
            {
                if (time_difference > TOLERANCE + TRIGGER_PERIOD)
                {
                    trigger_buf.pop();
                    ROS_INFO("The setting with bug!, td:%fsec", time_difference);
                    // ROS_INFO_STREAM("image time" << msg->header.stamp.toSec());
                    // ROS_INFO_STREAM("triger time" << trigger_buf.front().frame_stamp.toSec());
                }
                else
                {
                    ROS_INFO("td: %f sec", time_difference);

                    //refine image timestamp and publish
                    image_correct = *msg;
                    image_correct.header.stamp.sec = trigger_buf.front().frame_stamp.sec;
                    image_correct.header.stamp.nsec = trigger_buf.front().frame_stamp.nsec;
                    correct_img_pub.publish(image_correct);
                    trigger_buf.pop();
                    break;
                }
            }
            else
            {
                ROS_INFO("Throw first image!");
                break;
            }
        }
        else
        {
            ROS_INFO("Wait");
            break;
        }
    }
    m_buf.unlock();
}

//mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync");
    ros::NodeHandle nh;
    ros::Subscriber trigger_sub = nh.subscribe<mavros_msgs::CamIMUStamp>("/mavros/cam_imu_sync/cam_imu_stamp", 2, triggerCallback);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/cam0/arducam/triggered/camera/image_raw", 1, imageCallback);
    // ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/arducam/triggered/camera/image_raw", 1, imageCallback);
    correct_img_pub = nh.advertise<sensor_msgs::Image>("/sync/image", 2);

    ros::spin();

    return 0;
}
