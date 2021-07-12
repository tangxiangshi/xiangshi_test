#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

ros::Publisher pub_img, pub_match;
ros::Publisher pub_restart;

bool first_image_flag = true;
double first_image_time;
double last_image_time = 0;
int pub_count = 1;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
      first_image_flag = false;
      first_image_time = img_msg->header.stamp.toSec();
      last_image_time = img_msg->header.stamp.toSec();
      return;
    }
    // strange camera stream
    if(img_msg->header.stamp.toSec() - last_image_time > 1.0 || last_image_time > img_msg->header.stamp.toSec())
    {
      ROS_WARN("image discontinue! reset the feature tracker");
      first_image_flag = true;
      last_image_time = 0;
      pub_count = 1;
      std_msgs::Bool restart_flag;
      restart_flag.data = true;
      pub_restart.publish(restart_flag);
      return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    if(std::round(1.0 * pub_count / (last_image_time - first_image_time)) <= FREQ) // THIS CONDITION IS FOR IMAGE PUBLISH (NOT EVERY FRAME IS NEEDED) SO WE CONTROL THE UNDER FREQ FRAME TO PUB
    {
      PUB_THIS_FRAME = true;
      // if our frequency is almost equally to the setting freq we recount the pub_count and first_image_time;
      if(std::abs(1.0 * pub_count / (last_image_time - first_image_time) - FREQ) < 0.01 * FREQ)
      {
        pub_count = 0;
        first_image_time = img_msg->header.stamp.toSec();
      }
    }
    else {
      PUB_THIS_FRAME = false;
    }
    // transform ros msg to cv mat
    cv_bridge::CvImageConstPtr ptr;
    // change sensor_msg image 8uc1 to  mono8
    if(img_msg->encoding == "8UC1")
    {
      sensor_msgs::Image image;
      image.header = img_msg->header;
      image.height = img_msg->height;
      image.width = img_msg->width;
      image.is_bigendian = img_msg->is_bigendian;
      image.step = img_msg->step;
      image.data = img_msg->data;
      image.encoding = "mono8";
      ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    }
    else {
      ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
    }
    cv::Mat show_img = ptr->image;
    TicToc t_r;

    // for this step pub frame, we need to consider feature_point_cloud; and relevant parameters
    if(PUB_THIS_FRAME)
    {
      pub_count++;
      sensor_msgs::PointCloudConstPtr feature_points(new sensor_msgs::PointCloud);


    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

//  for(int i = 0; i < NUM_OF_CAM; i++)
//  {

//  }

  ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
  pub_match = n.advertise<sensor_msgs::Image>("feature_img", 1000);
  pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);

  ros::spin();
  return 0;


}
