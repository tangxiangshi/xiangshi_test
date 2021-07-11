#include "parameters.h"

int ROW;
int COL;
int FOCAL_LENGTH;

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;

int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int FISHEYE;
bool PUB_THIS_FRAME;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;
 // 获取参数“name”的value，写入到ans上
  //
  if(n.getParam(name,ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else(n.getParam(name, ans)
  {
    ROS_INFO_STREAM("Failed to load" << name)
  }

}

void readParameters(ros::NodeHandle &n)
{
  std::string config_file;

}
