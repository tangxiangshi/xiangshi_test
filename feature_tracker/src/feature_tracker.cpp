#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

FeatureTracker::FeatureTracker()
{

}

void FeatureTracker::readIntrinsicParameter(const std::string &calib_file)
{
  ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
