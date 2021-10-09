# ifndef BOXER_H_
#define BOXER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class BoundingBoxCalculator{

public:
struct BoundingBox
{
  BoundingBox(){}
  cv::Point3f corners[8]; 
  cv::Point3f size; 
  cv::Point3f center; 
 
  float angle; 
};

BoundingBox calcBoundingBox(const pcl::PointCloud<pcl::PointXYZI> &segment);

BoundingBox calcSimpleBox(const pcl::PointCloud<pcl::PointXYZI> &segment);
void cal_cosandsin();

void draw_box(const BoundingBox& box, const int& marker_id, visualization_msgs::Marker& marker, float scale);
void draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, visualization_msgs::Marker& marker);
private:

BoundingBox scaleBox(const BoundingBox &box, float scale);


float theta_ ;
std::vector<float> tabCos_,tabSin_;


};

#endif