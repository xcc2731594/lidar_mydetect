#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class GroundPlanefitBase
{
  public:
    
    GroundPlanefitBase( const float &planefit_thre = 0.2,
                        const float &abs_ground_height = -1.7,
                       const float &ground_thre = 0.2);
    ~GroundPlanefitBase() {}

     
    bool genPlane(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

     
    bool computeLocateZ(const pcl::PointXYZI &locate, float &locateZ);
 
    bool getLabelMat(cv::Mat &label_mat);

     
    bool getGroundPlaneModel(Eigen::VectorXf &plane_model_coefficients);

     
    bool computeNoground(pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);

  protected:
    float planefit_thre_, abs_ground_height_,ground_thre_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ori_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr proc_cloud_ptr_;
    bool has_input_;
    cv::Mat label_mat_;
    Eigen::VectorXf plane_model_coefficients_;

  private:
    float computeXYDis(const pcl::PointXYZI &pt);
    float computeBeta(const pcl::PointXYZI &pt);
    bool isInvalidPoint(const pcl::PointXYZI &pt);
    // bool computeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2);
};