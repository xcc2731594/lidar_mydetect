#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class PolarGridBase
{
  public:
 
    // PolarGridBase(const float &polar_angle = 1., const float &polar_range_size = 0.25,
    PolarGridBase(const float &polar_angle = 1., const float &polar_range_size = 0.20,
                  const float &max_range = 200., const float &max_z_height = 0.7);
    ~PolarGridBase() {}
 
    bool polarGridCluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                          std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters);

     
    bool genPolarGrowthGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

     
    bool genPolarGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

    void clusterstoColor(std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters,
                            pcl::PointCloud<pcl::PointXYZRGB>& color_cloud);



  protected:
    float max_range_, max_z_height_, polar_angle_, polar_range_size_;
    int MSegs_, NBins_;
    float polar_min_rad_;
    int dilation_size_;
    bool polar_has_input_;
    bool is_sort_grids_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr polar_leave_cloud_ptr_;
    std::vector<std::vector<std::vector<pcl::PointXYZI> > > polar_save_grid_pts_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr air_cloud_ptr_;

  private:
    // bool genPolarGrowthGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);
    float computeXYDis(const pcl::PointXYZI &pt);
    float computeBeta(const pcl::PointXYZI &pt);
    void polarGroundCluster(cv::Mat &label_mat);

    void polarUpGrowthCluster(const cv::Mat &ground_mat, 
                                std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters);
    int max_label_;
};