#include "cluster.h"
#include <stack>

PolarGridBase::PolarGridBase(const float &polar_angle, const float &polar_range_size,
                                  const float &max_range, const float &max_z_height)
{
    polar_angle_ = polar_angle;
    polar_range_size_ = polar_range_size;
    max_range_ = max_range;
    max_z_height_ = max_z_height;

    // polar_min_rad_ = (polar_angle_ / 180.) * M_PI;
    polar_min_rad_ = (polar_angle_ / (1.5*180.)) * M_PI;
    // MSegs_:角度方向的栅格数
    MSegs_ = (2 * M_PI + 0.001) / polar_min_rad_;
    // NBins_:长度方向的栅格数
    NBins_ = (max_range_ + 0.001) / polar_range_size_;
    // std::cout<<"MSegs_:"<<MSegs_<<std::endl;
    // std::cout<<"NBins_:"<<NBins_<<std::endl;
    polar_leave_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    polar_save_grid_pts_.resize(NBins_);
    for (int j = 0; j < NBins_; ++j)
    {
        polar_save_grid_pts_[j].resize(MSegs_);
    }
    air_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    dilation_size_ = 1;
    polar_has_input_ = false;
}

bool PolarGridBase::polarGridCluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                                     std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters)
{
    // 对栅格填入点
    if (!genPolarGrowthGrid(in_cloud_ptr))
    {
        // 从代码看,in_cloud_ptr为空的时候返回false
        return false;
    }
    cv::Mat ground_label_mat;
    // 对栅格中的点标label
    polarGroundCluster(ground_label_mat);
    // 遍历点,把同一label的点放一个cluster(vector)
    polarUpGrowthCluster(ground_label_mat, clusters);
    return true;
}

bool polarComputeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return pt1.z < pt2.z;
}
// 每次获得360度点云数据后，重新对polar_save_grid_pts栅格填入一次点
bool PolarGridBase::genPolarGrowthGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr)
{
    if (in_cloud_ptr->empty())
    {
        std::cout << "the input cloud is empty!!" << std::endl;
        return false;
    }

    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            polar_save_grid_pts_[j][i].clear();
        }
    }
    polar_leave_cloud_ptr_->clear();
    air_cloud_ptr_->clear();
    pcl::PointXYZI tmp_pt;
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        tmp_pt = in_cloud_ptr->points[i];
        if (isnan(tmp_pt.x) || isnan(tmp_pt.y) || isnan(tmp_pt.z) || isnan(tmp_pt.intensity))
            continue;
        // 计算点的距离
        float tmp_range = computeXYDis(tmp_pt);
        // 去除远离点
        if (tmp_range > max_range_)
        {
            polar_leave_cloud_ptr_->push_back(tmp_pt);
            continue;
        }
        // 去除空中点
        if (tmp_pt.z > max_z_height_)
        {
            air_cloud_ptr_->push_back(tmp_pt);
            continue;
        }
        // 计算点的角度
        float tmp_beta = computeBeta(tmp_pt);
        // 计算点对应的栅格坐标
        int x_index = tmp_beta / polar_min_rad_;
        int y_index = tmp_range / polar_range_size_;

        if(x_index==MSegs_)
            x_index--;
        if(y_index==NBins_)
            y_index--;
        polar_save_grid_pts_[y_index][x_index].push_back(tmp_pt);
    
    }

    polar_has_input_ = true;
    return true;
}

void PolarGridBase::polarGroundCluster(cv::Mat &label_mat)
{
    // bin_mat:栅格二维列表,只含有0,1(1:有点,0:无点)
    // label_mat:栅格二维列表,含有1,2,3,4,5..(label)
    cv::Mat bin_mat;
    bin_mat = cv::Mat::zeros(NBins_, MSegs_, CV_8U);
    // 给栅格二维列表赋值1/0
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (polar_save_grid_pts_[j][i].empty())
                continue;
            bin_mat.at<uchar>(j, i) = 1;
        }
    }
    // 膨胀算法
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * dilation_size_ + 1, 2 * dilation_size_ + 1),
                                                cv::Point(dilation_size_, dilation_size_));
    cv::dilate(bin_mat, bin_mat, element);

    label_mat.release();
    label_mat = cv::Mat::zeros(NBins_, MSegs_, CV_32S);

    int label = 0; // start by 1
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (bin_mat.at<uchar>(j, i) == 0)
                continue;
            if (label_mat.at<int>(j, i) != 0)
                continue;
            // 栈存放相邻点
            std::stack<cv::Point2i> neighborPixels;
            neighborPixels.push(cv::Point2i(i, j));
            label++;
            label_mat.at<int>(j, i) = label;
            while (!neighborPixels.empty())
            {
                // 取出一个
                cv::Point2i cur_index = neighborPixels.top();
                // 弹掉一个
                neighborPixels.pop();
                for (int jj = cur_index.y - 1; jj <= cur_index.y + 1; ++jj)
                {
                    // 长度方向超出边界
                    if (jj >= NBins_ || jj < 0)
                        continue;
                    for (int ii = cur_index.x - 1; ii <= cur_index.x + 1; ++ii)
                    {
                        int ii_index = ii;
                        // 角度方向360度循环
                        if (ii_index >= MSegs_)
                            ii_index -= MSegs_;
                        if (ii_index < 0)
                            ii_index += MSegs_;
                        // 这句的效果是cur点的上下左右(十字)满足if条件
                        if (jj == cur_index.y || ii_index == cur_index.x)
                        {
                            // bin_mat内容为0,跳过
                            if (bin_mat.at<uchar>(jj, ii_index) == 0)
                                continue;
                            // label_mat内容不为0,说明已经标注过,跳过
                            if (label_mat.at<int>(jj, ii_index) != 0)
                                continue;
                            label_mat.at<int>(jj, ii_index) = label;
                            neighborPixels.push(cv::Point2i(ii_index, jj));
                        }
                    }
                }
            }
        }
    }
    max_label_ = label;
}

void PolarGridBase::polarUpGrowthCluster(const cv::Mat &ground_mat, std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI> > tmp_clusters;
    tmp_clusters.clear();
    clusters.clear();
    tmp_clusters.resize(max_label_ + 1);
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (polar_save_grid_pts_[j][i].empty())
                continue;
            // 栅格不为空,遍历栅格中的点
            for (int k = 0; k < polar_save_grid_pts_[j][i].size(); ++k)
            {
                // ground_mat = label_mat,给对应label的点云(vector)添加点
                tmp_clusters[ground_mat.at<int>(j, i)].push_back(polar_save_grid_pts_[j][i][k]);
            }
        }
    }
    // 将tmp_clusters中的点转存到clusters,其中索引其实就是label
    for (int i = 0; i < tmp_clusters.size(); ++i)
    {
        if (tmp_clusters[i].points.size() < 9)
            continue;
        // 给点云容器(也是vector)添加点云
        clusters.push_back(tmp_clusters[i]);
    }
    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(tmp_clusters);
}

void PolarGridBase::clusterstoColor(std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters,pcl::PointCloud<pcl::PointXYZRGB>& color_cloud)
{
    for (int i = 0; i < clusters.size(); ++i)
    {
        int red = rand() % 256;
        int green = rand() % 256;
        int blue = rand() % 256;
        for (int j = 0; j < clusters[i].points.size(); ++j)
        {
            pcl::PointXYZRGB tmp_point;
            tmp_point.x = clusters[i].points[j].x;
            tmp_point.y = clusters[i].points[j].y;
            tmp_point.z = clusters[i].points[j].z;
            tmp_point.r = red;
            tmp_point.g = green;
            tmp_point.b = blue;
            color_cloud.push_back(tmp_point);
        }
    }
}

inline
float PolarGridBase::computeBeta(const pcl::PointXYZI &pt)
{
    float beta = atan2(pt.y, pt.x);
    beta = beta < 0. ? beta += 2 * M_PI : beta;
    beta = beta > (2 * M_PI) ? beta -= 2 * M_PI : beta;
    return beta;
}

inline
float PolarGridBase::computeXYDis(const pcl::PointXYZI &pt)
{
    return sqrtf(powf(pt.x, 2.) + powf(pt.y, 2.));
}