#include "boxer.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

// 把cos和sin计算好，后面直接查表速度快
void BoundingBoxCalculator::cal_cosandsin()
{
    theta_ = CV_PI / 180.;
    int numangle = cvRound(CV_PI / theta_);
    tabCos_.resize(numangle);
    tabSin_.resize(numangle);

    float ang = 0;
    for (int n = 0; n < numangle; ang += theta_, n++)
    {
        tabSin_[n] = sinf(ang);
        tabCos_[n] = cosf(ang);
    }
}

BoundingBoxCalculator::BoundingBox BoundingBoxCalculator::calcBoundingBox(const pcl::PointCloud<pcl::PointXYZI> &segment)
{
    int seg_size = segment.size();
    //异常处理
    if (seg_size < 1)
    {
        std::cout << "empty segment!" << std::endl;
        return BoundingBox();
    }

    //寻找最大最小范围
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(segment, min_point, max_point);

    //   cv::Point3f seg_center((max_point.x + min_point.x)*0.5f, (max_point.y + min_point.y)*0.5f, (max_point.z + min_point.z)*0.5f);
    //   float diag_len = cv::Point3f(max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z).getLength();
    //   float seg_ang = atan2f(seg_center.y, seg_center.x);

    //   if (diag_len < 5 && seg_center.getLength()<detect_radius_ || cosf(seg_ang)<-0.98f)
    //   {
    //     return calcSimpleBox(segment);
    //   }

    //------加速手段： 对于点数过多的segment，抽稀处理-------
    pcl::PointCloud<pcl::PointXYZI> sparse_segment;

    int max_seg_num = 600;
    
    int sparse_step = cvCeil((float)seg_size / max_seg_num);
    // 间隔sparse_step抽取点
    if (sparse_step > 1)
    {
        for (int i = 0; i < seg_size; i += sparse_step)
        {
            sparse_segment.points.push_back(segment.points[i]);
        }
    }
    else
    {
        sparse_segment = segment;
    }

    int sparse_segment_size = sparse_segment.size();

    float range_x = max_point.x - min_point.x;
    float range_y = max_point.y - min_point.y;

    float grid_size = 0.1f; //fmaxf(range_x, range_y)/50.f;
                            //   grid_size = grid_size > grid_size_ ? grid_size : grid_size_;

    float grid_size_inv = 1.f / grid_size;

    //为提升稳定性，在求hough line之前加入栅格化操作,https://www.cnblogs.com/liutianrui1/articles/10096098.html
    // cvRound()：返回跟参数最接近的整数值，即四舍五入;
    // cvFloor()：返回不大于参数的最大整数值，即向下取整;
    // cvCeil()：返回不小于参数的最小整数值，即向上取整;
    int grid_width = cvCeil(range_x * grid_size_inv);
    int grid_height = cvCeil(range_y * grid_size_inv);

    cv::Mat grid_map = cv::Mat::zeros(grid_height + 1, grid_width + 1, CV_8UC1);
    for (int i = 0; i < sparse_segment_size; ++i)
    {
        if (sparse_segment.points[i].z - min_point.z < (max_point.z - min_point.z) * 0.7f)
        {
            int r = (sparse_segment.points[i].y - min_point.y) * grid_size_inv;
            int c = (sparse_segment.points[i].x - min_point.x) * grid_size_inv;
            grid_map.at<uchar>(r, c) = 1;
        }
    }

    //-------------------------------------hough line detect------------------------------------
    // https://blog.csdn.net/leonardohaig/article/details/87907462
    // https://blog.csdn.net/zx3517288/article/details/47834337
    // https://www.cnblogs.com/liutianrui1/articles/10096098.html
    int numangle = tabSin_.size();
    // by xcc 同上
    // float theta_ = CV_PI / 180.;
    // int numangle = cvRound(CV_PI / theta_);

    // θ角范围0~180，ρ范围-ρ~ρ??
    // int numrho = cvRound((std::max(range_x , range_y) * 2 + 1) * grid_size_inv); //by xcc
    // 这边range_x + range_y是为了包含住ρ范围
    int numrho = cvRound(((range_x + range_y) * 2 + 1) * grid_size_inv);//使用grid_size代替hough变换的rho，即半径步长
    // +2是为了左右扩展么,膨胀
    int acc_num = (numangle + 2) * (numrho + 2);
    // _accum充当二维数组((numangle + 2) , (numrho + 2))
    cv::AutoBuffer<int> _accum(acc_num);
    int *accum = _accum;
    // 把accum清零,它是对较大的结构体或数组进行清零操作的一种最快方法
    memset(accum, 0, sizeof(accum[0]) * acc_num);

    // stage 1. fill accumulator
    int r_tans = (numrho - 1) * 0.5f;
    for (int i = 0; i < grid_map.rows; i++)
        for (int j = 0; j < grid_map.cols; j++)
        {
            if (grid_map.at<uchar>(i, j) > 0)
            { //处理非空
                for (int n = 0; n < numangle; n++)
                {
                    // Hough极坐标变换式,直角坐标系:i,j    极坐标系:r,n
                    // r_tans是为了把-ρ移到正区间
                    int r = cvRound(j * tabCos_[n] + i * tabSin_[n]) + r_tans;
                    accum[(n + 1) * (numrho + 2) + r + 1]++;
                }
            }
        }

    // find the max
    int acc_max = 0, idx_max = -1;
    for (int i = 0; i < acc_num; ++i)
    {
        if (accum[i] > acc_max)
        {
            acc_max = accum[i];
            idx_max = i;
        }
    }

    // 计算box主方向的旋转角度
    float angle = (cvFloor(idx_max / (numrho + 2)) - 1) * theta_;

    //-----------------------------------end hough line detect-------------------------------
    //根据主方向旋转点云
    float cos_ang = cosf(-angle), sin_ang = sinf(-angle);
    // 点云中心
    cv::Point2f center_point((min_point.x + max_point.x) * 0.5, (min_point.y + max_point.y) * 0.5);
    std::vector<float> rotate_pts_x(sparse_segment_size), rotate_pts_y(sparse_segment_size);
    for (int i = 0; i < sparse_segment_size; ++i) // TODO: 此处较耗时，需要优化
    {
        float x = (sparse_segment.points[i].x - center_point.x);
        float y = (sparse_segment.points[i].y - center_point.y);

        rotate_pts_x[i] = x * cos_ang - y * sin_ang;
        rotate_pts_y[i] = x * sin_ang + y * cos_ang;
    }

    float max_x = *std::max_element(rotate_pts_x.begin(), rotate_pts_x.end());
    float min_x = *std::min_element(rotate_pts_x.begin(), rotate_pts_x.end());
    float max_y = *std::max_element(rotate_pts_y.begin(), rotate_pts_y.end());
    float min_y = *std::min_element(rotate_pts_y.begin(), rotate_pts_y.end());

    cv::Point2f corners[4];
    corners[0] = cv::Point2f(min_x, min_y);
    corners[1] = cv::Point2f(max_x, min_y);
    corners[2] = cv::Point2f(max_x, max_y);
    corners[3] = cv::Point2f(min_x, max_y);

    //将得到的box顶点再变换回去
    cv::Point2f corners_back[4];
    cos_ang = cosf(angle), sin_ang = sinf(angle);

    for (int i = 0; i < 4; ++i)
    {
        corners_back[i].x = corners[i].x * cos_ang - corners[i].y * sin_ang + center_point.x;
        corners_back[i].y = corners[i].x * sin_ang + corners[i].y * cos_ang + center_point.y;
        
    }

    BoundingBox box;

    for (int i = 0; i < 4; ++i)
    {
        // 上下各四个点
        box.corners[i] = cv::Point3f(corners_back[i].x, corners_back[i].y, min_point.z);
        box.corners[i + 4] = cv::Point3f(corners_back[i].x, corners_back[i].y, max_point.z);
    }

    box.center = (box.corners[0] + box.corners[6]) * 0.5f;
    //   box.polar_center = cv::Point2f(box.center.getLength(), atan2f(box.center.y, box.center.x));

    box.size = cv::Point3f(max_x - min_x, max_y - min_y, max_point.z - min_point.z); //注意box的size是根据box自身坐标系计算的，是准确的
    box.angle = angle;

    if (box.size.x < box.size.y) //保证x朝向为最长边
    {
        float tmp = box.size.x;
        box.size.x = box.size.y;
        box.size.y = tmp;
        box.angle += CV_PI * 0.5;
    }

    //  box.anchor = box.corners[findBoxAnchor(box)];

    return box;
}

BoundingBoxCalculator::BoundingBox BoundingBoxCalculator::calcSimpleBox(const pcl::PointCloud<pcl::PointXYZI> &segment)
{
    if (segment.size() < 1)
    {
        std::cout << "empty segment!" << std::endl;

        return BoundingBox();
    }

    // step 1: 寻找最大最小范围
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(segment, min_point, max_point);

    cv::Point2f corners[4];
    corners[0] = cv::Point2f(min_point.x, min_point.y);
    corners[1] = cv::Point2f(max_point.x, min_point.y);
    corners[2] = cv::Point2f(max_point.x, max_point.y);
    corners[3] = cv::Point2f(min_point.x, max_point.y);

    BoundingBox box;

    for (int i = 0; i < 4; ++i)
    {
        box.corners[i] = cv::Point3f(corners[i].x, corners[i].y, min_point.z);
        box.corners[i + 4] = cv::Point3f(corners[i].x, corners[i].y, max_point.z);
    }

    box.center = (box.corners[0] + box.corners[6]) * 0.5f;
    // box.polar_center = cv::Point2f(box.center.getLength(), atan2f(box.center.y, box.center.x));
    box.size = cv::Point3f(max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);
    box.angle = 0;

    if (box.size.x < box.size.y) //保证x朝向为最长边
    {
        float tmp = box.size.x;
        box.size.x = box.size.y;
        box.size.y = tmp;
        box.angle += CV_PI * 0.5;
    }

    //  box.anchor = box.corners[findBoxAnchor(box)];

    return box;
}

BoundingBoxCalculator::BoundingBox BoundingBoxCalculator::scaleBox(const BoundingBox &box, float scale)
{
    if (fabsf(scale - 1.f) < 1e-6)
        return box;

    cv::Point3f center = box.center;

    BoundingBox box_out = box;

    for (int i = 0; i < 8; ++i)
    {
        box_out.corners[i] = (box.corners[i] - center) * scale + center;
    }

    box_out.size = box.size * scale;

    //  box_out.anchor = box.corners[findBoxAnchor(box_out)];

    return box_out;
}

void BoundingBoxCalculator::draw_box(const BoundingBox &box, const int &marker_id, visualization_msgs::Marker &marker, float scale)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "frame_id_711";
    std::vector<geometry_msgs::Point> cub_points;

    BoundingBox box_s = scaleBox(box, scale); // 放大scale倍后的框

    for (int i = 0; i < 8; ++i)
    {
        geometry_msgs::Point pts;
        pts.x = box_s.corners[i].x;
        pts.y = box_s.corners[i].y;
        pts.z = box_s.corners[i].z;
        cub_points.push_back(pts);
    }

    marker.scale.x = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    
    marker.color.a = 1;

    marker.lifetime = ros::Duration(0.1);

    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[0]);
    // horizontal high points for lines
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[4]);
    // vertical points for lines
    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[7]);
}


void BoundingBoxCalculator::draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, visualization_msgs::Marker& marker)
{
	//--------------标记跟踪信息----------
	marker.id = marker_id;
    marker.header.frame_id = "frame_id_711";
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x;
	marker.pose.position.y = pos.y;
	marker.pose.position.z = pos.z;
    marker.ns = "pos_info";
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.r = 1.0f;
	marker.color.g = 0.0;
	marker.color.b = 0.f;
	marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.1);
	marker.text = info;
}