#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ground_detect.h"
#include "cluster.h"
#include "boxer.h"
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>
// #include "tracker.h"

#include <pcl/filters/voxel_grid.h>


ros::NodeHandle *nh;
ros::Publisher points_pub, points_ground_pub, pub_marker, raw_points_pub;
GroundPlanefitBase *GroundPlanefitor;
PolarGridBase *PolarGridBasor;
BoundingBoxCalculator *boxer;
// Track *Tracker;

std::string to_string_with_precision(const double a_value,int precison);
// 自定义保留2为小数的函数
std::string to_string_with_precision(const double a_value,int precison)
{
    std::ostringstream out;
    out << std::fixed<<std::setprecision(precison) << a_value;
    return out.str();
};
void Callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    pcl::console::TicToc tt;
    tt.tic();
    double frame_time = points_msg->header.stamp.toSec();
    pcl::PCLPointCloud2 pcl_pc2;
    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*points_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 把pcl::PCLPointCloud2数据格式的点云转化为pcl::PointCloud<pointT>格式
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_raw);

    //体素滤波
    typename pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud_raw);
    voxel_filter.setLeafSize(0.3f,0.3f,0.3f);
    voxel_filter.filter(*cloud_raw_1);


    // 拟合地平面
    GroundPlanefitor->genPlane(cloud_raw);
    // 划分地面点和非地面点
    GroundPlanefitor->computeNoground(noground_cloud, ground_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    // 对非地面点进行极坐标聚类
    // PolarGridBasor->polarGridCluster(noground_cloud, clusters);
    // 水面上不用去地面点
    PolarGridBasor->polarGridCluster(cloud_raw_1, clusters);

    std::vector<BoundingBoxCalculator::BoundingBox> boxes;
    // std::vector<Track::tracker> trackers;
    //simple classify the object
    for (int i = 0; i < clusters.size(); ++i)
    {
        // 对同一类的点云计算外接框
        BoundingBoxCalculator::BoundingBox tmp_box = boxer->calcBoundingBox(clusters[i]);
        // BoundingBoxCalculator::BoundingBox tmp_box = boxer->calcSimpleBox(clusters[i]);
        // 限制框选最大物体
        // if (tmp_box.size.x * tmp_box.size.y > 18)
        //     continue;
        Eigen::Vector2f v1(tmp_box.center.x, tmp_box.center.y);
        float distance = v1.norm();
        // 距离大于8米的不画框,考虑删除
        // if (tmp_box.size.z > 1.7 && distance > 8)
        //     continue;
        // if (tmp_box.size.x / tmp_box.size.y > 4 && tmp_box.size.y < 0.4)
        // if (tmp_box.size.x / tmp_box.size.y > 3  )
        //     continue;
        // if (tmp_box.size.x * tmp_box.size.y > 10  )
        //     continue;
        //  if (tmp_box.size.x < 50 )
        //     continue;
        // 得到box
        boxes.push_back(tmp_box);
        // Track::tracker tmp_track;
        // tmp_track.num_points = clusters[i].points.size();
        // tmp_track.center[0] = tmp_box.center.x;
        // tmp_track.center[1] = tmp_box.center.y;
        // tmp_track.center[2] = 0;
        // tmp_track.size.x = tmp_box.size.x;
        // tmp_track.size.y = tmp_box.size.y;
        // tmp_track.size.z = tmp_box.size.z;
        // tmp_track.corners[0] = tmp_box.corners[0];
        // tmp_track.corners[1] = tmp_box.corners[1];
        // tmp_track.corners[2] = tmp_box.corners[2];
        // tmp_track.corners[3] = tmp_box.corners[3];
        // tmp_track.corners[4] = tmp_box.corners[4];
        // tmp_track.corners[5] = tmp_box.corners[5];
        // tmp_track.corners[6] = tmp_box.corners[6];
        // tmp_track.corners[7] = tmp_box.corners[7];
        // tmp_track.latest_tracked_time = frame_time;
        // trackers.push_back(tmp_track);
    }
    int box_count = 0;
    // 发布框选信息
    visualization_msgs::MarkerArray markers;
    // // 发布轨迹信息
    // visualization_msgs::MarkerArray pathers;
    markers.markers.clear();
    // pathers.markers.clear();


    // std::cout<<"boxes.size():"<<boxes.size()<<std::endl;
    // std::cout<<"trackers.size():"<<trackers.size()<<std::endl;

    

    for (int i = 0; i < boxes.size(); ++i)
    {
        BoundingBoxCalculator::BoundingBox box = boxes[i];
        visualization_msgs::Marker marker1;
        // 画box
        boxer->draw_box(box, box_count++, marker1, 1.1);// 1.1 框的放大比例
        cv::Point3f pos;
        pos.x = box.center.x;
        pos.y = box.center.y;
        pos.z = box.center.z + 1.7;
        // 计算距离用于显示
        Eigen::Vector2f v2(pos.x, pos.y);
        float distance = round(v2.norm()*100)/100.0;
        float angle = atan2( pos.y, pos.x)*180/3.1415;
        std::string info_dis;
        std::string info_ang;
        info_dis = to_string_with_precision(distance,2);
        info_ang = to_string_with_precision(angle,2);
        std::string name_distance = ",distance:";
        std::string name_angle = ",angle:";

        std::string info;
        float area = box.size.x * box.size.y;
        // if (area > 1.5 && area < 6)
        // {
        //     info = "Car";
        //     trackers[i].class_type = 1;
        // }

        if (box.size.x > 2.0 && box.size.x < 5.0 && box.size.y>0.8 && box.size.y<2.0)
        {
            info = "Car"+name_distance+info_dis+name_angle+info_ang;
            // trackers[i].class_type = 1;
        }
        // else if (box.size.x>10)
        // {
        //     info = "Ship";
        //     trackers[i].class_type = 4;
        // }
        else if (area >= 8)
        {
            info = "Truck:";
            // trackers[i].class_type = 2;
        }
        else if (area > 0.2 && area <= 1.5)
        {
            info = "Pedestrian:";
            // trackers[i].class_type = 3;
        }
        else if (box.size.x/box.size.y>5)
        {
            info = "wall:";
            // trackers[i].class_type = 3;
        }
        else
        {
            info = "Unkown:";
            // trackers[i].class_type = 4;
        }
        visualization_msgs::Marker marker2;
        // 写信息info
        boxer->draw_text(pos, info, box_count++, marker2);
        markers.markers.push_back(marker1);
        markers.markers.push_back(marker2);
    }
    // 发布trackers(调用tracker.cpp)
    // Tracker->getNewObjects(trackers);

    PolarGridBasor->clusterstoColor(clusters, color_cloud);
    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(color_cloud, output_points);
    output_points.header.frame_id = "frame_id_711";
    points_pub.publish(output_points);

    sensor_msgs::PointCloud2 output_ground_points;
    pcl::toROSMsg(*ground_cloud, output_ground_points);
    output_ground_points.header.frame_id = "frame_id_711";
    points_ground_pub.publish(output_ground_points);

    sensor_msgs::PointCloud2 tmp_msg;
    tmp_msg = *points_msg;
    tmp_msg.header.frame_id = "frame_id_711";
    raw_points_pub.publish(tmp_msg);
    // 发布boxer
    pub_marker.publish(markers);

    // std::cout << "the cost time of one frame is " << tt.toc() << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(clusters);
    std::vector<BoundingBoxCalculator::BoundingBox>().swap(boxes);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_localization");
    nh = new ros::NodeHandle;
    ros::NodeHandle pri_nh("~");
    float planefit_thre;
    pri_nh.getParam("planefit_thre", planefit_thre);
    float abs_ground_height;
    pri_nh.getParam("abs_ground_height", abs_ground_height);
    float ground_thre;
    pri_nh.getParam("ground_thre", ground_thre);

    std::string sub_points_topic;
    pri_nh.getParam("sub_points_topic", sub_points_topic);
    std::string pub_points_topic;
    pri_nh.getParam("pub_points_topic", pub_points_topic);
    std::string pub_ground_points_topic;
    pri_nh.getParam("pub_ground_points_topic", pub_ground_points_topic);
    std::string pub_raw_points_topic;
    pri_nh.getParam("pub_raw_points_topic", pub_raw_points_topic);
    std::string pub_array_topic;
    pri_nh.getParam("pub_array_topic", pub_array_topic);
    std::string sub_odo_topic;
    pri_nh.getParam("sub_odo_topic", sub_odo_topic);

    GroundPlanefitor = new GroundPlanefitBase(planefit_thre, abs_ground_height, ground_thre);
    PolarGridBasor = new PolarGridBase();
    // Tracker = new Track(*nh,sub_odo_topic);
    boxer = new BoundingBoxCalculator;
    boxer->cal_cosandsin();
    // std::cout<<"pub_array_topic:"<<pub_array_topic<<std::endl;
    points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_points_topic, 1, true);
    points_ground_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_ground_points_topic, 1, true);
    raw_points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_raw_points_topic, 1, true);
    pub_marker = nh->advertise<visualization_msgs::MarkerArray>(pub_array_topic, 1, true);

    // pub_path = nh->advertise<visualization_msgs::MarkerArray>("frame_id_711", 1, true);



    ros::Subscriber points_sub;
    points_sub = nh->subscribe(sub_points_topic, 10, Callback);
    // points_sub = nh->subscribe(sub_points_topic, 10, boost::bind(&Callback,_1,boxer));
    // 相同效果
    // points_sub = (*nh).subscribe(sub_points_topic, 10, &Callback);

    // ros::Rate loop_rate(10);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // };
    ros::spin();

    return 0;
}


/**
 *　　　　　　　 ┏┓　 ┏┓+ +
 *　　　　　　　┏┛┻━━━┛┻┓ + +
 *　　　　　　　┃　　　　　　┃ 　
 *　　　　　　　┃　　　━　　 ┃ ++ + + +
 *　　　　　　 ████━████  ┃+
 *　　　　　　　┃　　　　　　　┃ +
 *　　　　　　　┃　　　┻　　　┃
 *　　　　　　　┃　　　　　　┃ + +
 *　　　　　　　┗━┓　　　┏━┛
 *　　　　　　　　 ┃　　　┃　　　　　　　　　　　
 *　　　　　　　　 ┃　　　┃ + + + +
 *　　　　　　　　 ┃　　　┃　　　　Code is far away from bug with the animal protecting　　　　　　　
 *　　　　　　　　 ┃　　　┃ + 　　　　　　
 *　　　　　　　　 ┃　　　┃
 *　　　　　　　　 ┃　　　┃　　+　　　　　　　　　
 *　　　　　　　　 ┃　 　 ┗━━━┓ + +
 *　　　　　　　　 ┃ 　　　　   ┣┓
 *　　　　　　　　 ┃ 　　　　　 ┏┛
 *　　　　　　　　 ┗┓┓┏━┳┓┏┛ + + + +
 *　　　　　　　　  ┃┫┫ ┃┫┫
 *　　　　　　　　  ┗┻┛ ┗┻┛+ + + +
 */
