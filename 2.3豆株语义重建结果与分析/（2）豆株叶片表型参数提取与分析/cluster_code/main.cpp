//
// Created by xiao on 2022/9/8.
//
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>       // 根据索引提取点云
#include <pcl/filters/voxel_grid.h>            // 体素滤波
#include <pcl/kdtree/kdtree.h>                 // kd树

#include <pcl/visualization/pcl_visualizer.h>


using namespace std;

int
main(int argc, char** argv)
{
    //--------------------------读取桌面场景点云---------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("two.pcd", *cloud);
    cout << "读取点云: " << cloud->points.size() << " 个." << endl;

    //---------------------------体素滤波下采样----------------------------------
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    cout << "体素滤波后还有: " << cloud_filtered->points.size() << " 个." << endl;

    pcl::io::savePCDFileASCII("two_clustered.pcd", *cloud_filtered);

    return 0;
}
