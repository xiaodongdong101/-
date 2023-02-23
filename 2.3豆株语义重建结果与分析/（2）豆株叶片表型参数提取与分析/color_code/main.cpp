//
// Created by xiao on 2022/9/8.
//

#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <boost/thread/thread.hpp>

int
main(int argc, char** argv)
{

    time_t start, end, diff[5];
    start = time(0);
    // -----------------------------------------读取点云数据--------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("/home/xiao/coco1004/result1/ply-pcd/fused.pcd", *cloud) == -1)
    {
        PCL_ERROR("点云读取失败！！！");
        return (-1);
    }
    end = time(0);
    diff[0] = difftime(end, start);
    PCL_INFO("读取点云花费: %d秒。\n", diff[0]);
    // ------------------------------------法向量和表面曲率估计-----------------------------------
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);

    end = time(0);
    diff[1] = difftime(end, start) - diff[0];
    PCL_INFO("计算法向量花费: %d 秒。\n", diff[1]);
    // ------------------------------------基于颜色的区域生长-------------------------------------
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setMinClusterSize(2000);                        // 一个聚类需要的最小点数
    reg.setMaxClusterSize(11000);                    // 一个聚类需要的最大点数
    reg.setSearchMethod(tree);                         // 搜索方法
    reg.setDistanceThreshold(8);                      // 设置距离阈值，用于聚类相邻点搜索
    reg.setPointColorThreshold(4);                     // 设置两点颜色阈值
    reg.setRegionColorThreshold(40);                   // 设置两类区域颜色阈值
    /*可选参数*/
    reg.setSmoothModeFlag(true);                       // 设置是否使用平滑阈值，设置为true则需提供法线及法线夹角阈值
    reg.setInputNormals(normals);                      // 法向量和表面曲率
    reg.setSmoothnessThreshold(30 / 180.0 * M_PI);     // 设置平滑阈值，即法向量夹角的阈值
    reg.setCurvatureTestFlag(true);                    // 设置是否使用曲率的阈值，设置为true则需提供曲率及曲率阈值
    reg.setCurvatureThreshold(0.05);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);                             // 获取分割聚类的结果，分割结果保存在点云索引的向量中。
    end = time(0);
    diff[2] = difftime(end, start) - diff[0] - diff[1];
    PCL_INFO("基于颜色的区域生长算法运行时间为: %d秒\n", diff[2]);

    //-------------------------------------保存分割后的点云----------------------------------------
    int begin = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        printf("第%d块点云的点数为:%lld \n", begin + 1, cloud_cluster->points.size());

        std::stringstream ss;
        ss << "RGB_" << begin + 1 << ".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);

        begin++;
    }


    return (0);
}