//
// Created by xiao on 2022/9/8.
//

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    pcl::PCLPointCloud2 cloud;
    //加载ply文件
    pcl::PLYReader reader;
    reader.read("/home/xiao/coco1004/result1/ply-pcd/fused.ply", cloud);
    //将ply文件保存为pcd文件
    pcl::PCDWriter writer;
    writer.write("/home/xiao/coco1004/result1/ply-pcd/fused.pcd", cloud);

    return 0;
}