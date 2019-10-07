#include <iostream>
#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc,char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    
    pcl::io::load("cloud_merged.ply",*cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f,0.1f,0.1f);
    sor.filter(*cloud_filtered);

    pcl::io::save("save.ply",*cloud_filtered);

    return(0);
}
