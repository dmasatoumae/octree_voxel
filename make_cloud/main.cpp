#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

auto make_cloud(int width,int height,int z)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = width;
    cloud->height = height;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i = 0;i < cloud->points.size();++i)
    {
        cloud->points[i].x = 1024 * rand()/(RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand()/(RAND_MAX + 1.0f);
        cloud->points[i].z = z;
    }
    return cloud;
}
int main(int arrgc ,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bottom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_bottom = make_cloud(5000,1,1);
    cloud_top = make_cloud(5000,1,2.5);
    *cloud_merged = *cloud_bottom+*cloud_top;
    pcl::io::savePLYFileASCII("./data/cloud_bottom.ply",*cloud_bottom);
    pcl::io::savePLYFileASCII("./data/cloud_top.ply",*cloud_top);
    pcl::io::savePLYFileASCII("./data/cloud_merged.ply",*cloud_merged);
    return 0;
}
