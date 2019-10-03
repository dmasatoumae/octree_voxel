#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/auto_io.h>
#include <iostream>

class OctreeVoxel {
public:
    OctreeVoxel (std::string &filename,double resolution) :
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        octree(resolution)
    {
        if(!loadCloud(filename))
            return;
        displayedDepth = static_cast<int> (std::floor (octree.getTreeDepth() / 2.0));
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

    }
private:
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1{new pcl::PointCloud<pcl::PointXYZ>};
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
    bool loadCloud(std::string &filename);
    void saveCloud();
    int displayedDepth;

};

bool OctreeVoxel::loadCloud(std::string &filename){
    std::cout<<"Loading file"<<filename.c_str()<<std::endl;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::load(filename,*cloud))
    {
        return false;
    }
    std::cout<<"loadload";
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(*cloud,*cloud,nanIndexes);
    octree.setInputCloud(cloud);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    
    return true;
}

void OctreeVoxel::saveCloud()
{
    std::cout<<"save cloud"<<std::endl;
    pcl::io::save("save.ply",*cloud);
}
void OctreeVoxel::Voxel_filter(int depth)
{
    cloudVoxel->points.clear();

    pcl::PointXYZ pt_voxel_center;
    pcl::PointXYZ pt_centroid;

    std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
    double start = pcl::getTime();
    for (pcl::octree::OctrePointVoxelCentroid<pcl::PointXYZ>::FixedDepthIterater tree_it = 
            octree.fixed_depth_begin(depth);
            tree_it != octree.fixed_depth_end();++tree_it)
    {
        Eigen::Vector3f voxel_min, voxel_max;
        octree.getVoxelBounds(tree_it,voxel_min,voxel_max);
        
        pt_voxel_center.x=(voxel_min.x()+voxel_max.x())/2.0f;
        pt_voxel_center.y=(voxel_min.y()+voxel_max.y())/2.0f;
        pt_voxel_center.z=(voxel_min.z()+voxel_max.z())/2.0f;

        cloudVoxel->points.push_back(pt_voxel_center);
        

    


}
int main (int argc, char** argv)
{
    if (argc != 3 )
    {
        std::cerr << "ERROR: Syntax is octreevoxel <ply file> <resolution>"<<std::endl;
        std::cerr << "EXAMPLE: .octree_voxel bun0.ply 0.01" <<std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    OctreeVoxel octreeVo(cloud_path,atof(argv[2]));

    //octree.loadCloud(cloud_path);
    //std::cout<<"load_comp\n";
    //octree.saveCloud();
    std::cout<<"save_comp\n";

    return 0;
}
