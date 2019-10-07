#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/auto_io.h>
#include <iostream>

class OctreeVoxel {
public:
    OctreeVoxel (std::string &filename,double resolution) :
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloudVoxel (new pcl::PointCloud<pcl::PointXYZ>()),
        octree(resolution)
    {
        if(!loadCloud(filename))
            return;
        displayedDepth = static_cast<int> (std::floor (octree.getTreeDepth() / 2.0));
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();
        voxel_filter(displayedDepth);
        saveCloud();

    }
private:
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1{new pcl::PointCloud<pcl::PointXYZ>};
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
    bool loadCloud(std::string &filename);
    void saveCloud();
    void voxel_filter(int depth);
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
    pcl::io::save("save.ply",*cloudVoxel);
    std::cout<<"save cloud"<<std::endl;
}
void OctreeVoxel::voxel_filter(int depth)
{
    cloudVoxel->points.clear();

    pcl::PointXYZ pt_voxel_center;
    pcl::PointXYZ pt_centroid;

    std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
    double start = pcl::getTime();

    for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::FixedDepthIterator tree_it = 
            octree.fixed_depth_begin(depth);
            tree_it != octree.fixed_depth_end();++tree_it)
    {

        Eigen::Vector3f voxel_min, voxel_max;
        octree.getVoxelBounds(tree_it,voxel_min,voxel_max);
        
        pt_voxel_center.x=(voxel_min.x()+voxel_max.x())/2.0f;
        pt_voxel_center.y=(voxel_min.y()+voxel_max.y())/2.0f;
        pt_voxel_center.z=(voxel_min.z()+voxel_max.z())/2.0f;

        cloudVoxel->points.push_back(pt_voxel_center);

        if(octree.getTreeDepth()==(unsigned int) depth)
        {
            pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode* container = 
                static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode*> (tree_it.getCurrentOctreeNode ());

            container->getContainer().getCentroid(pt_centroid);
        }
        else
        {
            pcl::octree::OctreeKey dummy_key;
            pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
            octree.getVoxelCentroidsRecursive (static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::BranchNode*> (*tree_it), dummy_key, voxelCentroids);
            
            pcl::CentroidPoint<pcl::PointXYZ> centroid;
            for (const auto &voxelCentroid : voxelCentroids)
            {
                centroid.add (voxelCentroid);
                                              
            }

            centroid.get(pt_centroid);
        }
        //displayCloud->points.push_back(pt_centroid);
    }
    double end = pcl::getTime();
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
