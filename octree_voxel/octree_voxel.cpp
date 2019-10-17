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
        resolution_copy  = resolution;
        std::cout<<resolution<<std::endl;
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();
        voxel_filter(displayedDepth);
        search();
        saveCloud();

    }
private:
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1{new pcl::PointCloud<pcl::PointXYZ>};
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
    double resolution;
    double resolution_copy;
    double voxel_size;
    double volume=0.0;
    int octree_leaf_count=0;
    bool loadCloud(std::string &filename);
    void saveCloud();
    void voxel_filter(int depth);
    void search();
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
    int dep=0;
    for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::FixedDepthIterator tree_it = 
            octree.fixed_depth_begin(depth);
            tree_it != octree.fixed_depth_end();++tree_it)
    {
        octree_leaf_count++;

        Eigen::Vector3f voxel_min, voxel_max;
        octree.getVoxelBounds(tree_it,voxel_min,voxel_max);
        
        pt_voxel_center.x=(voxel_min.x()+voxel_max.x())/2.0f;
        pt_voxel_center.y=(voxel_min.y()+voxel_max.y())/2.0f;
        pt_voxel_center.z=(voxel_min.z()+voxel_max.z())/2.0f;
        if(octree_leaf_count==1)
            voxel_size=voxel_max.x()-voxel_min.x();
        //std::cout<<pt_voxel_center.x<<","<<pt_voxel_center.y<<","<<pt_voxel_center.z<<","<<std::endl;


        cloudVoxel->points.push_back(pt_voxel_center);

        if(octree.getTreeDepth()==(unsigned int) depth)
        {
            pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode* container = 
                static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode*> (tree_it.getCurrentOctreeNode ());

            container->getContainer().getCentroid(pt_centroid);
            dep++;
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
    //std::cout<<octree_leaf_count<<std::endl;
    //std::cout<<dep<<std::endl;
    double end = pcl::getTime();
}
void OctreeVoxel::search()
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> search_octree ((float)resolution_copy);
    search_octree.setInputCloud(cloudVoxel);
    search_octree.addPointsFromInputCloud();
    //float radius = (float)resolution_copy;
    float radius = 0.00001;
    int test_jouge_count = 0;
    //search_octree.getVoxelBounds()
    std::cout<<"aaaa"<<voxel_size<<std::endl;

    for(int i = 0;i<octree_leaf_count;i++)
    {
        /*
        std::cout<<
        cloudVoxel->points[i].x<<","<<
        cloudVoxel->points[i].y<<","<<
        cloudVoxel->points[i].z<<std::endl;
        */
        pcl::PointXYZ searchPoint;
        searchPoint.x=cloudVoxel->points[i].x;
        searchPoint.y=cloudVoxel->points[i].y;
        for(double searchP_z = voxel_size;searchP_z<=30;searchP_z+=voxel_size)
        {
            //for(float resolution)
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            searchPoint.z = cloudVoxel->points[i].z+searchP_z;
            if (search_octree.radiusSearch(searchPoint,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) >0)
            {
                for(size_t size = 0; size < pointIdxRadiusSearch.size();++size)
                {
                    /*
                    std::cout<<" tokore "<<std::endl;
                    
                    std::cout<<
                    searchPoint.x<<","<<
                    searchPoint.y<<","<<
                    searchPoint.z<<std::endl;
                    */
                    double height = searchPoint.z - cloudVoxel->points[i].z;
                    volume+=voxel_size*voxel_size*height;

                    test_jouge_count++;
                    

                }
            }

        }
    }
    std::cout<<"count="<<test_jouge_count<<std::endl;

    std::cout<<"volume="<<volume<<std::endl;
    /*
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator itit (&search_octree);
    int coucou = 0;
    while(*++itit)
    {
        coucou++;
        //std::cout << itit.getCurrentOctreeDepth() << ",";
    }
    std::cout<<coucou<<std::endl;

    
    for(int i=0;i<30;i++){
        searchPoint.x=
        searchPoint.y=
        searchPoint.z=
    }
    */

    
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
