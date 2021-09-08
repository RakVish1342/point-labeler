/*
// PCL includes
#include "pcd_custom_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/io.hpp> // SHOULD NOT BE INCLUDED AS THE FIRST LINE!! Leads to compilation error: ‘int pcl::getFieldIndex(const pcl::PointCloud<PointT>&, const string&, std::vector<pcl::PCLPointField>&)’ should have been declared inside ‘pcl’.


int main()
{
    // std::string fff = "/media/rxth/DATA2/tmp/kitti_data1_single_marked_test_custom/velodyne_pcd/cloud0.pcd";
    // std::string fff = "/media/rxth/DATA2/tmp/kitti_data1_single_marked_test_custom/velodyne_pcd/cloud0_quadrants_full.pcd";
    std::string fff = "/media/rxth/DATA2/tmp/kitti_data1_single_marked_test_custom/velodyne_pcd/cloud0_quadrants_seg.pcd";
    using PT_XYZIR = velodyne_pointcloud::PointXYZIR;
    pcl::PointCloud<PT_XYZIR>::Ptr cloud;


    std::cout << "HELLO" << std::endl;
    pcl::io::loadPCDFile<PT_XYZIR> (fff, *cloud);
    std::cout << "HELLO" << std::endl;

}

*/

#include <iostream>

int main()
{

    std::cout << "aasd" << std::endl;
}