#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <ros/ros.h>

#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelCloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dataCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dataCloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGBA>);

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addPointCloud(modelCloudDownsampled, "model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "model");
    viewer.addPointCloud(transformed, "transformed");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "transformed");
}


int main(int argv, char **args)
{
    // load files and convert to point type pcl::PointXYZRGBA
    pcl::PCLPointCloud2 cloud_blob;

    pcl::io::loadPCDFile("tablem.pcd", cloud_blob);
   
    pcl::fromPCLPointCloud2(cloud_blob, *modelCloud);

    pcl::io::loadPCDFile("tablelili.pcd", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *dataCloud);

    // make dense
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*modelCloud, *modelCloud, indices);
    pcl::removeNaNFromPointCloud(*dataCloud,  *dataCloud,  indices);

    // downsample clouds
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(modelCloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter(*modelCloudDownsampled);

    vg.setInputCloud(dataCloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter(*dataCloudDownsampled);


    // register clouds
    
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gicp;
    gicp.setInputSource(dataCloudDownsampled);
    gicp.setInputTarget(modelCloudDownsampled);

    gicp.align(*transformed);    

    std::cout << gicp.getFinalTransformation() << std::endl;
    
    // apply transform to data cloud (to fit model cloud)
    pcl::transformPointCloud(*dataCloudDownsampled, *transformed, gicp.getFinalTransformation());

    // show results
    pcl::visualization::CloudViewer viewer("Viewer for ICP");
    //viewer.showCloud(modelCloudDownsampled, "model");
    //viewer.showCloud(transformed, "transformed");

    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    while(!viewer.wasStopped())
    {
    }

    return 0;
}
