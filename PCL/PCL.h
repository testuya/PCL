#ifndef PCL_H_
#define PCL_H_
#pragma warning(disable:4996)


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/file_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/correspondence.h>
#include "PCLAdapter.h"
using namespace std;
 

class PCL{
private:
	void read(char *name,pcl::PointCloud<pcl::PointXYZ>::Ptr data);
	void filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
	double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &data);
	void normal_calculation(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::Normal>::Ptr &normal);
	void keypoints_calculation_iss(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoint);
	void feature_calculation_spinimage();
	void concatenate_field();
	void create_mesh();
	void visualize();
public:
	PCL();
	~PCL();
};


#endif