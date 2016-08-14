#include "PCL.h"

using namespace std;


pcl::PolygonMesh mesh;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

//キーポイント
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PolygonMesh triangles;


PCL::PCL(){
	read("181.ply", cloud);
	filtering(cloud,cloud_filtered);
	normal_calculation(cloud, normals);
	//concatenate_field();
	//create_mesh();
	keypoints_calculation_iss(cloud,cloud_keypoints);
	feature_calculation_spinimage();
	visualize();
}

PCL::~PCL(){

}


void PCL::read(char *name, pcl::PointCloud<pcl::PointXYZ>::Ptr data){
	pcl::io::loadPLYFile(name, *cloud);
	cout << "read : " << name << endl;
}

double PCL::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &data)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void PCL::filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output){
	//フィルタリング
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*output);
	cout << "filtering_do"<< endl;
}

void PCL::normal_calculation(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::Normal>::Ptr &normal){
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_normal->setInputCloud(data);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(data);
	n.setSearchMethod(tree_normal);
	n.setKSearch(20);
	n.compute(*normal);
	cout << "normal_calculation" << endl;
}

void PCL::keypoints_calculation_iss(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoint){
	float model_resolution = static_cast<float> (computeCloudResolution(cloud));
	double iss_salient_radius_ = 2 * model_resolution;
	double iss_non_max_radius_ = 4 * model_resolution;
	double iss_gamma_21_(0.975);
	double iss_gamma_32_(0.975);
	double iss_min_neighbors_(5);
	int iss_threads_(4);
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_keypoint(new pcl::search::KdTree<pcl::PointXYZ>);
	iss_detector.setSearchMethod(tree_keypoint);
	iss_detector.setSalientRadius(iss_salient_radius_);
	iss_detector.setNonMaxRadius(iss_non_max_radius_);
	iss_detector.setThreshold21(iss_gamma_21_);
	iss_detector.setThreshold32(iss_gamma_32_);
	iss_detector.setMinNeighbors(iss_min_neighbors_);
	iss_detector.setNumberOfThreads(iss_threads_);
	iss_detector.setInputCloud(data);
	iss_detector.compute(*keypoint);

	cout << "keypoints_calculation_iss" << endl;

}

void PCL::feature_calculation_spinimage(){
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
	spin_image_descriptor.setInputCloud(cloud);
	spin_image_descriptor.setInputNormals(normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	spin_image_descriptor.setSearchMethod(kdtree);
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153> >);
	spin_image_descriptor.setRadiusSearch(0.2);
	spin_image_descriptor.compute(*spin_images);	
}



void PCL::concatenate_field(){
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	cout << "concatenate_field" << endl;
}

void PCL::create_mesh(){
	//k分木作成
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_mesh(new pcl::search::KdTree<pcl::PointNormal>);
	tree_mesh->setInputCloud(cloud_with_normals);

	//物体の初期処理
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	gp3.setSearchRadius(0.025);//サーチの距離設定

	//パラメータの設定
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	//メッシュ作成
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree_mesh);
	gp3.reconstruct(triangles);
	cout << "create_mesh" << endl;
}

void PCL::visualize(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	//色の設定
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 100, 100, 100);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_keypoints, 0, 255, 0);

	//表示するものを選択
	//viewer->addPolygonMesh(triangles, "meshes", 0);
	viewer->addPointCloud(cloud, cloud_color_handler, "cloud");
	viewer->addPointCloud(cloud_keypoints, keypoints_color_handler, "keypoints");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
