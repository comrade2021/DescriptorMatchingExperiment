////----------------------------测试FPFH的多线程加速-----------------------------//
////过程：先计算room_scan1.pcd点云的法线，再计算所有点的FPFH特征，比较使用多线程加速前后的运行时间。
////结果：
////      现在可以使用FPFH的OMP加速了；
////      release运行模式下，OMP加速前后时间分别为 3.197s 和 0.957s ,加速前后差3.3倍（四核cpu）；
////      debug和release运行方式（未加速）情况下，时间分别为 128s 和 3s 。
//
//#include <pcl/point_types.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
//
//int main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
//
//	//read pcd file
//	pcl::io::loadPCDFile("./pcd/room_scan1.pcd", *cloud);
//	std::cout << "load pcd , size:" << cloud->size()<<std::endl;
//
//	//normal estimation
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setInputCloud(cloud);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	ne.setSearchMethod(ne_tree);
//	ne.setRadiusSearch(0.005);
//	ne.compute(*normals);
//	std::cout << "computed normal size:" << normals->size() << std::endl;
//
//
//	// Create the FPFH estimation class, and pass the input dataset+normals to it
//	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//	//pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//	//fpfh.setNumberOfThreads(4);
//	fpfh.setInputCloud(cloud);
//	fpfh.setInputNormals(normals);
//	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
//
//	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
//	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//
//	fpfh.setSearchMethod(tree);
//
//	// Output datasets
//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
//
//	// Use all neighbors in a sphere of radius 5cm
//	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//	fpfh.setRadiusSearch(0.01);
//
//	// Compute the features
//	clock_t start, end;
//	start = clock();
//	fpfh.compute(*fpfhs);
//	end = clock();
//	double cost_time = (double)(end - start) / CLOCKS_PER_SEC;
//	std::cout << "computed fpfh size:" << fpfhs->size() << std::endl;
//	std::cout << "fpfh cost time:" << cost_time<<"s" << std::endl;
//
//	// fpfhs->size () should have the same size as the input cloud->size ()*
//
//
//	return 0;
//}