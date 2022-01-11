////测试用其他直方图是否可以替代自定义特征
////1.kdtree_flann能不能用在自定义类型。
////2.之前自定义的点类型（FPFH_RGB_ORI）是否有错误
//#include <pcl/point_types.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/features/normal_3d_omp.h>
//#include "my_point_types.h"
//#include <pcl/features/pfh.h>
//#include <pcl/features/pfhrgb.h>
//
////近邻搜索的中心点
//#define PN 100
//
//int 
//main() {
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
//
//	pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2009-10-27/model1.ply", *cloud);
//
//	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setNumberOfThreads(4);
//	ne.setInputCloud(cloud);
//	ne.setRadiusSearch(0.01);
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_ne(new pcl::search::KdTree<pcl::PointXYZRGB>());
//	ne.setSearchMethod(tree_ne);
//	ne.compute(*normals);
//
//	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
//	fpfh.setInputCloud(cloud);
//	fpfh.setInputNormals(normals);
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZRGB>);
//	fpfh.setSearchMethod(tree_fpfh);
//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());//1.原始特征结果(33)
//	fpfh.setRadiusSearch(0.5);
//	fpfh.compute(*fpfhs);
//	
//	//查看PFHRGB的直方图数值总量  结果为200+200=400，pfh为100，fpfh为300
//	//pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb_es;
//	//pfhrgb_es.setInputCloud(cloud);
//	//pfhrgb_es.setInputNormals(normals);
//	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_pfhrgb(new pcl::search::KdTree<pcl::PointXYZRGB>);
//	//pfhrgb_es.setSearchMethod(tree_pfhrgb);
//	//pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb(new pcl::PointCloud<pcl::PFHRGBSignature250>());//1.原始特征结果(33)
//	//pfhrgb_es.setRadiusSearch(0.3);
//	//pfhrgb_es.compute(*pfhrgb);
//	//for (size_t i = 0; i < 250; i++)
//	//{
//	//	std::cout << pfhrgb->at(PN).histogram[i] << ",";
//	//}
//	//std::cout << std::endl << std::endl;
//
//
//	//2.自定义特征结果(125)
//	pcl::PointCloud<pcl::PFHSignature125>::Ptr fpfhs_ex125(new pcl::PointCloud<pcl::PFHSignature125>());
//	fpfhs_ex125->resize(fpfhs->size());
//	for (size_t i = 0; i < fpfhs->size(); i++)
//	{
//		for (size_t j = 0; j < 33; j++)
//		{
//			fpfhs_ex125->at(i).histogram[j] = fpfhs->at(i).histogram[j];
//		}
//		for (size_t k = 33; k < 125; k++)
//		{
//			fpfhs_ex125->at(i).histogram[k] = 0;
//		}
//	}	
//	//3.自定义特征结果(250)
//	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr fpfhs_ex250(new pcl::PointCloud<pcl::PFHRGBSignature250>());
//	fpfhs_ex250->resize(fpfhs->size());
//	for (size_t i = 0; i < fpfhs->size(); i++)
//	{
//		for (size_t j = 0; j < 33; j++)
//		{
//			fpfhs_ex250->at(i).histogram[j] = fpfhs->at(i).histogram[j];
//		}
//		for (size_t k = 33; k < 250; k++)
//		{
//			fpfhs_ex250->at(i).histogram[k] = 0;
//		}
//	}
//	//4.自定义特征结果(FPFH_RGB_ORI-158)
//	pcl::PointCloud<FPFH_RGB_ORI>::Ptr fpfhs_ex158(new pcl::PointCloud<FPFH_RGB_ORI>());
//	fpfhs_ex158->resize(fpfhs->size());
//	for (size_t i = 0; i < fpfhs->size(); i++)
//	{
//		for (size_t j = 0; j < 33; j++)
//		{
//			fpfhs_ex158->at(i).histogram[j] = fpfhs->at(i).histogram[j];
//		}
//		for (size_t k = 33; k < 158; k++)
//		{
//			fpfhs_ex158->at(i).histogram[k] = 0;
//		}
//	}
//
//
//	////1.原FPFH特征近邻搜索
//	//pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree_1;
//	//kdtree_1.setInputCloud(fpfhs);
//	//std::vector<int> indices(10);
//	//std::vector<float> sqr_distances(10);
//	//int found_neighs = kdtree_1.nearestKSearch(fpfhs->at(PN), 10, indices, sqr_distances);
//	//for (size_t i = 0; i < 10; i++)
//	//{
//	//	std::cout << indices[i] << "     " << sqr_distances[i] << std::endl;
//	//}
//	//std::cout << std::endl << std::endl;
//
//	//2.自定义FPFH特征近邻搜索(125)
//	pcl::KdTreeFLANN<pcl::PFHSignature125> kdtree_2;
//	kdtree_2.setInputCloud(fpfhs_ex125);
//	std::vector<int> indices_2(10);
//	std::vector<float> sqr_distances_2(10);
//	int found_neighs_2 = kdtree_2.nearestKSearch(fpfhs_ex125->at(PN), 10, indices_2, sqr_distances_2);
//	for (size_t i = 0; i < 10; i++)
//	{
//		std::cout << indices_2[i] << "     " << sqr_distances_2[i] << std::endl;
//	}
//	std::cout << std::endl << std::endl;
//
//	//3.自定义FPFH特征近邻搜索(250)
//	pcl::KdTreeFLANN<pcl::PFHRGBSignature250> kdtree_3;
//	kdtree_3.setInputCloud(fpfhs_ex250);
//	std::vector<int> indices_3(10);
//	std::vector<float> sqr_distances_3(10);
//	int found_neighs_3 = kdtree_3.nearestKSearch(fpfhs_ex250->at(PN), 10, indices_3, sqr_distances_3);
//	for (size_t i = 0; i < 10; i++)
//	{
//		std::cout << indices_3[i] << "     " << sqr_distances_3[i] << std::endl;
//	}
//	std::cout << std::endl << std::endl;
//	
//	//4.自定义FPFH特征近邻搜索(158)
//	pcl::KdTreeFLANN<FPFH_RGB_ORI> kdtree_4;
//	kdtree_4.setInputCloud(fpfhs_ex158);
//	std::vector<int> indices_4(10);
//	std::vector<float> sqr_distances_4(10);
//	int found_neighs_4 = kdtree_4.nearestKSearch(fpfhs_ex158->at(PN), 10, indices_4, sqr_distances_4);
//	for (size_t i = 0; i < 10; i++)
//	{
//		std::cout << indices_4[i] << "     " << sqr_distances_4[i] << std::endl;
//	}
//	std::cout << std::endl << std::endl;
//
//
//	//对比直方图内容
//	for (size_t i = 0; i < 33; i++)
//	{
//		std::cout << fpfhs->at(PN).histogram[i] << ",";
//	}
//	std::cout << std::endl << std::endl;
//
//	for (size_t i = 0; i < 125; i++)
//	{
//		std::cout << fpfhs_ex125->at(PN).histogram[i] << ",";
//	}
//	std::cout << std::endl << std::endl;
//	
//	for (size_t i = 0; i < 250; i++)
//	{
//		std::cout << fpfhs_ex250->at(PN).histogram[i] << ",";
//	}
//	std::cout << std::endl << std::endl;
//	
//	for (size_t i = 0; i < 158; i++)
//	{
//		std::cout << fpfhs_ex158->at(PN).histogram[i] << ",";
//	}
//	std::cout << std::endl << std::endl;
//}