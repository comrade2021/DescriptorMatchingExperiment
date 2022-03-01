#include "feature_extractor.h"
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_tests.h>
#include "cfh_rgb_pfh_rate.h"
#include "cfh_rgb_pfh_dot.h"
#include "cfh_rgb_fpfh_dot.hpp"
#include "cfh_rgb_fpfh_rate.hpp"

#ifndef NumberOfThreads
#define NumberOfThreads 2
#endif // !NumberOfThreads
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingContextOpenGL2);

 
void FeatureExtractor::computePFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
{
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computePFHRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature)
{
	pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	es.setRadiusSearch(radius);
	es.setNumberOfThreads(NumberOfThreads);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_RGB_PFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	CFH_Estimation_RGB_PFH_RATE ce;
	ce.setInputCloud(cloud);
	ce.setInputKeypoints(keypoints);
	ce.setInputNormal(normal);
	ce.setSearchRadius(radius);
	ce.computeFeature(*feature);
}

void FeatureExtractor::computeCFH_RGB_PFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	CFH_Estimation_RGB_PFH_DOT ce;
	ce.setInputCloud(cloud);
	ce.setInputKeypoints(keypoints);
	ce.setInputNormal(normal);
	ce.setSearchRadius(radius);
	ce.computeFeature(*feature);
}

void FeatureExtractor::computeCFH_RGB_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	pcl::CFH_Estimation_RGB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_RGB_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	/*pcl::CFH_Estimation_RGB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_125(new pcl::PointCloud<pcl::PFHSignature125>);
	es.compute(*feature_125);

	for (size_t i = 0; i < feature_125->size(); i++)
	{
		for (size_t j = 0; j < 33; j++)
		{
			feature->at(i).histogram[j] = feature_125->at(i).histogram[j];
		}
	}*/

	//==================33============================
	pcl::CFH_Estimation_RGB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);

	//可视化
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud_model");
	viewer->addSphere((*cloud)[0], 1.7, 0.5, 0.5, 0.0, "sphere");
	//可视化特征直方图
	pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("Feature Histogram");
	std::cout <<"getFieldsList FPFHSignature33 :" << pcl::getFieldsList<pcl::FPFHSignature33>(*feature)<<std::endl;
	std::cout <<"getFieldsList PointXYZRGB :" << pcl::getFieldsList<pcl::PointXYZRGB>(*cloud)<< std::endl;
	plotter->addFeatureHistogram(*feature,"fpfh",30,"feature");
	plotter->plot();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	for (size_t j = 0; j < 33; j++)
	{
		std::cout << feature->at(30).histogram[j] << ",";
	}
	std::cout << std::endl << std::endl;
}

void FeatureExtractor::computeFPFH_CFH_RGB_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
{
	//计算FPFH
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_s;
	es_s.setInputCloud(keypoints);
	es_s.setSearchSurface(cloud);
	es_s.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_s.setSearchMethod(tree_1);
	es_s.setRadiusSearch(radius);
	es_s.setNumberOfThreads(NumberOfThreads);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	es_s.compute(*fpfh_33);

	//计算CFH_RGB_FPFH_RATE
	pcl::CFH_Estimation_RGB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	es_c.setInputCloud(keypoints);
	es_c.setSearchSurface(cloud);
	es_c.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_c.setSearchMethod(tree_2);
	es_c.setRadiusSearch(radius);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	es_c.compute(*cfh_33);

	//拼接
	feature->resize(keypoints->size());
	for (size_t i = 0; i < feature->size(); i++)
	{
		for (size_t j = 0; j < 33; j++)
		{
			feature->at(i).histogram[j] = fpfh_33->at(i).histogram[j];
		}
		for (size_t k = 0; k < 33; k++)
		{
			feature->at(i).histogram[33+k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m <125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeSHOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::SHOT352>::Ptr feature)
{
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	es.setNumberOfThreads(NumberOfThreads);
	es.setRadiusSearch(radius);
	/*note: 此处可以不使用es.setLRFRadius，因为当未设置LRF半径时，会自动将其设置为radiusSearch的搜索半径。
	pcl库中的源码：lrf_estimator->setRadiusSearch((lrf_radius_ > 0 ? lrf_radius_ : search_radius_));*/
	es.compute(*feature);
}

void FeatureExtractor::computeCSHOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::SHOT1344>::Ptr feature)
{
	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	es.setNumberOfThreads(NumberOfThreads);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}
