#include "feature_extractor.h"
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_tests.h>
#include "cfh_rgb_pfh_rate.h"
#include "cfh_rgb_pfh_dot.h"
//此处需要引用hpp而非h，否则出现无法解析的错误
#include "cfh_rgb_fpfh_dot.hpp"
#include "cfh_rgb_fpfh_rate.hpp"
#include "cfh_rgb_fpfh_new.hpp"
#include "cfh_rgb_fpfh_L1.hpp"
#include "cfh_rgb_fpfh_L2.hpp"

#include "cfh_hsv_fpfh_dot.hpp"
#include "cfh_hsv_fpfh_rate.hpp"
#include "cfh_hsv_fpfh_L1.hpp"
#include "cfh_hsv_fpfh_L2.hpp"

#include "cfh_lab_fpfh_dot.hpp"
#include "cfh_lab_fpfh_rate.hpp"
#include "cfh_lab_fpfh_L1.hpp"
#include "cfh_lab_fpfh_L2.hpp"

#include "fpfh_sc_time.hpp"



#ifndef NumberOfThreads
#define NumberOfThreads 2
#endif // !NumberOfThreads
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <vtkAutoInit.h>

 
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
	//pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
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

void FeatureExtractor::computeCFH_RGB_FPFH_NEW(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
{
	// 44
	pcl::CFH_Estimation_RGB_FPFH_NEW<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> es;
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
	//==================33============================
	pcl::CFH_Estimation_RGB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_RGB_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_RGB_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_RGB_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_RGB_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_HSV_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	pcl::CFH_Estimation_HSV_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	//pcl::FPFH_SC_TIME_Estimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_HSV_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_HSV_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_HSV_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_HSV_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_HSV_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_HSV_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_LAB_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	pcl::CFH_Estimation_LAB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_LAB_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_LAB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_LAB_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_LAB_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}

void FeatureExtractor::computeCFH_LAB_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//==================33============================
	pcl::CFH_Estimation_LAB_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree_f);
	es.setRadiusSearch(radius);
	es.compute(*feature);
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

void FeatureExtractor::computeFPFH_CFH_RGB_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
{
	//计算FPFH
	//pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_s;
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
	pcl::CFH_Estimation_RGB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_RGB_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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
	pcl::CFH_Estimation_RGB_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_RGB_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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
	pcl::CFH_Estimation_RGB_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_HSV_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
{
	//计算FPFH
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_s;
	//pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_s;
	es_s.setInputCloud(keypoints);
	es_s.setSearchSurface(cloud);
	es_s.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_s.setSearchMethod(tree_1);
	es_s.setRadiusSearch(radius);
	es_s.setNumberOfThreads(NumberOfThreads);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	es_s.compute(*fpfh_33);

	//!!!!!相当于执行两遍FPFH，太慢，以后尝试融合到一起
	//计算CFH_HSV_FPFH_RATE
	pcl::CFH_Estimation_HSV_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_HSV_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_HSV_FPFH_RATE
	pcl::CFH_Estimation_HSV_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_HSV_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_HSV_FPFH_RATE
	pcl::CFH_Estimation_HSV_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_HSV_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_HSV_FPFH_RATE
	pcl::CFH_Estimation_HSV_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_LAB_FPFH_RATE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_LAB_FPFH_RATE
	pcl::CFH_Estimation_LAB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	es_c.setInputCloud(keypoints);
	es_c.setSearchSurface(cloud);
	es_c.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_c.setSearchMethod(tree_2);
	es_c.setRadiusSearch(radius);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	es_c.compute(*cfh_33);
	
	
	////TEST------------计算CFH_LAB_FPFH_RATE,提前转换色彩空间
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	//keypoints_c->resize(keypoints->size());
	//for (size_t i = 0; i < keypoints_c->size(); i++)
	//{
	//	int red_1 = keypoints->at(i).r;
	//	int green_1 = keypoints->at(i).g;
	//	int blue_1 = keypoints->at(i).b;

	//	colorutil::RGB rgb_color_1(red_1 / 255.0, green_1 / 255.0, blue_1 / 255.0);

	//	colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
	//	colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);

	//	keypoints_c->at(i).x = keypoints->at(i).x;
	//	keypoints_c->at(i).y = keypoints->at(i).y;
	//	keypoints_c->at(i).z = keypoints->at(i).z;
	//	keypoints_c->at(i).r = keypoints->at(i).r;
	//	keypoints_c->at(i).g = keypoints->at(i).g;
	//	keypoints_c->at(i).b = keypoints->at(i).b;
	//}
	//
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_c->resize(cloud->size());
	//for (size_t i = 0; i < cloud_c->size(); i++)
	//{
	//	int red_1 = cloud->at(i).r;
	//	int green_1 = cloud->at(i).g;
	//	int blue_1 = cloud->at(i).b;

	//	colorutil::RGB rgb_color_1(red_1 / 255.0, green_1 / 255.0, blue_1 / 255.0);

	//	colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
	//	colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);

	//	cloud_c->at(i).x = cloud->at(i).x;
	//	cloud_c->at(i).y = cloud->at(i).y;
	//	cloud_c->at(i).z = cloud->at(i).z;
	//	cloud_c->at(i).r = cloud->at(i).r;
	//	cloud_c->at(i).g = cloud->at(i).g;
	//	cloud_c->at(i).b = cloud->at(i).b;
	//}
	//pcl::CFH_Estimation_LAB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	//es_c.setInputCloud(keypoints_c);
	//es_c.setSearchSurface(cloud_c);
	//es_c.setInputNormals(normal);
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//es_c.setSearchMethod(tree_2);
	//es_c.setRadiusSearch(radius);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	//es_c.compute(*cfh_33);

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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_LAB_FPFH_DOT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_LAB_FPFH_DOT
	pcl::CFH_Estimation_LAB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	es_c.setInputCloud(keypoints);
	es_c.setSearchSurface(cloud);
	es_c.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_c.setSearchMethod(tree_2);
	es_c.setRadiusSearch(radius);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	es_c.compute(*cfh_33);


	////TEST------------计算CFH_LAB_FPFH_DOT,提前转换色彩空间
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	//keypoints_c->resize(keypoints->size());
	//for (size_t i = 0; i < keypoints_c->size(); i++)
	//{
	//	int red_1 = keypoints->at(i).r;
	//	int green_1 = keypoints->at(i).g;
	//	int blue_1 = keypoints->at(i).b;

	//	colorutil::RGB rgb_color_1(red_1 / 255.0, green_1 / 255.0, blue_1 / 255.0);

	//	colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
	//	colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);

	//	keypoints_c->at(i).x = keypoints->at(i).x;
	//	keypoints_c->at(i).y = keypoints->at(i).y;
	//	keypoints_c->at(i).z = keypoints->at(i).z;
	//	keypoints_c->at(i).r = keypoints->at(i).r;
	//	keypoints_c->at(i).g = keypoints->at(i).g;
	//	keypoints_c->at(i).b = keypoints->at(i).b;
	//}
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_c->resize(cloud->size());
	//for (size_t i = 0; i < cloud_c->size(); i++)
	//{
	//	int red_1 = cloud->at(i).r;
	//	int green_1 = cloud->at(i).g;
	//	int blue_1 = cloud->at(i).b;

	//	colorutil::RGB rgb_color_1(red_1 / 255.0, green_1 / 255.0, blue_1 / 255.0);

	//	colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
	//	colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);

	//	cloud_c->at(i).x = cloud->at(i).x;
	//	cloud_c->at(i).y = cloud->at(i).y;
	//	cloud_c->at(i).z = cloud->at(i).z;
	//	cloud_c->at(i).r = cloud->at(i).r;
	//	cloud_c->at(i).g = cloud->at(i).g;
	//	cloud_c->at(i).b = cloud->at(i).b;
	//}
	//pcl::CFH_Estimation_LAB_FPFH_DOT<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	//es_c.setInputCloud(keypoints_c);
	//es_c.setSearchSurface(cloud_c);
	//es_c.setInputNormals(normal);
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//es_c.setSearchMethod(tree_2);
	//es_c.setRadiusSearch(radius);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	//es_c.compute(*cfh_33);

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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_LAB_FPFH_L1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_LAB_FPFH_RATE
	pcl::CFH_Estimation_LAB_FPFH_L1<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_LAB_FPFH_L2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	//计算CFH_LAB_FPFH_RATE
	pcl::CFH_Estimation_LAB_FPFH_L2<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
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
			feature->at(i).histogram[33 + k] = cfh_33->at(i).histogram[k];
		}
		for (size_t m = 66; m < 125; m++)
		{
			feature->at(i).histogram[m] = 0;
		}
	}
}

void FeatureExtractor::computeFPFH_CFH_RGB_FPFH_NEW(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature)
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

	////计算CFH_RGB_FPFH_RATE
	//pcl::CFH_Estimation_RGB_FPFH_NEW<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es_c;
	//es_c.setInputCloud(keypoints);
	//es_c.setSearchSurface(cloud);
	//es_c.setInputNormals(normal);
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//es_c.setSearchMethod(tree_2);
	//es_c.setRadiusSearch(radius);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_33(new pcl::PointCloud<pcl::FPFHSignature33>);
	//es_c.compute(*cfh_33);
	
	//计算CFH_RGB_FPFH_RATE
	pcl::CFH_Estimation_RGB_FPFH_NEW<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> es_c;
	es_c.setInputCloud(keypoints);
	es_c.setSearchSurface(cloud);
	es_c.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es_c.setSearchMethod(tree_2);
	es_c.setRadiusSearch(radius);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cfh_44(new pcl::PointCloud<pcl::PFHSignature125>);
	es_c.compute(*cfh_44);
	
	//拼接
	feature->resize(keypoints->size());
	for (size_t i = 0; i < feature->size(); i++)
	{
		for (size_t j = 0; j < 33; j++)
		{
			feature->at(i).histogram[j] = fpfh_33->at(i).histogram[j];
		}
		for (size_t k = 0; k < 44; k++)
		{
			feature->at(i).histogram[33 + k] = cfh_44->at(i).histogram[k];
		}
		for (size_t m = 77; m < 125; m++)
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
	//pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> es;
	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> es;
	es.setInputCloud(keypoints);
	es.setSearchSurface(cloud);
	es.setInputNormals(normal);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	es.setSearchMethod(tree);
	//es.setNumberOfThreads(NumberOfThreads);
	es.setRadiusSearch(radius);
	es.compute(*feature);
}
