#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkAutoInit.h>
#include <string>
#include <iostream>
#include "keypoints_detector.h"
#include "my_normal_estimation.h"
#include "toolbox.h"
#include "validator.h"
#include <pcl/common/transforms.h>
#include <pcl/features/impl/boundary.hpp>
#include "file_reader.h"
#include "experiment.h"
#include <pcl/features/fpfh_omp.h>
#include "cfh_rgb_rate_33.h"

#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include "keypoints_detector.h"
#include "cfh_rgb_dot.h"
#include "cfh_rgb_fpfh_rate.hpp"

//int main() {
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//    pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2010-03-03/Scena1/scene1.ply", *cloud);
//
//    //keypoints
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//    KeypointsDetector kd;
//    kd.computeRandomSampleKeypoints(cloud, keypoints, 20);
//
//    //3.提取关键点（体素下采样）
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_1(new pcl::PointCloud<pcl::PointXYZRGB>);
//    KeypointsDetector key_detector;
//    key_detector.computeVoxelGridKeypoints(cloud, keypoints_m_1,0.7);
//    std::cout << "keypoints_m_1->size():   " << keypoints_m_1->size() << std::endl;
//
//    //normal
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
//    pcl::NormalEstimationOMP < pcl::PointXYZRGB, pcl::Normal> ne;
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZRGB>());
//    ne.setSearchMethod(tree_1);
//    ne.setInputCloud(cloud);
//    ne.setRadiusSearch(0.4);
//    ne.setNumberOfThreads(4);
//    ne.compute(*normals);
//
//    ////feature
//    //CFH_Estimation_RGB_RATE_33 ce;
//    //ce.setInputCloud(cloud);
//    //ce.setInputKeypoints(keypoints);
//    //ce.setInputNormal(normals);
//    //ce.setSearchRadius(1.0);
//    //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
//    //ce.computeFeature(*fpfh);
//    
//    //feature
//    pcl::CFH_Estimation_RGB_FPFH_RATE<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> es;
//    
//    es.setInputCloud(keypoints);
//    es.setSearchSurface(cloud);
//    es.setInputNormals(normals);
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGB>());
//    es.setSearchMethod(tree_f);
//    es.setRadiusSearch(1.7);
//    //es.setNumberOfThreads(4);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
//    es.compute(*fpfh);
//
//    //求直方图和
//    for (size_t i = 0; i < fpfh->size(); i++)
//    {
//        double sum=0;
//        for (size_t j = 0; j < 33; j++)
//        {
//            sum += fpfh->at(i).histogram[j];
//        }
//        std::cout << sum << std::endl;
//    }
//    //直方图值
//    for (size_t i = 0; i < fpfh->size(); i++)
//    {
//        for (size_t j = 0; j < 33; j++)
//        {
//            std::cout << fpfh->at(i).histogram[j]<<",";
//        }
//        std::cout << std::endl<<std::endl;
//    }
//}



 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingContextOpenGL2);

//正在测试的描述符数量
#define NUM_OF_DESCRIPTOR_TESTED 1
//正在测试的场景数量
#define NUM_OF_SCNEN 12
//匹配阈值分组数量
#define NUM_ALPHA 20

int
main() {
	//计时器
	clock_t startTime, endTime;
	startTime = clock();
	std::vector<int> time_vec;

	//dataset3(group:1,2,3,4)
	Experiment exp;
	ToolBox tb;
	std::vector<double> sum_precision(NUM_ALPHA * NUM_OF_DESCRIPTOR_TESTED);
	std::vector<double> sum_recall(NUM_ALPHA * NUM_OF_DESCRIPTOR_TESTED);

	//std::vector<std::string> features_name = { "CFH_RGB_PFH_RATE","CFH_RGB_PFH_DOT"};
	std::vector<std::string> features_name = {"CFH_RGB_FPFH_RATE"};
	/*
	测试新的特征时，注意：

	1. main的fatures_name对应于experiment::perform的features_name，须手动设置为与experiment中当前实验安排一致；
	
	2. 手动设置 NUM_OF_DESCRIPTOR_TESTED

	3. 手动设置NUM_ALPHA，须与Experiment::computeALLPR_2::alpha_devide一致

	4. experiment.h、main()、experiment.cpp 的 matches_sum_cfh_rgb_pfh_rate_等内容，须一致,顺序必须保持一致。

	5. 由于feature_extractor中的match等方法参数已有重载，因此对于新特征，只需要写一个计算方法就行。
	*/

	std::vector<double> precision;
	std::vector<double> recall;
	exp.setDatasetName("dataset3");
	exp.setModelNumber(2);
	exp.setAlphaUpperLimit(1.0);
	////group-1-scene1
	//exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene1.ini");
	//exp.setOutputTXTPath("./dataset3/dataset3-group1-scene1-out.txt");
	//exp.perform(precision, recall);
	//tb.accumulateVector(sum_precision, precision);
	//tb.accumulateVector(sum_recall, recall);
	////group-1-scene2
	//exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene2.ini");
	//exp.setOutputTXTPath("./dataset3/dataset3-group1-scene2-out.txt");
	//exp.perform(precision, recall);
	//tb.accumulateVector(sum_precision, precision);
	//tb.accumulateVector(sum_recall, recall);
	////group-1-scene3
	//exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene3.ini");
	//exp.setOutputTXTPath("./dataset3/dataset3-group1-scene3-out.txt");
	//exp.perform(precision, recall);
	//tb.accumulateVector(sum_precision, precision);
	//tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-2-scene1
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena1/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group2-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene2
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena1/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group2-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene3
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena1/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group2-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene4
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena1/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group2-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-3-scene1
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena2/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group3-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene2
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena2/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group3-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene3
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena2/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group3-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene4
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena2/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group3-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-4-scene1
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena3/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group4-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene2
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena3/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group4-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene3
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena3/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group4-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene4
	exp.setIniPath("./dataset3/3D models/CVLab/2010-03-03/Scena3/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group4-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);


	//计算多次场景实验结果平均值
	for (size_t i = 0; i < sum_precision.size(); i++)
	{
		sum_precision[i] /= NUM_OF_SCNEN;
	}
	for (size_t i = 0; i < sum_recall.size(); i++)
	{
		sum_recall[i] /= NUM_OF_SCNEN;
	}
	exp.writePRToTXT("./dataset3/result.txt", sum_precision, sum_recall, features_name);

	//计算多次场景实验结果平均值
	//注意！此处顺序必须与features_name一致
	std::vector<double> vector_p;
	std::vector<double> vector_r;
	double p = 0;
	double r = 0;
	////pfh-pr   ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_pfh_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_pfh_[i]) / exp.matches_sum_pfh_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_pfh_[i]) / exp.corresponding_regions_sum_pfh_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	////fpfh-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_[i]) / exp.matches_sum_fpfh_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_[i]) / exp.corresponding_regions_sum_fpfh_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	////CFH_RGB_PFH_RATE  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_pfh_rate_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_pfh_rate_[i]) / exp.matches_sum_cfh_rgb_pfh_rate_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_pfh_rate_[i]) / exp.corresponding_regions_sum_cfh_rgb_pfh_rate_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	
	////CFH_RGB_PFH_DOT  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_pfh_dot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_pfh_dot_[i]) / exp.matches_sum_cfh_rgb_pfh_dot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_pfh_dot_[i]) / exp.corresponding_regions_sum_cfh_rgb_pfh_dot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	//CFH_RGB_FPFH_RATE  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_cfh_rgb_fpfh_rate_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_rate_[i]) / exp.matches_sum_cfh_rgb_fpfh_rate_[i];
			r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_rate_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_rate_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	////fpfh-rgb-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_rgb_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_rgb_[i]) / exp.matches_sum_fpfh_rgb_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_rgb_[i]) / exp.corresponding_regions_sum_fpfh_rgb_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	////pfhrgb-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_pfhrgb_[i]==0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_pfhrgb_[i]) / exp.matches_sum_pfhrgb_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_pfhrgb_[i]) / exp.corresponding_regions_sum_pfhrgb_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	////shot-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_shot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_shot_[i]) / exp.matches_sum_shot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_shot_[i]) / exp.corresponding_regions_sum_shot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	////cshot-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cshot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cshot_[i]) / exp.matches_sum_cshot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cshot_[i]) / exp.corresponding_regions_sum_cshot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	exp.writePRToTXT("./dataset3/result_2.txt", vector_p, vector_r, features_name);

	std::ofstream outfile;   //输出流
	outfile.open("./dataset3/time.txt", ios::app);
	if (!outfile.is_open())
		std::cout << "Open file failure" << std::endl;
	for (size_t i = 0; i < time_vec.size(); i++)
	{
		outfile << time_vec[i];
		outfile << "\n";
	}
	outfile.close();
}












//	////计时器
//	//clock_t startTime, endTime;
//	//startTime = clock();
//
//	//int num_corr_key = 2*100;//对应关键点的数量,n∗每个场景1000个特征（n是场景中存在的模型数量）
//	//int num_s_noise = 100;//场景关键点中的干扰点数量
//	//double resolution_model, resolution_scene;//网格分辨率
//	//double radius_normal_m = 0.0;//法线的计算半径
//	//double radius_normal_s = 0.0;
//	//double radius_feature_m = 0.0;//特征的计算半径
//	//double radius_feature_s = 0.0;
//	//double alpha = 0.8;//特征匹配阈值
//	//double beta = 10;//距离误差阈值
//
//	//Eigen::Matrix4f ground_truth = Eigen::Matrix4f::Identity();
//	//ground_truth << 0.33333331, 0.91068363, -0.24401692, -1.100000,
//	//	-0.24401692, 0.33333331, 0.91068363, -0.500000,
//	//	0.91068363, -0.24401692, 0.33333331, -3.000000,
//	//	0, 0, 0, 1;
//	//std::cout << std::fixed << std::setprecision(15) << ground_truth << std::endl;
//
//	////Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//	////// Define a rotation matrix
//	////float theta = M_PI / 4; // The angle of rotation in radians
//	////transform_1(0, 0) = std::cos(theta);
//	////transform_1(0, 1) = -sin(theta);
//	////transform_1(1, 0) = sin(theta);
//	////transform_1(1, 1) = std::cos(theta);
//	////// Define a translation of 2.5 meters on the x axis.
//	////transform_1(0, 3) = 0.5;
//
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZRGB>);//点云
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model1(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model2(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
//	////pcl::IndicesPtr indicesptr_m(new std::vector<int>());//关键点索引
//	////pcl::IndicesPtr indicesptr_s(new std::vector<int>());
//	//pcl::PointCloud<pcl::Normal>::Ptr normal_m(new pcl::PointCloud<pcl::Normal>);//法线
//	//pcl::PointCloud<pcl::Normal>::Ptr normal_s(new pcl::PointCloud<pcl::Normal>);
//	////读取模型和场景点云
//	//pcl::io::loadPLYFile("./pcd/2009-10-27/Scene1.ply", *cloud_scene);
//	//pcl::io::loadPLYFile("./pcd/2009-10-27/model1.ply", *cloud_model1);
//	//pcl::io::loadPLYFile("./pcd/2009-10-27/model2.ply", *cloud_model2);
//	//*cloud_model = *cloud_model1;
//	//*cloud_model += *cloud_model2;
//	//std::cout << "points number of model: " << cloud_model->size() << std::endl;
//	//std::cout << "points number of scene: " << cloud_scene->size() << std::endl;
//
//	////计算点云的网格分辨率
//	//ToolBox tb;
//	//resolution_model = tb.computeMeshResolution(cloud_model);
//	//resolution_scene = tb.computeMeshResolution(cloud_scene);
//	//double cm_r = tb.computeCloudResolution(cloud_model);
//	//double cs_r = tb.computeCloudResolution(cloud_scene);
//	//std::cout << "mesh resolution of model: " << resolution_model << std::endl;
//	//std::cout << "mesh resolution of scene: " << resolution_scene << std::endl;
//	//std::cout << "cloud resolution of model: " << cm_r << std::endl;
//	//std::cout << "cloud resolution of scene: " << cs_r << std::endl;
//	////设置与分辨率有关的参数
//	//radius_normal_m = 4 * resolution_model;//法线的计算半径
//	//radius_normal_s = 4 * resolution_scene;
//	//radius_feature_m = 15 * resolution_model;//特征的计算半径
//	//radius_feature_s = 15 * resolution_scene;
//	//beta = 10 * resolution_model;
//
//	////计算法线
//	//MyNormalEstimation ne;
//	//ne.computeNormal(cloud_model, cloud_scene, radius_normal_m, radius_normal_s, normal_m, normal_s);
//	//std::cout << "Normal Estimation Completed. " << normal_m->size() << "   " << normal_s->size() << std::endl;
//
//	////检测关键点（随机采样）
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m(new pcl::PointCloud<pcl::PointXYZRGB>);//初始的模型关键点（未过滤无效点）
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m(new pcl::PointCloud<pcl::PointXYZRGB>);//模型关键点云
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s(new pcl::PointCloud<pcl::PointXYZRGB>);//场景关键点云
//	//KeypointsDetector key_detector;
//	//key_detector.compute(cloud_model, cloud_scene, keypoints_m, keypoints_s, num_corr_key, num_s_noise, ground_truth, radius_normal_s);
//	//std::cout << "Keypoints Detection Completed. " << "  model keypoints: " << keypoints_m->size() << "  scene keypoints: " << keypoints_s->size() << std::endl;
//	///*KeypointsDetector key_detector;
//	//key_detector.computeRandomSample(cloud_model, cloud_scene, keypoints_unfiltered_m, key_num_m);
//	//std::cout << "Keypoints(random) Sampling  Completed. " << "  model keypoints (unfiltered): " << keypoints_unfiltered_m->size() << std::endl;
//	//key_detector.filterValidKeypoints(cloud_model, cloud_scene, keypoints_unfiltered_m, ground_truth, radius_normal_s, keypoints_m, keypoints_s);
//	//std::cout << "Keypoints Filtering  Completed. " << "  model keypoints: " << keypoints_m->size() << "  scene keypoints: " << keypoints_s->size() << std::endl;*/
//
//	//////检测关键点（ISS3D）
//	////pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_unfiltered_m(new pcl::PointCloud<pcl::PointXYZRGB>);//初始的模型关键点（未过滤无效点）
//	////pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_m(new pcl::PointCloud<pcl::PointXYZRGB>);
//	////pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_s(new pcl::PointCloud<pcl::PointXYZRGB>);
//	////key_detector.computeISS3DKeypoints(cloud_model, cloud_scene, normal_m, normal_s, keypoints_iss_unfiltered_m, resolution_model);
//	////std::cout << "Keypoints(ISS3D) Sampling Completed. " << "  model keypoints (unfiltered): " << keypoints_iss_unfiltered_m->size() << std::endl;
//	////key_detector.filterValidKeypoints(cloud_model, cloud_scene, keypoints_iss_unfiltered_m, ground_truth, radius_normal_s, keypoints_iss_m, keypoints_iss_s);
//	////std::cout << "Keypoints Filtering Completed. " << "  model keypoints: " << keypoints_iss_m->size() << "  scene keypoints: " << keypoints_iss_s->size() << std::endl;
//
//	////计算某一特定特征
//	////1.pfh
//	//pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_m(new pcl::PointCloud<pcl::PFHSignature125>());
//	//pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_s(new pcl::PointCloud<pcl::PFHSignature125>());
//	//FeatureExtractorPFH ex_pfh;
//	//ex_pfh.computePFH(cloud_model, cloud_scene, keypoints_m, keypoints_s, normal_m, normal_s, radius_feature_m, radius_feature_s, *pfh_m, *pfh_s);
//	//std::cout << "Feature PFH Extraction Completed." << "  " << pfh_m->size() << "  " << pfh_s->size() << std::endl;
//	//////2.fpfh
//	////pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_m(new pcl::PointCloud<pcl::FPFHSignature33>());
//	////pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_s(new pcl::PointCloud<pcl::FPFHSignature33>());
//	////FeatureExtractorFPFH ex_fpfh;
//	////ex_fpfh.computeFPFH(cloud_model, cloud_scene, keypoints_m, keypoints_s, normal_m, normal_s, radius_feature_m, radius_feature_s, *fpfh_m, *fpfh_s);
//	////std::cout << "Feature FPFH Extraction Completed." << "  " << fpfh_m->size() << "  " << fpfh_s->size() << std::endl;
//
//	////特征匹配
//	////1.pfh特征匹配
//	//pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());//最近邻
//	//pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());//次近邻
//	//pcl::CorrespondencesPtr corrs_matched(new pcl::Correspondences());//匹配结果
//	//FeatureMatchPFH match_pfh;
//	//match_pfh.computeNN(keypoints_m, keypoints_s, pfh_m, pfh_s, corrs_n1, corrs_n2);
//	//match_pfh.filterNN(corrs_n1, corrs_n2, alpha, corrs_matched);
//	//std::cout << "Feature PFH Matching Completed.  " << "alpha: " << alpha << std::endl;
//	//std::cout << "Number of Feature Matched: " << corrs_matched->size() << std::endl;
//
//	////验证匹配
//	//Validator va;
//	//pcl::CorrespondencesPtr corr_distence(new pcl::Correspondences());//存储着对应点之间的三维空间距离（距离偏差）
//	//va.calculateDeviation(keypoints_m, keypoints_s, ground_truth, corrs_matched, corr_distence);
//	//pcl::CorrespondencesPtr corr_verified(new pcl::Correspondences());//存储距离偏差在阈值内的对应点
//	//va.verify(corr_distence, corr_verified, beta);
//	//std::cout << "Feature Verification Completed. " << "beta: " << beta << std::endl;
//	//std::cout << "Number of Correct Correspondence: " << corr_verified->size() << std::endl;
//
//	////计算当前阈值参数下的精度和召回率
//	//double p = (static_cast<double>(corr_verified->size())) / (corrs_matched->size());
//	//double r = (static_cast<double>(corr_verified->size())) / (keypoints_m->size());
//	//std::cout << "precision: " << p << std::endl;
//	//std::cout << "recall: " << r << std::endl;
//
//
//	////可视化
//	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	//viewer->setBackgroundColor(0, 0, 0);
//	//
//	////显示原始点云
//	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_model, "cloud_model");
//	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_scene, "cloud_scene");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_model");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_scene");
//	////显示关键点
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handle_1(keypoints_m, 255, 0, 0);//模型点云关键点为红色
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handle_2(keypoints_s, 255, 0, 0);//场景点云关键点为红色
//	//viewer->addPointCloud<pcl::PointXYZRGB>(keypoints_m, color_handle_1, "keypoints_m");
//	//viewer->addPointCloud<pcl::PointXYZRGB>(keypoints_s, color_handle_2, "keypoints_s");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints_m");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints_s");
//
//	//////显示匹配对应
//	////viewer->addCorrespondences<pcl::PointXYZRGB>(keypoints_m, keypoints_s, *corrs_matched,"correspondences_matched");
//	////viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 1, "correspondences_matched");
//	////viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences_matched");
//	//////显示经过验证的正确匹配对应
//	////viewer->addCorrespondences<pcl::PointXYZRGB>(keypoints_m, keypoints_s, *corr_verified,"correspondences_verified");
//	////viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 1, "correspondences_verified");
//	////viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "correspondences_verified");
//
//	////viewer->addLine<pcl::PointXYZRGB>((*cloud)[0],(*cloud)[cloud->size() - 1], "line");
//	//viewer->addSphere((*cloud_model)[0], 0.922109, 0.5, 0.5, 0.0, "sphere");
//
//	////显示法线
//	////viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_model, normal_m, 15, 0.05, "normal_m");
//	////viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_scene, normal_s, 15, 0.05, "normal_s");
//	//viewer->addCoordinateSystem(1.0);
//	//viewer->initCameraParameters();
//
//	//////可视化特征直方图
//	////pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("PFH Feature Histogram");
//	////std::cout <<"getFieldsList PFHSignature125 :" << pcl::getFieldsList<pcl::PFHSignature125>(*pfh_m)<<std::endl;
//	////std::cout <<"getFieldsList PointXYZRGB :" << pcl::getFieldsList<pcl::PointXYZRGB>(*cloud_model)<< std::endl;
//	////plotter->addFeatureHistogram(*pfh_m,"pfh",5,"pfh_m");
//	////plotter->plot();
//
//	//endTime = clock();//计时结束
//	//cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
//	//while (!viewer->wasStopped())
//	//{
//	//	viewer->spinOnce(100);
//	//}
//
//	//return 0;
//}