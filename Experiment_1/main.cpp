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
#include "cfh_rgb_pfh_rate.h"

#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include "keypoints_detector.h"
#include "cfh_rgb_pfh_dot.h"
#include "cfh_rgb_fpfh_rate.hpp"

#ifndef NumberOfThreads
#define NumberOfThreads 2
#endif // !NumberOfThreads

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
//    ne.setNumberOfThreads(NumberOfThreads);
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
//    //es.setNumberOfThreads(NumberOfThreads);
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
#define NUM_OF_DESCRIPTOR_TESTED 6
//正在测试的场景数量 dataset3=15 dataset4=16
#define NUM_OF_SCNEN 15//换数据集时候记得改这里！！！！！！！
//匹配阈值分组数量
#define NUM_ALPHA 20

#define DATASET_3
//#define DATASET_4
//#define DATASET_5

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

	std::vector<std::string> features_name = { "FPFH_CFH_HSV_FPFH_RATE","FPFH_CFH_HSV_FPFH_DOT","FPFH_CFH_LAB_FPFH_RATE" ,"FPFH_CFH_LAB_FPFH_DOT" ,"FPFH_CFH_LAB_FPFH_L1" ,"FPFH_CFH_LAB_FPFH_L2" };
	//std::vector<std::string> features_name = {"FPFH","FPFH_CFH_RGB_FPFH_RATE","FPFH_CFH_HSV_FPFH_RATE","FPFH_CFH_LAB_FPFH_RATE","FPFH_CFH_LAB_FPFH_DOT","PFHRGB","CSHOT"};
	//std::vector<std::string> features_name = {"FPFH"};


	/*
	测试新的特征时，注意：

	1. main的fatures_name对应于experiment::perform的features_name，须手动设置为与experiment中当前实验安排一致；
	
	2. 手动设置 NUM_OF_DESCRIPTOR_TESTED

	3. 手动设置NUM_ALPHA，须与Experiment::computeALLPR_2::alpha_devide一致

	4. experiment.h、main()、experiment.cpp 的 matches_sum_cfh_rgb_pfh_rate_等内容，须一致,顺序必须保持一致。

	5. 由于feature_extractor中的match等方法参数已有重载，因此对于新特征，只需要写一个计算方法就行。

	6. feature_extractor.cpp处需要引用hpp而非h，否则出现无法解析的错误
	*/

	std::vector<double> precision;
	std::vector<double> recall;

#ifdef DATASET_3
	//设置数据集名称
	exp.setDatasetName("dataset3");
	exp.setModelNumber(2);//设置每个场景对应几个模型
	exp.setAlphaUpperLimit(1.0);//alpha阈值上限
	//group-1-scene1
	exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group1-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-1-scene2
	exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group1-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-1-scene3
	exp.setIniPath("./dataset3/3D models/CVLab/2009-10-27/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset3/dataset3-group1-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
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
#endif // DATASET_3

#ifdef DATASET_4
	//设置数据集名称
	exp.setDatasetName("dataset4");
	exp.setModelNumber(2);//设置每个场景对应几个模型
	exp.setAlphaUpperLimit(1.0);//alpha阈值上限
	//group-1-scene1
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene1/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group1-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-1-scene2
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene1/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group1-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-1-scene3
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene1/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group1-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-1-scene4
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene1/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group1-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-2-scene1
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene2/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group2-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene2
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene2/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group2-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene3
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene2/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group2-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-2-scene4
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene2/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group2-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-3-scene1
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene3/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group3-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene2
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene3/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group3-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene3
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene3/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group3-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-3-scene4
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene3/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group3-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//group-4-scene1
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene4/ConfigScene1.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group4-scene1-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene2
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene4/ConfigScene2.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group4-scene2-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene3
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene4/ConfigScene3.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group4-scene3-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//group-4-scene4
	exp.setIniPath("./dataset4/3D models/CVLab/2010-06-12/Scene4/ConfigScene4.ini");
	exp.setOutputTXTPath("./dataset4/dataset4-group4-scene4-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
#endif // DATASET_4

#ifdef DATASET_5
	//设置数据集名称
	exp.setDatasetName("dataset5");
	exp.setModelNumber(2);//设置每个场景对应几个模型
	exp.setAlphaUpperLimit(1.0);//alpha阈值上限
	//001
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene001.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene001-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//003
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene003.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene003-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//006
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene006.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene006-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//010
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene010.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene010-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//011
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene011.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene011-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//013
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene013.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene013-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//014
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene014.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene014-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//018
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene018.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene018-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//019
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene019.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene019-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//022
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene022.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene022-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//030
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene030.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene030-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	endTime = clock();//计时结束
	time_vec.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);

	//031
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene031.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene031-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//032
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene032.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene032-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//036
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene036.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene036-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
	//039
	exp.setIniPath("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/configwithbestview/Configscene039.ini");
	exp.setOutputTXTPath("./dataset5/dataset5-scene039-out.txt");
	exp.perform(precision, recall);
	tb.accumulateVector(sum_precision, precision);
	tb.accumulateVector(sum_recall, recall);
#endif // DATASET_5

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
#ifdef DATASET_3
	exp.writePRToTXT("./dataset3/result.txt", sum_precision, sum_recall, features_name);
#endif // DATASET_3

#ifdef DATASET_4
	exp.writePRToTXT("./dataset4/result.txt", sum_precision, sum_recall, features_name);
#endif // DATASET_4

#ifdef DATASET_5
	exp.writePRToTXT("./dataset5/result.txt", sum_precision, sum_recall, features_name);
#endif // DATASET_5


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

	////CFH_RGB_FPFH_RATE  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_fpfh_rate_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_rate_[i]) / exp.matches_sum_cfh_rgb_fpfh_rate_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_rate_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_rate_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_RGB_FPFH_DOT  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_fpfh_dot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_dot_[i]) / exp.matches_sum_cfh_rgb_fpfh_dot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_dot_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_dot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_RGB_FPFH_L1  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_fpfh_L1_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_L1_[i]) / exp.matches_sum_cfh_rgb_fpfh_L1_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_L1_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_L1_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	 
	////CFH_RGB_FPFH_L2  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_fpfh_L2_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_L2_[i]) / exp.matches_sum_cfh_rgb_fpfh_L2_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_L2_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_L2_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	 
	////CFH_RGB_FPFH_NEW  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_rgb_fpfh_new_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_new_[i]) / exp.matches_sum_cfh_rgb_fpfh_new_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_rgb_fpfh_new_[i]) / exp.corresponding_regions_sum_cfh_rgb_fpfh_new_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_HSV_FPFH_RATE  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_hsv_fpfh_rate_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_rate_[i]) / exp.matches_sum_cfh_hsv_fpfh_rate_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_rate_[i]) / exp.corresponding_regions_sum_cfh_hsv_fpfh_rate_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_HSV_FPFH_DOT  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_hsv_fpfh_dot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_dot_[i]) / exp.matches_sum_cfh_hsv_fpfh_dot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_dot_[i]) / exp.corresponding_regions_sum_cfh_hsv_fpfh_dot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_HSV_FPFH_L1  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_hsv_fpfh_L1_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_L1_[i]) / exp.matches_sum_cfh_hsv_fpfh_L1_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_L1_[i]) / exp.corresponding_regions_sum_cfh_hsv_fpfh_L1_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_HSV_FPFH_L2  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_hsv_fpfh_L2_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_L2_[i]) / exp.matches_sum_cfh_hsv_fpfh_L2_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_hsv_fpfh_L2_[i]) / exp.corresponding_regions_sum_cfh_hsv_fpfh_L2_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_LAB_FPFH_RATE  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_lab_fpfh_rate_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_rate_[i]) / exp.matches_sum_cfh_lab_fpfh_rate_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_rate_[i]) / exp.corresponding_regions_sum_cfh_lab_fpfh_rate_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_LAB_FPFH_DOT  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_lab_fpfh_dot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_dot_[i]) / exp.matches_sum_cfh_lab_fpfh_dot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_dot_[i]) / exp.corresponding_regions_sum_cfh_lab_fpfh_dot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_LAB_FPFH_L1  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_lab_fpfh_L1_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_L1_[i]) / exp.matches_sum_cfh_lab_fpfh_L1_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_L1_[i]) / exp.corresponding_regions_sum_cfh_lab_fpfh_L1_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////CFH_LAB_FPFH_L2  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_cfh_lab_fpfh_L2_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_L2_[i]) / exp.matches_sum_cfh_lab_fpfh_L2_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_cfh_lab_fpfh_L2_[i]) / exp.corresponding_regions_sum_cfh_lab_fpfh_L2_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	//----------------------S+C-----------------------------

	////fpfh_cfh_rgb_fpfh_dot-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_rgb_fpfh_dot_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_dot_[i]) / exp.matches_sum_fpfh_cfh_rgb_fpfh_dot_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_dot_[i]) / exp.corresponding_regions_sum_fpfh_cfh_rgb_fpfh_dot_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	 
	////fpfh_cfh_rgb_fpfh_rate-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_rgb_fpfh_rate_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_rate_[i]) / exp.matches_sum_fpfh_cfh_rgb_fpfh_rate_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_rate_[i]) / exp.corresponding_regions_sum_fpfh_cfh_rgb_fpfh_rate_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////fpfh_cfh_rgb_fpfh_L1-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_rgb_fpfh_L1_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_L1_[i]) / exp.matches_sum_fpfh_cfh_rgb_fpfh_L1_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_L1_[i]) / exp.corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L1_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	
	////fpfh_cfh_rgb_fpfh_L2-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_rgb_fpfh_L2_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_L2_[i]) / exp.matches_sum_fpfh_cfh_rgb_fpfh_L2_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_L2_[i]) / exp.corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L2_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}
	 
	////fpfh_cfh_rgb_fpfh_new-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_rgb_fpfh_new_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_new_[i]) / exp.matches_sum_fpfh_cfh_rgb_fpfh_new_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_rgb_fpfh_new_[i]) / exp.corresponding_regions_sum_fpfh_cfh_rgb_fpfh_new_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	//fpfh_cfh_hsv_fpfh_rate-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_hsv_fpfh_rate_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_rate_[i]) / exp.matches_sum_fpfh_cfh_hsv_fpfh_rate_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_rate_[i]) / exp.corresponding_regions_sum_fpfh_cfh_hsv_fpfh_rate_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	//fpfh_cfh_hsv_fpfh_dot-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_hsv_fpfh_dot_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_dot_[i]) / exp.matches_sum_fpfh_cfh_hsv_fpfh_dot_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_dot_[i]) / exp.corresponding_regions_sum_fpfh_cfh_hsv_fpfh_dot_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	////fpfh_cfh_hsv_fpfh_L1-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_hsv_fpfh_L1_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_L1_[i]) / exp.matches_sum_fpfh_cfh_hsv_fpfh_L1_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_L1_[i]) / exp.corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L1_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	////fpfh_cfh_hsv_fpfh_L2-pr  ! 如果不是正在测试的特征，请将此处注释
	//for (size_t i = 0; i < 20; i++)
	//{
	//	if (exp.matches_sum_fpfh_cfh_hsv_fpfh_L2_[i] == 0)
	//	{
	//		p = 1;
	//		r = 0;
	//	}
	//	else
	//	{
	//		p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_L2_[i]) / exp.matches_sum_fpfh_cfh_hsv_fpfh_L2_[i];
	//		r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_hsv_fpfh_L2_[i]) / exp.corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L2_[i];
	//	}
	//	vector_p.push_back(p);
	//	vector_r.push_back(r);
	//}

	

	//fpfh_cfh_lab_fpfh_rate-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_lab_fpfh_rate_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_rate_[i]) / exp.matches_sum_fpfh_cfh_lab_fpfh_rate_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_rate_[i]) / exp.corresponding_regions_sum_fpfh_cfh_lab_fpfh_rate_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	//fpfh_cfh_lab_fpfh_dot-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_lab_fpfh_dot_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_dot_[i]) / exp.matches_sum_fpfh_cfh_lab_fpfh_dot_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_dot_[i]) / exp.corresponding_regions_sum_fpfh_cfh_lab_fpfh_dot_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	//fpfh_cfh_lab_fpfh_L1-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_lab_fpfh_L1_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_L1_[i]) / exp.matches_sum_fpfh_cfh_lab_fpfh_L1_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_L1_[i]) / exp.corresponding_regions_sum_fpfh_cfh_lab_fpfh_L1_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}

	//fpfh_cfh_lab_fpfh_L2-pr  ! 如果不是正在测试的特征，请将此处注释
	for (size_t i = 0; i < 20; i++)
	{
		if (exp.matches_sum_fpfh_cfh_lab_fpfh_L2_[i] == 0)
		{
			p = 1;
			r = 0;
		}
		else
		{
			p = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_L2_[i]) / exp.matches_sum_fpfh_cfh_lab_fpfh_L2_[i];
			r = static_cast<double>(exp.correct_matches_sum_fpfh_cfh_lab_fpfh_L2_[i]) / exp.corresponding_regions_sum_fpfh_cfh_lab_fpfh_L2_[i];
		}
		vector_p.push_back(p);
		vector_r.push_back(r);
	}
	
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

#ifdef DATASET_3
	exp.writePRToTXT("./dataset3/result_2.txt", vector_p, vector_r, features_name);
	std::ofstream outfile;   //输出流
	outfile.open("./dataset3/time.txt", ios::app);
#endif // DATASET_3

#ifdef DATASET_4
	exp.writePRToTXT("./dataset4/result_2.txt", vector_p, vector_r, features_name);
	std::ofstream outfile;   //输出流
	outfile.open("./dataset4/time.txt", ios::app);
#endif // DATASET_4

#ifdef DATASET_5
	exp.writePRToTXT("./dataset5/result_2.txt", vector_p, vector_r, features_name);
	std::ofstream outfile;   //输出流
	outfile.open("./dataset5/time.txt", ios::app);
#endif // DATASET_5


	


	if (!outfile.is_open())
		std::cout << "Open file failure" << std::endl;
	for (size_t i = 0; i < time_vec.size(); i++)
	{
		outfile << time_vec[i];
		outfile << "\n";
	}
	outfile.close();
}