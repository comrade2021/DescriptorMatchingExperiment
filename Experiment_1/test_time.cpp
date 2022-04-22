///*对比特征描述符的时间性能：在场景和模型点云上计算特征、匹配特征（NNDR方式，阈值为0）*/
//
////注意：FPFH、FPFH(S+C)、CSHOT都不能用OMP加速，在feature_extractor.cpp中修改
//
//#include <pcl/pcl_base.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <vtkAutoInit.h>
//#include <string>
//#include <iostream>
//#include "keypoints_detector.h"
//#include "my_normal_estimation.h"
//#include "toolbox.h"
//#include <pcl/features/normal_3d_omp.h>
//#include "keypoints_detector.h"
//#include "feature_extractor.h"
//#include "feature_match.h"
//#include <pcl/filters/random_sample.h>
//
//#define NUM_OF_KEYPOINTS 2000	//每次采集的关键点数量
//#define RADIUS_K_OF_NORMAL 10	//用于法线估计的邻域点数量(k近邻方式)
////#define RADIUS_COEFF_OF_FEATURE 5	//特征的计算半径为N倍的网格分辨率
//
//int main() {
//	int RADIUS_COEFF_OF_FEATURE;
//	//int NUM_OF_KEYPOINTS=0;
//	double threshold_alpha = 0;//匹配时的alpha阈值
//
//	//测试特征计算时间
//	for (size_t ii = 4; ii <= 8; ii = ii +1)
//	{
//		/*NUM_OF_KEYPOINTS = ii;
//		std::cout << "------------------------------" << NUM_OF_KEYPOINTS << "------------------------------" << std::endl;*/
//		
//		RADIUS_COEFF_OF_FEATURE = ii;
//		std::cout << "------------------------------" << RADIUS_COEFF_OF_FEATURE << "------------------------------" << std::endl;
//
//		//计时器
//		clock_t startTime, endTime;
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZRGB>);//点云
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
//		pcl::PointCloud<pcl::Normal>::Ptr normal_m(new pcl::PointCloud<pcl::Normal>);//法线
//		pcl::PointCloud<pcl::Normal>::Ptr normal_s(new pcl::PointCloud<pcl::Normal>);
//		//读取模型和场景点云
//		pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2009-10-27/Scene1.ply", *cloud_model);
//		pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2009-10-27/Scene2.ply", *cloud_scene);
//		std::cout << "point number of model:   " << cloud_model->size() << std::endl;
//		std::cout << "point number of scene:   " << cloud_scene->size() << std::endl;
//		std::cout << std::endl;
//
//		//计算点云的网格分辨率
//		ToolBox tb;
//		float resolution_model = tb.computeMeshResolution(cloud_model);
//		float resolution_scene = tb.computeMeshResolution(cloud_scene);
//		std::cout << "mesh resolution of model:   " << resolution_model << std::endl;
//		std::cout << "mesh resolution of scene:   " << resolution_scene << std::endl;
//		std::cout << std::endl;
//
//		//设置与分辨率有关的参数
//		float radius_feature_m = RADIUS_COEFF_OF_FEATURE * resolution_model;//特征的计算半径
//		float radius_feature_s = RADIUS_COEFF_OF_FEATURE * resolution_model;//为了特征的一致性，两个半径都是resolution_model的倍数
//
//		//计算法线
//		startTime = clock();//计时开始
//		MyNormalEstimation ne;
//		ne.computeNormal_K(cloud_model, RADIUS_K_OF_NORMAL, normal_m);
//		ne.computeNormal_K(cloud_scene, RADIUS_K_OF_NORMAL, normal_s);
//		endTime = clock();//计时结束
//		double time_normal = (double)(endTime - startTime);
//		std::cout << "radius_k_of_normal:   " << RADIUS_K_OF_NORMAL << std::endl;
//		std::cout << "radius_of_feature:   " << RADIUS_COEFF_OF_FEATURE << "*MR" << std::endl;
//		std::cout << "radius_of_feature_m:   " << radius_feature_m << std::endl;
//		std::cout << "radius_of_feature_s:   " << radius_feature_s << std::endl;
//		std::cout << std::endl;
//		//法线的运行时间
//		std::cout << "time_normal_estimation:   " << time_normal  << std::endl;
//		std::cout << std::endl;
//
//		//检测关键点
//		startTime = clock();//计时开始
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_ran_m(new pcl::PointCloud<pcl::PointXYZRGB>);
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_ran_s(new pcl::PointCloud<pcl::PointXYZRGB>);
//		pcl::RandomSample<pcl::PointXYZRGB> rs;
//		rs.setInputCloud(cloud_model);
//		rs.setSeed(3866);//系统启动以来的嘀嗒时间作为随机种子
//		rs.setSample(NUM_OF_KEYPOINTS);
//		rs.filter(*keypoints_ran_m);
//
//		rs.setInputCloud(cloud_scene);
//		rs.setSeed(3866);//系统启动以来的嘀嗒时间作为随机种子
//		rs.setSample(NUM_OF_KEYPOINTS);
//		rs.filter(*keypoints_ran_s);
//		endTime = clock();//计时结束
//		double time_keypoints = (double)(endTime - startTime);
//		std::cout << "num_of_keypoints_m:   " << NUM_OF_KEYPOINTS << std::endl;
//		std::cout << "num_of_keypoints_s:   " << NUM_OF_KEYPOINTS << std::endl;
//		std::cout << "time_keypoints_ran_sample:   " << time_keypoints  << std::endl;
//		std::cout << std::endl;
//
//		//计算特征的平均邻域点数量
//		startTime = clock();//计时开始
//		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_search;
//		pcl::PointXYZRGB searchPoint;
//		int count_m = 0;
//		int count_s = 0;
//		//model
//		kdtree_search.setInputCloud(cloud_model);
//		for (size_t i = 0; i < keypoints_ran_m->size(); i++)
//		{
//			searchPoint.x = keypoints_ran_m->at(i).x;
//			searchPoint.y = keypoints_ran_m->at(i).y;
//			searchPoint.z = keypoints_ran_m->at(i).z;
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			int k = kdtree_search.radiusSearch(searchPoint, radius_feature_m, pointIdxRadiusSearch, pointRadiusSquaredDistance);
//			count_m = count_m + (k - 1);//减一是由于近邻搜索的最近点是关键点本身
//		}
//		double ave_m = count_m / keypoints_ran_m->size();
//		//scene
//		kdtree_search.setInputCloud(cloud_scene);
//		for (size_t i = 0; i < keypoints_ran_s->size(); i++)
//		{
//			searchPoint.x = keypoints_ran_s->at(i).x;
//			searchPoint.y = keypoints_ran_s->at(i).y;
//			searchPoint.z = keypoints_ran_s->at(i).z;
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			int k = kdtree_search.radiusSearch(searchPoint, radius_feature_m, pointIdxRadiusSearch, pointRadiusSquaredDistance);
//			count_s = count_s + (k - 1);//减一是由于近邻搜索的最近点是关键点本身
//		}
//		double ave_s = count_s / keypoints_ran_s->size();
//		std::cout << "ave_points_m:   " << ave_m << std::endl;
//		std::cout << "ave_points_s:   " << ave_s << std::endl;
//		endTime = clock();//计时结束
//		double time_average = (double)(endTime - startTime);
//		std::cout << "time_compute_average:   " << time_average  << std::endl;
//		std::cout << std::endl;
//
//
//		//测试4种描述子的计算和匹配时间
//		//1. FPFH
//		//计算特征：FPFH
//		startTime = clock();//计时开始
//		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
//		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
//		FeatureExtractor ex001;
//		ex001.computeFPFH(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_fpfh);
//		ex001.computeFPFH(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_fpfh);
//		endTime = clock();//计时结束
//		double time_feature_fpfh = (double)(endTime - startTime);
//		std::cout << "time_feature_fpfh:   " << time_feature_fpfh  << std::endl;
//		//特征匹配(求出最近和次近邻)
//		//匹配：FPFH
//		startTime = clock();//计时开始
//		pcl::CorrespondencesPtr corrs_match_n1_fpfh(new pcl::Correspondences());//最近邻对应
//		pcl::CorrespondencesPtr corrs_match_n2_fpfh(new pcl::Correspondences());//次近邻对应
//		FeatureMatch matcher001;
//		matcher001.computeNN(feature_m_fpfh, feature_s_fpfh, corrs_match_n1_fpfh, corrs_match_n2_fpfh);
//		matcher001.computeNN(feature_s_fpfh,feature_m_fpfh, corrs_match_n1_fpfh, corrs_match_n2_fpfh);
//		//matcher001.match_r(feature_m_fpfh, feature_s_fpfh, corrs_match_n1_fpfh,0);
//		endTime = clock();//计时结束
//		double time_match_fpfh = (double)(endTime - startTime);
//		std::cout << "time_match_fpfh:   " << time_match_fpfh  << std::endl;
//		std::cout << std::endl;
//
//		////2. FPFH_CFH_HSV_FPFH_RATE
//		////计算特征：FPFH_SC_TIME,模拟对66的计算
//		//startTime = clock();//计时开始
//		//pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m_fpfhsc_t(new pcl::PointCloud<pcl::FPFHSignature33>());
//		//pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s_fpfhsc_t(new pcl::PointCloud<pcl::FPFHSignature33>());
//		//FeatureExtractor ex002_t;
//		//ex002_t.computeCFH_HSV_FPFH_RATE(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_fpfhsc_t);
//		//ex002_t.computeCFH_HSV_FPFH_RATE(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_fpfhsc_t);
//		//endTime = clock();//计时结束
//		//double time_feature_fpfh_sc = (double)(endTime - startTime);
//		//std::cout << "time_feature_fpfh_sc:   " << time_feature_fpfh_sc  << std::endl;
//		//
//		////计算特征：FPFH_CFH_HSV_FPFH_RATE
//		//pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m_fpfhsc(new pcl::PointCloud<pcl::PFHSignature125>());
//		//pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s_fpfhsc(new pcl::PointCloud<pcl::PFHSignature125>());
//		//FeatureExtractor ex002;
//		//ex002.computeFPFH_CFH_HSV_FPFH_RATE(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_fpfhsc);
//		//ex002.computeFPFH_CFH_HSV_FPFH_RATE(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_fpfhsc);
//
//		////特征匹配(求出最近和次近邻)
//		////匹配：FPFH_CFH_HSV_FPFH_RATE
//		//startTime = clock();//计时开始
//		//pcl::CorrespondencesPtr corrs_match_n1_fpfhsc(new pcl::Correspondences());//最近邻对应
//		//pcl::CorrespondencesPtr corrs_match_n2_fpfhsc(new pcl::Correspondences());//次近邻对应
//		//FeatureMatch matcher002;
//		//matcher002.computeNN(feature_m_fpfhsc,feature_s_fpfhsc, corrs_match_n1_fpfhsc, corrs_match_n2_fpfhsc);
//		////matcher002.match_r(feature_m_fpfhsc, feature_s_fpfhsc, corrs_match_n1_fpfhsc, 0);
//		//endTime = clock();//计时结束
//		//double time_match_fpfh_sc = (double)(endTime - startTime);
//		//std::cout << "time_match_fpfh_sc:   " << time_match_fpfh_sc  << std::endl;
//		//std::cout << std::endl;
//
//		//2. FPFH_CFH_LAB_FPFH_DOT
//		startTime = clock();//计时开始
//		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m_fpfhsc1(new pcl::PointCloud<pcl::PFHSignature125>());
//		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s_fpfhsc1(new pcl::PointCloud<pcl::PFHSignature125>());
//		FeatureExtractor ex0021;
//		ex0021.computeFPFH_CFH_LAB_FPFH_DOT(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_fpfhsc1);
//		ex0021.computeFPFH_CFH_LAB_FPFH_DOT(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_fpfhsc1);
//		endTime = clock();//计时结束
//		double time_feature_fpfhsc1 = (double)(endTime - startTime);
//		std::cout << "time_feature_fpfhsc1:   " << time_feature_fpfhsc1 << std::endl;
//
//		//特征匹配(求出最近和次近邻)
//		startTime = clock();//计时开始
//		pcl::CorrespondencesPtr corrs_match_n1_fpfhsc1(new pcl::Correspondences());//最近邻对应
//		pcl::CorrespondencesPtr corrs_match_n2_fpfhsc1(new pcl::Correspondences());//次近邻对应
//		FeatureMatch matcher0021;
//		matcher0021.computeNN(feature_m_fpfhsc1,feature_s_fpfhsc1, corrs_match_n1_fpfhsc1, corrs_match_n2_fpfhsc1);
//		//matcher002.match_r(feature_m_fpfhsc, feature_s_fpfhsc, corrs_match_n1_fpfhsc, 0);
//		endTime = clock();//计时结束
//		double time_match_fpfh_sc1 = (double)(endTime - startTime);
//		std::cout << "time_match_fpfh_sc1:   " << time_match_fpfh_sc1  << std::endl;
//		std::cout << std::endl;
//		
//		//2. FPFH_CFH_LAB_FPFH_RATE
//		startTime = clock();//计时开始
//		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m_fpfhsc11(new pcl::PointCloud<pcl::PFHSignature125>());
//		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s_fpfhsc11(new pcl::PointCloud<pcl::PFHSignature125>());
//		FeatureExtractor ex00211;
//		ex00211.computeFPFH_CFH_LAB_FPFH_RATE(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_fpfhsc11);
//		ex00211.computeFPFH_CFH_LAB_FPFH_RATE(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_fpfhsc11);
//		endTime = clock();//计时结束
//		double time_feature_fpfhsc11 = (double)(endTime - startTime);
//		std::cout << "time_feature_fpfhsc11:   " << time_feature_fpfhsc11 << std::endl;
//
//		//特征匹配(求出最近和次近邻)
//		startTime = clock();//计时开始
//		pcl::CorrespondencesPtr corrs_match_n1_fpfhsc11(new pcl::Correspondences());//最近邻对应
//		pcl::CorrespondencesPtr corrs_match_n2_fpfhsc11(new pcl::Correspondences());//次近邻对应
//		FeatureMatch matcher00211;
//		matcher00211.computeNN(feature_m_fpfhsc11,feature_s_fpfhsc11, corrs_match_n1_fpfhsc11, corrs_match_n2_fpfhsc11);
//		//matcher002.match_r(feature_m_fpfhsc, feature_s_fpfhsc, corrs_match_n1_fpfhsc, 0);
//		endTime = clock();//计时结束
//		double time_match_fpfh_sc11 = (double)(endTime - startTime);
//		std::cout << "time_match_fpfh_sc11:   " << time_match_fpfh_sc11  << std::endl;
//		std::cout << std::endl;
//
//
//
//		////3. PFHRGB
//		////计算特征：PFHRGB
//		//startTime = clock();//计时开始
//		//pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m_pfhrgb(new pcl::PointCloud<pcl::PFHRGBSignature250>());
//		//pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s_pfhrgb(new pcl::PointCloud<pcl::PFHRGBSignature250>());
//		//FeatureExtractor ex003;
//		//ex003.computePFHRGB(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_pfhrgb);
//		//ex003.computePFHRGB(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_pfhrgb);
//		//endTime = clock();//计时结束
//		//double time_feature_pfhrgb = (double)(endTime - startTime);
//		//std::cout << "time_feature_pfhrgb:   " << time_feature_pfhrgb  << std::endl;
//		////特征匹配(求出最近和次近邻)
//		////匹配：PFHRGB
//		//startTime = clock();//计时开始
//		//pcl::CorrespondencesPtr corrs_match_n1_pfhrgb(new pcl::Correspondences());//最近邻对应
//		//pcl::CorrespondencesPtr corrs_match_n2_pfhrgb(new pcl::Correspondences());//次近邻对应
//		//FeatureMatch matcher003;
//		//matcher003.computeNN(feature_m_pfhrgb, feature_s_pfhrgb, corrs_match_n1_pfhrgb, corrs_match_n2_pfhrgb);
//		////matcher003.match_r(feature_m_pfhrgb, feature_s_pfhrgb, corrs_match_n1_pfhrgb, 0);
//		//endTime = clock();//计时结束
//		//double time_match_pfhrgb = (double)(endTime - startTime);
//		//std::cout << "time_match_pfhrgb:   " << time_match_pfhrgb  << std::endl;
//		//std::cout << std::endl;
//
//		////特征匹配(求出最近和次近邻)//125 test
//		////匹配：PFHRGB
//		//pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m_fpfhsc111(new pcl::PointCloud<pcl::PFHSignature125>());
//		//pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s_fpfhsc111(new pcl::PointCloud<pcl::PFHSignature125>());
//		//for (size_t i = 0; i < feature_m_pfhrgb->size(); i++)
//		//{
//		//	for (size_t j = 0; j < 125; j++)
//		//	{
//		//		feature_m_fpfhsc111->at(i).histogram[j] = feature_m_pfhrgb->at(i).histogram[j];
//		//	}
//		//}
//		//for (size_t i = 0; i < feature_s_pfhrgb->size(); i++)
//		//{
//		//	for (size_t j = 0; j < 125; j++)
//		//	{
//		//		feature_s_fpfhsc111->at(i).histogram[j] = feature_s_pfhrgb->at(i).histogram[j];
//		//	}
//		//}
//
//		//startTime = clock();//计时开始
//		//pcl::CorrespondencesPtr corrs_match_n1_pfhrgb1(new pcl::Correspondences());//最近邻对应
//		//pcl::CorrespondencesPtr corrs_match_n2_pfhrgb1(new pcl::Correspondences());//次近邻对应
//		//
//		//FeatureMatch matcher0031;
//		//matcher0031.computeNN(feature_m_fpfhsc111, feature_s_fpfhsc111, corrs_match_n1_pfhrgb1, corrs_match_n2_pfhrgb1);
//		////matcher003.match_r(feature_m_pfhrgb, feature_s_pfhrgb, corrs_match_n1_pfhrgb, 0);
//		//endTime = clock();//计时结束
//		//double time_match_pfhrgb1 = (double)(endTime - startTime);
//		//std::cout << "time_match_pfhrgb 111:   " << time_match_pfhrgb1 << std::endl;
//		//std::cout << std::endl;
//
//		////4. CSHOT
//		////计算特征：CSHOT
//		//startTime = clock();//计时开始
//		//pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m_cshot(new pcl::PointCloud<pcl::SHOT1344>());
//		//pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s_cshot(new pcl::PointCloud<pcl::SHOT1344>());
//		//FeatureExtractor ex004;
//		//ex004.computeCSHOT(cloud_model, keypoints_ran_m, normal_m, radius_feature_m, feature_m_cshot);
//		//ex004.computeCSHOT(cloud_scene, keypoints_ran_s, normal_s, radius_feature_s, feature_s_cshot);
//		//endTime = clock();//计时结束
//		//double time_feature_cshot = (double)(endTime - startTime);
//		//std::cout << "time_feature_cshot:   " << time_feature_cshot  << std::endl;
//		////特征匹配(求出最近和次近邻)
//		////匹配：CSHOT
//		//startTime = clock();//计时开始
//		//pcl::CorrespondencesPtr corrs_match_n1_cshot(new pcl::Correspondences());//最近邻对应
//		//pcl::CorrespondencesPtr corrs_match_n2_cshot(new pcl::Correspondences());//次近邻对应
//		//FeatureMatch matcher004;
//		//matcher004.computeNN(feature_m_cshot, feature_s_cshot, corrs_match_n1_cshot, corrs_match_n2_cshot);
//		////matcher004.match_r(feature_m_cshot, feature_s_cshot, corrs_match_n1_cshot, 0);
//		//endTime = clock();//计时结束
//		//double time_match_cshot = (double)(endTime - startTime);
//		//std::cout << "time_match_cshot:   " << time_match_cshot  << std::endl;
//		//std::cout << std::endl;
//
//		std::cout << "------------------------------" << "END" << "------------------------------" << std::endl;
//	}
//
//	
//
//
//
//	////可视化
//	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	//viewer->setBackgroundColor(0, 0, 0);
//
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
//	//cout << "The run time is: " << (double)(endTime - startTime)  << endl;
//	//while (!viewer->wasStopped())
//	//{
//	//	viewer->spinOnce(100);
//	//}
//
//	return 0;
//}