﻿#include "experiment.h"
#include <ctime>
#include "file_reader.h"
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <pcl/search/kdtree.h>
#include "toolbox.h"
#include "my_normal_estimation.h"
#include "keypoints_detector.h"
#include "feature_extractor.h"
#include "feature_match.h"
#include "validator.h"

//#define PFH
//#define FPFH
//#define CFH_RGB_PFH_RATE
//#define CFH_RGB_PFH_DOT

//--------------C------------------

//#define CFH_RGB_FPFH_RATE
//#define CFH_RGB_FPFH_DOT
//#define CFH_RGB_FPFH_L1
//#define CFH_RGB_FPFH_L2
//#define CFH_RGB_FPFH_NEW

//#define CFH_HSV_FPFH_RATE
//#define CFH_HSV_FPFH_DOT
//#define CFH_HSV_FPFH_L1
//#define CFH_HSV_FPFH_L2

//#define CFH_LAB_FPFH_RATE
//#define CFH_LAB_FPFH_DOT
//#define CFH_LAB_FPFH_L1
//#define CFH_LAB_FPFH_L2

//------------S+C--------------------

//#define FPFH_CFH_RGB_FPFH_RATE
//#define FPFH_CFH_RGB_FPFH_DOT
//#define FPFH_CFH_RGB_FPFH_L1
//#define FPFH_CFH_RGB_FPFH_L2
//#define FPFH_CFH_RGB_FPFH_NEW

#define FPFH_CFH_HSV_FPFH_RATE
#define FPFH_CFH_HSV_FPFH_DOT
//#define FPFH_CFH_HSV_FPFH_L1
//#define FPFH_CFH_HSV_FPFH_L2

#define FPFH_CFH_LAB_FPFH_RATE
#define FPFH_CFH_LAB_FPFH_DOT//LAB+CIE2000的直方图+33
#define FPFH_CFH_LAB_FPFH_L1
#define FPFH_CFH_LAB_FPFH_L2

//#define PFHRGB
//#define SHOT
//#define CSHOT

#define CSHOT_KEY
//#define VOXEL_KEY

void Experiment::perform(std::vector<double>& precision, std::vector<double>& recall)
{
	//清空vector
	std::vector<double>().swap(precision);
	std::vector<double>().swap(recall);

	std::vector<std::string> features_name;
	switch (model_number_)
	{
	case 2:
		std::vector<double>().swap(precision);
		std::vector<double>().swap(recall);
		computeALLPR_2(alpha_upper_limit_, precision, recall, features_name);
		writePRToTXT(output_txt_path_, precision, recall, features_name);
		break;
	default:
		PCL_ERROR("The number of models corresponding to each scene is incorrect\n");
		break;
	}

}

void Experiment::setModelNumber(int model_number)
{
	model_number_ = model_number;
}

void Experiment::setDatasetName(std::string dataset_name)
{
	dataset_name_ = dataset_name;
}

void Experiment::setIniPath(std::string ini_path)
{
	ini_path_ = ini_path;
}

void Experiment::setAlphaUpperLimit(double alpha_upper_limit)
{
	alpha_upper_limit_ = alpha_upper_limit;
}

void Experiment::setOutputTXTPath(std::string output_txt_path)
{
	output_txt_path_ = output_txt_path;
}

void Experiment::writePRToTXT(std::string file_path, std::vector<double>& precision, std::vector<double>& recall, std::vector<std::string>& features_name)
{
	size_t alpha_devide = 20;
	std::ofstream outfile;   //输出流
	outfile.open(file_path, ios::app);
	if (!outfile.is_open())
		std::cout << "Open file failure" << std::endl;
	for (size_t i = 0; i < features_name.size(); i++)
	{
		outfile << features_name[i] + "-P" + "\t";
		for (size_t j = 0; j < alpha_devide; j++)
		{
			outfile << precision[i * alpha_devide + j];
			if (j != (alpha_devide-1))
			{
				outfile << "\t";
			}
			else
			{
				outfile << "\n";
			}
		}
		outfile << features_name[i] + "-R" + "\t";
		for (size_t k = 0; k < alpha_devide; k++)
		{
			outfile << recall[i * alpha_devide + k];
			if (k != alpha_devide-1)
			{
				outfile << "\t";
			}
			else
			{
				outfile << "\n";
			}
		}
	}
	outfile.close();
}

void Experiment::computeALLPR_2(double alpha_upper_limit, std::vector<double>& precision, std::vector<double>& recall, std::vector<std::string>& features_name)
{
	std::cout << "========================================= Calculate the PR value once =========================================" << std::endl;
	std::cout << "model:2   scene:1 " << std::endl;
	//参数
	int num_key_m_1 = 100;//关键点数量
	int num_key_m_2 = 100;
	int num_key_s = 300;
	int noi = 100;

	double mr_s = 0;//网格分辨率
	double mr_1 = 0;
	double mr_2 = 0;
	double normal_radius_m_1 = 0.0;//法线的计算半径
	double normal_radius_m_2 = 0.0;
	double normal_radius_s = 0.0;
	double radius_feature_m_1 = 0.0;//特征的计算半径
	double radius_feature_m_2 = 0.0;
	double radius_feature_s = 0.0;
	double beta_threshold = 0.0;
	size_t alpha_devide = 20;//每条PR曲线的点数
	std::cout << "num_key_m: " << num_key_m_1 << std::endl;
	std::cout << "num_key_s: " << num_key_s << std::endl;

	//读取配置文件
	std::string scene_path;
	std::vector<std::string> model_path;
	std::vector<std::string> model_matrix_path;
	FileReader fr;
	fr.readINI(dataset_name_, ini_path_, scene_path, model_path, model_matrix_path);
	std::cout << "---------------------------- Read INI ----------------------------" << std::endl;
	std::cout << "dataset_name_: " << dataset_name_ << std::endl;
	std::cout << "ini_path_: " << ini_path_ << std::endl;
	std::cout << "scene_path: " << scene_path << std::endl;
	std::cout << "model_path[0]: " << model_path[0] << std::endl;
	std::cout << "model_path[1]: " << model_path[1] << std::endl;
	std::cout << "model_matrix_path[0]: " << model_matrix_path[0] << std::endl;
	std::cout << "model_matrix_path[1]: " << model_matrix_path[1] << std::endl;

	//读取变换矩阵
	Eigen::Matrix4f ground_truth_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f ground_truth_2 = Eigen::Matrix4f::Identity();
	fr.readGroundTruth(model_matrix_path[0], ground_truth_1);
	fr.readGroundTruth(model_matrix_path[1], ground_truth_2);
	std::cout << "------------------------ Load GroundTruth ------------------------" << std::endl;
	std::cout << "ground_truth_1: " << ground_truth_1 << std::endl;
	std::cout << "ground_truth_2: " << ground_truth_2 << std::endl;

	//读取源点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile(scene_path, *cloud_scene);
	pcl::io::loadPLYFile(model_path[0], *cloud_model_1);
	pcl::io::loadPLYFile(model_path[1], *cloud_model_2);
	std::cout << "---------------------------- Load PLY ----------------------------" << std::endl;
	std::cout << "cloud_scene: " << cloud_scene->size() << std::endl;
	std::cout << "cloud_model_1: " << cloud_model_1->size() << std::endl;
	std::cout << "cloud_model_2: " << cloud_model_2->size() << std::endl;

	//--------------------------------匹配实验--------------------------------//
	//1.计算点云的网格分辨率
	ToolBox tb;
	mr_s = tb.computeMeshResolution(cloud_scene);
	mr_1 = tb.computeMeshResolution(cloud_model_1);
	mr_2 = tb.computeMeshResolution(cloud_model_2);
	double res_scene = tb.computeCloudResolution(cloud_scene);
	std::cout << "------------------- Calculate Mesh Resolution --------------------" << std::endl;
	std::cout << "mr_s: " << mr_s << std::endl;
	std::cout << "mr_1: " << mr_1 << std::endl;
	std::cout << "mr_2: " << mr_2 << std::endl;
	std::cout << "scene density: " << res_scene << std::endl;

	// !!!------------TESTING------------!!!!!!!!!!
	//设置参数(与分辨率相关)
	normal_radius_s = 4 * mr_s;//法线的计算半径
	normal_radius_m_1 = 4 * mr_1;
	normal_radius_m_2 = 4 * mr_1;
	radius_feature_m_1 = 30 * mr_s;//特征的计算半径//10,15,20//4,6,8,10
	radius_feature_m_2 = 30 * mr_s;
	radius_feature_s = 30 * mr_s;
	beta_threshold = 10 * res_scene;//距离阈值
	std::cout << "--------------------- Initialize Parameters ----------------------" << std::endl;
	std::cout << "normal_radius_s : " << normal_radius_s << std::endl;
	std::cout << "normal_radius_m_1 = : " << normal_radius_m_1 << std::endl;
	std::cout << "normal_radius_m_2 = : " << normal_radius_m_2 << std::endl;
	std::cout << "radius_feature_s = : " << radius_feature_s << std::endl;
	std::cout << "radius_feature_m_1 = : " << radius_feature_m_1 << std::endl;
	std::cout << "radius_feature_m_2 = : " << radius_feature_m_2 << std::endl;
	std::cout << "beta_threshold = 10 * res_scene: " << beta_threshold << std::endl;

#ifdef CSHOT_KEY
	//3.提取关键点（随机采样）（使用变换矩阵，含有真实对应，与CSHOT论文相同）
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s(new pcl::PointCloud<pcl::PointXYZRGB>);
	KeypointsDetector key_detector;
	key_detector.compute(cloud_model_1, cloud_scene, keypoints_m_1, keypoints_s_1,  ground_truth_1, normal_radius_m_1, num_key_m_1);
	key_detector.compute(cloud_model_2, cloud_scene, keypoints_m_2, keypoints_s_2,  ground_truth_2, normal_radius_m_2, num_key_m_2);
	*keypoints_m += *keypoints_m_1;//keypoints_m内容[1000关键点(一)，1000关键点(二)]
	*keypoints_m += *keypoints_m_2;
	*keypoints_s += *keypoints_s_1;//keypoints_s内容[<=1000关键点(对应一)，<=1000关键点(对应二)，>=1000干扰点]
	*keypoints_s += *keypoints_s_2;
	//int noi = num_key_s - (keypoints_s->size());
	key_detector.addNoisePoints(cloud_scene, keypoints_s, noi);//!注意：虽然真实对应数量在变化，但几种描述子在一次场景实验中使用的关键点是完全相同的，结果依然是公平的。
	std::cout << "----------------------- Sampling Keypoints Test -----------------------" << std::endl;
	std::cout << "keypoints_s corr_regions/noise : " <<keypoints_s->size()- noi <<"/" << noi << std::endl;
	std::cout << "keypoints_m size: " << keypoints_m->size() << std::endl;
	std::cout << "keypoints_s size: " << keypoints_s->size() << std::endl;
#endif // CSHOT_KEY

#ifdef VOXEL_KEY
	//3.提取关键点（体素下采样）（使用变换矩阵，含有真实对应，与四篇论文之一相同，为了避免实验中由“近邻比值匹配”和“不同大小模型使用相同数量关键点”导致的曲线波动）
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s(new pcl::PointCloud<pcl::PointXYZRGB>);
	KeypointsDetector key_detector;
	key_detector.compute_v(cloud_model_1, cloud_scene, keypoints_m_1, keypoints_s_1, ground_truth_1, normal_radius_m_1, 0.7f);
	key_detector.compute_v(cloud_model_2, cloud_scene, keypoints_m_2, keypoints_s_2, ground_truth_2, normal_radius_m_2, 0.7f);
	*keypoints_m += *keypoints_m_1;//keypoints_m内容[模型下采样关键点(一)，模型下采样关键点(二)]
	*keypoints_m += *keypoints_m_2;
	*keypoints_s += *keypoints_s_1;//keypoints_s内容[场景关键点(对应模型一)，场景关键点(对应模型二)，干扰点]
	*keypoints_s += *keypoints_s_2;
	int noi = ((keypoints_s->size()) / 2);
	key_detector.addNoisePoints(cloud_scene, keypoints_s, noi);//!注意：虽然真实对应数量在变化，但几种描述子在一次场景实验中使用的关键点是完全相同的，结果依然是公平的。
	std::cout << "----------------------- Sampling Keypoints Test -----------------------" << std::endl;
	std::cout << "keypoints_m_1 size: " << keypoints_m_1->size() << std::endl;//2800
	std::cout << "keypoints_m_2 size: " << keypoints_m_2->size() << std::endl;//1200
	std::cout << "keypoints_m size: " << keypoints_m->size() << std::endl;
	//std::cout << "corr_regions/noise in keypoints_s : " <<keypoints_s->size()- noi <<"/" << noi << std::endl;
	std::cout << "keypoints_s size: " << keypoints_s->size() << std::endl;
#endif // VOXEL_KEY

	//2.计算法线
	MyNormalEstimation ne;
	pcl::PointCloud<pcl::Normal>::Ptr normal_m_1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_m_2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_s(new pcl::PointCloud<pcl::Normal>);
	/*ne.computeNormal(cloud_model_1, normal_radius_m_1, normal_m_1);
	ne.computeNormal(cloud_model_2, normal_radius_m_2, normal_m_2);
	ne.computeNormal(cloud_scene, normal_radius_s, normal_s);*/
	ne.computeNormal_K(cloud_model_1, 12, normal_m_1);
	ne.computeNormal_K(cloud_model_2, 12, normal_m_2);
	ne.computeNormal_K(cloud_scene, 12, normal_s);
	std::cout << "----------------------- Calculate Normals ------------------------" << std::endl;
	std::cout << "normal_m_1 size: " << normal_m_1->size() << std::endl;
	std::cout << "normal_m_2 size: " << normal_m_2->size() << std::endl;
	std::cout << "normal_s size: " << normal_s->size() << std::endl;

	////3.提取关键点（随机采样）（不使用变换矩阵）
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s(new pcl::PointCloud<pcl::PointXYZRGB>);
	//KeypointsDetector key_detector;
	//key_detector.computeRandomSampleKeypoints(cloud_model_1, keypoints_m_1, num_key_m_1);
	//key_detector.computeRandomSampleKeypoints(cloud_model_2, keypoints_m_2, num_key_m_2);
	//key_detector.computeRandomSampleKeypoints(cloud_scene, keypoints_s, num_key_s);
	//std::cout << "----------------------- Sampling Keypoints -----------------------" << std::endl;
	//std::cout << "keypoints_m_1 size: " << keypoints_m_1->size() << std::endl;
	//std::cout << "keypoints_m_2 size: " << keypoints_m_2->size() << std::endl;
	//std::cout << "keypoints_s size: " << keypoints_s->size() << std::endl;
	///* 拼接关键点云，拼接PFH特征，这是为了保持特征索引和关键点索引的一致。
	//* 拼接后，关键点索引=特征索引，这样可以由特征索引间接获取关键点坐标。 */
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m(new pcl::PointCloud<pcl::PointXYZRGB>);//拼接的关键点云
	//*keypoints_m += *keypoints_m_1;
	//*keypoints_m += *keypoints_m_2;

#ifdef PFH
	//4.计算特征的PR曲线（PFH）
	//计算特征（PFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_pfh;
	ex_pfh.computePFH(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, pfh_m_1);
	ex_pfh.computePFH(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, pfh_m_2);
	ex_pfh.computePFH(cloud_scene, keypoints_s, normal_s, radius_feature_s, pfh_s);
	//拼接特征（PFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*pfh_m += *pfh_m_1;
	*pfh_m += *pfh_m_2;
	//计算PR曲线
	//将阈值等分为15份，分别计算PR值
	features_name.push_back("PFH");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(pfh_m, pfh_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		//all_correspondences: corresponding regions between two scan(model+scene) of the same scene.<scene_index,model_index,distence>
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_pfh_[i] += correct_matches->size();
		matches_sum_pfh_[i] += corrs_match->size();
		corresponding_regions_sum_pfh_[i] += all_correspondences->size();
		std::cout << "------------------------------- PFH --------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // PFH

#ifdef FPFH
	//5.计算特征的PR曲线（FPFH）
	//计算特征（FPFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_fpfh;
	ex_fpfh.computeFPFH(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1/2.0, fpfh_m_1);
	ex_fpfh.computeFPFH(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, fpfh_m_2);
	ex_fpfh.computeFPFH(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, fpfh_s);
	std::cout << "----------------------- Calculate FPFH Features ------------------------" << std::endl;
	std::cout << "feature_m_1 size: " << fpfh_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << fpfh_m_2->size() << std::endl;
	std::cout << "feature_s size: " << fpfh_s->size() << std::endl;
	//拼接特征（FPFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*fpfh_m += *fpfh_m_1;
	*fpfh_m += *fpfh_m_2;
	//计算PR曲线
	features_name.push_back("FPFH");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(fpfh_m, fpfh_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_[i] += correct_matches->size();
		matches_sum_fpfh_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_[i]:   " << correct_matches_sum_fpfh_[i] << std::endl;
		std::cout << "matches_sum_fpfh_[i]:   " << matches_sum_fpfh_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_[i]:   " << corresponding_regions_sum_fpfh_[i] << std::endl;
		std::cout << "------------------------------- FPFH -------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH

#ifdef CFH_RGB_PFH_RATE
	//5.计算特征的PR曲线（cfh_rgb_pfh_rate）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh;
	ex_cfh.computeCFH_RGB_PFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh_m_1);
	ex_cfh.computeCFH_RGB_PFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh_m_2);
	ex_cfh.computeCFH_RGB_PFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh_s);
	std::cout << "----------------------- Calculate CFH_RGB_PFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh_m += *cfh_m_1;
	*cfh_m += *cfh_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_PFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh_m, cfh_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_pfh_rate_[i] += correct_matches->size();
		matches_sum_cfh_rgb_pfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_pfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_pfh_rate_[i]:   " << correct_matches_sum_cfh_rgb_pfh_rate_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_pfh_rate_[i]:   " << matches_sum_cfh_rgb_pfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_pfh_rate_[i]:   " << corresponding_regions_sum_cfh_rgb_pfh_rate_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_PFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_PFH_RATE

#ifdef CFH_RGB_PFH_DOT
	//5.计算特征的PR曲线（cfh_rgb_pfh_dot）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh001_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh001_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh001_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh001;
	ex_cfh001.computeCFH_RGB_PFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh001_m_1);
	ex_cfh001.computeCFH_RGB_PFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh001_m_2);
	ex_cfh001.computeCFH_RGB_PFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh001_s);
	std::cout << "----------------------- Calculate CFH_RGB_PFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh001_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh001_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh001_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh001_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh001_m += *cfh001_m_1;
	*cfh001_m += *cfh001_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_PFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh001_m, cfh001_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_pfh_dot_[i] += correct_matches->size();
		matches_sum_cfh_rgb_pfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_pfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_pfh_dot_[i]:   " << correct_matches_sum_cfh_rgb_pfh_dot_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_pfh_dot_[i]:   " << matches_sum_cfh_rgb_pfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_pfh_dot_[i]:   " << corresponding_regions_sum_cfh_rgb_pfh_dot_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_PFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_PFH_DOT

//--------------------C---------------------------

#ifdef CFH_RGB_FPFH_RATE
	//5.计算特征的PR曲线（cfh_rgb_fpfh_rate）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh002_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh002_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh002_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh002;
	ex_cfh002.computeCFH_RGB_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh002_m_1);
	ex_cfh002.computeCFH_RGB_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh002_m_2);
	ex_cfh002.computeCFH_RGB_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh002_s);
	std::cout << "----------------------- Calculate CFH_RGB_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh002_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh002_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh002_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh002_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh002_m += *cfh002_m_1;
	*cfh002_m += *cfh002_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh002_m, cfh002_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_fpfh_rate_[i] += correct_matches->size();
		matches_sum_cfh_rgb_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_fpfh_rate_[i]:   " << correct_matches_sum_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_fpfh_rate_[i]:   " << matches_sum_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_fpfh_rate_[i]:   " << corresponding_regions_sum_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_FPFH_RATE

#ifdef CFH_RGB_FPFH_DOT
	//5.计算特征的PR曲线（cfh_rgb_fpfh_dot）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh003_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh003_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh003_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh003;
	ex_cfh003.computeCFH_RGB_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh003_m_1);
	ex_cfh003.computeCFH_RGB_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh003_m_2);
	ex_cfh003.computeCFH_RGB_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh003_s);
	std::cout << "----------------------- Calculate CFH_RGB_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh003_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh003_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh003_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh003_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh003_m += *cfh003_m_1;
	*cfh003_m += *cfh003_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh003_m, cfh003_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_fpfh_dot_[i] += correct_matches->size();
		matches_sum_cfh_rgb_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_fpfh_dot_[i]:   " << correct_matches_sum_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_fpfh_dot_[i]:   " << matches_sum_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_fpfh_dot_[i]:   " << corresponding_regions_sum_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_FPFH_DOT

#ifdef CFH_RGB_FPFH_L1
	//5.计算特征的PR曲线（cfh_rgb_fpfh_L1）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh004_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh004_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh004_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh004;
	ex_cfh004.computeCFH_RGB_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh004_m_1);
	ex_cfh004.computeCFH_RGB_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh004_m_2);
	ex_cfh004.computeCFH_RGB_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh004_s);
	std::cout << "----------------------- Calculate CFH_RGB_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh004_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh004_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh004_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh004_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh004_m += *cfh004_m_1;
	*cfh004_m += *cfh004_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh004_m, cfh004_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_fpfh_L1_[i] += correct_matches->size();
		matches_sum_cfh_rgb_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_fpfh_L1_[i]:   " << correct_matches_sum_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_fpfh_L1_[i]:   " << matches_sum_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_fpfh_L1_[i]:   " << corresponding_regions_sum_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_FPFH_L1

#ifdef CFH_RGB_FPFH_L2
	//5.计算特征的PR曲线（cfh_rgb_fpfh_L2）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh005_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh005_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh005_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh005;
	ex_cfh005.computeCFH_RGB_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh005_m_1);
	ex_cfh005.computeCFH_RGB_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh005_m_2);
	ex_cfh005.computeCFH_RGB_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh005_s);
	std::cout << "----------------------- Calculate CFH_RGB_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh005_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh005_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh005_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh005_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh005_m += *cfh005_m_1;
	*cfh005_m += *cfh005_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh005_m, cfh005_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_fpfh_L2_[i] += correct_matches->size();
		matches_sum_cfh_rgb_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_fpfh_L2_[i]:   " << correct_matches_sum_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_fpfh_L2_[i]:   " << matches_sum_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_fpfh_L2_[i]:   " << corresponding_regions_sum_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_FPFH_L2

#ifdef CFH_RGB_FPFH_NEW
	//5.计算特征的PR曲线（cfh_rgb_fpfh_new）
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cfh005_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cfh005_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cfh005_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_cfh005;
	ex_cfh005.computeCFH_RGB_FPFH_NEW(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh005_m_1);
	ex_cfh005.computeCFH_RGB_FPFH_NEW(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh005_m_2);
	ex_cfh005.computeCFH_RGB_FPFH_NEW(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh005_s);
	std::cout << "----------------------- Calculate CFH_RGB_FPFH_NEW Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh005_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh005_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh005_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cfh005_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*cfh005_m += *cfh005_m_1;
	*cfh005_m += *cfh005_m_2;
	//计算PR曲线
	features_name.push_back("CFH_RGB_FPFH_NEW");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh005_m, cfh005_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_rgb_fpfh_new_[i] += correct_matches->size();
		matches_sum_cfh_rgb_fpfh_new_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_rgb_fpfh_new_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_rgb_fpfh_new_[i]:   " << correct_matches_sum_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "matches_sum_cfh_rgb_fpfh_new_[i]:   " << matches_sum_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_rgb_fpfh_new_[i]:   " << corresponding_regions_sum_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "------------------------------- CFH_RGB_FPFH_NEW ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_RGB_FPFH_NEW

#ifdef CFH_HSV_FPFH_RATE
	//5.计算特征的PR曲线（cfh_hsv_fpfh_rate）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh006_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh006_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh006_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh006;
	ex_cfh006.computeCFH_HSV_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh006_m_1);
	ex_cfh006.computeCFH_HSV_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh006_m_2);
	ex_cfh006.computeCFH_HSV_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh006_s);
	std::cout << "----------------------- Calculate CFH_HSV_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh006_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh006_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh006_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh006_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh006_m += *cfh006_m_1;
	*cfh006_m += *cfh006_m_2;
	//计算PR曲线
	features_name.push_back("CFH_HSV_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh006_m, cfh006_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_hsv_fpfh_rate_[i] += correct_matches->size();
		matches_sum_cfh_hsv_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_hsv_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_hsv_fpfh_rate_[i]:   " << correct_matches_sum_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_cfh_hsv_fpfh_rate_[i]:   " << matches_sum_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_hsv_fpfh_rate_[i]:   " << corresponding_regions_sum_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- CFH_HSV_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_HSV_FPFH_RATE

#ifdef CFH_HSV_FPFH_DOT
	//5.计算特征的PR曲线（cfh_hsv_fpfh_dot）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh007_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh007_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh007_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh007;
	ex_cfh007.computeCFH_HSV_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh007_m_1);
	ex_cfh007.computeCFH_HSV_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh007_m_2);
	ex_cfh007.computeCFH_HSV_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh007_s);
	std::cout << "----------------------- Calculate CFH_HSV_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh007_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh007_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh007_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh007_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh007_m += *cfh007_m_1;
	*cfh007_m += *cfh007_m_2;
	//计算PR曲线
	features_name.push_back("CFH_HSV_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh007_m, cfh007_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_hsv_fpfh_dot_[i] += correct_matches->size();
		matches_sum_cfh_hsv_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_hsv_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_hsv_fpfh_dot_[i]:   " << correct_matches_sum_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_cfh_hsv_fpfh_dot_[i]:   " << matches_sum_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_hsv_fpfh_dot_[i]:   " << corresponding_regions_sum_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- CFH_HSV_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_HSV_FPFH_DOT

#ifdef CFH_HSV_FPFH_L1
	//5.计算特征的PR曲线（cfh_hsv_fpfh_L1）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh008_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh008_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh008_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh008;
	ex_cfh008.computeCFH_HSV_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh008_m_1);
	ex_cfh008.computeCFH_HSV_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh008_m_2);
	ex_cfh008.computeCFH_HSV_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh008_s);
	std::cout << "----------------------- Calculate CFH_HSV_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh008_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh008_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh008_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh008_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh008_m += *cfh008_m_1;
	*cfh008_m += *cfh008_m_2;
	//计算PR曲线
	features_name.push_back("CFH_HSV_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh008_m, cfh008_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_hsv_fpfh_L1_[i] += correct_matches->size();
		matches_sum_cfh_hsv_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_hsv_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_hsv_fpfh_L1_[i]:   " << correct_matches_sum_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_cfh_hsv_fpfh_L1_[i]:   " << matches_sum_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_hsv_fpfh_L1_[i]:   " << corresponding_regions_sum_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- CFH_HSV_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_HSV_FPFH_L1

#ifdef CFH_HSV_FPFH_L2
	//5.计算特征的PR曲线（cfh_hsv_fpfh_L2）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh009_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh009_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh009_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh009;
	ex_cfh009.computeCFH_HSV_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh009_m_1);
	ex_cfh009.computeCFH_HSV_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh009_m_2);
	ex_cfh009.computeCFH_HSV_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh009_s);
	std::cout << "----------------------- Calculate CFH_HSV_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh009_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh009_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh009_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh009_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh009_m += *cfh009_m_1;
	*cfh009_m += *cfh009_m_2;
	//计算PR曲线
	features_name.push_back("CFH_HSV_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh009_m, cfh009_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_hsv_fpfh_L2_[i] += correct_matches->size();
		matches_sum_cfh_hsv_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_hsv_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_hsv_fpfh_L2_[i]:   " << correct_matches_sum_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_cfh_hsv_fpfh_L2_[i]:   " << matches_sum_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_hsv_fpfh_L2_[i]:   " << corresponding_regions_sum_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- CFH_HSV_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_HSV_FPFH_L2

#ifdef CFH_LAB_FPFH_RATE
	//5.计算特征的PR曲线（cfh_lab_fpfh_rate）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh010_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh010_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh010_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh010;
	ex_cfh010.computeCFH_LAB_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh010_m_1);
	ex_cfh010.computeCFH_LAB_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh010_m_2);
	ex_cfh010.computeCFH_LAB_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh010_s);
	std::cout << "----------------------- Calculate CFH_LAB_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh010_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh010_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh010_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh010_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh010_m += *cfh010_m_1;
	*cfh010_m += *cfh010_m_2;
	//计算PR曲线
	features_name.push_back("CFH_LAB_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh010_m, cfh010_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_lab_fpfh_rate_[i] += correct_matches->size();
		matches_sum_cfh_lab_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_lab_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_lab_fpfh_rate_[i]:   " << correct_matches_sum_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_cfh_lab_fpfh_rate_[i]:   " << matches_sum_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_lab_fpfh_rate_[i]:   " << corresponding_regions_sum_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- CFH_LAB_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_LAB_FPFH_RATE

#ifdef CFH_LAB_FPFH_DOT
	//5.计算特征的PR曲线（cfh_lab_fpfh_dot）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh011_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh011_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh011_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh011;
	ex_cfh011.computeCFH_LAB_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh011_m_1);
	ex_cfh011.computeCFH_LAB_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh011_m_2);
	ex_cfh011.computeCFH_LAB_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh011_s);
	std::cout << "----------------------- Calculate CFH_LAB_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh011_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh011_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh011_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh011_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh011_m += *cfh011_m_1;
	*cfh011_m += *cfh011_m_2;
	//计算PR曲线
	features_name.push_back("CFH_LAB_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh011_m, cfh011_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_lab_fpfh_dot_[i] += correct_matches->size();
		matches_sum_cfh_lab_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_lab_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_lab_fpfh_dot_[i]:   " << correct_matches_sum_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_cfh_lab_fpfh_dot_[i]:   " << matches_sum_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_lab_fpfh_dot_[i]:   " << corresponding_regions_sum_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- CFH_LAB_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_LAB_FPFH_DOT

#ifdef CFH_LAB_FPFH_L1
	//5.计算特征的PR曲线（cfh_lab_fpfh_L1）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh012_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh012_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh012_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh012;
	ex_cfh012.computeCFH_LAB_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh012_m_1);
	ex_cfh012.computeCFH_LAB_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh012_m_2);
	ex_cfh012.computeCFH_LAB_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh012_s);
	std::cout << "----------------------- Calculate CFH_LAB_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh012_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh012_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh012_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh012_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh012_m += *cfh012_m_1;
	*cfh012_m += *cfh012_m_2;
	//计算PR曲线
	features_name.push_back("CFH_LAB_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh012_m, cfh012_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_lab_fpfh_L1_[i] += correct_matches->size();
		matches_sum_cfh_lab_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_lab_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_lab_fpfh_L1_[i]:   " << correct_matches_sum_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_cfh_lab_fpfh_L1_[i]:   " << matches_sum_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_lab_fpfh_L1_[i]:   " << corresponding_regions_sum_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- CFH_LAB_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_LAB_FPFH_L1

#ifdef CFH_LAB_FPFH_L2
	//5.计算特征的PR曲线（cfh_lab_fpfh_L2）
	//计算特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh013_m_1(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh013_m_2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh013_s(new pcl::PointCloud<pcl::FPFHSignature33>());
	FeatureExtractor ex_cfh013;
	ex_cfh013.computeCFH_LAB_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cfh013_m_1);
	ex_cfh013.computeCFH_LAB_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cfh013_m_2);
	ex_cfh013.computeCFH_LAB_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, cfh013_s);
	std::cout << "----------------------- Calculate CFH_LAB_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << cfh013_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << cfh013_m_2->size() << std::endl;
	std::cout << "feature_s size: " << cfh013_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cfh013_m(new pcl::PointCloud<pcl::FPFHSignature33>());
	*cfh013_m += *cfh013_m_1;
	*cfh013_m += *cfh013_m_2;
	//计算PR曲线
	features_name.push_back("CFH_LAB_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cfh013_m, cfh013_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cfh_lab_fpfh_L2_[i] += correct_matches->size();
		matches_sum_cfh_lab_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_cfh_lab_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_cfh_lab_fpfh_L2_[i]:   " << correct_matches_sum_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_cfh_lab_fpfh_L2_[i]:   " << matches_sum_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_cfh_lab_fpfh_L2_[i]:   " << corresponding_regions_sum_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- CFH_LAB_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CFH_LAB_FPFH_L2

//----------------------S+C-----------------------

#ifdef FPFH_CFH_RGB_FPFH_RATE
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc002_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc002_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc002_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc002;
	ex_sc002.computeFPFH_CFH_RGB_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1/2.0, sc002_m_1);
	ex_sc002.computeFPFH_CFH_RGB_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, sc002_m_2);
	ex_sc002.computeFPFH_CFH_RGB_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, sc002_s);
	std::cout << "----------------------- Calculate FPFH_CFH_RGB_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc002_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc002_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc002_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc002_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc002_m += *sc002_m_1;
	*sc002_m += *sc002_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_RGB_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc002_m, sc002_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_rgb_fpfh_rate_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_rgb_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_rgb_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_rgb_fpfh_rate_[i]:   " << correct_matches_sum_fpfh_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_rgb_fpfh_rate_[i]:   " << matches_sum_fpfh_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_rgb_fpfh_rate_[i]:   " << corresponding_regions_sum_fpfh_cfh_rgb_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_RGB_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_RGB_FPFH_RATE

#ifdef FPFH_CFH_RGB_FPFH_DOT
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc001_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc001_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc001_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc001;
	ex_sc001.computeFPFH_CFH_RGB_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc001_m_1);
	ex_sc001.computeFPFH_CFH_RGB_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc001_m_2);
	ex_sc001.computeFPFH_CFH_RGB_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc001_s);
	std::cout << "----------------------- Calculate FPFH_CFH_RGB_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc001_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc001_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc001_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc001_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc001_m += *sc001_m_1;
	*sc001_m += *sc001_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_RGB_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc001_m, sc001_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_rgb_fpfh_dot_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_rgb_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_rgb_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_rgb_fpfh_dot_[i]:   " << correct_matches_sum_fpfh_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_rgb_fpfh_dot_[i]:   " << matches_sum_fpfh_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_rgb_fpfh_dot_[i]:   " << corresponding_regions_sum_fpfh_cfh_rgb_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_RGB_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_RGB_FPFH_DOT

#ifdef FPFH_CFH_RGB_FPFH_L1
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc003_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc003_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc003_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc003;
	ex_sc003.computeFPFH_CFH_RGB_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc003_m_1);
	ex_sc003.computeFPFH_CFH_RGB_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc003_m_2);
	ex_sc003.computeFPFH_CFH_RGB_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc003_s);
	std::cout << "----------------------- Calculate FPFH_CFH_RGB_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc003_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc003_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc003_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc003_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc003_m += *sc003_m_1;
	*sc003_m += *sc003_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_RGB_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc003_m, sc003_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_rgb_fpfh_L1_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_rgb_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_rgb_fpfh_L1_[i]:   " << correct_matches_sum_fpfh_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_rgb_fpfh_L1_[i]:   " << matches_sum_fpfh_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L1_[i]:   " << corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_RGB_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_RGB_FPFH_L1

#ifdef FPFH_CFH_RGB_FPFH_L2
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc004;
	ex_sc004.computeFPFH_CFH_RGB_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc004_m_1);
	ex_sc004.computeFPFH_CFH_RGB_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc004_m_2);
	ex_sc004.computeFPFH_CFH_RGB_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc004_s);
	std::cout << "----------------------- Calculate FPFH_CFH_RGB_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc004_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc004_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc004_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc004_m += *sc004_m_1;
	*sc004_m += *sc004_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_RGB_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc004_m, sc004_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_rgb_fpfh_L2_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_rgb_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_rgb_fpfh_L2_[i]:   " << correct_matches_sum_fpfh_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_rgb_fpfh_L2_[i]:   " << matches_sum_fpfh_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L2_[i]:   " << corresponding_regions_sum_fpfh_cfh_rgb_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_RGB_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_RGB_FPFH_L2

#ifdef FPFH_CFH_RGB_FPFH_NEW
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc004;
	ex_sc004.computeFPFH_CFH_RGB_FPFH_NEW(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc004_m_1);
	ex_sc004.computeFPFH_CFH_RGB_FPFH_NEW(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc004_m_2);
	ex_sc004.computeFPFH_CFH_RGB_FPFH_NEW(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc004_s);
	std::cout << "----------------------- Calculate FPFH_CFH_RGB_FPFH_NEW Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc004_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc004_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc004_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc004_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc004_m += *sc004_m_1;
	*sc004_m += *sc004_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_RGB_FPFH_NEW");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc004_m, sc004_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_rgb_fpfh_new_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_rgb_fpfh_new_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_rgb_fpfh_new_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_rgb_fpfh_new_[i]:   " << correct_matches_sum_fpfh_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_rgb_fpfh_new_[i]:   " << matches_sum_fpfh_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_rgb_fpfh_new_[i]:   " << corresponding_regions_sum_fpfh_cfh_rgb_fpfh_new_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_RGB_FPFH_NEW ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_RGB_FPFH_NEW

#ifdef FPFH_CFH_HSV_FPFH_RATE
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc005_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc005_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc005_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc005;
	ex_sc005.computeFPFH_CFH_HSV_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1 / 2.0, sc005_m_1);
	ex_sc005.computeFPFH_CFH_HSV_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, sc005_m_2);
	ex_sc005.computeFPFH_CFH_HSV_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, sc005_s);
	std::cout << "----------------------- Calculate FPFH_CFH_HSV_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc005_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc005_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc005_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc005_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc005_m += *sc005_m_1;
	*sc005_m += *sc005_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_HSV_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc005_m, sc005_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_hsv_fpfh_rate_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_hsv_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_hsv_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_hsv_fpfh_rate_[i]:   " << correct_matches_sum_fpfh_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_hsv_fpfh_rate_[i]:   " << matches_sum_fpfh_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_hsv_fpfh_rate_[i]:   " << corresponding_regions_sum_fpfh_cfh_hsv_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_HSV_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_HSV_FPFH_RATE

#ifdef FPFH_CFH_HSV_FPFH_DOT
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc006_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc006_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc006_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc006;
	ex_sc006.computeFPFH_CFH_HSV_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc006_m_1);
	ex_sc006.computeFPFH_CFH_HSV_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc006_m_2);
	ex_sc006.computeFPFH_CFH_HSV_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc006_s);
	std::cout << "----------------------- Calculate FPFH_CFH_HSV_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc006_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc006_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc006_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc006_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc006_m += *sc006_m_1;
	*sc006_m += *sc006_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_HSV_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc006_m, sc006_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_hsv_fpfh_dot_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_hsv_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_hsv_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_hsv_fpfh_dot_[i]:   " << correct_matches_sum_fpfh_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_hsv_fpfh_dot_[i]:   " << matches_sum_fpfh_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_hsv_fpfh_dot_[i]:   " << corresponding_regions_sum_fpfh_cfh_hsv_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_HSV_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_HSV_FPFH_DOT

#ifdef FPFH_CFH_HSV_FPFH_L1
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc007_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc007_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc007_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc007;
	ex_sc007.computeFPFH_CFH_HSV_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc007_m_1);
	ex_sc007.computeFPFH_CFH_HSV_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc007_m_2);
	ex_sc007.computeFPFH_CFH_HSV_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc007_s);
	std::cout << "----------------------- Calculate FPFH_CFH_HSV_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc007_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc007_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc007_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc007_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc007_m += *sc007_m_1;
	*sc007_m += *sc007_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_HSV_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc007_m, sc007_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_hsv_fpfh_L1_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_hsv_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_hsv_fpfh_L1_[i]:   " << correct_matches_sum_fpfh_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_hsv_fpfh_L1_[i]:   " << matches_sum_fpfh_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L1_[i]:   " << corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_HSV_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_HSV_FPFH_L1

#ifdef FPFH_CFH_HSV_FPFH_L2
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc008_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc008_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc008_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc008;
	ex_sc008.computeFPFH_CFH_HSV_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc008_m_1);
	ex_sc008.computeFPFH_CFH_HSV_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc008_m_2);
	ex_sc008.computeFPFH_CFH_HSV_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc008_s);
	std::cout << "----------------------- Calculate FPFH_CFH_HSV_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc008_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc008_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc008_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc008_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc008_m += *sc008_m_1;
	*sc008_m += *sc008_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_HSV_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc008_m, sc008_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_hsv_fpfh_L2_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_hsv_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_hsv_fpfh_L2_[i]:   " << correct_matches_sum_fpfh_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_hsv_fpfh_L2_[i]:   " << matches_sum_fpfh_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L2_[i]:   " << corresponding_regions_sum_fpfh_cfh_hsv_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_HSV_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_HSV_FPFH_L2

#ifdef FPFH_CFH_LAB_FPFH_RATE
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc009_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc009_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc009_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc009;
	ex_sc009.computeFPFH_CFH_LAB_FPFH_RATE(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1 / 2.0, sc009_m_1);
	ex_sc009.computeFPFH_CFH_LAB_FPFH_RATE(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, sc009_m_2);
	ex_sc009.computeFPFH_CFH_LAB_FPFH_RATE(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, sc009_s);
	std::cout << "----------------------- Calculate FPFH_CFH_LAB_FPFH_RATE Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc009_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc009_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc009_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc009_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc009_m += *sc009_m_1;
	*sc009_m += *sc009_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_LAB_FPFH_RATE");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc009_m, sc009_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_lab_fpfh_rate_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_lab_fpfh_rate_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_lab_fpfh_rate_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_lab_fpfh_rate_[i]:   " << correct_matches_sum_fpfh_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_lab_fpfh_rate_[i]:   " << matches_sum_fpfh_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_lab_fpfh_rate_[i]:   " << corresponding_regions_sum_fpfh_cfh_lab_fpfh_rate_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_LAB_FPFH_RATE ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_LAB_FPFH_RATE

#ifdef FPFH_CFH_LAB_FPFH_DOT
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc010_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc010_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc010_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc010;
	ex_sc010.computeFPFH_CFH_LAB_FPFH_DOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1/2.0, sc010_m_1);
	ex_sc010.computeFPFH_CFH_LAB_FPFH_DOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, sc010_m_2);
	ex_sc010.computeFPFH_CFH_LAB_FPFH_DOT(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, sc010_s);
	std::cout << "----------------------- Calculate FPFH_CFH_LAB_FPFH_DOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc010_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc010_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc010_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc010_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc010_m += *sc010_m_1;
	*sc010_m += *sc010_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_LAB_FPFH_DOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc010_m, sc010_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_lab_fpfh_dot_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_lab_fpfh_dot_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_lab_fpfh_dot_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_lab_fpfh_dot_[i]:   " << correct_matches_sum_fpfh_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_lab_fpfh_dot_[i]:   " << matches_sum_fpfh_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_lab_fpfh_dot_[i]:   " << corresponding_regions_sum_fpfh_cfh_lab_fpfh_dot_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_LAB_FPFH_DOT ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_LAB_FPFH_DOT

#ifdef FPFH_CFH_LAB_FPFH_L1
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc011_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc011_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc011_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc011;
	ex_sc011.computeFPFH_CFH_LAB_FPFH_L1(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1 / 2.0, sc011_m_1);
	ex_sc011.computeFPFH_CFH_LAB_FPFH_L1(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2 / 2.0, sc011_m_2);
	ex_sc011.computeFPFH_CFH_LAB_FPFH_L1(cloud_scene, keypoints_s, normal_s, radius_feature_s / 2.0, sc011_s);
	std::cout << "----------------------- Calculate FPFH_CFH_LAB_FPFH_L1 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc011_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc011_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc011_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc011_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc011_m += *sc011_m_1;
	*sc011_m += *sc011_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_LAB_FPFH_L1");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc011_m, sc011_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_lab_fpfh_L1_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_lab_fpfh_L1_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_lab_fpfh_L1_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_lab_fpfh_L1_[i]:   " << correct_matches_sum_fpfh_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_lab_fpfh_L1_[i]:   " << matches_sum_fpfh_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_lab_fpfh_L1_[i]:   " << corresponding_regions_sum_fpfh_cfh_lab_fpfh_L1_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_LAB_FPFH_L1 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_LAB_FPFH_L1

#ifdef FPFH_CFH_LAB_FPFH_L2
	//计算特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc012_m_1(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc012_m_2(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc012_s(new pcl::PointCloud<pcl::PFHSignature125>());
	FeatureExtractor ex_sc012;
	ex_sc012.computeFPFH_CFH_LAB_FPFH_L2(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, sc012_m_1);
	ex_sc012.computeFPFH_CFH_LAB_FPFH_L2(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, sc012_m_2);
	ex_sc012.computeFPFH_CFH_LAB_FPFH_L2(cloud_scene, keypoints_s, normal_s, radius_feature_s, sc012_s);
	std::cout << "----------------------- Calculate FPFH_CFH_LAB_FPFH_L2 Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << sc012_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << sc012_m_2->size() << std::endl;
	std::cout << "feature_s size: " << sc012_s->size() << std::endl;
	//拼接特征（CFH）
	pcl::PointCloud<pcl::PFHSignature125>::Ptr sc012_m(new pcl::PointCloud<pcl::PFHSignature125>());
	*sc012_m += *sc012_m_1;
	*sc012_m += *sc012_m_2;
	//计算PR曲线
	features_name.push_back("FPFH_CFH_LAB_FPFH_L2");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(sc012_m, sc012_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_fpfh_cfh_lab_fpfh_L2_[i] += correct_matches->size();
		matches_sum_fpfh_cfh_lab_fpfh_L2_[i] += corrs_match->size();
		corresponding_regions_sum_fpfh_cfh_lab_fpfh_L2_[i] += all_correspondences->size();
		std::cout << "correct_matches_sum_fpfh_cfh_lab_fpfh_L2_[i]:   " << correct_matches_sum_fpfh_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "matches_sum_fpfh_cfh_lab_fpfh_L2_[i]:   " << matches_sum_fpfh_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "corresponding_regions_sum_fpfh_cfh_lab_fpfh_L2_[i]:   " << corresponding_regions_sum_fpfh_cfh_lab_fpfh_L2_[i] << std::endl;
		std::cout << "------------------------------- FPFH_CFH_LAB_FPFH_L2 ------------------------------" << std::endl;
		std::cout << "----------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying -----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "--------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // FPFH_CFH_LAB_FPFH_L2

#ifdef PFHRGB

	//6.计算特征的PR曲线（PFHRGB）
	//计算特征（PFHRGB）
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb_m_1(new pcl::PointCloud<pcl::PFHRGBSignature250>());
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb_m_2(new pcl::PointCloud<pcl::PFHRGBSignature250>());
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb_s(new pcl::PointCloud<pcl::PFHRGBSignature250>());
	FeatureExtractor ex_pfhrgb;
	ex_pfhrgb.computePFHRGB(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, pfhrgb_m_1);
	ex_pfhrgb.computePFHRGB(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, pfhrgb_m_2);
	ex_pfhrgb.computePFHRGB(cloud_scene, keypoints_s, normal_s, radius_feature_s, pfhrgb_s);
	//拼接特征（PFHRGB）
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb_m(new pcl::PointCloud<pcl::PFHRGBSignature250>());
	*pfhrgb_m += *pfhrgb_m_1;
	*pfhrgb_m += *pfhrgb_m_2;
	//计算PR曲线
	features_name.push_back("PFHRGB");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(pfhrgb_m, pfhrgb_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_pfhrgb_[i] += correct_matches->size();
		matches_sum_pfhrgb_[i] += corrs_match->size();
		corresponding_regions_sum_pfhrgb_[i] += all_correspondences->size();
		std::cout << "----------------------------- PFHRGB ------------------------------" << std::endl;
		std::cout << "---------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying ----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "-------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // PFHRGB
#ifdef SHOT

	//7.计算特征的PR曲线（SHOT）
	//计算特征（SHOT）
	pcl::PointCloud<pcl::SHOT352>::Ptr shot_m_1(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr shot_m_2(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr shot_s(new pcl::PointCloud<pcl::SHOT352>());
	FeatureExtractor ex_shot;
	ex_shot.computeSHOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, shot_m_1);
	ex_shot.computeSHOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, shot_m_2);
	ex_shot.computeSHOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, shot_s);
	std::cout << "----------------------- Calculate SHOT Features -----------------------" << std::endl;
	std::cout << "feature_m_1 size: " << shot_m_1->size() << std::endl;
	std::cout << "feature_m_2 size: " << shot_m_2->size() << std::endl;
	std::cout << "feature_s size: " << shot_s->size() << std::endl;
	//拼接特征（SHOT）
	pcl::PointCloud<pcl::SHOT352>::Ptr shot_m(new pcl::PointCloud<pcl::SHOT352>());
	*shot_m += *shot_m_1;
	*shot_m += *shot_m_2;
	//计算PR曲线
	features_name.push_back("SHOT");
	for (size_t i = 0; i <= alpha_devide-1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide-1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(shot_m, shot_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_shot_[i] += correct_matches->size();
		matches_sum_shot_[i] += corrs_match->size();
		corresponding_regions_sum_shot_[i] += all_correspondences->size();
		std::cout << "------------------------------ SHOT -------------------------------" << std::endl;
		std::cout << "---------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying ----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "-------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // SHOT
#ifdef CSHOT
	//8.计算特征的PR曲线（CSHOT）
	//计算特征（CSHOT）
	pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_m_1(new pcl::PointCloud<pcl::SHOT1344>());
	pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_m_2(new pcl::PointCloud<pcl::SHOT1344>());
	pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_s(new pcl::PointCloud<pcl::SHOT1344>());
	FeatureExtractor ex_cshot;
	ex_cshot.computeCSHOT(cloud_model_1, keypoints_m_1, normal_m_1, radius_feature_m_1, cshot_m_1);
	ex_cshot.computeCSHOT(cloud_model_2, keypoints_m_2, normal_m_2, radius_feature_m_2, cshot_m_2);
	ex_cshot.computeCSHOT(cloud_scene, keypoints_s, normal_s, radius_feature_s, cshot_s);
	//拼接特征（CSHOT）
	pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_m(new pcl::PointCloud<pcl::SHOT1344>());
	*cshot_m += *cshot_m_1;
	*cshot_m += *cshot_m_2;
	//计算PR曲线
	features_name.push_back("CSHOT");
	for (size_t i = 0; i <= alpha_devide - 1; i++)
	{
		// 匹配：corrs_match
		pcl::CorrespondencesPtr corrs_match(new pcl::Correspondences());//特征匹配结果,<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离>
		double threshold_alpha = alpha_upper_limit * (i / static_cast<double>(alpha_devide - 1));//匹配时的alpha阈值
		FeatureMatch matcher;
		matcher.match_r(cshot_m, cshot_s, corrs_match, threshold_alpha);
		// 验证：correct matches
		pcl::CorrespondencesPtr correct_matches(new pcl::Correspondences());
		Validator va;
		va.findCorrectMatches(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, corrs_match, correct_matches);
		// 验证：correspondences(possible correct matches)
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
		va.findALLCorrespondences(keypoints_m_1, ground_truth_1, keypoints_m_2, ground_truth_2, keypoints_s, beta_threshold, all_correspondences);
		// 计算当前阈值下的PR值
		double p;
		double r;
		if ((corrs_match->size()) == 0)
		{
			p = 1;
			r = 0;
			std::cout << "########warning!#########  ##corrs_match->size()==0" << std::endl;
		}
		else
		{
			p = (static_cast<double>(correct_matches->size())) / static_cast<double>((corrs_match->size()));
			r = (static_cast<double>(correct_matches->size())) / static_cast<double>((all_correspondences->size()));
		}
		if ((all_correspondences->size()) == 0)
		{
			PCL_ERROR("########warning!#########  ##all_correspondences->size()==0");
		}
		precision.push_back(p);
		recall.push_back(r);
		correct_matches_sum_cshot_[i] += correct_matches->size();
		matches_sum_cshot_[i] += corrs_match->size();
		corresponding_regions_sum_cshot_[i] += all_correspondences->size();
		std::cout << "----------------------------- CSHOT -------------------------------" << std::endl;
		std::cout << "---------------------------- Matching -----------------------------" << std::endl;
		std::cout << "all_match / keypoints_s: " << corrs_match->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "---------------------------- verifying ----------------------------" << std::endl;
		std::cout << "correct_matches / all_match: " << correct_matches->size() << "/" << corrs_match->size() << std::endl;
		std::cout << "all_correspondences / keypoints_s: " << all_correspondences->size() << "/" << keypoints_s->size() << std::endl;
		std::cout << "-------------------------- Calculate PR ---------------------------" << std::endl;
		std::cout << "threshold of feature matching: " << threshold_alpha << std::endl;
		std::cout << "threshold of verified: " << beta_threshold << std::endl;
		std::cout << "precision: " << p << std::endl;
		std::cout << "recall: " << r << std::endl;
		std::cout << std::endl << std::endl << std::endl << std::endl;
	}
#endif // CSHOT

}