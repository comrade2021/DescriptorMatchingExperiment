#pragma once
//match:PFH,PFHRGB,FPFH
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include "my_point_types.h"

class FeatureMatch
{
public:
	/// <summary>
	/// 对每个场景特征，先在模型中寻找欧式距离最近的对应特征，再选出距离小于阈值的对应。
	/// </summary>
	/// <param name="pfh_m">模型特征</param>
	/// <param name="pfh_s">场景特征</param>
	/// <param name="corr">输出对应,内容分别为<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离></param>
	/// <param name="threshold">特征距离上限</param>
	void match(
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match(
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match(
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match(//FPFH_RGB_ORI
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m,
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match(
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match(
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);

	/// <summary>
	/// 根据特征的最近邻和次近邻比值，为每个场景关键点匹配模型关键点。
	/// </summary>
	/// <param name="pfh_m">模型特征</param>
	/// <param name="pfh_s">场景特征</param>
	/// <param name="corr">输出对应,内容分别为<index_query:场景特征,index_match:匹配到的模型特征,distence:特征间欧式距离></param>
	/// <param name="threshold">最近邻和次近邻比值的阈值</param>
	void match_r(
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match_r(
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match_r(
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match_r(//FPFH_RGB_ORI
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m,
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match_r(
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);
	void match_r(
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s,
		pcl::CorrespondencesPtr corr,
		double threshold);

public:
	//根据最近邻和次近邻比值完成特征匹配（NNDR方式）
	void filterNN(
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2,
		double alpha,
		pcl::CorrespondencesPtr corr_out);//参数：最近邻对应，次近邻对应，“最近邻/次近邻”的上限阈值，被采纳的最近邻对应结果[模型关键点索引，场景关键点索引，对应特征的欧氏距离]

	//计算模型特征的最近邻和次近邻场景特征
	void computeNN(
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
	void computeNN(
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m,
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
	void computeNN(
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
	void computeNN(
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m,
		pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
	void computeNN(
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT352>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
	void computeNN(
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m,
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s,
		pcl::CorrespondencesPtr model_scene_corrs_n1,
		pcl::CorrespondencesPtr model_scene_corrs_n2);//n1:最近邻，n2:次近邻  两者对应点内容：[模型关键点索引，场景关键点索引，对应特征的欧氏距离]
};
