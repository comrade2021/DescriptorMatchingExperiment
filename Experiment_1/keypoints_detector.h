#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class KeypointsDetector
{
public:

	/// <summary>
	/// 计算出一对“模型-场景”的真实对应关键点。（保证对应数量）
	/// </summary>
	/// <param name="cloud_model"></param>
	/// <param name="cloud_scene"></param>
	/// <param name="keypoints_m"></param>
	/// <param name="keypoints_s"></param>
	/// <param name="num_corr_key"></param>
	/// <param name="ground_truth"></param>
	/// <param name="radius_for_inspection">根据场景点云此半径内的点数判断有效的对应关键点</param>
	void compute(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
		int num_corr_key,//对应数量
		Eigen::Matrix4f ground_truth,
		double radius_for_inspection);//检测无效对应关键点的搜索半径
									  
	/// <summary>
	/// 计算出一对“模型-场景”的真实对应关键点。（不关心对应数量）
	/// </summary>
	/// <param name="cloud_model"></param>
	/// <param name="cloud_scene"></param>
	/// <param name="keypoints_m"></param>
	/// <param name="keypoints_s"></param>
	/// <param name="ground_truth"></param>
	/// <param name="radius_for_inspection">根据场景点云此半径内的点数判断有效的对应关键点</param>
	void compute(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
		Eigen::Matrix4f ground_truth,
		double radius_for_inspection,
		int keypoints_num);//检测无效对应关键点的搜索半径

	/// <summary>
	/// 计算出一对“模型-场景”的真实对应关键点。（模型关键点由体素下采样得到）
	/// </summary>
	/// <param name="cloud_model"></param>
	/// <param name="cloud_scene"></param>
	/// <param name="keypoints_m"></param>
	/// <param name="keypoints_s"></param>
	/// <param name="ground_truth"></param>
	/// <param name="radius_for_inspection"></param>
	/// <param name="leaf_size"></param>
	void compute_v(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, Eigen::Matrix4f ground_truth, double radius_for_inspection,double leaf_size);

	/// <summary>
	/// 随机采样关键点
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输出的关键点</param>
	/// <param name="num">需要的关键点数量</param>
	void computeRandomSampleKeypoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		int num);
	
	/// <summary>
	/// 体素质心下采样获取关键点
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输出的关键点</param>
	/// <param name="L1">体素尺寸,单位：米</param>
	/// <param name="L2"></param>
	/// <param name="L3"></param>
	void computeVoxelGridKeypoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		double leaf_size);

	/// <summary>
	/// 从cloud中随机选点，添加至keypoints，作为噪声关键点
	/// </summary>
	/// <param name="cloud"></param>
	/// <param name="keypoints"></param>
	/// <param name="n_add">即将添加的噪声关键点数量</param>
	void addNoisePoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		int n_add);
	//获取模型点云的关键点-随机采样算法
	void computeRandomSample(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,//初次检测到的模型关键点
		int num_m);
protected:
	//获取模型点云的关键点-ISS3D关键点检测算法
	void computeISS3DKeypoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
		pcl::PointCloud<pcl::Normal>::Ptr normal_m,
		pcl::PointCloud<pcl::Normal>::Ptr normal_s,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,
		double model_resolution);

	/// <summary>
	/// 根据模型关键点和真实变换，计算场景关键点，再过滤出有效的模型和场景的对应关键点
	/// </summary>
	/// <param name="cloud_model"></param>
	/// <param name="cloud_scene"></param>
	/// <param name="keypoints_unfiltered_m">初次检测到的模型关键点</param>
	/// <param name="ground_truth">模型到场景的真实变换，数据集中已经标注</param>
	/// <param name="radius_for_inspection">检查关键点有效性的近邻搜索半径，此半径内近邻点数少于模型近邻点数一半的场景关键点为无效点，建议设置为法线估计半径</param>
	/// <param name="keypoints_m"></param>
	/// <param name="keypoints_s"></param>
	void filterValidKeypoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,//
		Eigen::Matrix4f ground_truth,//
		double radius_for_inspection,//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s);

	//裁剪多余点
	void cut(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
		int n_cut); //n_cut:模型关键点需要修剪到多少

	
};
