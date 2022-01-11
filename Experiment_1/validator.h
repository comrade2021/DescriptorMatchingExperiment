#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>

class Validator
{
public:
	//计算匹配的关键点和真实对应点的三维空间欧式距离
	void calculateDeviation(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
		Eigen::Matrix4f transform_matrix,//模型到场景的真实变换矩阵
		pcl::CorrespondencesPtr corr_in,//特征匹配对应（corr->distence为特征之间的距离）
		pcl::CorrespondencesPtr corr_out);//对应点间的距离差值（corr->distence为特征匹配阶段“匹配到的场景对应点”和“真实场景对应点”的坐标之间的欧氏距离）

	//用β阈值验证匹配结果，只保留距离偏差小于β的点
	void verify(
		pcl::CorrespondencesPtr corr_in,
		pcl::CorrespondencesPtr corr_out,
		double beta);

	/// <summary>
	/// 查找匹配对应中的正确对应。
	/// 1.将模型关键点进行位姿变换，获取模型关键点的真实对应点
	/// 2.计算真实点和匹配点的距离
	/// 3.筛选出交并比超过50%的正确匹配对应结果（由球心距离间接计算）
	/// 注：思考，当场景中物体放置紧邻时，可能出现匹配点在错误物体上，而真实点为空，又由于靠近真实点，导致该匹配结果验证为正确匹配，但是当前半径和物体间距下，这种歪打正着的情况很少，可以忽略。
	/// </summary>
	/// <param name="keypoints_m1">模型一的关键点</param>
	/// <param name="ground_truth_1">模型一到场景的变换矩阵</param>
	/// <param name="keypoints_m2">模型二的关键点</param>
	/// <param name="ground_truth_2">模型二到场景的变换矩阵</param>
	/// <param name="keypoints_s">场景的关键点</param>
	/// <param name="beta_threshold">允许的距离偏差阈值</param>
	/// <param name="corrs_match">匹配结果</param>
	/// <param name="correct_matches">正确的匹配结果</param>
	void findCorrectMatches(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1, 
		Eigen::Matrix4f ground_truth_1, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2, 
		Eigen::Matrix4f ground_truth_2, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, 
		double beta_threshold, 
		pcl::CorrespondencesPtr corrs_match, 
		pcl::CorrespondencesPtr correct_matches);

	/// <summary>
	/// 计算模型到场景的全部对应区域数量
	/// 1.转换出模型关键点的真实对应点
	/// 2.检查每个真实点周围是否存在重叠率高的场景关键点。
	/// 3.存储对应结果。由于是刚性变换，可知场景点变换回模型也同样有相邻对应，比如将真实点和相邻的场景点同时变换回模型中，可以发现场景点也有近邻。
	/// </summary>
	/// <param name="keypoints_m1">模型一的关键点</param>
	/// <param name="ground_truth_1">模型一到场景的变换矩阵</param>
	/// <param name="keypoints_m2">模型二的关键点</param>
	/// <param name="ground_truth_2">模型二到场景的变换矩阵</param>
	/// <param name="keypoints_s">场景的关键点</param>
	/// <param name="beta_threshold">允许的距离偏差阈值</param>
	/// <param name="all_correspondences">模型到场景的全部对应区域</param>
	void findALLCorrespondences(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1,
		Eigen::Matrix4f ground_truth_1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2,
		Eigen::Matrix4f ground_truth_2,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
		double beta_threshold,
		pcl::CorrespondencesPtr all_correspondences);

};