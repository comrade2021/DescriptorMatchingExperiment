#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MyNormalEstimation
{
public:
	
	//计算模型和场景法线，滤除近邻点太少的离散点云
	void computeNormal(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_model,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_scene,
		double radius_search_m,
		double radius_search_s,
		pcl::PointCloud<pcl::Normal>::Ptr out_normals_model,
		pcl::PointCloud<pcl::Normal>::Ptr out_normals_scene);
	
	/// <summary>
	/// 计算点云法线
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="radius">计算法线的搜索半径</param>
	/// <param name="normal">输出法线</param>
	void computeNormal(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		double radius,
		pcl::PointCloud<pcl::Normal>::Ptr normal);

private:

};