#pragma once
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "my_point_types.h"

class FeatureExtractor
{
public:
	/// <summary>
	/// 计算PFH特征
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="pfh">输出特征</param>
	void computePFH(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature);
	
	/// <summary>
	/// 计算PFHRGB特征
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="pfh">输出特征</param>
	void computePFHRGB(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature);
	
	/// <summary>
	/// 计算FPFH特征
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="pfh">输出特征</param>
	void computeFPFH(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature);

	/// 计算色彩直方图（RGB pfh结构 rate）
	void computeCFH_RGB_PFH_RATE(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature);
	
	/// 计算色彩直方图（RGB pfh结构 dot）
	void computeCFH_RGB_PFH_DOT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature);

	/// 计算色彩直方图（RGB fpfh结构 rate）
	void computeCFH_RGB_FPFH_RATE(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature);
	
	/// 计算色彩直方图（RGB fpfh结构 dot）
	void computeCFH_RGB_FPFH_DOT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature);

	/// <summary>
	/// 几何（FPFH）与色彩（cfh_rgb_fpfh_rate）的拼接特征，共66维，结果存储于pfh125直方图
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="feature">输出特征</param>
	void computeFPFH_CFH_RGB_FPFH_RATE(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr feature);

	/// <summary>
	/// 计算SHOT特征
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="pfh">输出特征</param>
	void computeSHOT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::SHOT352>::Ptr feature);

	/// <summary>
	/// 计算CSHOT特征
	/// </summary>
	/// <param name="cloud">输入点云</param>
	/// <param name="keypoints">输入关键点</param>
	/// <param name="normal">输入法线</param>
	/// <param name="radius">特征计算半径</param>
	/// <param name="pfh">输出特征</param>
	void computeCSHOT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
		pcl::PointCloud<pcl::Normal>::Ptr normal,
		double radius,
		pcl::PointCloud<pcl::SHOT1344>::Ptr feature);

};
