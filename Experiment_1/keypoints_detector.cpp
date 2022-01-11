#include "keypoints_detector.h"
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/impl/boundary.hpp>
#include <pcl/filters/random_sample.h>
#include <boost/winapi/time.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

//计算关键点有三个步骤：
//1.在模型点云上检测关键点。
//2.将（1）中获得的模型关键点，进行坐标变换（来自数据集标注的真实变换矩阵），得到场景中的对应关键点。
//3.变换得到场景关键点可能没有足够临近点，需要把这些孤立的对应关键点滤除。最后保证每个模型上能够有1000个关键点。
//4.在场景关键点中，除了能够和模型对齐的关键点，还要添加随机选取的1000干扰点。场景关键点数量=模型数量*1000+场景干扰点数量。
//KeypointsDetector::filterValidKeypoints函数的功能是上述的（2）（3）步骤

void KeypointsDetector::compute(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, 
	int num_corr_key,
	Eigen::Matrix4f ground_truth,
	double radius_for_inspection)
{
	int n = 1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_s(new pcl::PointCloud<pcl::PointXYZRGB>);
	computeRandomSample(cloud_model, keypoints_unfiltered_m,n * num_corr_key);
	filterValidKeypoints(cloud_model, cloud_scene, keypoints_unfiltered_m, ground_truth, radius_for_inspection, keypoints_m, keypoints_s);
	//过滤后关键点不能少于num_corr_key
	while (keypoints_m->size() < num_corr_key)
	{
		++n;
		if ((static_cast<unsigned long long>(n) * num_corr_key) >= cloud_model->size())
		{
			std::cout << "keypoint detector error: num_corr_key is too large!" << std::endl;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		computeRandomSample(cloud_model, keypoints_unfiltered_m_temp, n * num_corr_key);
		filterValidKeypoints(cloud_model, cloud_scene, keypoints_unfiltered_m_temp, ground_truth, radius_for_inspection, keypoints_m, keypoints_s);
	}
	cut(keypoints_m, keypoints_s, num_corr_key);//选择固定数量的关键点
}

void KeypointsDetector::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, Eigen::Matrix4f ground_truth, double radius_for_inspection, int keypoints_num)
{
	pcl::Indices index_s;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_s(new pcl::PointCloud<pcl::PointXYZRGB>);

	computeRandomSample(cloud_model, keypoints_m, keypoints_num);
	pcl::transformPointCloud(*keypoints_m, *keypoints_unfiltered_s, ground_truth);
	//筛除在场景中近邻点太少的关键点，保证关键点的真实有效，场景关键点的近邻点数至少为模型的一半
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_m;
	kdtree_m.setInputCloud(cloud_model);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_s;
	kdtree_s.setInputCloud(cloud_scene);
	//检查近邻点数量
	for (size_t i = 0; i < keypoints_m->size(); i++)
	{
		std::vector<int> pointIdxRadiusSearch_1;
		std::vector<float> pointRadiusSquaredDistance_1;
		std::vector<int> pointIdxRadiusSearch_2;
		std::vector<float> pointRadiusSquaredDistance_2;
		int num_m = kdtree_m.radiusSearch(keypoints_m->at(i), radius_for_inspection, pointIdxRadiusSearch_1, pointRadiusSquaredDistance_1);
		int num_s = kdtree_s.radiusSearch(keypoints_unfiltered_s->at(i), radius_for_inspection, pointIdxRadiusSearch_2, pointRadiusSquaredDistance_2);
		if ((num_s >= (num_m/3))&&(num_s!=0)) //场景关键点周围要有足够近邻点
		{
			index_s.push_back(i);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_s(new pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_s, index_s));
	pcl::copyPointCloud(*temp_s, *keypoints_s);
}

void KeypointsDetector::compute_v(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, Eigen::Matrix4f ground_truth, double radius_for_inspection, double leaf_size)
{
	pcl::Indices index_s;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_s(new pcl::PointCloud<pcl::PointXYZRGB>);

	computeVoxelGridKeypoints(cloud_model, keypoints_m, leaf_size);
	pcl::transformPointCloud(*keypoints_m, *keypoints_unfiltered_s, ground_truth);
	//筛除在场景中近邻点太少的关键点，保证关键点的真实有效，场景关键点的近邻点数至少为模型的一半
	//先得到模型关键点再获取场景对应关键点，之后滤除不可用的场景关键点
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_m;
	kdtree_m.setInputCloud(cloud_model);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_s;
	kdtree_s.setInputCloud(cloud_scene);
	//检查近邻点数量
	for (size_t i = 0; i < keypoints_m->size(); i++)
	{
		std::vector<int> pointIdxRadiusSearch_1;
		std::vector<float> pointRadiusSquaredDistance_1;
		std::vector<int> pointIdxRadiusSearch_2;
		std::vector<float> pointRadiusSquaredDistance_2;
		int num_m = kdtree_m.radiusSearch(keypoints_m->at(i), radius_for_inspection, pointIdxRadiusSearch_1, pointRadiusSquaredDistance_1);
		int num_s = kdtree_s.radiusSearch(keypoints_unfiltered_s->at(i), radius_for_inspection, pointIdxRadiusSearch_2, pointRadiusSquaredDistance_2);
		if ((num_s >= (num_m / 3)) && (num_s != 0)) //场景关键点周围要有足够近邻点
		{
			index_s.push_back(i);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_s(new pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_s, index_s));
	pcl::copyPointCloud(*temp_s, *keypoints_s);
}

void KeypointsDetector::computeRandomSampleKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, int num)
{
	pcl::RandomSample<pcl::PointXYZRGB> rs;
	rs.setInputCloud(cloud);
	rs.setSeed(GetTickCount());//系统启动以来的嘀嗒时间作为随机种子

	rs.setSample(num);
	rs.filter(*keypoints);
}

void KeypointsDetector::computeRandomSample(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,
	int num_m)
{
	//模型点云的随机采样
	pcl::RandomSample<pcl::PointXYZRGB> rs1;
	rs1.setInputCloud(cloud_model);
	rs1.setSeed(GetTickCount());//系统启动以来的嘀嗒时间作为随机种子
	rs1.setSample(num_m);
	rs1.filter(*keypoints_unfiltered_m);
}

void KeypointsDetector::computeISS3DKeypoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
	pcl::PointCloud<pcl::Normal>::Ptr normal_m,
	pcl::PointCloud<pcl::Normal>::Ptr normal_s,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,
	double model_resolution)
{
	pcl::ISSKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZRGB> iss_detector;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//计算模型的ISS3D关键点
	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(4 * model_resolution);
	iss_detector.setNonMaxRadius(2 * model_resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(cloud_model);
	iss_detector.setNormals(normal_m);
	iss_detector.compute(*keypoints_unfiltered_m);
}

void KeypointsDetector::filterValidKeypoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_m,
	Eigen::Matrix4f ground_truth,
	double radius_for_inspection,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s)
{
	pcl::Indices index_m;
	pcl::Indices index_s;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_unfiltered_s(new pcl::PointCloud<pcl::PointXYZRGB>);
	//将模型关键点进行坐标变换，得到对应的场景关键点（均未过滤）
	pcl::transformPointCloud(*keypoints_unfiltered_m, *keypoints_unfiltered_s, ground_truth);
	//筛除在场景中近邻点太少的关键点，保证关键点的真实有效，场景关键点的近邻点数至少为模型的一半
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_m;
	kdtree_m.setInputCloud(cloud_model);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_s;
	kdtree_s.setInputCloud(cloud_scene);
	//检查关键点的近邻点数量
	for (size_t i = 0; i < keypoints_unfiltered_m->size(); i++)
	{
		std::vector<int> pointIdxRadiusSearch_1;
		std::vector<float> pointRadiusSquaredDistance_1;
		std::vector<int> pointIdxRadiusSearch_2;
		std::vector<float> pointRadiusSquaredDistance_2;
		int num_m = kdtree_m.radiusSearch(keypoints_unfiltered_m->at(i), radius_for_inspection, pointIdxRadiusSearch_1, pointRadiusSquaredDistance_1);
		int num_s = kdtree_s.radiusSearch(keypoints_unfiltered_s->at(i), radius_for_inspection, pointIdxRadiusSearch_2, pointRadiusSquaredDistance_2);
		if (num_m==0)
		{
			std::cout << "keypoints_detector warning: neighborhood of keypoints（unfiltered,model） is zero!" << std::endl;
		}
		if ((num_s>=(num_m/3))&&(num_m>=4)) //场景关键点周围要有足够近邻点
		{
			index_m.push_back(i);
			index_s.push_back(i);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_m(new pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_m, index_m));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_s(new pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_s, index_s));
	pcl::copyPointCloud(*temp_m, *keypoints_m);
	pcl::copyPointCloud(*temp_s, *keypoints_s);

	//*keypoints_m = pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_m, index_m);//使用copypointcloud
	//*keypoints_s = pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_unfiltered_s, index_s);
}

void KeypointsDetector::cut(pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, int n_cut)
{
	pcl::Indices index;
	pcl::RandomSample<pcl::PointXYZRGB> rs_c;
	rs_c.setInputCloud(keypoints_m);
	rs_c.setSeed(time(0)+1);
	rs_c.setSample(n_cut);
	rs_c.filter(index);

	*keypoints_m = pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_m, index);
	*keypoints_s = pcl::PointCloud<pcl::PointXYZRGB>(*keypoints_s, index);
}

void KeypointsDetector::computeVoxelGridKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, double leaf_size)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*keypoints);
}

void KeypointsDetector::addNoisePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, int n_add)
{
	pcl::RandomSample<pcl::PointXYZRGB> rs_a;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr noise(new pcl::PointCloud<pcl::PointXYZRGB>);
	rs_a.setInputCloud(cloud);
	rs_a.setSeed(time(0)+2);
	rs_a.setSample(n_add);
	rs_a.filter(*noise);
	*keypoints += *noise;
}

