#include "my_normal_estimation.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>

#ifndef NumberOfThreads
#define NumberOfThreads 2
#endif // !NumberOfThreads

void MyNormalEstimation::computeNormal(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_model, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_scene, 
	double radius_search_m, 
	double radius_search_s, 
	pcl::PointCloud<pcl::Normal>::Ptr out_normals_model, 
	pcl::PointCloud<pcl::Normal>::Ptr out_normals_scene)
{
	//估计法线
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_1,ne_2;//创建法线估计类的对象
	ne_1.setNumberOfThreads(NumberOfThreads);//设置使用的线程数量（cpu核心数量）
	ne_2.setNumberOfThreads(NumberOfThreads);
	ne_1.setInputCloud(input_cloud_model);//设置待计算的输入点云
	ne_2.setInputCloud(input_cloud_scene);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_m(new pcl::search::KdTree<pcl::PointXYZRGB>());//创建kd树对象，用来加速近邻点搜索
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_s(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne_1.setSearchMethod(tree_m);
	ne_2.setSearchMethod(tree_s);
	ne_1.setRadiusSearch(radius_search_m);//设置法线估计的搜索半径
	ne_2.setRadiusSearch(radius_search_s);
	ne_1.compute(*out_normals_model);//计算并输出法线点云
	ne_2.compute(*out_normals_scene);

	//重新计算一遍法线，填补无效的法线点
	pcl::PointCloud<pcl::Normal>::Ptr nk_1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr nk_2(new pcl::PointCloud<pcl::Normal>);
	ne_1.setRadiusSearch(0);
	ne_1.setKSearch(5);
	ne_1.compute(*nk_1);
	ne_2.setRadiusSearch(0);
	ne_2.setKSearch(5);
	ne_2.compute(*nk_2);

	//填充第一次计算后值为NaN的法线
	int i_1 = 0;
	int	i_2 = 0;
	for (int i = 0; i < out_normals_model->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*out_normals_model)[i]))
		{
			out_normals_model->at(i).normal_x = nk_1->at(i).normal_x;
			out_normals_model->at(i).normal_y = nk_1->at(i).normal_y;
			out_normals_model->at(i).normal_z = nk_1->at(i).normal_z;
			i_1++;
		}
	}
	for (int i = 0; i < out_normals_scene->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*out_normals_scene)[i]))
		{
			out_normals_scene->at(i).normal_x = nk_2->at(i).normal_x;
			out_normals_scene->at(i).normal_y = nk_2->at(i).normal_y;
			out_normals_scene->at(i).normal_z = nk_2->at(i).normal_z;
			i_2++;
		}
	}
	//PCL_WARN(" Refilled [%d] invalid normals\n", i_1);
	//PCL_WARN(" Refilled [%d] invalid normals\n", i_2);
	//再次检查法线空值
	for (int i = 0; i < out_normals_model->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*out_normals_model)[i]))
		{
			PCL_WARN("out_normals_model[%d] is not finite\n", i);
		}
	}
	for (int i = 0; i < out_normals_scene->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*out_normals_scene)[i]))
		{
			PCL_WARN("out_normals_scene[%d] is not finite\n", i);
		}
	}
}

void MyNormalEstimation::computeNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;//创建法线估计类
	ne.setNumberOfThreads(NumberOfThreads);//设置使用的线程数量（cpu核心数量）
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());//使用kd树进行近邻搜索
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(radius);//设置法线估计的搜索半径
	ne.compute(*normal);

	//重新计算一次法线，补充由于搜索半径内近邻点太少导致的无效法线，这次的搜索方式为Ksearch
	pcl::PointCloud<pcl::Normal>::Ptr normal_k(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0);
	ne.setKSearch(5);
	ne.compute(*normal_k);

	//填充第一次计算后值为NaN的法线
	int i_1 = 0;
	for (int i = 0; i < normal->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*normal)[i]))
		{
			normal->at(i).normal_x = normal_k->at(i).normal_x;
			normal->at(i).normal_y = normal_k->at(i).normal_y;
			normal->at(i).normal_z = normal_k->at(i).normal_z;
			i_1++;
		}
	}
	PCL_WARN(" Refilled [%d] invalid normals\n", i_1);
	//再次检查法线空值
	for (int i = 0; i < normal->size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>((*normal)[i]))
		{
			PCL_WARN("Normal [%d] is not finite\n", i);
		}
	}
}

void MyNormalEstimation::computeNormal_K(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int radius, pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;//创建法线估计类
	ne.setNumberOfThreads(NumberOfThreads);//设置使用的线程数量（cpu核心数量）
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());//使用kd树进行近邻搜索
	ne.setSearchMethod(tree);
	ne.setKSearch(radius);
	ne.compute(*normal);
}
