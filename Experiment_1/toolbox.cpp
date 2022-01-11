#include "toolbox.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/point_tests.h>

float ToolBox::computeMeshResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
	using KdTreeInPtr = typename pcl::KdTree<pcl::PointXYZRGB>::Ptr;
	KdTreeInPtr tree = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB>>(false);
	tree->setInputCloud(input);

	pcl::Indices nn_indices(9);
	std::vector<float> nn_distances(9);

	float sum_distances = 0.0;//总距离
	std::vector<float> avg_distances(input->size());//平均距离
	//遍历源数据
	for (std::size_t i = 0; i < input->size(); ++i) {
		tree->nearestKSearch((*input)[i], 9, nn_indices, nn_distances);//查询每个点的九个近邻点

		float avg_dist_neighbours = 0.0;
		for (std::size_t j = 1; j < nn_indices.size(); j++)//不计算第一个近邻点的距离，因为它总是零
			avg_dist_neighbours += std::sqrt(nn_distances[j]);//累加查询点到这九个点的距离

		avg_dist_neighbours /= static_cast<float>(nn_indices.size());//计算查询点到近邻点的平均距离

		avg_distances[i] = avg_dist_neighbours;//avg_distances[]存储“每个点与近邻点间的平均距离”
		sum_distances += avg_dist_neighbours;//累加“每个点与近邻点间的平均距离”
	}

	//计算平均长度的中位数
	std::sort(avg_distances.begin(), avg_distances.end());
	float median;
	if (avg_distances.size() % 2 != 0)
	{
		median = avg_distances[(static_cast<int>(avg_distances.size()) / 2)];
	}
	else
	{
		median = (avg_distances[(static_cast<int>(avg_distances.size()) / 2)] + avg_distances[(static_cast<int>(avg_distances.size()) / 2)-1]) / 2;
	}

	return median;
}

double ToolBox::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);

	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void ToolBox::accumulateVector(std::vector<double>& vec_acc, std::vector<double>& vec)
{
	if (vec_acc.size()==vec.size())
	{
		for (size_t i = 0; i < vec_acc.size(); i++)
		{
			vec_acc[i] += vec[i];
		}
	}
	else
	{
		PCL_ERROR("ERROR: ToolBox::accumulateVector(std::vector<double>& vec_acc, std::vector<double>& vec),vec_acc.size()!=vec.size()!");
	}
	
}
