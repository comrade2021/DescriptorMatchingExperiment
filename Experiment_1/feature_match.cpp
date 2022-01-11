#include "feature_match.h"
#include <pcl/kdtree/kdtree_flann.h>

void FeatureMatch::computeNN(
	pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m,
	pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	//利用KDTree寻找模型到场景的对应
	pcl::KdTreeFLANN<pcl::PFHSignature125> kdtree;
	kdtree.setInputCloud(feature_m);
	//利用直方图特征的欧式距离，查找最近邻和次近邻特征点，特征索引即关键点索引
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		//查找场景特征的两个近邻点
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		//分为最近邻和次近邻
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);//注意区分Correspondences 和 Correspondence，否则会对这里的vector感到疑惑。
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::computeNN(
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m,
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	pcl::KdTreeFLANN<pcl::PFHRGBSignature250> kdtree;
	kdtree.setInputCloud(feature_m);
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::computeNN(
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree;
	kdtree.setInputCloud(feature_m);
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::computeNN(
	pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m,
	pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	pcl::KdTreeFLANN<FPFH_RGB_ORI> kdtree;
	kdtree.setInputCloud(feature_m);
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		PCL_WARN("%f\n", sqr_distances[0]);
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::computeNN(
	pcl::PointCloud<pcl::SHOT352>::Ptr feature_m,
	pcl::PointCloud<pcl::SHOT352>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	pcl::KdTreeFLANN<pcl::SHOT352> kdtree;
	kdtree.setInputCloud(feature_m);
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::computeNN(
	pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m,
	pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s,
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2)
{
	pcl::KdTreeFLANN<pcl::SHOT1344> kdtree;
	kdtree.setInputCloud(feature_m);
	for (std::size_t i = 0; i < feature_s->size(); ++i)
	{
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 2, indices, sqr_distances);
		if (sqr_distances[0] <= sqr_distances[1])
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[1], sqr_distances[1]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
		else {
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[1], sqr_distances[1]);
			pcl::Correspondence corr_n2(static_cast<int> (i), indices[0], sqr_distances[0]);
			model_scene_corrs_n1->push_back(corr_n1);
			model_scene_corrs_n2->push_back(corr_n2);
		}
	}
}

void FeatureMatch::filterNN(
	pcl::CorrespondencesPtr model_scene_corrs_n1,
	pcl::CorrespondencesPtr model_scene_corrs_n2,
	double alpha,
	pcl::CorrespondencesPtr corr_out)
{
	//检查两个对应关系中，最近邻和次近邻索引总数是否一致
	if (model_scene_corrs_n1->size() != model_scene_corrs_n2->size())
	{
		std::cout << "error: when feature mating,size of model_scene_corrs_n1 !=  size of model_scene_corrs_n2!" << std::endl;
	}

	//遍历对应，当最近邻和次近邻的比值小于alpla时，就判断该最近邻点为匹配点，将对应索引添加到输出结果
	for (size_t i = 0; i < model_scene_corrs_n1->size(); i++)
	{
		//检查查询点是否相同
		if (model_scene_corrs_n1->at(i).index_query != model_scene_corrs_n2->at(i).index_query)
		{
			std::cout << "error: The index_query in the two correspondences are not the same!" << std::endl;
		}
		double d1 = model_scene_corrs_n1->at(i).distance;
		double d2 = model_scene_corrs_n2->at(i).distance;
		//检查最近邻和次近邻距离值是否合法
		if (d1 > d2 || d1 < 0 || d2 < 0)
		{
			std::cout << "error: The nearest neighbor distance is greater than the next nearest neighbor distance! OR distance < 0 !" << std::endl;
		}
		//计算最近邻和次近邻的距离之比
		double r = 0.0;
		if (d1 == d2)
		{
			r = 1;
		}
		else
		{
			r = d1 / d2;
		}
		//记录符合阈值的匹配关系
		if (r <= alpha)
		{
			pcl::Correspondence corr(model_scene_corrs_n1->at(i).index_query, model_scene_corrs_n1->at(i).index_match, model_scene_corrs_n1->at(i).distance);
			corr_out->push_back(corr);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	//1.寻找最近邻
	//2.判断距离
	//3.输出对应
	pcl::KdTreeFLANN<pcl::PFHSignature125> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		//为场景特征查找一个近邻
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs==1&&sqr_distances[0]<=threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::KdTreeFLANN<pcl::PFHRGBSignature250> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs==1&&sqr_distances[0]<=threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs==1&&sqr_distances[0]<=threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m, pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::KdTreeFLANN<FPFH_RGB_ORI> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs == 1 && sqr_distances[0] <= threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<pcl::SHOT352>::Ptr feature_m, pcl::PointCloud<pcl::SHOT352>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::KdTreeFLANN<pcl::SHOT352> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs==1&&sqr_distances[0]<=threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match(pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m, pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::KdTreeFLANN<pcl::SHOT1344> kdtree;
	kdtree.setInputCloud(feature_m);
	for (size_t i = 0; i < feature_s->size(); i++)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);
		int found_neighs = kdtree.nearestKSearch(feature_s->at(i), 1, indices, sqr_distances);
		if (found_neighs==1&&sqr_distances[0]<=threshold)
		{
			pcl::Correspondence corr_n1(static_cast<int> (i), indices[0], sqr_distances[0]);
			corr->push_back(corr_n1);
		}
	}
}

void FeatureMatch::match_r(pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_m, pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}

void FeatureMatch::match_r(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_m, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}

void FeatureMatch::match_r(pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_m, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}

void FeatureMatch::match_r(pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_m, pcl::PointCloud<FPFH_RGB_ORI>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}

void FeatureMatch::match_r(pcl::PointCloud<pcl::SHOT352>::Ptr feature_m, pcl::PointCloud<pcl::SHOT352>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}

void FeatureMatch::match_r(pcl::PointCloud<pcl::SHOT1344>::Ptr feature_m, pcl::PointCloud<pcl::SHOT1344>::Ptr feature_s, pcl::CorrespondencesPtr corr, double threshold)
{
	pcl::CorrespondencesPtr corrs_n1(new pcl::Correspondences());
	pcl::CorrespondencesPtr corrs_n2(new pcl::Correspondences());
	computeNN(feature_m, feature_s, corrs_n1, corrs_n2);
	filterNN(corrs_n1, corrs_n2, threshold, corr);
}
