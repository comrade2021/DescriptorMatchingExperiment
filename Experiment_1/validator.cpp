#include "validator.h"
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

void Validator::calculateDeviation(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s,
	Eigen::Matrix4f transform_matrix,
	pcl::CorrespondencesPtr corr_in,
	pcl::CorrespondencesPtr corr_out)
{
	for (size_t i = 0; i < corr_in->size(); i++)
	{
		Eigen::Transform<float, 3, Eigen::Affine> transform_affine3f(transform_matrix); //Matrix4f(四维)矩阵转换为Affine3f(四维)矩阵

		pcl::index_t index_query = corr_in->at(i).index_query;//获取匹配的关键点的索引
		pcl::index_t index_match = corr_in->at(i).index_match;

		pcl::PointXYZRGB point = keypoints_m->at(index_query);//模型关键点
		pcl::PointXYZRGB predicted_point = keypoints_s->at(index_match);//初次匹配的结果，也就是预测的对应特征点
		pcl::PointXYZRGB actual_point = pcl::transformPoint(point, transform_affine3f);//真实对应特征点
		float distence = (predicted_point.getVector3fMap() - actual_point.getVector3fMap()).norm();//计算真实对应点和匹配点的距离
		pcl::Correspondence corr(index_query, index_match, distence);
		corr_out->push_back(corr);
	}
}

void Validator::verify(
	pcl::CorrespondencesPtr corr_in, 
	pcl::CorrespondencesPtr corr_out, 
	double beta)
{
	for (size_t i = 0; i < corr_in->size(); i++)
	{
		if (corr_in->at(i).distance<=beta)
		{
			pcl::Correspondence corr(corr_in->at(i).index_query, corr_in->at(i).index_match, corr_in->at(i).distance);
			corr_out->push_back(corr);
		}
	}
}

void Validator::findCorrectMatches(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1, 
	Eigen::Matrix4f ground_truth_1, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2, 
	Eigen::Matrix4f ground_truth_2, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, 
	double beta_threshold,
	pcl::CorrespondencesPtr corrs_match, 
	pcl::CorrespondencesPtr correct_matches)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_gt(new pcl::PointCloud<pcl::PointXYZRGB>);//模型关键点在场景中的真实对应点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*keypoints_m1, *keypoints_m1_gt, ground_truth_1);
	pcl::transformPointCloud(*keypoints_m2, *keypoints_m2_gt, ground_truth_2);
	*keypoints_m_gt += *keypoints_m1_gt;
	*keypoints_m_gt += *keypoints_m2_gt;
	//计算真实点、和对应点的距离
	for (size_t i = 0; i < corrs_match->size(); i++)
	{
		int index_s = corrs_match->at(i).index_query;
		int index_m = corrs_match->at(i).index_match;
		pcl::PointXYZRGB point_s = keypoints_s->at(index_s);
		pcl::PointXYZRGB point_m_gt = keypoints_m_gt->at(index_m);//keypoints_m_gt与keypoints_m的索引是一致的
		double distence = (point_s.getVector3fMap() - point_m_gt.getVector3fMap()).norm();//计算两点距离

		//两个半径为1的球体相交，当球心距离为0.773926时，两球体交并比为50%，因此两球心距离小于0.773926时，交并比超过50%。
		//double t = feature_radius * 0.773926;
		if (distence <= beta_threshold)
		{
			pcl::Correspondence co(index_s, index_m, distence);
			correct_matches->push_back(co);
		}
	}
}

void Validator::findALLCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1, Eigen::Matrix4f ground_truth_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2, Eigen::Matrix4f ground_truth_2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_s, double beta_threshold, pcl::CorrespondencesPtr all_correspondences)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m_gt(new pcl::PointCloud<pcl::PointXYZRGB>);//模型关键点在场景中的真实对应点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m1_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_m2_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*keypoints_m1, *keypoints_m1_gt, ground_truth_1);
	pcl::transformPointCloud(*keypoints_m2, *keypoints_m2_gt, ground_truth_2);
	*keypoints_m_gt += *keypoints_m1_gt;
	*keypoints_m_gt += *keypoints_m2_gt;
	pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(keypoints_s);
	for (size_t i = 0; i < keypoints_m_gt->size(); i++)
	{
		std::vector<int> indices;
		std::vector<float> distence;
		//double t = feature_radius * 0.773926;
		int num = kdtree.radiusSearch(keypoints_m_gt->points[i], beta_threshold, indices, distence);
		if (num >= 1)
		{
			pcl::Correspondence co(indices[0], i, distence[0]);//<场景索引，模型索引，距离>
			all_correspondences->push_back(co);
		}
	}
}


