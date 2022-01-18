#include "cfh_rgb_dot.h"
#include <pcl/features/pfh_tools.h>

void CFH_Estimation_RGB_DOT::computeFeature(pcl::PointCloud<pcl::FPFHSignature33>& output)
{
    // resize output
    output.resize(input_keypoints_->size());//为output分配空间，否则会出现向量下标越界

    /// nr_subdiv for RGB
    cfh_histogram_.setZero(nr_subdiv_);
    cfh_tuple_.setZero(1);

    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
    {
        pcl::Indices nn_indices;
        std::vector<float> nn_dists;
        searchForNeighbors(idx, search_radius_, nn_indices, nn_dists);
        //// Estimate the PFH signature(only RGB) at each patch
        computePointSignature(*input_cloud_, nn_indices, nr_subdiv_, cfh_histogram_);

        std::copy_n(cfh_histogram_.data(), cfh_histogram_.size(), output[idx].histogram);
    }
}

void CFH_Estimation_RGB_DOT::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) { input_cloud_ = input_cloud; }
void CFH_Estimation_RGB_DOT::setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals) { input_normals_ = input_normals; }
void CFH_Estimation_RGB_DOT::setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints) { input_keypoints_ = input_keypoints; }
void CFH_Estimation_RGB_DOT::setSearchRadius(double search_radius) { search_radius_ = search_radius; }


//此处有错，输出indices不为空，但数值都是0
void CFH_Estimation_RGB_DOT::searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(input_cloud_);
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch((*input_keypoints_)[index], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        indices.resize(pointIdxRadiusSearch.size());//清空
        distances.resize(pointRadiusSquaredDistance.size());//清空
        for (size_t i = 0; i < indices.size(); i++)
        {
            indices.at(i) = pointIdxRadiusSearch.at(i);
        }
        for (size_t i = 0; i < indices.size(); i++)
        {
            distances.at(i) = pointRadiusSquaredDistance.at(i);
        }
    }
    else
    {
        PCL_WARN("CFH_Estimation_RGB_DOT::searchForNeighbors: kdtree.radiusSearch() == 0");
    }

}

void CFH_Estimation_RGB_DOT::computePointSignature(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::Indices& indices, int nr_split, Eigen::VectorXf& cfh_histogram)
{
    int h_index;

    // Clear the resultant point histogram
    cfh_histogram.setZero();

    // Factorization constant (直方图数值总和为200)
    float hist_incr = 100.0f / static_cast<float> (indices.size() * (indices.size() - 1) / 2);

    // Iterate over all the points in the neighborhood（所有邻域点的"排列"）
    for (const auto& index_i : indices)
    {
        for (const auto& index_j : indices)
        {
            // Avoid unnecessary returns
            if (index_i == index_j)
                continue;

            // Compute the pair NNi to NNj
            if (!computeRGBPairFeatures(cloud, index_i, index_j,cfh_tuple_[0]))
                continue;

            // 计算每个特征值应该放入的的bin
            float feature_value = nr_split * ((cfh_tuple_[0] + 1.0) * 0.5);
            f_index_[0] = static_cast<int> (std::floor(feature_value));
            for (auto& feature : f_index_)
            {
                feature = std::min(nr_split - 1, std::max(0, feature));//bin序号范围: [0,nr_split - 1]
            }

            // Copy into the histogram
            h_index = f_index_[0];
            cfh_histogram[h_index] += hist_incr;
        }
    }
}

bool CFH_Estimation_RGB_DOT::computeRGBPairFeatures(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int p_idx, int q_idx, float& f1)
{
    float r1 = cloud[p_idx].r / static_cast<float>(255) - 0.5;
    float g1 = cloud[p_idx].g / static_cast<float>(255) - 0.5;
    float b1 = cloud[p_idx].b / static_cast<float>(255) - 0.5;
    float r2 = cloud[q_idx].r / static_cast<float>(255) - 0.5;
    float g2 = cloud[q_idx].g / static_cast<float>(255) - 0.5;
    float b2 = cloud[q_idx].b / static_cast<float>(255) - 0.5;
    Eigen::Vector4f colors1(r1,g1,b1,0), colors2(r2,g2,b2,0);
    colors1.normalize();
    colors2.normalize();

    computeRGBPairFeatures(colors1, colors2, f1);
    return (true);
}

bool CFH_Estimation_RGB_DOT::computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1)
{
    
    f1 = colors1.dot(colors2);
    return (true);
}