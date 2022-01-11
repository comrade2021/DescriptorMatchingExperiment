#include "cfh_rgb_rate_33.h"
#include <pcl/features/pfh_tools.h>

void CFH_Estimation_RGB_RATE_33::computeFeature(pcl::PointCloud<pcl::FPFHSignature33>& output)
{
    //resize output
    output.resize(input_keypoints_->size());//为output分配空间，否则会出现向量下标越界

    /// nr_subdiv 33 for RGB
    pfhrgb_histogram_.setZero(3* nr_subdiv_);
    pfhrgb_tuple_.setZero(3);

    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
    {
        pcl::Indices nn_indices;
        std::vector<float> nn_dists;
        searchForNeighbors(idx, search_radius_, nn_indices, nn_dists);

        //// Estimate the PFH signature(only RGB) at each patch
        computePointPFHRGBSignature(*input_cloud_, nn_indices, nr_subdiv_, pfhrgb_histogram_);

        std::copy_n(pfhrgb_histogram_.data(), pfhrgb_histogram_.size(), output[idx].histogram);
    }

}

void CFH_Estimation_RGB_RATE_33::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) { input_cloud_ = input_cloud; }
void CFH_Estimation_RGB_RATE_33::setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals) { input_normals_ = input_normals; }
void CFH_Estimation_RGB_RATE_33::setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints) { input_keypoints_ = input_keypoints; }
void CFH_Estimation_RGB_RATE_33::setSearchRadius(double search_radius) { search_radius_ = search_radius; }


//此处有错，输出indices不为空，但数值都是0
void CFH_Estimation_RGB_RATE_33::searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances)
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
        PCL_WARN("CFH_Estimation_RGB_RATE_33::searchForNeighbors: kdtree.radiusSearch() == 0");
    }

}

void CFH_Estimation_RGB_RATE_33::computePointPFHRGBSignature(pcl::PointCloud<pcl::PointXYZRGB>& cloud,pcl::Indices& indices, int nr_split, Eigen::VectorXf& pfhrgb_histogram)
{
    int h_index;

    // Clear the resultant point histogram
    pfhrgb_histogram.setZero();

    // Factorization constant (直方图数值总和为200)
    float hist_incr = 100.0f / static_cast<float> (3 * indices.size() * (indices.size() - 1) / 2);

    // Iterate over all the points in the neighborhood（所有邻域点的"排列"）
    for (const auto& index_i : indices)
    {
        for (const auto& index_j : indices)
        {
            // Avoid unnecessary returns
            if (index_i == index_j)
                continue;

            // Compute the pair NNi to NNj
            if (!computeRGBPairFeatures(cloud, index_i, index_j,
                pfhrgb_tuple_[0], pfhrgb_tuple_[1], pfhrgb_tuple_[2]))
                continue;

            // Normalize the f1, f2, f3 features and push them in the histogram
            // color ratios are in [-1, 1]
            for (int i = 0; i < 3; ++i)
            {
                float feature_value = nr_split * ((pfhrgb_tuple_[i] + 1.0) * 0.5);
                f_index_[i] = static_cast<int> (std::floor(feature_value));
            }
            for (auto& feature : f_index_)
            {
                feature = std::min(nr_split - 1, std::max(0, feature));
            }

            // Copy into the histogram
            h_index = 0;
            for (int d = 0; d < 3; ++d)
            {
                h_index = d * 11 + f_index_[d];
                pfhrgb_histogram[h_index] += hist_incr;
            }
        }
    }
}

bool CFH_Estimation_RGB_RATE_33::computeRGBPairFeatures(pcl::PointCloud<pcl::PointXYZRGB>& cloud,int p_idx, int q_idx, float& f1, float& f2, float& f3)
{
    Eigen::Vector4i colors1(cloud[p_idx].r, cloud[p_idx].g, cloud[p_idx].b, 0),
        colors2(cloud[q_idx].r, cloud[q_idx].g, cloud[q_idx].b, 0);
    computeRGBPairFeatures(colors1,colors2,f1, f2, f3);
    return (true);
}

bool CFH_Estimation_RGB_RATE_33::computeRGBPairFeatures(const Eigen::Vector4i& colors1,const Eigen::Vector4i& colors2, float& f1, float& f2, float& f3)
{
    // everything before was standard 4D-Darboux frame feature pair
    // now, for the experimental color stuff
    f1 = (colors2[0] != 0) ? static_cast<float> (colors1[0]) / colors2[0] : 1.0f;
    f2 = (colors2[1] != 0) ? static_cast<float> (colors1[1]) / colors2[1] : 1.0f;
    f3 = (colors2[2] != 0) ? static_cast<float> (colors1[2]) / colors2[2] : 1.0f;

    // make sure the ratios are in the [-1, 1] interval
    if (f1 > 1.0f) f1 = -1.0f / f1;
    if (f2 > 1.0f) f2 = -1.0f / f2;
    if (f3 > 1.0f) f3 = -1.0f / f3;

    return (true);
}