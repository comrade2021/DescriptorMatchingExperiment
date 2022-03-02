#pragma once

#include "cfh_hsv_fpfh_L2.h"

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/features/pfh_tools.h>

#include <set> // for std::set

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::computePairFeatures(const pcl::PointCloud<PointInT>& cloud, int p_idx, int q_idx, float& f1)
{
    Eigen::Vector4i colors1(cloud[p_idx].r, cloud[p_idx].g, cloud[p_idx].b, 0),
        colors2(cloud[q_idx].r, cloud[q_idx].g, cloud[q_idx].b, 0);
    Eigen::Vector4f hsv1;
    Eigen::Vector4f hsv2;
    RGBtoHSV(colors1, hsv1);
    RGBtoHSV(colors2, hsv2);

    hsv1[0] = hsv1[0] / static_cast<float>(360);//HSV中H的范围是0-360，所以需要归一化
    hsv2[0] = hsv2[0] / static_cast<float>(360);

    if ((hsv1[0] < 0)) hsv1[0] = 0;//处理异常值
    if ((hsv1[0] > 1)) hsv1[0] = 1;
    if ((hsv1[1] < 0)) hsv1[1] = 0;
    if ((hsv1[1] > 1)) hsv1[1] = 1;
    if ((hsv1[2] < 0)) hsv1[2] = 0;
    if ((hsv1[2] > 1)) hsv1[2] = 1;

    if ((hsv2[0] < 0)) hsv2[0] = 0;
    if ((hsv2[0] > 1)) hsv2[0] = 1;
    if ((hsv2[1] < 0)) hsv2[1] = 0;
    if ((hsv2[1] > 1)) hsv2[1] = 1;
    if ((hsv2[2] < 0)) hsv2[2] = 0;
    if ((hsv2[2] > 1)) hsv2[2] = 1;

    //hsv1[0] -= 0.5;//将特征向量移动到坐标系中心
    //hsv1[1] -= 0.5;
    //hsv1[2] -= 0.5;
    //hsv2[0] -= 0.5;
    //hsv2[1] -= 0.5;
    //hsv2[2] -= 0.5;

    //范围在0-100
    hsv1[0] *= 100;
    hsv1[1] *= 100;
    hsv1[2] *= 100;
    hsv2[0] *= 100;
    hsv2[1] *= 100;
    hsv2[2] *= 100;

    float r1 = hsv1[0];
    float g1 = hsv1[1];
    float b1 = hsv1[2];
    float r2 = hsv2[0];
    float g2 = hsv2[1];
    float b2 = hsv2[2];

    float L2 = sqrt((r1 - r2) * (r1 - r2) + (g1 - g2) * (g1 - g2) + (b1 - b2) * (b1 - b2));

    float odist = L2 / static_cast<float>(sqrt(3) * 100);//范围：0 - 1

    if (odist < 0)
    {
        f1 = 0;
    }
    if (odist > 1.0)
    {
        f1 = 1.0;
    }

    f1 = odist;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1)
{
    return (true);
}

template<typename PointInT, typename PointNT, typename PointOutT>
inline void pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::RGBtoHSV(const Eigen::Vector4i& in, Eigen::Vector4f& out)
{
    const unsigned char max = std::max(in[0], std::max(in[1], in[2]));
    const unsigned char min = std::min(in[0], std::min(in[1], in[2]));

    out[2] = static_cast <float> (max) / 255.f;//V 范围0-1

    if (max == 0) // division by zero
    {
        out[1] = 0.f;
        out[0] = 0.f; // h = -1.f;
        return;
    }

    const float diff = static_cast <float> (max - min);
    out[1] = diff / static_cast <float> (max);//S 范围0-1

    if (min == max) // diff == 0 -> division by zero
    {
        out[0] = 0;
        return;
    }

    if (max == in[0]) out[0] = 60.f * (static_cast <float> (in[1] - in[2]) / diff);
    else if (max == in[1]) out[0] = 60.f * (2.f + static_cast <float> (in[2] - in[0]) / diff);
    else                  out[0] = 60.f * (4.f + static_cast <float> (in[0] - in[1]) / diff); // max == b

    if (out[0] < 0.f) out[0] += 360.f;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::computePointSPFHSignature(const pcl::PointCloud<PointInT>& cloud, pcl::index_t p_idx, int row, const pcl::Indices& indices, Eigen::MatrixXf& hist_f1)
{
    Eigen::Vector4f pfh_tuple;
    // Get the number of bins from the histograms size
    // @TODO: use arrays
    int nr_bins_f1 = static_cast<int> (hist_f1.cols());

    // Factorization constant
    float hist_incr = 300.0f / static_cast<float>(indices.size() - 1);

    // Iterate over all the points in the neighborhood
    for (const auto& index : indices)
    {
        // Avoid unnecessary returns
        if (p_idx == index)
            continue;

        // Compute the pair P to NNi
        if (!computePairFeatures(cloud, p_idx, index, pfh_tuple[0]))
            continue;

        // Normalize the f1, f2, f3 features and push them in the histogram
        int h_index = static_cast<int> (std::floor(nr_bins_f1 * (pfh_tuple[0])));
        if (h_index < 0)           h_index = 0;
        if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
        hist_f1(row, h_index) += hist_incr;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::weightPointSPFHSignature(
    const Eigen::MatrixXf& hist_f1, const pcl::Indices& indices, const std::vector<float>& dists, Eigen::VectorXf& fpfh_histogram)
{
    assert(indices.size() == dists.size());
    // @TODO: use arrays
    double sum_f1 = 0.0;
    float weight = 0.0, val_f1;

    // Get the number of bins from the histograms size
    const auto nr_bins_f1 = hist_f1.cols();

    // Clear the histogram
    fpfh_histogram.setZero(nr_bins_f1);

    // Use the entire patch
    for (std::size_t idx = 0; idx < indices.size(); ++idx)
    {
        // Minus the query point itself
        if (dists[idx] == 0)
            continue;

        // Standard weighting function used
        weight = 1.0f / sqrt(dists[idx]);

        // Weight the SPFH of the query point with the SPFH of its neighbors
        for (Eigen::MatrixXf::Index f1_i = 0; f1_i < nr_bins_f1; ++f1_i)
        {
            val_f1 = hist_f1(indices[idx], f1_i) * weight;
            sum_f1 += val_f1;
            fpfh_histogram[f1_i] += val_f1;
        }
    }

    if (sum_f1 != 0)
        sum_f1 = 300.0 / sum_f1;           // histogram values sum up to 300


    // Adjust final FPFH values
    const auto denormalize_with = [](auto factor)
    {
        return [=](const auto& data) { return data * factor; };
    };

    auto last = fpfh_histogram.data();
    std::transform(last, last + nr_bins_f1, last, denormalize_with(sum_f1));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::computeSPFHSignatures(std::vector<int>& spfh_hist_lookup,
    Eigen::MatrixXf& hist_f1)
{
    // Allocate enough space to hold the NN search results
    // \note This resize is irrelevant for a radiusSearch ().
    pcl::Indices nn_indices(k_);
    std::vector<float> nn_dists(k_);

    std::set<int> spfh_indices;// 集合，用来存储所有需要计算spfh的点的索引
    spfh_hist_lookup.resize(surface_->size());

    // Build a list of (unique) indices for which we will need to compute SPFH signatures
    // (We need an SPFH signature for every point that is a neighbor of any point in input_[indices_])
    if (surface_ != input_ ||
        indices_->size() != surface_->size())
    {
        for (const auto& p_idx : *indices_)
        {
            if (this->searchForNeighbors(p_idx, search_parameter_, nn_indices, nn_dists) == 0)
                continue;

            spfh_indices.insert(nn_indices.begin(), nn_indices.end());
        }
    }
    else
    {
        // Special case: When a feature must be computed at every point, there is no need for a neighborhood search
        for (std::size_t idx = 0; idx < indices_->size(); ++idx)
            spfh_indices.insert(static_cast<int> (idx));
    }

    // Initialize the arrays that will store the SPFH signatures
    // 准备空间，存放计算的spfh特征直方图
    std::size_t data_size = spfh_indices.size();
    hist_f1.setZero(data_size, nr_bins_f1_);

    // Compute SPFH signatures for every point that needs them
    std::size_t i = 0;
    for (const auto& p_idx : spfh_indices)
    {
        // Find the neighborhood around p_idx
        if (this->searchForNeighbors(*surface_, p_idx, search_parameter_, nn_indices, nn_dists) == 0)
            continue;

        // Estimate the SPFH signature around p_idx
        computePointSPFHSignature(*surface_, p_idx, i, nn_indices, hist_f1);

        // Populate a lookup table for converting a point index to its corresponding row in the spfh_hist_* matrices
        // 将所有计算了spfh的点的点云索引和存放所有spfh的hist_f1对应，用于查找某点的spfh
        spfh_hist_lookup[p_idx] = i;
        i++;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_HSV_FPFH_L2<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut& output)
{
    // Allocate enough space to hold the NN search results
    // \note This resize is irrelevant for a radiusSearch ().
    pcl::Indices nn_indices(k_);
    std::vector<float> nn_dists(k_);

    std::vector<int> spfh_hist_lookup;
    computeSPFHSignatures(spfh_hist_lookup, hist_f1_);

    output.is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (input_->is_dense)
    {
        // Iterate over the entire index vector
        for (std::size_t idx = 0; idx < indices_->size(); ++idx)
        {
            if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
            {
                for (Eigen::Index d = 0; d < fpfh_histogram_.size(); ++d)
                    output[idx].histogram[d] = std::numeric_limits<float>::quiet_NaN();

                output.is_dense = false;
                continue;
            }

            // ... and remap the nn_indices values so that they represent row indices in the spfh_hist_* matrices
            // instead of indices into surface_->points
            for (auto& nn_index : nn_indices)
                nn_index = spfh_hist_lookup[nn_index];

            // Compute the FPFH signature (i.e. compute a weighted combination of local SPFH signatures) ...
            weightPointSPFHSignature(hist_f1_, nn_indices, nn_dists, fpfh_histogram_);

            // ...and copy it into the output cloud
            std::copy_n(fpfh_histogram_.data(), fpfh_histogram_.size(), output[idx].histogram);
        }
    }
    else
    {
        // Iterate over the entire index vector
        for (std::size_t idx = 0; idx < indices_->size(); ++idx)
        {
            if (!isFinite((*input_)[(*indices_)[idx]]) ||
                this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
            {
                for (Eigen::Index d = 0; d < fpfh_histogram_.size(); ++d)
                    output[idx].histogram[d] = std::numeric_limits<float>::quiet_NaN();

                output.is_dense = false;
                continue;
            }

            // ... and remap the nn_indices values so that they represent row indices in the spfh_hist_* matrices
            // instead of indices into surface_->points
            for (auto& nn_index : nn_indices)
                nn_index = spfh_hist_lookup[nn_index];

            // Compute the FPFH signature (i.e. compute a weighted combination of local SPFH signatures) ...
            weightPointSPFHSignature(hist_f1_, nn_indices, nn_dists, fpfh_histogram_);

            // ...and copy it into the output cloud
            std::copy_n(fpfh_histogram_.data(), fpfh_histogram_.size(), output[idx].histogram);
        }
    }
}