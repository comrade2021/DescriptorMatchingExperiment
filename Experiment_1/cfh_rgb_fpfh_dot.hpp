#pragma once

#include "cfh_rgb_fpfh_dot.h"

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/features/pfh_tools.h>

#include <set> // for std::set

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::computePairFeatures(const pcl::PointCloud<PointInT>& cloud, int p_idx, int q_idx, float& f1)
{
    float r1 = cloud[p_idx].r / static_cast<float>(255) - 0.5;
    float g1 = cloud[p_idx].g / static_cast<float>(255) - 0.5;
    float b1 = cloud[p_idx].b / static_cast<float>(255) - 0.5;
    float r2 = cloud[q_idx].r / static_cast<float>(255) - 0.5;
    float g2 = cloud[q_idx].g / static_cast<float>(255) - 0.5;
    float b2 = cloud[q_idx].b / static_cast<float>(255) - 0.5;
    Eigen::Vector4f colors1(r1, g1, b1, 0), colors2(r2, g2, b2, 0);
    colors1.normalize();
    colors2.normalize();

    computeRGBPairFeatures(colors1, colors2, f1);
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1)
{
    f1 = colors1.dot(colors2);
    if (f1 < (-1.0)) f1 = -1;
    if (f1 > 1.0) f1 = 1;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::computePointSPFHSignature(const pcl::PointCloud<PointInT>& cloud, pcl::index_t p_idx, int row, const pcl::Indices& indices, Eigen::MatrixXf& hist_f1)
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
        int h_index = static_cast<int> (std::floor(nr_bins_f1 * ((pfh_tuple[0] + 1.0) * 0.5)));
        if (h_index < 0)           h_index = 0;
        if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
        hist_f1(row, h_index) += hist_incr;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::weightPointSPFHSignature(
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
        weight = 1.0f / dists[idx];

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
pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::computeSPFHSignatures(std::vector<int>& spfh_hist_lookup,
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
pcl::CFH_Estimation_RGB_FPFH_DOT<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut& output)
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