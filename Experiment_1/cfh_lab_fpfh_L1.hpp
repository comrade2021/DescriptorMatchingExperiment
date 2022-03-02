#pragma once

#include "cfh_lab_fpfh_L1.h"

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/features/pfh_tools.h>
#include <pcl/common/colors.h>
#include <set> // for std::set

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::computePairFeatures(const pcl::PointCloud<PointInT>& cloud, int p_idx, int q_idx, float& f1)
{
    unsigned char red_1 = cloud[p_idx].r;
    unsigned char green_1 = cloud[p_idx].g;
    unsigned char blue_1 = cloud[p_idx].b;
    unsigned char red_2 = cloud[q_idx].r;
    unsigned char green_2 = cloud[q_idx].g;
    unsigned char blue_2 = cloud[q_idx].b;

    float L1, a1, b1;
    float L2, a2, b2;

    RGB2CIELAB(red_1, green_1, blue_1, L1, a1, b1);
    RGB2CIELAB(red_2, green_2, blue_2, L2, a2, b2);

    L1 /= 100.0f;
    a1 /= 120.0f;
    b1 /= 120.0f;   //normalized LAB components (0<L<1, -1<a<1, -1<b<1)
    L2 /= 100.0f;
    a2 /= 120.0f;
    b2 /= 120.0f;   //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

    //0-1
    a1 = (a1 + 1.0) / 2.0;
    b1 = (b1 + 1.0) / 2.0;
    a2 = (a2 + 1.0) / 2.0;
    b2 = (b2 + 1.0) / 2.0;

    //处理异常值
    Eigen::Vector4f lab1(L1, a1, b1, 0),lab2(L2, a2, b2, 0);
    if ((lab1[0] < 0)) lab1[0] = 0;
    if ((lab1[0] > 1)) lab1[0] = 1;
    if ((lab1[1] < 0)) lab1[1] = 0;
    if ((lab1[1] > 1)) lab1[1] = 1;
    if ((lab1[2] < 0)) lab1[2] = 0;
    if ((lab1[2] > 1)) lab1[2] = 1;

    if ((lab2[0] < 0)) lab2[0] = 0;
    if ((lab2[0] > 1)) lab2[0] = 1;
    if ((lab2[1] < 0)) lab2[1] = 0;
    if ((lab2[1] > 1)) lab2[1] = 1;
    if ((lab2[2] < 0)) lab2[2] = 0;
    if ((lab2[2] > 1)) lab2[2] = 1;

    //范围在0-100
    lab1[0] *= 100;
    lab1[1] *= 100;
    lab1[2] *= 100;
    lab2[0] *= 100;
    lab2[1] *= 100;
    lab2[2] *= 100;

    //计算L1距离
    float r11 = lab1[0];
    float g11 = lab1[1];
    float b11 = lab1[2];
    float r22 = lab2[0];
    float g22 = lab2[1];
    float b22 = lab2[2];

    float L11 = abs(r11 - r22) + abs(g11 - g22) + abs(b11 - b22);

    float mdist = L11 / static_cast<float>(100 + 100 + 100);//范围：0 - 1
    if (mdist < 0)
    {
        f1 = 0;
    }
    if (mdist > 1.0)
    {
        f1 = 1.0;
    }

    f1 = mdist;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
inline bool pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1)
{
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
std::array<float, 256>
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::sRGB_LUT = pcl::RGB2sRGB_LUT<float, 8>();

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
std::array<float, 4000>
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::sXYZ_LUT = pcl::XYZ2LAB_LUT<float, 4000>();

template<typename PointInT, typename PointNT, typename PointOutT>
inline void pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::RGB2CIELAB(unsigned char R, unsigned char G, unsigned char B, float& L, float& A, float& B2)
{
    float fr = sRGB_LUT[R];
    float fg = sRGB_LUT[G];
    float fb = sRGB_LUT[B];

    // Use white = D65
    const float x = fr * 0.412453f + fg * 0.357580f + fb * 0.180423f;
    const float y = fr * 0.212671f + fg * 0.715160f + fb * 0.072169f;
    const float z = fr * 0.019334f + fg * 0.119193f + fb * 0.950227f;

    float vx = x / 0.95047f;
    float vy = y;
    float vz = z / 1.08883f;

    vx = sXYZ_LUT[int(vx * 4000)];
    vy = sXYZ_LUT[int(vy * 4000)];
    vz = sXYZ_LUT[int(vz * 4000)];

    L = 116.0f * vy - 16.0f;
    if (L > 100)
        L = 100.0f;

    A = 500.0f * (vx - vy);
    if (A > 120)
        A = 120.0f;
    else if (A < -120)
        A = -120.0f;

    B2 = 200.0f * (vy - vz);
    if (B2 > 120)
        B2 = 120.0f;
    else if (B2 < -120)
        B2 = -120.0f;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::computePointSPFHSignature(const pcl::PointCloud<PointInT>& cloud, pcl::index_t p_idx, int row, const pcl::Indices& indices, Eigen::MatrixXf& hist_f1)
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
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::weightPointSPFHSignature(
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
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::computeSPFHSignatures(std::vector<int>& spfh_hist_lookup,
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
pcl::CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut& output)
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