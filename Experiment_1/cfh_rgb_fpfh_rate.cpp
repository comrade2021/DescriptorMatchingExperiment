#include "cfh_rgb_fpfh_rate.h"
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <set>

void CFH_Estimation_RGB_FPFH_RATE::computeFeature(pcl::PointCloud<pcl::FPFHSignature33>& output)
{
    //resize output
    output.resize(input_keypoints_->size());//为output分配空间，否则会出现向量下标越界

	std::vector<int> spfh_hist_lookup;
	computeSPFHSignatures(spfh_hist_lookup, hist_f1_, hist_f2_, hist_f3_);

	output.is_dense = true;
	// Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
	if (input_cloud_->is_dense)
	{
        std::cout << "----------### 1" << std::endl;

		// Iterate over the entire index vector
		for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
		{
            std::cout << "----------### 1 -----"<<idx << std::endl;

			pcl::Indices nn_indices;
			std::vector<float> nn_dists;
			if (searchForNeighbors(idx, search_radius_, nn_indices, nn_dists) == 0)
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
			weightPointSPFHSignature(hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, fpfh_histogram_);
            std::cout << "----------### 1  weightPointSPFHSignature " << std::endl;

			// ...and copy it into the output cloud
			std::copy_n(fpfh_histogram_.data(), fpfh_histogram_.size(), output[idx].histogram);
            std::cout << "----------### 1  copy_n " << std::endl;

		}
        std::cout << "----------### 1 end" << std::endl;

	}
	else
	{
        std::cout << "----------### 2" << std::endl;
		// Iterate over the entire index vector
		for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
		{
			pcl::Indices nn_indices;
			std::vector<float> nn_dists;
			if (!pcl::isFinite((*input_keypoints_)[idx]) ||
				searchForNeighbors(idx, search_radius_, nn_indices, nn_dists) == 0)
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
			weightPointSPFHSignature(hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, fpfh_histogram_);

			// ...and copy it into the output cloud
			std::copy_n(fpfh_histogram_.data(), fpfh_histogram_.size(), output[idx].histogram);
		}
	}
}

void CFH_Estimation_RGB_FPFH_RATE::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) { input_cloud_ = input_cloud; }
void CFH_Estimation_RGB_FPFH_RATE::setSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface) { surface_ = surface; }
void CFH_Estimation_RGB_FPFH_RATE::setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals) { input_normals_ = input_normals; }
void CFH_Estimation_RGB_FPFH_RATE::setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints) { input_keypoints_ = input_keypoints; }
void CFH_Estimation_RGB_FPFH_RATE::setSearchRadius(double search_radius) { search_radius_ = search_radius; }

int CFH_Estimation_RGB_FPFH_RATE::searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(input_cloud_);
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int re = kdtree.radiusSearch((*input_keypoints_)[index], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (re>0)
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
    return(re);
}
int CFH_Estimation_RGB_FPFH_RATE::searchForNeighbors(pcl::PointCloud<pcl::PointXYZRGB> &search_cloud, std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(input_cloud_);
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int re = kdtree.radiusSearch((search_cloud)[index], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (re > 0)
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
    return(re);
}

bool CFH_Estimation_RGB_FPFH_RATE::computePairFeatures(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<pcl::Normal>& normals, int p_idx, int q_idx, float& f1, float& f2, float& f3, float& f4)
{
    Eigen::Vector4i colors1(cloud[p_idx].r, cloud[p_idx].g, cloud[p_idx].b, 0),
        colors2(cloud[q_idx].r, cloud[q_idx].g, cloud[q_idx].b, 0);
    //f4
    Eigen::Vector4f dp2p1 = cloud[q_idx].getVector4fMap() - cloud[p_idx].getVector4fMap();
    dp2p1[3] = 0.0f;
    float distence = dp2p1.norm();

    computePairFeatures(colors1, colors2, f1, f2, f3, distence);
    return (true);
}

bool CFH_Estimation_RGB_FPFH_RATE::computePairFeatures(const Eigen::Vector4i& colors1, const Eigen::Vector4i& colors2, float& f1, float& f2, float& f3, float& f4)
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

void CFH_Estimation_RGB_FPFH_RATE::computePointSPFHSignature(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<pcl::Normal>& normals, pcl::index_t p_idx, int row, const pcl::Indices& indices, Eigen::MatrixXf& hist_f1, Eigen::MatrixXf& hist_f2, Eigen::MatrixXf& hist_f3)
{
    Eigen::Vector4f pfh_tuple;
    // Get the number of bins from the histograms size
    // @TODO: use arrays
    int nr_bins_f1 = static_cast<int> (hist_f1.cols());
    int nr_bins_f2 = static_cast<int> (hist_f2.cols());
    int nr_bins_f3 = static_cast<int> (hist_f3.cols());

    // Factorization constant
    float hist_incr = 100.0f / static_cast<float>(indices.size() - 1);

    // Iterate over all the points in the neighborhood
    for (const auto& index : indices)
    {
        // Avoid unnecessary returns
        if (p_idx == index)
            continue;

        // Compute the pair P to NNi
        if (!computePairFeatures(cloud, normals, p_idx, index, pfh_tuple[0], pfh_tuple[1], pfh_tuple[2], pfh_tuple[3]))
            continue;

        // Normalize the f1, f2, f3 features and push them in the histogram
        int h_index = static_cast<int> (std::floor(nr_bins_f1 * ((pfh_tuple[0] + 1.0) * 0.5)));
        if (h_index < 0)           h_index = 0;
        if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
        hist_f1(row, h_index) += hist_incr;

        h_index = static_cast<int> (std::floor(nr_bins_f2 * ((pfh_tuple[1] + 1.0) * 0.5)));
        if (h_index < 0)           h_index = 0;
        if (h_index >= nr_bins_f2) h_index = nr_bins_f2 - 1;
        hist_f2(row, h_index) += hist_incr;

        h_index = static_cast<int> (std::floor(nr_bins_f3 * ((pfh_tuple[2] + 1.0) * 0.5)));
        if (h_index < 0)           h_index = 0;
        if (h_index >= nr_bins_f3) h_index = nr_bins_f3 - 1;
        hist_f3(row, h_index) += hist_incr;
    }
}

void CFH_Estimation_RGB_FPFH_RATE::weightPointSPFHSignature(const Eigen::MatrixXf& hist_f1, const Eigen::MatrixXf& hist_f2, const Eigen::MatrixXf& hist_f3, const pcl::Indices& indices, const std::vector<float>& dists, Eigen::VectorXf& fpfh_histogram)
{
    assert(indices.size() == dists.size());
    // @TODO: use arrays
    double sum_f1 = 0.0, sum_f2 = 0.0, sum_f3 = 0.0;
    float weight = 0.0, val_f1, val_f2, val_f3;

    // Get the number of bins from the histograms size
    const auto nr_bins_f1 = hist_f1.cols();
    const auto nr_bins_f2 = hist_f2.cols();
    const auto nr_bins_f3 = hist_f3.cols();
    const auto nr_bins_f12 = nr_bins_f1 + nr_bins_f2;

    // Clear the histogram
    fpfh_histogram.setZero(nr_bins_f1 + nr_bins_f2 + nr_bins_f3);

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

        for (Eigen::MatrixXf::Index f2_i = 0; f2_i < nr_bins_f2; ++f2_i)
        {
            val_f2 = hist_f2(indices[idx], f2_i) * weight;
            sum_f2 += val_f2;
            fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
        }

        for (Eigen::MatrixXf::Index f3_i = 0; f3_i < nr_bins_f3; ++f3_i)
        {
            val_f3 = hist_f3(indices[idx], f3_i) * weight;
            sum_f3 += val_f3;
            fpfh_histogram[f3_i + nr_bins_f12] += val_f3;
        }
    }

    if (sum_f1 != 0)
        sum_f1 = 100.0 / sum_f1;           // histogram values sum up to 100
    if (sum_f2 != 0)
        sum_f2 = 100.0 / sum_f2;           // histogram values sum up to 100
    if (sum_f3 != 0)
        sum_f3 = 100.0 / sum_f3;           // histogram values sum up to 100

      // Adjust final FPFH values
    const auto denormalize_with = [](auto factor)
    {
        return [=](const auto& data) { return data * factor; };
    };

    auto last = fpfh_histogram.data();
    last = std::transform(last, last + nr_bins_f1, last, denormalize_with(sum_f1));
    last = std::transform(last, last + nr_bins_f2, last, denormalize_with(sum_f2));
    std::transform(last, last + nr_bins_f3, last, denormalize_with(sum_f3));
}

void CFH_Estimation_RGB_FPFH_RATE::computeSPFHSignatures(std::vector<int>& spfh_hist_lookup, Eigen::MatrixXf& hist_f1, Eigen::MatrixXf& hist_f2, Eigen::MatrixXf& hist_f3)
{
    std::set<int> spfh_indices;//指示需要计算spfh的点索引，因为需要计算SPFH的点包括：特征点与它的所有搜索半径内的近邻点
    spfh_hist_lookup.resize(surface_->size());

    // Build a list of (unique) indices for which we will need to compute SPFH signatures
    // (We need an SPFH signature for every point that is a neighbor of any point in input_[indices_])
    // 判断是否将输入点云直接作为关键点云
    if (surface_ != input_cloud_ ||
        input_keypoints_->size() != surface_->size())
    {
        for (std::size_t p_idx = 0; p_idx < input_keypoints_->size(); ++p_idx)
        {
            pcl::Indices nn_indices;
            std::vector<float> nn_dists;
            if (searchForNeighbors(p_idx, search_radius_, nn_indices, nn_dists) == 0)
                continue;

            spfh_indices.insert(nn_indices.begin(), nn_indices.end());
        }
    }
    else
    {
        // Special case: When a feature must be computed at every point, there is no need for a neighborhood search
        for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
            spfh_indices.insert(static_cast<int> (idx));
    }

    // Initialize the arrays that will store the SPFH signatures
    std::size_t data_size = spfh_indices.size();
    hist_f1.setZero(data_size, nr_bins_f1_);
    hist_f2.setZero(data_size, nr_bins_f2_);
    hist_f3.setZero(data_size, nr_bins_f3_);

    // Compute SPFH signatures for every point that needs them
    std::size_t i = 0;
    for (const auto& p_idx : spfh_indices)
    {
        pcl::Indices nn_indices;
        std::vector<float> nn_dists;
        // Find the neighborhood around p_idx
        if (searchForNeighbors(*surface_, p_idx, search_radius_, nn_indices, nn_dists) == 0)
            continue;

        // Estimate the SPFH signature around p_idx
        computePointSPFHSignature(*surface_, *input_normals_, p_idx, i, nn_indices, hist_f1, hist_f2, hist_f3);
        std::cout << i << std::endl;
        // Populate a lookup table for converting a point index to its corresponding row in the spfh_hist_* matrices
        spfh_hist_lookup[p_idx] = i;
        i++;
    }
    std::cout << "----------end computeSPFHSignatures" << std::endl;

}