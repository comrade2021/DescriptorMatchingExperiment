//#include "cfh.h"
//#include <pcl/features/pfh_tools.h>
//
//void CFHEstimation::computeFeature(pcl::PointCloud<pcl::PFHRGBSignature250>& output)
//{
//    //resize output
//    output.resize(input_keypoints_->size());//为output分配空间，否则会出现向量下标越界
//
//    /// nr_subdiv^3 for RGB and nr_subdiv^3 for the angular features
//    pfhrgb_histogram_.setZero(2 * nr_subdiv_ * nr_subdiv_ * nr_subdiv_);
//    pfhrgb_tuple_.setZero(7);
//
//    // Iterating over the entire index vector
//    for (std::size_t idx = 0; idx < input_keypoints_->size(); ++idx)
//    {
//        pcl::Indices nn_indices;
//        std::vector<float> nn_dists;
//        searchForNeighbors(idx, search_radius_, nn_indices, nn_dists);
//
//        //// Estimate the PFH signature at each patch
//        computePointPFHRGBSignature(*input_cloud_, *input_normals_, nn_indices, nr_subdiv_, pfhrgb_histogram_);
//
//        std::copy_n(pfhrgb_histogram_.data(), pfhrgb_histogram_.size(), output[idx].histogram);
//    }
//
//}
//
//void CFHEstimation::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) { input_cloud_ = input_cloud; }
//void CFHEstimation::setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals) { input_normals_ = input_normals; }
//void CFHEstimation::setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints) { input_keypoints_ = input_keypoints; }
//void CFHEstimation::setSearchRadius(double search_radius) { search_radius_ = search_radius; }
//
//
////此处有错，输出indices不为空，但数值都是0
//void CFHEstimation::searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances)
//{
//    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//    kdtree.setInputCloud(input_cloud_);
//    // Neighbors within radius search
//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;
//    if (kdtree.radiusSearch((*input_keypoints_)[index], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//    {
//        indices.resize(pointIdxRadiusSearch.size());//清空
//        distances.resize(pointRadiusSquaredDistance.size());//清空
//        for (size_t i = 0; i < indices.size(); i++)
//        {
//            indices.at(i) = pointIdxRadiusSearch.at(i);
//        }
//        for (size_t i = 0; i < indices.size(); i++)
//        {
//            distances.at(i)=pointRadiusSquaredDistance.at(i);
//        }
//    }
//    else
//    {
//        PCL_WARN("CFHEstimation::searchForNeighbors: kdtree.radiusSearch() == 0");
//    }
//
//}
//
//void CFHEstimation::computePointPFHRGBSignature(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::Normal>& normals, pcl::Indices& indices, int nr_split, Eigen::VectorXf& pfhrgb_histogram)
//{
//    int h_index, h_p;
//
//    // Clear the resultant point histogram
//    pfhrgb_histogram.setZero();
//
//    // Factorization constant
//    float hist_incr = 100.0f / static_cast<float> (indices.size() * (indices.size() - 1) / 2);
//
//    // Iterate over all the points in the neighborhood
//    for (const auto& index_i : indices)
//    {
//        for (const auto& index_j : indices)
//        {
//            // Avoid unnecessary returns
//            if (index_i == index_j)
//                continue;
//
//            // Compute the pair NNi to NNj
//            if (!computeRGBPairFeatures(cloud, normals, index_i, index_j,
//                pfhrgb_tuple_[0], pfhrgb_tuple_[1], pfhrgb_tuple_[2], pfhrgb_tuple_[3],
//                pfhrgb_tuple_[4], pfhrgb_tuple_[5], pfhrgb_tuple_[6]))
//                continue;
//
//            // Normalize the f1, f2, f3, f5, f6, f7 features and push them in the histogram
//            // f1值为[-pi,+pi]之间的一个数(atan2（y，x）求的是y/x的反正切)。
//            f_index_[0] = static_cast<int> (std::floor(nr_split * ((pfhrgb_tuple_[0] + M_PI) * d_pi_)));
//            // @TODO: confirm "not to do for i == 3"
//            for (int i = 1; i < 3; ++i)
//            {
//                const float feature_value = nr_split * ((pfhrgb_tuple_[i] + 1.0) * 0.5);
//                f_index_[i] = static_cast<int> (std::floor(feature_value));
//            }
//            // color ratios are in [-1, 1]
//            for (int i = 4; i < 7; ++i)
//            {
//                const float feature_value = nr_split * ((pfhrgb_tuple_[i] + 1.0) * 0.5);
//                f_index_[i] = static_cast<int> (std::floor(feature_value));
//            }
//            for (auto& feature : f_index_)
//            {
//                feature = std::min(nr_split - 1, std::max(0, feature));
//            }
//
//            // Copy into the histogram
//            h_index = 0;
//            h_p = 1;
//            for (int d = 0; d < 3; ++d)
//            {
//                h_index += h_p * f_index_[d];
//                h_p *= nr_split;
//            }
//            pfhrgb_histogram[h_index] += hist_incr;
//
//            // and the colors
//            h_index = 125;
//            h_p = 1;
//            for (int d = 4; d < 7; ++d)
//            {
//                h_index += h_p * f_index_[d];
//                h_p *= nr_split;
//            }
//            pfhrgb_histogram[h_index] += hist_incr;
//        }
//    }
//}
//
//bool CFHEstimation::computeRGBPairFeatures(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::Normal>& normals, int p_idx, int q_idx, float& f1, float& f2, float& f3, float& f4, float& f5, float& f6, float& f7)
//{
//    Eigen::Vector4i colors1(cloud[p_idx].r, cloud[p_idx].g, cloud[p_idx].b, 0),
//        colors2(cloud[q_idx].r, cloud[q_idx].g, cloud[q_idx].b, 0);
//    computeRGBPairFeatures(cloud[p_idx].getVector4fMap(), normals[p_idx].getNormalVector4fMap(),
//        colors1,
//        cloud[q_idx].getVector4fMap(), normals[q_idx].getNormalVector4fMap(),
//        colors2,
//        f1, f2, f3, f4, f5, f6, f7);
//    return (true);
//}
//
//bool CFHEstimation::computeRGBPairFeatures(const Eigen::Vector4f& p1, const Eigen::Vector4f& n1, const Eigen::Vector4i& colors1, const Eigen::Vector4f& p2, const Eigen::Vector4f& n2, const Eigen::Vector4i& colors2, float& f1, float& f2, float& f3, float& f4, float& f5, float& f6, float& f7)
//{
//    Eigen::Vector4f dp2p1 = p2 - p1;
//    dp2p1[3] = 0.0f;
//    f4 = dp2p1.norm();
//
//    if (f4 == 0.0f)
//    {
//        PCL_DEBUG("Euclidean distance between points is 0!\n");
//        f1 = f2 = f3 = f4 = 0.0f;
//        return (false);
//    }
//
//    Eigen::Vector4f n1_copy = n1,
//        n2_copy = n2;
//    n1_copy[3] = n2_copy[3] = 0.0f;
//    float angle1 = n1_copy.dot(dp2p1) / f4;
//
//    f3 = angle1;
//
//    // Create a Darboux frame coordinate system u-v-w
//    // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
//    Eigen::Vector4f v = dp2p1.cross3(n1_copy);
//    v[3] = 0.0f;
//    float v_norm = v.norm();
//    if (v_norm == 0.0f)
//    {
//        PCL_DEBUG("Norm of Delta x U is 0!\n");
//        f1 = f2 = f3 = f4 = 0.0f;
//        return (false);
//    }
//    // Normalize v
//    v /= v_norm;
//
//    Eigen::Vector4f w = n1_copy.cross3(v);
//    // Do not have to normalize w - it is a unit vector by construction
//
//    v[3] = 0.0f;
//    f2 = v.dot(n2_copy);
//    w[3] = 0.0f;
//    // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
//    f1 = std::atan2(w.dot(n2_copy), n1_copy.dot(n2_copy)); // @todo optimize this
//
//    // everything before was standard 4D-Darboux frame feature pair
//    // now, for the experimental color stuff
//    f5 = (colors2[0] != 0) ? static_cast<float> (colors1[0]) / colors2[0] : 1.0f;
//    f6 = (colors2[1] != 0) ? static_cast<float> (colors1[1]) / colors2[1] : 1.0f;
//    f7 = (colors2[2] != 0) ? static_cast<float> (colors1[2]) / colors2[2] : 1.0f;
//
//    // make sure the ratios are in the [-1, 1] interval
//    if (f5 > 1.0f) f5 = -1.0f / f5;
//    if (f6 > 1.0f) f6 = -1.0f / f6;
//    if (f7 > 1.0f) f7 = -1.0f / f7;
//
//    return (true);
//}
