#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>

/// 色彩直方图计算 cfh_rgb_fpfh_rate
/// 色彩空间：RGB
/// 度量方式：点对的RGB三元组各通道的比值，点对结构与fpfh一样，加权方式也和fpfh一样。
/// 维度：33维
class CFH_Estimation_RGB_FPFH_RATE
{
public:
    void computeFeature(pcl::PointCloud<pcl::FPFHSignature33>& output);
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void setSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface);
    void setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals);
    void setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints);
    void setSearchRadius(double search_radius);
    int searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances);
    int searchForNeighbors(pcl::PointCloud<pcl::PointXYZRGB> &search_cloud,std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances);
    
    inline void setNrSubdivisions(int nr_bins_f1, int nr_bins_f2, int nr_bins_f3)
    {
        nr_bins_f1_ = nr_bins_f1;
        nr_bins_f2_ = nr_bins_f2;
        nr_bins_f3_ = nr_bins_f3;
    }


	bool computePairFeatures(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<pcl::Normal>& normals,int p_idx, int q_idx, float& f1, float& f2, float& f3, float& f4);
    bool computePairFeatures(const Eigen::Vector4i& colors1, const Eigen::Vector4i& colors2, float& f1, float& f2, float& f3, float& f4);

	void computePointSPFHSignature(
        const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
        const pcl::PointCloud<pcl::Normal>& normals,
        pcl::index_t p_idx, int row,
        const pcl::Indices& indices,
        Eigen::MatrixXf& hist_f1, 
        Eigen::MatrixXf& hist_f2, 
        Eigen::MatrixXf& hist_f3);

	void weightPointSPFHSignature(
		const Eigen::MatrixXf& hist_f1,
		const Eigen::MatrixXf& hist_f2,
		const Eigen::MatrixXf& hist_f3,
		const pcl::Indices& indices,
		const std::vector<float>& dists,
		Eigen::VectorXf& fpfh_histogram);

	void computeSPFHSignatures(
        std::vector<int>& spfh_hist_lookup,
        Eigen::MatrixXf& hist_f1, 
        Eigen::MatrixXf& hist_f2, 
        Eigen::MatrixXf& hist_f3);

private:
    /** \brief The number of subdivisions for each angular feature interval. */
    int nr_bins_f1_=11, nr_bins_f2_ = 11, nr_bins_f3_ = 11;

    /** \brief Placeholder for the f1 histogram. */
    Eigen::MatrixXf hist_f1_;

    /** \brief Placeholder for the f2 histogram. */
    Eigen::MatrixXf hist_f2_;

    /** \brief Placeholder for the f3 histogram. */
    Eigen::MatrixXf hist_f3_;

    /** \brief Placeholder for a point's FPFH signature. */
    Eigen::VectorXf fpfh_histogram_;

    /** \brief Float constant = 1.0 / (2.0 * M_PI) */
    float d_pi_= 1.0f / (2.0f * static_cast<float> (M_PI));

    //other parameter
    int k_;
    double search_radius_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_;
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_;
};