#pragma once
//点对间差异度量方式：一维的曼哈顿距离
#include <pcl/features/feature.h>
#include <array>  // for sRGB_LUT, sXYZ_LUT

namespace pcl
{
    template <typename PointInT, typename PointNT, typename PointOutT = pcl::FPFHSignature33>
    class CFH_Estimation_LAB_FPFH_L1 : public FeatureFromNormals<PointInT, PointNT, PointOutT>
    {
    public:
        using Ptr = shared_ptr<CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT> >;
        using ConstPtr = shared_ptr<const CFH_Estimation_LAB_FPFH_L1<PointInT, PointNT, PointOutT> >;
        using Feature<PointInT, PointOutT>::feature_name_;
        using Feature<PointInT, PointOutT>::getClassName;
        using Feature<PointInT, PointOutT>::indices_;
        using Feature<PointInT, PointOutT>::k_;
        using Feature<PointInT, PointOutT>::search_parameter_;
        using Feature<PointInT, PointOutT>::input_;
        using Feature<PointInT, PointOutT>::surface_;
        using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

        using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

        /** \brief Empty constructor. */
        CFH_Estimation_LAB_FPFH_L1() :
            nr_bins_f1_(33),
            d_pi_(1.0f / (2.0f * static_cast<float> (M_PI)))
        {
            feature_name_ = "CFH_Estimation_LAB_FPFH_L1";
        };

        /*计算两点RGB三元组之间的点积*/
        bool
            computePairFeatures(const pcl::PointCloud<PointInT>& cloud, int p_idx, int q_idx, float& f1);

        /*计算两点RGB三元组之间的点积，由于长度已经规范化，所以结果是向量之间夹角的余弦(cos)，代替原来的pcl::pcl::computePairFeatures */
        bool
            computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1);

        static void
            RGB2CIELAB(unsigned char R, unsigned char G, unsigned char B, float& L, float& A, float& B2);

        static std::array<float, 256> sRGB_LUT;
        static std::array<float, 4000> sXYZ_LUT;

        /** \brief Estimate the SPFH (Simple Point Feature Histograms) individual signatures of the three angular
          * (f1, f2, f3) features for a given point based on its spatial neighborhood of 3D points with normals
          * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
          * \param[in] normals the dataset containing the surface normals at each point in \a cloud
          * \param[in] p_idx the index of the query point (source)
          * \param[in] row the index row in feature histogramms
          * \param[in] indices the k-neighborhood point indices in the dataset
          * \param[out] hist_f1 the resultant SPFH histogram for feature f1
          * \param[out] hist_f2 the resultant SPFH histogram for feature f2
          * \param[out] hist_f3 the resultant SPFH histogram for feature f3
          */
        void
            computePointSPFHSignature(const pcl::PointCloud<PointInT>& cloud, pcl::index_t p_idx, int row, const pcl::Indices& indices, Eigen::MatrixXf& hist_f1);

        /** \brief Weight the SPFH (Simple Point Feature Histograms) individual histograms to create the final FPFH
          * (Fast Point Feature Histogram) for a given point based on its 3D spatial neighborhood
          * \param[in] hist_f1 the histogram feature vector of \a f1 values over the given patch
          * \param[in] hist_f2 the histogram feature vector of \a f2 values over the given patch
          * \param[in] hist_f3 the histogram feature vector of \a f3 values over the given patch
          * \param[in] indices the point indices of p_idx's k-neighborhood in the point cloud
          * \param[in] dists the distances from p_idx to all its k-neighbors
          * \param[out] fpfh_histogram the resultant FPFH histogram representing the feature at the query point
          */
        void
            weightPointSPFHSignature(const Eigen::MatrixXf& hist_f1,
                const pcl::Indices& indices,
                const std::vector<float>& dists,
                Eigen::VectorXf& fpfh_histogram);

        /** \brief Set the number of subdivisions for each angular feature interval.
          * \param[in] nr_bins_f1 number of subdivisions for the first angular feature
          * \param[in] nr_bins_f2 number of subdivisions for the second angular feature
          * \param[in] nr_bins_f3 number of subdivisions for the third angular feature
          * cfh_rgb_fpfh_dot的点对差异度量结果只有一个点积，而不是fpfh的三个角度。
          */
        inline void
            setNrSubdivisions(int nr_bins_f1)
        {
            nr_bins_f1_ = nr_bins_f1;
        }

        /** \brief Get the number of subdivisions for each angular feature interval.
          * \param[out] nr_bins_f1 number of subdivisions for the first angular feature
          * \param[out] nr_bins_f2 number of subdivisions for the second angular feature
          * \param[out] nr_bins_f3 number of subdivisions for the third angular feature
           */
        inline void
            getNrSubdivisions(int& nr_bins_f1)
        {
            nr_bins_f1 = nr_bins_f1_;
        }

    protected:

        /** \brief Estimate the set of all SPFH (Simple Point Feature Histograms) signatures for the input cloud
          * \param[out] spf_hist_lookup a lookup table for all the SPF feature indices
          * \param[out] hist_f1 the resultant SPFH histogram for feature f1
          * \param[out] hist_f2 the resultant SPFH histogram for feature f2
          * \param[out] hist_f3 the resultant SPFH histogram for feature f3
          */
        void
            computeSPFHSignatures(std::vector<int>& spf_hist_lookup,
                Eigen::MatrixXf& hist_f1);

        /** \brief The number of subdivisions for each angular feature interval. */
        int nr_bins_f1_;

        /** \brief Placeholder for the f1 histogram. */
        Eigen::MatrixXf hist_f1_;

        /** \brief Placeholder for a point's FPFH signature. */
        Eigen::VectorXf fpfh_histogram_;

        /** \brief Float constant = 1.0 / (2.0 * M_PI) */
        float d_pi_;

    private:
        /** \brief Estimate the Fast Point Feature Histograms (FPFH) descriptors at a set of points given by
          * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
          * setSearchMethod ()
          * \param[out] output the resultant point cloud model dataset that contains the FPFH feature estimates
          */
        void computeFeature(PointCloudOut& output) override;
    };
}