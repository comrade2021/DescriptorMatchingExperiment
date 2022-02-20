#pragma once
#include <pcl/point_types.h>
#include <pcl/features/pfhrgb.h>

/// 色彩直方图计算 cfh_rgb_pfh_dot
/// 色彩空间：RGB
/// 度量方式：点对的RGB三元组的点积，由于规范化，实际上就是它们的cos，点对结构与pfhrgb一样
/// 维度：nr_subdiv_=33维
class CFH_Estimation_RGB_PFH_DOT
{
public:
    void computeFeature(pcl::PointCloud<pcl::FPFHSignature33>& output);
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr input_normals);
    void setInputKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints);
    void setSearchRadius(double search_radius);

    /// <summary>
    /// 计算某一关键点的所有邻域点（半径搜索）
    /// </summary>
    /// <param name="index">"查询点"在关键点云中的索引</param>
    /// <param name="radius">搜索半径</param>
    /// <param name="indices">[输出]所有邻域点的索引（搜索点云为输入点云input_cloud_，而不是关键点云input_keypoints_）</param>
    /// <param name="distances">[输出]查询点到每个邻域点的距离</param>
    void searchForNeighbors(std::size_t index, double radius, pcl::Indices& indices, std::vector<float>& distances);

    /// <summary>
    /// 依据给定的邻域点集计算某一点的色彩特征直方图
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="indices">邻域点在点云中的索引</param>
    /// <param name="nr_split">每个特征的细分区间数量，等于nr_subdiv_</param>
    /// <param name="cfh_histogram">输出的特征直方图</param>
    void computePointSignature(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::Indices& indices, int nr_split, Eigen::VectorXf& cfh_histogram);

    /// <summary>
    /// 计算两点之间的色彩特征。输入参数是点云及点对的索引。
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="p_idx"></param>
    /// <param name="q_idx"></param>
    /// <param name="f1"></param>
    /// <param name="f2"></param>
    /// <param name="f3"></param>
    /// <returns></returns>
    bool computeRGBPairFeatures(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int p_idx, int q_idx, float& f1);

    /// <summary>
    /// 计算两点之间的色彩特征（两点RGB向量的点积）
    /// </summary>
    /// <param name="colors1"></param>
    /// <param name="colors2"></param>
    /// <param name="f1"></param>
    /// <returns></returns>
    bool computeRGBPairFeatures(const Eigen::Vector4f& colors1, const Eigen::Vector4f& colors2, float& f1);

private:
    /** \brief The number of subdivisions for each angular feature interval. */
    int nr_subdiv_ = 33;

    /** \brief Placeholder for a point's PFHRGB signature. */
    Eigen::VectorXf cfh_histogram_;

    /** \brief Placeholder for a PFHRGB 7-tuple. */
    Eigen::VectorXf cfh_tuple_;

    /** \brief Placeholder for a histogram index. */
    int f_index_[1];//指示每个值应该放入33个角度区间的哪一个。

    /** \brief Float constant = 1.0 / (2.0 * M_PI) */
    float d_pi_ = (1.0f / (2.0f * static_cast<float> (M_PI)));


    //other parameter
    int k_;
    double search_radius_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints_;
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_;
};
