#pragma once
#include <pcl/point_types.h>
#include <pcl/features/pfhrgb.h>
//#include <pcl/features/feature.h>

class CFHEstimation
{
public:
	void computeFeature(pcl::PointCloud<pcl::PFHRGBSignature250> &output);//计算11+11+11维拼接而得的色彩直方图特征

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
    /// 依据给定的邻域点集计算某一点的特征直方图。
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="normals">点云法线</param>
    /// <param name="indices">邻域点在点云中的索引</param>
    /// <param name="nr_split">每个特征的细分区间数量，等于nr_subdiv_</param>
    /// <param name="pfhrgb_histogram">输出的特征直方图</param>
    void computePointPFHRGBSignature(pcl::PointCloud<pcl::PointXYZRGB>& cloud,pcl::PointCloud<pcl::Normal>& normals, pcl::Indices& indices, int nr_split, Eigen::VectorXf& pfhrgb_histogram);

    /// <summary>
    /// 计算两个点之间的pfhrgb特征，三个角度+一个距离+三个色彩比值，输入参数为点云中点对的索引
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="normals"></param>
    /// <param name="p_idx">p点在cloud中的索引</param>
    /// <param name="q_idx">q点在cloud中的索引</param>
    /// <param name="f1"></param>
    /// <param name="f2"></param>
    /// <param name="f3"></param>
    /// <param name="f4"></param>
    /// <param name="f5"></param>
    /// <param name="f6"></param>
    /// <param name="f7"></param>
    bool computeRGBPairFeatures(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::Normal>& normals,
        int p_idx, int q_idx,
        float& f1, float& f2, float& f3, float& f4, float& f5, float& f6, float& f7);

    /// <summary>
    /// 计算两个点之间的pfhrgb特征，三个角度+一个距离+三个色彩比值，输入参数为点对的坐标值等
    /// </summary>
    /// <param name="p1"></param>
    /// <param name="n1"></param>
    /// <param name="colors1"></param>
    /// <param name="p2"></param>
    /// <param name="n2"></param>
    /// <param name="colors2"></param>
    /// <param name="f1"></param>
    /// <param name="f2"></param>
    /// <param name="f3"></param>
    /// <param name="f4"></param>
    /// <param name="f5"></param>
    /// <param name="f6"></param>
    /// <param name="f7"></param>
    /// <returns></returns>
    bool computeRGBPairFeatures(const Eigen::Vector4f& p1, const Eigen::Vector4f& n1, const Eigen::Vector4i& colors1,
        const Eigen::Vector4f& p2, const Eigen::Vector4f& n2, const Eigen::Vector4i& colors2,
        float& f1, float& f2, float& f3, float& f4, float& f5, float& f6, float& f7);

private:
    /** \brief The number of subdivisions for each angular feature interval. */
    int nr_subdiv_=5;

    /** \brief Placeholder for a point's PFHRGB signature. */
    Eigen::VectorXf pfhrgb_histogram_;

    /** \brief Placeholder for a PFHRGB 7-tuple. */
    Eigen::VectorXf pfhrgb_tuple_;

    /** \brief Placeholder for a histogram index. */
    int f_index_[7];//指示每个角度值应该放入5个角度区间的哪一个。

    /** \brief Float constant = 1.0 / (2.0 * M_PI) */
    //float d_pi_;
    float d_pi_ = (1.0f / (2.0f * static_cast<float> (M_PI)));


    //other parameter
    int k_;
    double search_radius_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_keypoints_;
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_;
};
