#pragma once
#include <string>
#include <vector>
#include "my_point_types.h"


class Experiment
{
public:
	void setModelNumber(int model_number);
	void setDatasetName(std::string dataset_name);
	void setIniPath(std::string ini_path);
	void writePRToTXT(std::string file_path, std::vector<double>& precision, std::vector<double>& recall, std::vector<std::string>& features_name);
	void setAlphaUpperLimit(double alpha_upper_limit);
	void setOutputTXTPath(std::string output_txt_path);

	/// <summary>
	/// 根据某一个α值计算所有描述子的PR曲线，"2"表示一个场景对应两个模型的实验配置
	/// </summary>
	/// <param name="alpha">本次使用的alpha值</param>
	void computeALLPR_2(double alpha_upper_limit, std::vector<double>& precision, std::vector<double>& recall, std::vector<std::string>& features_name);

	/// <summary>
	/// 执行一次实验，计算并输出一次所有描述子的PR值
	/// </summary>
	/// <param name="precision">输出的精准率，包括所有描述子的</param>
	/// <param name="recall">输出的召回率，包括所有描述子的</param>
	void perform(std::vector<double>& precision, std::vector<double>& recall);

	int correct_matches_sum_pfh_[20] = {};
	int matches_sum_pfh_[20] = {};
	int corresponding_regions_sum_pfh_[20] = {};

	int correct_matches_sum_fpfh_[20] = {0};
	int matches_sum_fpfh_[20] = {0};
	int corresponding_regions_sum_fpfh_[20] = {0};
	
	// -------------------for color---------------------- //
	int correct_matches_sum_cfh_rgb_pfh_rate_[20] = {0};
	int matches_sum_cfh_rgb_pfh_rate_[20] = {0};
	int corresponding_regions_sum_cfh_rgb_pfh_rate_[20] = {0};
	
	int correct_matches_sum_cfh_rgb_pfh_dot_[20] = {0};
	int matches_sum_cfh_rgb_pfh_dot_[20] = {0};
	int corresponding_regions_sum_cfh_rgb_pfh_dot_[20] = {0};

	// -------------------------------------------------- //

	int correct_matches_sum_fpfh_rgb_[20] = {0};
	int matches_sum_fpfh_rgb_[20] = {0};
	int corresponding_regions_sum_fpfh_rgb_[20] = {0};

	int correct_matches_sum_pfhrgb_[20] = {};
	int matches_sum_pfhrgb_[20] = {};
	int corresponding_regions_sum_pfhrgb_[20] = {};

	int correct_matches_sum_shot_[20] = {};
	int matches_sum_shot_[20] = {};
	int corresponding_regions_sum_shot_[20] = {};

	int correct_matches_sum_cshot_[20] = {};
	int matches_sum_cshot_[20] = {};
	int corresponding_regions_sum_cshot_[20] = {};

private:
	int model_number_ = 0;//每个场景对应的模型数量,目前可以为2
	double alpha_upper_limit_;//alpha阈值的上限，0到alpha_upper_limit是特征匹配的阈值变化范围。

	std::string dataset_name_; //数据集名称，数据集应该放到程序根目录，文件夹名称与此相同
	std::string ini_path_; //配置文件（ConfigScene1.ini）的相对路径
	std::string output_txt_path_;//txt文件的路径
};