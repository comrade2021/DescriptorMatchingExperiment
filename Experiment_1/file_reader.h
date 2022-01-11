#pragma once
#include <pcl/common/transforms.h>

class FileReader
{
public:
	/// <summary>
	/// 读取xf文件中的变换矩阵
	/// </summary>
	/// <param name="xf_path">ground_truth.xf文件地址</param>
	/// <param name="ground_truth">输出变换矩阵</param>
	void readGroundTruth(std::string xf_path,Eigen::Matrix4f &ground_truth);
	
	/// <summary>
	/// 读取INI文件，获取场景路径和对应模型路径。
	/// </summary>
	/// <param name="ini_path">ConfigScene1.ini文件的地址</param>
	/// <param name="scene_path">输出场景点云地址</param>
	/// <param name="model_path">输出模型点云地址</param>
	/// <param name="model_matrix">输出模型到场景的矩阵文件路径</param>
	void readINI(std::string dataset_name,std::string ini_path,std::string &scene_path,std::vector<std::string> &model_path, std::vector<std::string>& model_matrix_path);

	/// <summary>
	/// 用"/"替换"\"
	/// </summary>
	/// <param name="str">含有"\"的字符串</param>
	void replaceC(std::string& str);

	/// <summary>
	/// 从INI文件中读取每个场景对应的模型数量
	/// </summary>
	/// <param name="ini_path"></param>
	/// <returns></returns>
	static int readNumber(std::string ini_path);

private:

};