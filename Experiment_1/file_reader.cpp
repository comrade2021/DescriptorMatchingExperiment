#include "file_reader.h"
#include <Windows.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector> 
#include <string>
#include <sstream>
#include <iomanip>

void FileReader::readGroundTruth(std::string xf_path, Eigen::Matrix4f& ground_truth)
{
	std::ifstream infile;   //输入流
	std::vector<double> vec;

	infile.open(xf_path, std::ios::in);
	if (!infile.is_open())
		std::cout << "Open file failure:"<< xf_path << std::endl;
	while (!infile.eof())            // 若未到文件结束一直循环
	{
		double a;
		infile >> a;
		vec.push_back(a);
	}
	int k = 0;
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			ground_truth(i, j) = vec[k];
			k++;
		}
	}
	infile.close();   //关闭文件
}

void FileReader::readINI(std::string dataset_name, std::string ini_path, std::string& scene_path, std::vector<std::string>& model_path, std::vector<std::string>& model_matrix_path)
{
	int number = 0;
	LPTSTR url = new char[MAX_PATH];//ini文件地址
	
	strcpy(url,ini_path.c_str());//从参数获取ini文件地址
	number = GetPrivateProfileInt("MODELS", "NUMBER", 0, url);//获取模型数量
	//读取每个模型的路径和变换矩阵
	for (int i = 0; i < number; i++)
	{
		//获取模型文件地址
		std::string model_name = "MODEL_" + std::to_string(i);//键
		LPTSTR model_path_lp = new char[MAX_PATH];//值(用LPTSTR暂存函数输出)
		GetPrivateProfileString("MODELS", model_name.c_str(), "", model_path_lp, MAX_PATH, url);
		model_path.push_back(model_path_lp);//输出模型地址
		replaceC(model_path[i]);
		model_path[i] = "./" + dataset_name + model_path[i];
		//获取模型矩阵文件地址
		std::string model_matrix_name = "MODEL_" + std::to_string(i) + "_GROUNDTRUTH";//键
		LPTSTR matrix_path_lp = new char[MAX_PATH];//值
		GetPrivateProfileString("MODELS", model_matrix_name.c_str(), "", matrix_path_lp, MAX_PATH, url);
		model_matrix_path.push_back(matrix_path_lp);//输出模型矩阵地址
		replaceC(model_matrix_path[i]);
		model_matrix_path[i] = "./" + dataset_name + model_matrix_path[i];
	}
	//读取场景信息
	LPTSTR scene_path_lp = new char[MAX_PATH];
	GetPrivateProfileString("SCENE", "PATH", "", scene_path_lp, MAX_PATH, url);
	scene_path = scene_path_lp;
	replaceC(scene_path);
	scene_path = "./" + dataset_name + scene_path;
	delete[] url;
}

/* 用/替换\ */
void FileReader::replaceC(std::string& str) {
	int pos;
	pos = str.find("\\");
	while (pos != -1) {
		// str.length()求字符的长度，注意str必须是string类型
		str.replace(pos, std::string("\\").length(), "/");
		pos = str.find("\\");
	}
}

int FileReader::readNumber(std::string ini_path)
{
	int number = 0;
	LPTSTR url = new char[MAX_PATH];//ini文件地址
	strcpy(url, ini_path.c_str());//从参数获取ini文件地址
	number = GetPrivateProfileInt("MODELS", "NUMBER", 0, url);//获取模型数量
	return number;
}
