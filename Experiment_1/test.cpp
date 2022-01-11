//#pragma warning(disable:4996)
//#include <Windows.h>
//#include <stdio.h>
//#include <iostream>
//
//#include <vector> 
//#include <string> 
//#include <fstream>
//
//int main() {
//	using namespace std;
//	/*LPTSTR lpPath = new char[MAX_PATH];
//
//	strcpy(lpPath, "./dataset/IniFileName.ini");
//
//	WritePrivateProfileString("LiMing", "Sex", "Man", lpPath);
//	WritePrivateProfileString("LiMing", "Age", "20", lpPath);
//
//	WritePrivateProfileString("Fangfang", "Sex", "Woman", lpPath);
//	WritePrivateProfileString("Fangfang", "Age", "21", lpPath);
//
//	delete[] lpPath;*/
//
//
//	/*LPTSTR lpPath = new char[MAX_PATH];
//	LPTSTR LiMingSex = new char[6];
//	int LiMingAge;
//	LPTSTR FangfangSex = new char[6];
//	int FangfangAge;
//
//	strcpy(lpPath, "./dataset/IniFileName.ini");
//
//	GetPrivateProfileString("LiMing", "Sex", "", LiMingSex, 6, lpPath);
//	LiMingAge = GetPrivateProfileInt("LiMing", "Age", 0, lpPath);
//
//	GetPrivateProfileString("Fangfang", "Sex", "", FangfangSex, 6, lpPath);
//	FangfangAge = GetPrivateProfileInt("Fangfang", "Age", 0, lpPath);
//
//	std::cout << LiMingSex << std::endl;
//	std::cout << LiMingAge << std::endl;
//	std::cout << FangfangSex << std::endl;
//	std::cout << FangfangAge << std::endl;
//
//	delete[] lpPath;*/
//
//	////read string
//	//std::ifstream myfile("./dataset/in.txt");
//	//std::ofstream outfile("./dataset/out.xf", std::ios::app);
//	//std::string temp;
//	//if (!myfile.is_open())
//	//{
//	//	std::cout << "未成功打开文件" << std::endl;
//	//}
//	//while (std::getline(myfile, temp))
//	//{
//	//	std::cout << temp << std::endl;
//	//	outfile << temp;
//	//	outfile << std::endl;
//	//}
//	//myfile.close();
//	//outfile.close();
//	//return 0;
//
//
//	//vector<int> cost;
//	//int v, w, weight;
//	//ifstream infile;   //输入流
//	//ofstream outfile;   //输出流
//
//	//infile.open("./dataset/in2.txt", ios::in);
//	//if (!infile.is_open())
//	//	cout << "Open file failure" << endl;
//	//while (!infile.eof())            // 若未到文件结束一直循环
//	//{
//	//	infile >> v >> w >> weight;
//	//	cost.push_back(v);
//	//	cost.push_back(w);
//	//	cost.push_back(weight);
//	//	cout << v << " " << w << " " << weight << " " << endl;
//	//}
//	//infile.close();   //关闭文件
//
//	//outfile.open("./dataset/out2.txt", ios::app);   //每次写都定位的文件结尾，不会丢失原来的内容，用out则会丢失原来的内容
//	//if (!outfile.is_open())
//	//	cout << "Open file failure" << endl;
//	//for (size_t i = 0; i < 9; i++)
//	//{
//	//	outfile << cost[i] << endl;  //在result.txt中写入结果
//	//}
//	//outfile.close();
//
//	//return 0;
//}