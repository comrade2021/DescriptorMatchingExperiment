//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//#include "feature_extractor.h"
//#include "my_normal_estimation.h"
//#include "keypoints_detector.h"
//#include <vtkAutoInit.h>
//
//
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//VTK_MODULE_INIT(vtkRenderingContextOpenGL2);
//
//
//
//#define INCRE 0.02
//#define WID 1
//
//int
//main()
//{
//	pcl::PointCloud<pcl::PointXYZRGB> cloud;
//	int t = 0;
//	for (float x = -WID; x <= WID; x += INCRE)
//	{
//		for (float y = -WID; y <= WID; y += INCRE)
//		{
//			if ((x*x+y*y)<=1.0)
//			{
//				t++;
//
//			}
//		}
//	}
//
//	// Fill in the cloud data
//	cloud.width = t;
//	cloud.height = 1;
//	cloud.is_dense = false;
//	cloud.resize(cloud.width * cloud.height);
//
//	int idx = 0;
//	////第一种纹理
//	//for (float x = -WID; x <= WID; x+= INCRE)
//	//{
//	//	for (float y = -WID; y <= WID; y += INCRE)
//	//	{
//	//		if ((x * x + y * y) <= 1.0)//只要一个圆
//	//		{
//	//			cloud.at(idx).x = x;
//	//			cloud.at(idx).y = y;
//	//			cloud.at(idx).z = 0;
//	//			if (x > 0)
//	//			{
//	//				cloud.at(idx).r = 0;
//	//				cloud.at(idx).g = 0;
//	//				cloud.at(idx).b = 128;
//	//			}
//	//			else
//	//			{
//	//				cloud.at(idx).r = 255;
//	//				cloud.at(idx).g = 255;
//	//				cloud.at(idx).b = 0;
//	//			}
//	//			idx++;
//	//		}
//	//	}
//	//}
//
//	//// 第二种纹理
//	//for (float x = -WID; x <= WID; x += INCRE)
//	//{
//	//	for (float y = -WID; y <= WID; y += INCRE)
//	//	{
//	//		if ((x * x + y * y) <= 1.0)//只要一个圆
//	//		{
//	//			cloud.at(idx).x = x;
//	//			cloud.at(idx).y = y;
//	//			cloud.at(idx).z = 0;
//	//			//分配色彩纹理
//	//			if (sqrt(x*x+y*y)<(1/sqrt(2)))
//	//			{
//	//				cloud.at(idx).r = 0;
//	//				cloud.at(idx).g = 0;
//	//				cloud.at(idx).b = 128;
//	//			}
//	//			else
//	//			{
//	//				cloud.at(idx).r = 255;
//	//				cloud.at(idx).g = 255;
//	//				cloud.at(idx).b = 0;
//	//			}
//	//			idx++;
//	//		}
//	//	}
//	//}
//
//	//// 第三种纹理
//	//for (float x = -WID; x <= WID; x += INCRE)
//	//{
//	//	for (float y = -WID; y <= WID; y += INCRE)
//	//	{
//	//		if ((x * x + y * y) <= 1.0)//只要一个圆
//	//		{
//	//			cloud.at(idx).x = x;
//	//			cloud.at(idx).y = y;
//	//			cloud.at(idx).z = 0;
//	//			//分配色彩纹理
//	//			if ((x>-0.1&&x<0.1)||(x > 0.3 && x < 0.5) || (x > 0.7 && x < 0.9) || (x > -0.5 && x < -0.3) || (x > -0.9 && x < -0.7))
//	//			{
//	//				cloud.at(idx).r = 0;
//	//				cloud.at(idx).g = 0;
//	//				cloud.at(idx).b = 128;
//	//			}
//	//			else
//	//			{
//	//				cloud.at(idx).r = 255;
//	//				cloud.at(idx).g = 255;
//	//				cloud.at(idx).b = 0;
//	//			}
//	//			idx++;
//	//		}
//	//	}
//	//}
//	
//	//// 第4种纹理
//	//for (float x = -WID; x <= WID; x += INCRE)
//	//{
//	//	for (float y = -WID; y <= WID; y += INCRE)
//	//	{
//	//		if ((x * x + y * y) <= 1.0)//只要一个圆
//	//		{
//	//			cloud.at(idx).x = x;
//	//			cloud.at(idx).y = y;
//	//			cloud.at(idx).z = 0;
//	//			//分配色彩纹理
//	//			if ((x>0&&y>0)||(x<0&&y<0))
//	//			{
//	//				cloud.at(idx).r = 0;
//	//				cloud.at(idx).g = 0;
//	//				cloud.at(idx).b = 128;
//	//			}
//	//			else
//	//			{
//	//				cloud.at(idx).r = 255;
//	//				cloud.at(idx).g = 255;
//	//				cloud.at(idx).b = 0;
//	//			}
//	//			idx++;
//	//		}
//	//	}
//	//}
//
//	// 第5种纹理
//	for (float x = -WID; x <= WID; x+= INCRE)
//	{
//		for (float y = -WID; y <= WID; y += INCRE)
//		{
//			if ((x * x + y * y) <= 1.0)//只要一个圆
//			{
//				cloud.at(idx).x = x;
//				cloud.at(idx).y = y;
//				cloud.at(idx).z = 0;
//				if (x > 0)
//				{
//					cloud.at(idx).r = 0;
//					cloud.at(idx).g = 0;
//					cloud.at(idx).b = 128;
//				}
//				else
//				{
//					cloud.at(idx).r = 220;
//					cloud.at(idx).g = 20;
//					cloud.at(idx).b = 60;
//				}
//				idx++;
//			}
//		}
//	}
//
//	//for (auto& point : cloud)
//	//{
//	//	point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//	//	point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//	//	point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//	//}
//
//	pcl::io::savePCDFileASCII("./pcd/texture_1.pcd", cloud);
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::io::loadPCDFile("./pcd/texture_1.pcd", *cloud_1);
//
//	
//
//
//	//可视化直方图
//	//keypoints
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_1(new pcl::PointCloud<pcl::PointXYZRGB>());
//	keypoint_1->width = 1;
//	keypoint_1->height = 1;
//	keypoint_1->is_dense = false;
//	keypoint_1->resize(1);
//	keypoint_1->at(0).x = 0;
//	keypoint_1->at(0).y = 0;
//	keypoint_1->at(0).z = 0;
//	keypoint_1->at(0).r = 0;
//	keypoint_1->at(0).g = 0;
//	keypoint_1->at(0).b = 0;
//	//normal
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
//	MyNormalEstimation ne;
//	ne.computeNormal_K(cloud_1, 10, normals);
//	//feature
//	pcl::PointCloud<pcl::PFHSignature125>::Ptr feature_1(new pcl::PointCloud<pcl::PFHSignature125>());
//	FeatureExtractor ex;
//	ex.computeFPFH_CFH_HSV_FPFH_RATE(cloud_1, keypoint_1, normals, 1.0, feature_1);
//	for (size_t i = 0; i < 66; i++)
//	{
//		std::cout << feature_1->at(0).histogram[i] << std::endl;
//	}
//	//可视化特征直方图
//	pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("plotter");
//	/*std::cout <<"getFieldsList PFHSignature125 :" << pcl::getFieldsList<pcl::PFHSignature125>(*pfh_m)<<std::endl;
//	std::cout <<"getFieldsList PointXYZRGB :" << pcl::getFieldsList<pcl::PointXYZRGB>(*cloud_model)<< std::endl;*/
//	//plotter->addFeatureHistogram(*feature_1,"pfh",0,"histogram_1");
//	plotter->addFeatureHistogram(*feature_1,70,"feature_1");
//	plotter->plot();
//	//plotter->setXRange(0, 66);
//	plotter->setXTitle("");
//	plotter->setYTitle("");
//
//	/*while (!plotter->wasStopped())
//	{
//		plotter->spinOnce(100);
//		plotter->clearPlots();
//	}*/
//
//	//点云可视化
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.addPointCloud(cloud_1, "cloud_1");
//	pcl::PointXYZRGB p1(0,0,0,255,255,255);
//	viewer.addSphere(p1, 0.03, 0, 0, 0, "sphere");
//	//viewer.addCoordinateSystem(1.0, "coor", 0);
//	viewer.setBackgroundColor(211, 211, 211); // Setting background to a dark grey
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_1");
//
//	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
//		viewer.spinOnce();
//	}
//
//	return (0);
//}