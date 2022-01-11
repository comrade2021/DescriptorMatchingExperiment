//#include <iostream>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//
//int
//main() {
//	// Load file | Works with PCD and PLY files
//	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	//pcl::io::loadPLYFile("./pcd/Armadillo_vres2_small_scaled.ply", *source_cloud);
//	pcl::io::loadPLYFile("./pcd/bun_zipper.ply", *source_cloud);
//
//	//METHOD #1: Using a Matrix4f  
//	//This is the "manual" method, perfect to understand but error prone !
//	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//
//	// Define a rotation matrix
//	float theta = M_PI / 4; // The angle of rotation in radians
//	transform_1(0, 0) = std::cos(theta);
//	transform_1(0, 1) = -sin(theta);
//	transform_1(1, 0) = sin(theta);
//	transform_1(1, 1) = std::cos(theta);
//
//	// Define a translation of 2.5 meters on the x axis.
//	transform_1(0, 3) = 0.5;
//
//	// Print the transformation
//	printf("Method #1: using a Matrix4f\n");
//	std::cout << transform_1 << std::endl;
//
//	// Executing the transformation
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);
//
//	//Affine3f(四维)矩阵与Matrix4f(四维)矩阵之间的转换
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_af3(new pcl::PointCloud<pcl::PointXYZ>());
//	Eigen::Transform<float, 3, Eigen::Affine> transform_2(transform_1);
//	pcl::transformPointCloud(*source_cloud, *transformed_cloud_af3, transform_2);
//
//	// Visualization
//	printf("\nPoint cloud colors :  white  = original point cloud\n"
//		"                        red  = transformed point cloud\n");
//	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
//
//	// Define R,G,B colors for the point cloud
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
//	// We add the point cloud to the viewer and pass the color handler
//	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
//
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 255, 255, 255); // white
//	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
//	//viewer.addPointCloud(transformed_cloud_af3, transformed_cloud_color_handler, "transformed_cloud_af3");
//
//	for (int i = 0; i < 100; i++)
//	{
//		viewer.addLine(source_cloud->at(i), transformed_cloud->at(i), std::to_string(i));
//	}
//
//	viewer.addCoordinateSystem(1.0, "cloud", 0);
//	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//	//viewer.setPosition(800, 400); // Setting visualiser window position
//
//	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
//		viewer.spinOnce();
//	}
//
//	return 0;
//}