//#include <iostream>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//
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
//int
//main() 
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::io::loadPLYFile("./dataset5/3D models/CVLab/Kinect/MeshRegistration/Duck/duck002.ply", *cloud_1);
//	pcl::io::loadPLYFile("./dataset5/3D models/CVLab/Kinect/ObjectRecognition/Scenes/2011_06_27/scene011.ply", *cloud_1);
//	//pcl::io::loadPLYFile("./dataset4/3D models/CVLab/2010-06-12/Scene1/Scene1.ply", *cloud_1);
//	//pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2009-10-27/model2.ply", *cloud_1);
//	//pcl::io::loadPLYFile("./dataset3/3D models/CVLab/2009-10-27/Scene1.ply", *cloud_1);
//
//	//点云可视化
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.addPointCloud(cloud_1, "cloud_1");
//	//viewer.addCoordinateSystem(1.0, "coor", 0);
//	viewer.setBackgroundColor(255, 255, 255); // Setting background to a dark grey
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_1");
//
//	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
//		viewer.spinOnce();
//	}
//
//	return (0);
//
//}