// line_match.cpp : 定义控制台应用程序的入口点。
#include"pch.h"
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include<iostream>
#include<math.h>
#include <string> 
#include<fstream> 
#include"Bazier.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <vtkAutoInit.h>

#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeType,vtkRenderingOpenGL) 
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#pragma once
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;


typedef pcl::PointXYZI ptype;
typedef pcl::PointCloud<ptype>::Ptr ptrtype;

U16 pts_sourceID;
float p_gpsT;

ptrtype readlas(string filepath)
{
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(filepath.c_str());
	LASreader* lasreader = lasreadopener.open();
	size_t count = lasreader->header.number_of_point_records;
	pcl::PointCloud<ptype>::Ptr pointCloudPtr(new pcl::PointCloud<ptype>);
	pointCloudPtr->resize(count);
	size_t j = 0;
	pts_sourceID = lasreader->point.get_point_source_ID();
	p_gpsT = lasreader->point.get_gps_time();
	while (lasreader->read_point() && j < count)
	{
		pointCloudPtr->points[j].x = lasreader->point.get_x();
		pointCloudPtr->points[j].y = lasreader->point.get_y();
		pointCloudPtr->points[j].z = lasreader->point.get_z();
		pointCloudPtr->points[j].intensity = lasreader->point.get_gps_time();
		j++;
	}
	pointCloudPtr->resize(j);
	pointCloudPtr->width = count;
	pointCloudPtr->height = 1;
	pointCloudPtr->is_dense = false;
	return pointCloudPtr;
}

int writelas(string filepath,ptrtype point_cloud)
{
	//存入las文件
	LASwriteOpener lasWriterOpener;
	lasWriterOpener.set_file_name(filepath.c_str());
	//init header
	LASheader lasHeader;
	lasHeader.x_scale_factor = 0.0001;
	lasHeader.y_scale_factor = 0.0001;
	lasHeader.z_scale_factor = 0.0001;
	/*
	lasHeader.x_offset = mls_rect->points[0].x + pmin.x;
	lasHeader.y_offset = mls_rect->points[0].y + pmin.y;
	lasHeader.z_offset = mls_rect->points[0].z + pmin.z;
	*/
	lasHeader.x_offset = point_cloud->points[0].x;
	lasHeader.y_offset = point_cloud->points[0].y;
	lasHeader.z_offset = point_cloud->points[0].z;
	lasHeader.point_data_format = 3;
	lasHeader.point_data_record_length = 34;

	//open laswriter
	LASwriter* lasWriter = lasWriterOpener.open(&lasHeader);

	// init point
	LASpoint lasPoint;
	lasPoint.init(&lasHeader, lasHeader.point_data_format, lasHeader.point_data_record_length, 0);

	// write points
	double minX = DBL_MAX, minY = DBL_MAX, minZ = DBL_MAX;
	double maxX = -DBL_MAX, maxY = -DBL_MAX, maxZ = -DBL_MAX;
	for (int j = 0; j < point_cloud->size(); j++)
	{
		// populate the point
		lasPoint.set_x(point_cloud->points[j].x);
		lasPoint.set_y(point_cloud->points[j].y);
		lasPoint.set_z(point_cloud->points[j].z);
		lasPoint.set_intensity(point_cloud->points[j].intensity);
		lasPoint.set_point_source_ID(pts_sourceID);
		lasPoint.set_gps_time(p_gpsT);
		lasPoint.set_R(0);
		lasPoint.set_G(0);
		lasPoint.set_B(0);
		lasPoint.set_classification(0);

		// write the point
		lasWriter->write_point(&lasPoint);

		// add it to the inventory
		lasWriter->update_inventory(&lasPoint);

		//range
		double x = point_cloud->points[j].x;
		double y = point_cloud->points[j].y;
		double z = point_cloud->points[j].z;
		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
		if (z < minZ) minZ = z;
		if (z > maxZ) maxZ = z;
	}

	// update the boundary
	lasHeader.set_bounding_box(minX, minY, minZ, maxX, maxY, maxZ);

	// update the header
	lasWriter->update_header(&lasHeader, true);

	// close the writer
	lasWriter->close();
	delete lasWriter;
	lasWriter = nullptr;
	return 0;
}
void getFiles(string path, vector<string>& files)
{
	//文件句柄
	intptr_t   hFile = 0;
	//文件信息，声明一个存储文件信息的结构体
	struct _finddata_t fileinfo;
	string p;//字符串，存放路径
	p.assign(path).append("\\*.las");   //寻找特定文件后缀
	if ((hFile = _findfirst(p.c_str(), &fileinfo)) != -1)//若查找成功，则进入
	{
		do
		{
			//如果是目录,迭代之（即文件夹内还有文件夹）
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				//文件名不等于"."&&文件名不等于".."
				//.表示当前目录
				//..表示当前目录的父目录
				//判断时，两者都要忽略，不然就无限递归跳不出去了！
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			//如果不是,加入列表
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose函数结束查找
		_findclose(hFile);
	}
}
void cloudvisual(ptrtype cloud, const char* name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));

	pcl::visualization::PointCloudColorHandlerGenericField<ptype> fildColor(cloud, "z"); // 按照z字段进行渲染

	viewer->addPointCloud<ptype>(cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main()
{
	/*
	pcl::PointCloud<ptype>::Ptr mls_cloud(new pcl::PointCloud<ptype>);
	std::istringstream mls_path("E:", std::ios::in);
	mls_cloud = readlas(mls_path);
	std::ofstream outfile("E:\\test.txt", std::ios::out);
	if (outfile)
	{
		for (int i = 0; i < output_y.size(); i++)
		{
			outfile << std::fixed << setprecision(3) << output_x[i] << " " << output_y[i] << std::endl;
		}
	}
	outfile.close();
	*/
	

	string filePath = "G:\\pc_file\\las";//自己设置目录
	vector<string> files;

	////获取该路径下的所有文件
	getFiles(filePath, files);
	char str[30];
	int size = files.size();
	for (int i = 0; i < size; i++)
	{
		pcl::PointCloud<ptype>::Ptr pointCloudPtr(new pcl::PointCloud<ptype>);
		pointCloudPtr=readlas(files[i].c_str());
		//cloudvisual(pointCloudPtr, "1");
		stringstream output_file;
		output_file<< "G:\\pc_file\\las\\" << i << ".las";
		writelas(output_file.str(), pointCloudPtr);
		cout << i << endl;
	}

	return 0;
}
