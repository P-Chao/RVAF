#include "StereoLayer.h"

namespace svaf{


StereoLayer::StereoLayer()
{
}

StereoLayer::StereoLayer(LayerParameter& layer)
	: Layer(layer)
{
}

StereoLayer::~StereoLayer()
{
}

void StereoLayer::pcdsave(string filename, vector<Point3f>& points, bool is_dense){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = points.size();
	cloud.height = 1;
	cloud.is_dense = is_dense;
	cloud.points.resize(points.size());
	for (size_t i = 0; i < points.size(); ++i){
		cloud.points[i].x = points[i].x;
		cloud.points[i].y = points[i].y;
		cloud.points[i].z = points[i].z;
	}
	if (!cloud.empty()){
		pcl::io::savePCDFileASCII(filename, cloud);
		LOG(INFO) << filename << " Saved <" << cloud.size() << "> Points.";
	}
	else{
		LOG(ERROR) << "PCL Write Error, Empty Cloud.";
	}
	
	return;
}

void StereoLayer::pcdread(string filename, vector<Point3f>& points){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1){
		LOG(FATAL) << " Couldn't Open " << filename;
		return;
	}

	points.clear();
	for (size_t i = 0; i < cloud->points.size(); ++i){
		points.push_back(Point3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
	}
	return;
}

void StereoLayer::pcdsave(string filename, pcl::PointCloud<pcl::PointXYZ>& cloud, bool is_dense){
	pcl::io::savePCDFileASCII(filename, cloud);
	LOG(INFO) << filename << " Saved <" << cloud.size() << "> Points.";
	return;
}

void StereoLayer::pcdread(string filename, pcl::PointCloud<pcl::PointXYZ>& cloud){
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1){
		LOG(FATAL) << " Couldn't Open " << filename;
		return;
	}
}

void StereoLayer::pclconvert(pcl::PointCloud<pcl::PointXYZ>& cloud, vector<Point3f>& inpoints){
	cloud.clear();
	cloud.width = inpoints.size();
	cloud.height = 1;
	cloud.resize(inpoints.size());
	for (int i = 0; i < inpoints.size(); ++i){
		cloud.points[i].x = inpoints[i].x;
		cloud.points[i].y = inpoints[i].y;
		cloud.points[i].z = inpoints[i].z;
	}

}

void StereoLayer::pclconvert(vector<Point3f>& points, pcl::PointCloud<pcl::PointXYZ>& incloud){
	points.clear();
	points.resize(incloud.size());
	for (int i = 0; i < incloud.size(); ++i){
		points.push_back(Point3f(incloud.points[i].x, incloud.points[i].y, incloud.points[i].z));
	}
}

}
