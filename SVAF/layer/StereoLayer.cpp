/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

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
	} else{
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

void StereoLayer::pcdsave(string filename, pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool is_dense){
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

std::vector<float> StereoLayer::computeEularAngles(Eigen::Matrix4f& R, bool israd){
	std::vector<float> result(3, 0);
	const float pi = 3.14159265397932384626433;

	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = pi - theta1;
		float psi1 = atan2(R(2, 1) / cos(theta1), R(2, 2) / cos(theta1));
		float psi2 = atan2(R(2, 0) / cos(theta2), R(2, 2) / cos(theta2));
		float pfi1 = atan2(R(1, 0) / cos(theta1), R(0, 0) / cos(theta1));
		float pfi2 = atan2(R(1, 0) / cos(theta2), R(0, 0) / cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	} else{
		float phi = 0;
		float delta = atan2(R(0, 1), R(0, 2));
		if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN){ // R(2,0) == -1
			theta = pi / 2;
			psi = phi + delta;
		} else{
			theta = -pi / 2;
			psi = -phi + delta;
		}
	}

	// psi is along x-axis, theta is along y-axis, pfi is along z axis
	if (israd){ // for rad 
		result[0] = psi;
		result[1] = theta;
		result[2] = pfi;
		LOG(INFO) << "Result Rad Angle:  ( " << result[0] << ", "
			<< result[1] << ", " << result[2] << " )";
	} else{
		result[0] = psi * 180 / pi;
		result[1] = theta * 180 / pi;
		result[2] = pfi * 180 / pi;
		LOG(INFO) << "Result Degree Angle:  ( " << result[0] << ", "
			<< result[1] << ", " << result[2] << " )";
	}
	return result;
}

}
