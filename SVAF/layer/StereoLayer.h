/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include <pcl\io\pcd_io.h>
#include <pcl\point_types.h>
#include "Layer.h"


namespace svaf{

class StereoLayer :
	public Layer
{
public:
	StereoLayer();
	explicit StereoLayer(LayerParameter&);
	~StereoLayer();

protected:
	void pcdsave(string filename, vector<Point3f>& points, bool is_dense = false);
	void pcdread(string filename, vector<Point3f>& points);
	void pcdsave(string filename, pcl::PointCloud<pcl::PointXYZ>& cloud, bool is_dense = false);
	void pcdsave(string filename, pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool is_dense = false);
	void pcdread(string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
	void pclconvert(pcl::PointCloud<pcl::PointXYZ>& cloud, vector<Point3f>& inpoints);
	void pclconvert(vector<Point3f>& points, pcl::PointCloud<pcl::PointXYZ>& incloud);
	void pcdcenterlocation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double & x, double & y, double & z);
	std::vector<float> computeEularAngles(Eigen::Matrix4f& R, bool israd = true);

};

}

