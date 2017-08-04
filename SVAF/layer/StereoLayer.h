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
	void pcdread(string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
	void pclconvert(pcl::PointCloud<pcl::PointXYZ>& cloud, vector<Point3f>& inpoints);
	void pclconvert(vector<Point3f>& points, pcl::PointCloud<pcl::PointXYZ>& incloud);

};

}

