#include "IAEstimateLayer.h"
#include <math.h>

#include "../../PointCloudRegistration/features.h"
#include "../../PointCloudRegistration/filters.h"
#include "../../PointCloudRegistration/registration.h"
#include "../../PointCloudRegistration/sac_ia.h"
#include "../../PointCloudRegistration/visualization.h"

namespace svaf{

IAEstimateLayer::IAEstimateLayer(LayerParameter& layer) : StereoLayer(layer){
	targetfile = layer.sacia_param().pcd_filename();
	max_iter = layer.sacia_param().ia_param().max_iter();
	min_cors = layer.sacia_param().ia_param().min_cors();
	max_cors = layer.sacia_param().ia_param().max_cors();
	voxel_grid = layer.sacia_param().ia_param().voxel_grid();
	norm_rad = layer.sacia_param().ia_param().norm_rad();
	feat_rad = layer.sacia_param().ia_param().feat_rad();

	x = layer.sacia_param().coor_param().x();
	y = layer.sacia_param().coor_param().y();
	z = layer.sacia_param().coor_param().z();
	a = layer.sacia_param().coor_param().a();
	b = layer.sacia_param().coor_param().b();
	c = layer.sacia_param().coor_param().c();

	pcdread(targetfile, targetpcd);
}

IAEstimateLayer::~IAEstimateLayer()
{
}

bool IAEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_NOTNULL(param);
	pWorld_ = (World*)param;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

	if (pWorld_->pointW.empty()){
		if (pWorld_->pointL.empty()){
			LOG(ERROR) << "\nLoop Cut Short\n";
			return false;
		}
		pclconvert(*source, pWorld_->pointL);
		LOG(INFO) << "SAC-IA use Left Camera Coordinate.";
	} else{
		pclconvert(*source, pWorld_->pointW);
		LOG(INFO) << "SAC-IA use World Coordinate.";
	}
	
	target = targetpcd.makeShared();
	
	__t.StartWatchTimer();
	// downsample the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds(new pcl::PointCloud<pcl::PointXYZ>);

	pc::voxelFilter(source, cloud1ds, voxel_grid);
	pc::voxelFilter(target, cloud2ds, voxel_grid);

	// compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = pc::getNormals(cloud1ds, norm_rad);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 = pc::getNormals(cloud2ds, norm_rad);

	// compute local features
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1 = pc::getFeatures(cloud1ds, normals1, feat_rad);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2 = pc::getFeatures(cloud2ds, normals2, feat_rad);

	// sac_ia
	auto sac_ia = pc::align(cloud1ds, cloud2ds, features1, features2, max_iter, min_cors, max_cors);
	__t.ReadWatchTimer("SAC-IA Time");

	Eigen::Matrix4f init_transform = sac_ia.getFinalTransformation(); // make transform of cloud2 -> cloud2
	LOG(INFO) << "SAC-IA Matrix:\n" << init_transform;
	LOG(INFO) << "SAC-IA scores: " << sac_ia.getFitnessScore();
	
	std::vector<float> angle = computeEularAngles(init_transform, false);
	pWorld_->a = angle[0];
	pWorld_->b = angle[1];
	pWorld_->c = angle[2];

	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}
	
	if (Layer::task_type == PC_REGISTRATION){
		__bout = true;
	} else{
		__bout = false;
	}

	if (__show || __save || __bout){
		pcl::transformPointCloud(*target, *target, init_transform);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged = pc::coloredMerge(source, target);
		if (__save){
			pcdsave(string("tmp/R_") + Circuit::time_id_ + ".pcd", *merged);
		}
		if (__show){
			pc::viewPair(cloud1ds, cloud2ds, source, target);
		}
		if (__bout){
			Mat im;
			float x, y, z, r, g, b;
			Block block("ia_ori", im, false, false, __bout);
			block.isOutput3DPoint = true;
			for (int i = 0; i < cloud1ds->points.size(); ++i){
				x = cloud1ds->points[i].x;
				y = cloud1ds->points[i].y;
				z = cloud1ds->points[i].z;
				block.point3d.push_back(Point3f(x, y, x));
				block.color3d.push_back(Color3f(0.0f, 1.0f, 0.0f));
			}
			for (int i = 0; i < cloud2ds->points.size(); ++i){
				x = cloud2ds->points[i].x;
				y = cloud2ds->points[i].y;
				z = cloud2ds->points[i].z;
				r = 0; g = 0; b = 1.0f;
				block.point3d.push_back(Point3f(x, y, x));
				block.color3d.push_back(Color3f(1.0f, 0.0f, 0.0f));
			}
			disp.push_back(block);

			Block block2("ia_est", im, false, false, __bout);
			block2.isOutput3DPoint = true;
			for (int i = 0; i < source->points.size(); ++i){
				x = source->points[i].x;
				y = source->points[i].y;
				z = source->points[i].z;
				block2.point3d.push_back(Point3f(x, y, x));
				block2.color3d.push_back(Color3f(0.0f, 1.0f, 0.0f));
			}
			for (int i = 0; i < target->points.size(); ++i){
				x = target->points[i].x;
				y = target->points[i].y;
				z = target->points[i].z;
				r = 0; g = 0; b = 1.0f;
				block2.point3d.push_back(Point3f(x, y, x));
				block2.color3d.push_back(Color3f(1.0f, 0.0f, 0.0f));
			}
			disp.push_back(block2);
		}
	}

	return true;
}

}
