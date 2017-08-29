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
	
	if (__show || __save){
		pcl::transformPointCloud(*target, *target, init_transform);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged = pc::coloredMerge(source, target);
		if (__save){
			pcdsave(string("tmp/R_") + Circuit::time_id_ + ".pcd", *merged);
		}
		if (__show){
			pc::viewPair(cloud1ds, cloud2ds, source, target);
		}
	}

	

	return true;
}

std::vector<float> IAEstimateLayer::computeEularAngles(Eigen::Matrix4f& R, bool israd){
	std::vector<float> result(3, 0);
	const float pi = 3.14159265397932384626433;

	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = pi - theta1;
		float psi1 = atan2(R(2,1)/cos(theta1), R(2,2)/cos(theta1));
		float psi2 = atan2(R(2,0)/cos(theta2), R(2,2)/cos(theta2));
		float pfi1 = atan2(R(1,0)/cos(theta1), R(0,0)/cos(theta1));
		float pfi2 = atan2(R(1,0)/cos(theta2), R(0,0)/cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	} else{
		float phi = 0;
		float delta = atan2(R(0,1), R(0,2));
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
