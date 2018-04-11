/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
NDT点云配准
*/

#include "NDTEstimateLayer.h"
#include <math.h>

#include "../../PointCloudRegistration/features.h"
#include "../../PointCloudRegistration/filters.h"
#include "../../PointCloudRegistration/registration.h"
#include "../../PointCloudRegistration/sac_ia.h"
#include "../../PointCloudRegistration/visualization.h"

#include <pcl/registration/ndt.h>

namespace svaf{

NDTEstimateLayer::NDTEstimateLayer(LayerParameter& layer) : StereoLayer(layer)
{
	// 读入初始配准参数
	targetfile = layer.iandt_param().pcd_filename();		// 目标点云文件
	max_iter = layer.iandt_param().sacia_param().max_iter();	// 最大迭代次数
	min_cors = layer.iandt_param().sacia_param().min_cors();	// 最小响应
	max_cors = layer.iandt_param().sacia_param().max_cors();	// 最大响应
	voxel_grid = layer.iandt_param().sacia_param().voxel_grid();	// 方格滤波器方格大小
	norm_rad = layer.iandt_param().sacia_param().norm_rad();	// 法向量半径
	feat_rad = layer.iandt_param().sacia_param().feat_rad();	// 特征半径

	// 目标点云位置
	x = layer.iandt_param().coor_param().x();
	y = layer.iandt_param().coor_param().y();
	z = layer.iandt_param().coor_param().z();
	a = layer.iandt_param().coor_param().a();
	b = layer.iandt_param().coor_param().b();
	c = layer.iandt_param().coor_param().c();

	// 读入点云文件
	pcdread(targetfile, targetpcd);
}

NDTEstimateLayer::~NDTEstimateLayer()
{
}

bool NDTEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_NOTNULL(param);
	pWorld_ = (World*)param;

	double xx, yy, zz;
	if (images[0].useroi){
		xx = pWorld_->x;
		yy = pWorld_->y;
		zz = pWorld_->z;
	}

	// 点云智能指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

	// 自动从相机坐标系或世界坐标系中选择数据作为待配准点云
	if (pWorld_->pointW.empty()){
		if (pWorld_->pointL.empty()){
			LOG(ERROR) << "\nLoop Cut Short\n";
			return false;
		}
		pclconvert(*source, pWorld_->pointL);
		LOG(INFO) << "SAC-IA use Left Camera Coordinate.";
		RLOG("SAC-IA use Left Camera Coordinate.");
	} else{
		pclconvert(*source, pWorld_->pointW);
		LOG(INFO) << "SAC-IA use World Coordinate.";
		RLOG("SAC-IA use World Coordinate");
	}

	target = targetpcd.makeShared(); // deep copy

	__t.StartWatchTimer();
	// downsample the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds(new pcl::PointCloud<pcl::PointXYZ>);

	pc::voxelFilter(source, cloud1ds, voxel_grid);
	pc::voxelFilter(target, cloud2ds, voxel_grid);

	// compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = pc::getNormals(cloud1ds, norm_rad);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 = pc::getNormals(cloud2ds, norm_rad);

	// c9ompute local features
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1 = pc::getFeatures(cloud1ds, normals1, feat_rad);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2 = pc::getFeatures(cloud2ds, normals2, feat_rad);

	// sac_ia
	auto sac_ia = pc::align(cloud1ds, cloud2ds, features1, features2, max_iter, min_cors, max_cors);
	__t.ReadWatchTimer("SAC-IA TIme");
	
	// 计算得到配准变换矩阵
	Eigen::Matrix4f Matrix = sac_ia.getFinalTransformation(); // make transform of cloud2
	LOG(INFO) << "SAC-IA Matrix:\n" << Matrix;
	LOG(INFO) << "SAC-IA scores: " << sac_ia.getFitnessScore();

	//Matrix = Matrix.inverse();

	//std::vector<float> angle = computeEularAngles(Matrix, false);
	pWorld_->fetchtype = 1;
	//pWorld_->a = a + angle[0];
	//pWorld_->b = b + angle[1];
	//pWorld_->c = c + angle[2];
	//pWorld_->x = x + Matrix(0, 3);
	//pWorld_->y = y + Matrix(1, 3);
	//pWorld_->z = z + Matrix(2, 3);
	//LOG(INFO) << "Result Angle   :" << " a: " << pWorld_->a << " b: " << pWorld_->b << " c: " << pWorld_->c;
	//LOG(INFO) << "Result Position:" << " x: " << pWorld_->x << " y: " << pWorld_->y << " z: " << pWorld_->z;

	// Matrix * [x y z 1]' = [rx ry rz 1]';
	//pWorld_->x = Matrix(0, 0) * x + Matrix(0, 1) * y + Matrix(0, 2) * z + Matrix(0, 3);
	//pWorld_->y = Matrix(1, 0) * x + Matrix(1, 1) * y + Matrix(1, 2) * z + Matrix(1, 3);
	//pWorld_->z = Matrix(2, 0) * x + Matrix(2, 1) * y + Matrix(2, 2) * z + Matrix(2, 3);

	// pcd center location
	//pcdcenterlocation(source, pWorld_->x, pWorld_->y, pWorld_->z);

	// ndt
	pcl::transformPointCloud(*cloud2ds, *cloud2ds, Matrix);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
	
	src = cloud1ds;
	tgt = cloud2ds;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	ndt.setTransformationEpsilon(0.1);
	ndt.setStepSize(10);

	ndt.setResolution(10);
	ndt.setMaximumIterations(100);
	ndt.setInputCloud(src);
	ndt.setInputTarget(tgt);

	pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*reg_result);

	Eigen::Matrix4f Ti = ndt.getFinalTransformation();
	Eigen::Matrix4f Tiv = Ti.inverse();
	Eigen::Matrix4f RT = Matrix * Tiv;
	vector<float> ndt_angles = computeEularAngles(RT, false);
	
	pWorld_->a = a + ndt_angles[0];
	pWorld_->b = b + ndt_angles[1];
	pWorld_->c = c + ndt_angles[2];
	//pWorld_->x = x + RT(0, 0);
	//pWorld_->y = y + RT(0, 1);
	//pWorld_->z = z + RT(0, 2);
	// compute eular angle
	//pWorld_->a = a + angle[2] + ndt_angles[2];
	//pWorld_->b = b + angle[1] + ndt_angles[1];
	//pWorld_->c = c + angle[0] + ndt_angles[0];



	// pcd center location
	pcdcenterlocation(source, pWorld_->x, pWorld_->y, pWorld_->z);

	// roi center location
	//if (images[0].useroi){
		//pWorld_->x = xx;
		//pWorld_->y = yy;
		//pWorld_->z = zz;
	//}

	LOG(INFO) << "Result Position:" << " x: " << pWorld_->x << " y: " << pWorld_->y << " z: " << pWorld_->z;

	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	if (Layer::task_type == PC_REGISTRATION){
		__bout = true;
	}
	else{
		__bout = false;
	}

	// 将点云接触输出或保存到tmp文件夹下
	if (__show || __save || __bout){
		pcl::transformPointCloud(*target, *target, Matrix);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged = pc::coloredMerge(source, target);
		if (__save){
			pcdsave(string("tmp/R_") + Circuit::time_id_ + ".pcd", *merged);
		}
		if (__show){
			pc::viewPair(cloud1ds, cloud2ds, source, target);
		}
		if (__bout){
			Mat im;
			float tmpx, tmpy, tmpz, tmpr, tmpg, tmpb;
			Block block("ia_ori", im, false, false, __bout);
			block.isOutput3DPoint = true;
			for (int i = 0; i < cloud1ds->points.size(); ++i){
				tmpx = cloud1ds->points[i].x;
				tmpy = cloud1ds->points[i].y;
				tmpz = cloud1ds->points[i].z;
				block.point3d.push_back(Point3f(tmpx, tmpy, tmpz));
				block.color3d.push_back(Color3f(0.0f, 1.0f, 0.0f));
			}
			for (int i = 0; i < cloud2ds->points.size(); ++i){
				tmpx = cloud2ds->points[i].x;
				tmpy = cloud2ds->points[i].y;
				tmpz = cloud2ds->points[i].z;
				block.point3d.push_back(Point3f(tmpx, tmpy, tmpz));
				block.color3d.push_back(Color3f(1.0f, 0.0f, 0.0f));
			}
			disp.push_back(block);

			Block block2("ia_est", im, false, false, __bout);
			block2.isOutput3DPoint = true;
			for (int i = 0; i < source->points.size(); ++i){
				tmpx = source->points[i].x;
				tmpy = source->points[i].y;
				tmpz = source->points[i].z;
				block2.point3d.push_back(Point3f(tmpx, tmpy, tmpz));
				block2.color3d.push_back(Color3f(0.0f, 1.0f, 0.0f));
			}
			for (int i = 0; i < target->points.size(); ++i){
				tmpx = target->points[i].x;
				tmpy = target->points[i].y;
				tmpz = target->points[i].z;
				block2.point3d.push_back(Point3f(tmpx, tmpy, tmpz));
				block2.color3d.push_back(Color3f(1.0f, 0.0f, 0.0f));
			}
			disp.push_back(block2);
		}
	}

	return true;
}

}
