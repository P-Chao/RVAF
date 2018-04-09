/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
利用矩阵相乘完成空间坐标转换
*/

#include "MatrixMulLayer.h"
#include <sstream>

namespace svaf{

// 在类构架时就从文件读取坐标变换矩阵
MatrixMulLayer::MatrixMulLayer(LayerParameter& layer) : StereoLayer(layer)
{
	if (layer.mxmul_param().has_col0() && layer.mxmul_param().has_col1() && layer.mxmul_param().has_col2()){
		// 从配置文件脚本中读取变换矩阵参数
		string col[3];
		col[0] = layer.mxmul_param().col0();
		col[1] = layer.mxmul_param().col1();
		col[2] = layer.mxmul_param().col2();
		stringstream ss;
		for (int i = 0; i < 2; ++i){
			ss.clear();
			ss << col[i];
			for (int j = 0; j < 3; ++j){
				ss >> M[i][j];
			}
		}
		LOG(INFO) << "Matrix Opened From Proto.";
	} else if(layer.mxmul_param().has_filename()){
		// 从二进制文件中读取变换矩阵参数
		string filename = layer.mxmul_param().filename();
		FILE *fp = fopen(filename.c_str(), "rb+");
		CHECK_NOTNULL(fp);
		fread(&M[0], 4, 4, fp);
		fread(&M[1], 4, 4, fp);
		fread(&M[2], 4, 4, fp);
		fclose(fp);
		LOG(INFO) << "Matrix Opened From File " << filename;
	} else{
		// 无矩阵变换参数，创建失败
		LOG(FATAL) << "Matrix Create Failed!";
		RLOG("Matrix Create Failed!");
	}
	
	char loginfo[160];
	string logstr;
	sprintf(loginfo, "\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n", M[0][0], M[0][1], M[0][2], M[0][3]); logstr += loginfo;
	sprintf(loginfo, "\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n", M[1][0], M[1][1], M[1][2], M[1][3]); logstr += loginfo;
	sprintf(loginfo, "\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n", M[2][0], M[2][1], M[2][2], M[2][3]); logstr += loginfo;
	LOG(INFO) << "world = M * camera, M = \n" << logstr;
	RLOG(string("world = M * camera, M = \n") + logstr);
}

MatrixMulLayer::~MatrixMulLayer()
{
}

bool MatrixMulLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	auto pWorld = (World*)param;
	// 通过矩阵相乘将左相机坐标变换为世界坐标
	pWorld->pointW.clear();
	__t.StartWatchTimer();
	// center roi
	if (images[0].useroi){
		double xx = pWorld->x * M[0][0] + pWorld->y * M[0][1] + pWorld->z * M[0][2] + M[0][3];
		double yy = pWorld->x * M[1][0] + pWorld->y * M[1][1] + pWorld->z * M[1][2] + M[1][3];
		double zz = pWorld->x * M[2][0] + pWorld->y * M[2][1] + pWorld->z * M[2][2] + M[2][3];
		pWorld->x = xx;
		pWorld->y = yy;
		pWorld->z = zz;
	}

	for (int i = 0; i < pWorld->pointL.size(); ++i){
		Point3f point;
		point.x = pWorld->pointL[i].x * M[0][0] + pWorld->pointL[i].y * M[0][1] + pWorld->pointL[i].z * M[0][2] + M[0][3];
		point.y = pWorld->pointL[i].x * M[1][0] + pWorld->pointL[i].y * M[1][1] + pWorld->pointL[i].z * M[1][2] + M[1][3];
		point.z = pWorld->pointL[i].x * M[2][0] + pWorld->pointL[i].y * M[2][1] + pWorld->pointL[i].z * M[2][2] + M[2][3];
		pWorld->pointW.push_back(point);
	}
	__t.ReadWatchTimer("RT Transform Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}
	
	// 保存世界坐标系下的点云结果
	if (__save){
		pcdsave(string("tmp/W_") + Circuit::time_id_ + ".pcd", pWorld->pointW); // world coord
	}

	if (Layer::task_type == PC_MULMATRIX){
		__bout = true;
	} else{
		__bout = false;
	}

	if (__bout){
		Mat im;
		Block block("Point Cloud World", im, false, false, __bout);
		block.isOutput3DPoint = true;
		block.point3d = pWorld->pointW;
		disp.push_back(block);
	}

	// 输出信息并记录日志
	char loginfo[160];
	string logstr;
	for (int i = 0; i < (std::min)((int)pWorld->xl.size(), INT_MAX); ++i){
		sprintf(loginfo, "(%8.3f, %8.3f) (%8.3f, %8.3f)\t(%8.3f, %8.3f, %8.3f)\n",
			pWorld->xl[i].x, pWorld->xl[i].y, pWorld->xr[i].x, pWorld->xr[i].y,
			pWorld->pointW[i].x, pWorld->pointW[i].y, pWorld->pointW[i].z);
		logstr += loginfo;
	}
	LOG(INFO) << "Left Camera Point: \n \t xl\t\t\t xr \t\t\t pointW\n" << logstr;

	if (pWorld->xl.size() == 1){
		pWorld->x = pWorld->pointW[0].x;
		pWorld->y = pWorld->pointW[0].y;
		pWorld->z = pWorld->pointW[0].z;
		pWorld->a = 0;
		pWorld->b = 0;
		pWorld->c = 0;
		pWorld->fetchtype = 1;
	}

	return true;
}

}
