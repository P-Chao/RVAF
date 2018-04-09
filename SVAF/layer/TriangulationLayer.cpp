/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
调用Matlab工具箱从二维视差恢复三维坐标
*/

#include "TriangulationLayer.h"
#include <direct.h>

namespace svaf{

// 构造函数
TriangulationLayer::TriangulationLayer(LayerParameter& layer) : StereoLayer(layer)
{
	// 设置工具箱路径
	if (layer.triang_param().has_toolbox_dir()){
		toolbox_dir = layer.triang_param().toolbox_dir();
	}else{
		toolbox_dir = "D:/Program Files/Matlab/R2016a/toolbox/calib/";
	}

	// 设置标定文件路径
	if (layer.triang_param().has_calibmat_dir()){
		calibmat_dir = layer.triang_param().calibmat_dir();
	}else{
		calibmat_dir = "./calib/";
	}
	isMatlabVisible = layer.triang_param().visible(); // 设置Matlab窗口是否可见
	isSavePointCloud = layer.triang_param().savepc(); // 是否保存点云
	OpenMatlab(); // 打开Matlab
}

// 析构函数
TriangulationLayer::~TriangulationLayer()
{
	CloseMatlab(); // 关闭Matlab
}

// 运行算法
bool TriangulationLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_NOTNULL(images[0].pMatch);
	
	// 读取之前算法产生的数据
	pWorld_ = (World *)param;
	pWorld_->xl.clear();
	pWorld_->xr.clear();
	pWorld_->pointL.clear();
	pWorld_->pointR.clear();
	Block& image0 = images[0];
	Block& image1 = *images[0].pMatch;

	// Run For ROI
	if (images.size() > 1 && images[1].useroi){
		double xx, yy, zz;
		ComputeWorld(image0.xct, image0.yct, image1.xct, image1.yct,
			xx, yy, zz);
		pWorld_->x = xx;
		pWorld_->y = yy;
		pWorld_->z = zz;
	}

	// 将左右图像对应点转存起来
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}
		Point2f pt0 = image0.points[i];
		Point2f pt1 = image1.points[image0.ptidx[i]];
		pt0.x += image0.roi.x;
		pt0.y += image0.roi.y;
		pt1.x += image1.roi.x;
		pt1.y += image1.roi.y;
		pWorld_->xl.push_back(pt0);
		pWorld_->xr.push_back(pt1);
	}
	
	//CHECK_GE(pWorld_->xl.size(), 4) << "Need More Point.";
	if (pWorld_->xl.size() < 4){
		LOG(ERROR) << "More Than 4 Point Needed to Estimate Position and Attitude.";
	}

	isMatlabVisible = layer.triang_param().visible();
	char loginfo[120];
	string logstr;
	
	// 由二维坐标计算三维坐标
	__t.StartWatchTimer();
	ComputeWorld();
	__t.ReadWatchTimer("Triang Compute Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	if (Layer::task_type == PC_TRIANGLE){
		__bout = true;
	} else{
		__bout = false;
	}

	// 输出点云
	if (__bout){
		Mat im;
		Block block("Point Cloud Camera", im, false, false, __bout);
		block.isOutput3DPoint = true;
		block.point3d = pWorld_->pointW;
		disp.push_back(block);
	}

	// 命令行显示
	if (pWorld_->pointL.size() < 20){
		for (int i = 0; i < pWorld_->xl.size(); ++i){
			sprintf(loginfo, "(%8.3f, %8.3f) (%8.3f, %8.3f)\t(%8.3f, %8.3f, %8.3f)\n",
				pWorld_->xl[i].x, pWorld_->xl[i].y, pWorld_->xr[i].x, pWorld_->xr[i].y,
				pWorld_->pointL[i].x, pWorld_->pointL[i].y, pWorld_->pointL[i].z);
			logstr += loginfo;
		}
		LOG(INFO) << "Left Camera Point: \n \t xl\t\t\t xr \t\t\t pointL\n" << logstr;
	}else{
		LOG(INFO) << "Left Camera Point Count " << pWorld_->pointL.size();
	}
	
	// 保存点云
	if (__save){
		pcdsave(string("tmp/C_") + Circuit::time_id_ + ".pcd", pWorld_->pointL);// camera coord
		//LOG(INFO) << "ref_pointcloud.pc Point Cloud File Has Been Saved.";
	}

	HWND hw = ::FindWindow(NULL, "right Adaboost");
	if (hw != NULL){
		::SetForegroundWindow(hw);
	}

	return true;
}

// 调用Matlab工具箱，由左右视图坐标计算三维坐标
void TriangulationLayer::ComputeWorld(){
	// 如果之前没有打开Matlab，则打开Matlab
	if (!m_Ep){
		OpenMatlab();
	}
	const int pointCount = pWorld_->xl.size();
	pWorld_->pointL.resize(pointCount);
	pWorld_->pointR.resize(pointCount);

	// 将左右图像对应点坐标读入Matlab
	Matlab::mxArray* leftPoint = Matlab::mxCreateDoubleMatrix(2, pointCount, Matlab::mxREAL);
	Matlab::mxArray* rightPoint = Matlab::mxCreateDoubleMatrix(2, pointCount, Matlab::mxREAL);
	for (int i = 0; i < pointCount; ++i){
		Matlab::mxGetPr(leftPoint)[2 * i] = pWorld_->xl[i].x;
		Matlab::mxGetPr(leftPoint)[2 * i + 1] = pWorld_->xl[i].y;
		Matlab::mxGetPr(rightPoint)[2 * i] = pWorld_->xr[i].x;
		Matlab::mxGetPr(rightPoint)[2 * i + 1] = pWorld_->xr[i].y;
	}
	// 调用工具箱计算三维坐标
	Matlab::engEvalString(m_Ep, "clear xL, clear xR");
	Matlab::engPutVariable(m_Ep, "xL", leftPoint);
	Matlab::engPutVariable(m_Ep, "xR", rightPoint);
	if (pWorld_->rectified){
		Matlab::engEvalString(m_Ep, "[XL, XR] = stereo_triangulation(xL, xR, om_new, T_new, fc_left_new, cc_left_new, kc_left_new, alpha_c_left_new, fc_right_new, cc_right_new, kc_right_new, alpha_c_right_new); ");
	} else{
		Matlab::engEvalString(m_Ep, "[XL, XR] = stereo_triangulation(xL, xR, om, T, fc_left, cc_left, kc_left, alpha_c_left, fc_right, cc_right, kc_right, alpha_c_right); ");
	}
	
	// 从Matlab读出左右相机坐标系下的三维坐标
	Matlab::mxArray* xL = Matlab::engGetVariable(m_Ep, "XL");
	Matlab::mxArray* xR = Matlab::engGetVariable(m_Ep, "XR");

	CHECK_NOTNULL(xL);
	CHECK_NOTNULL(xR);

	for (int i = 0; i < pointCount; ++i){
		pWorld_->pointL[i].x = Matlab::mxGetPr(xL)[3 * i + 0];
		pWorld_->pointL[i].y = Matlab::mxGetPr(xL)[3 * i + 1];
		pWorld_->pointL[i].z = Matlab::mxGetPr(xL)[3 * i + 2];
		pWorld_->pointR[i].x = Matlab::mxGetPr(xR)[3 * i + 0];
		pWorld_->pointR[i].y = Matlab::mxGetPr(xR)[3 * i + 1];
		pWorld_->pointR[i].z = Matlab::mxGetPr(xR)[3 * i + 2];
	}

	// 调用Matlab的Figure窗口显示三维点云
	if (__show){
		Matlab::engEvalString(m_Ep, "figure(1), scatter3(XL(1, :), XL(2, :), XL(3, :));");
		HWND hw = ::FindWindow(NULL, "right Adaboost");
		if (hw != NULL){
			::SetForegroundWindow(hw);
		}
	}

	Matlab::mxDestroyArray(leftPoint);
	Matlab::mxDestroyArray(rightPoint);

	// openCV版本
	//Matlab::mxArray* om_new = Matlab::engGetVariable(m_Ep, "om_new");
	//Matlab::mxArray* T_new = Matlab::engGetVariable(m_Ep, "T_new");
	//Matlab::mxArray* KK_left_new = Matlab::engGetVariable(m_Ep, "KK_left_new");
	//Matlab::mxArray* KK_right_new = Matlab::engGetVariable(m_Ep, "KK_right_new");

	//Mat proj1(3, 4, CV_32FC1);
	//Mat proj2(3, 4, CV_32FC1);
	//Mat om(3, 1, CV_32FC1);
	//Mat R(3, 3, CV_32FC1);
	//Mat T(3, 1, CV_32FC1);
	//Mat KK_left(3, 3, CV_32FC1);
	//Mat KK_right(3, 3, CV_32FC1);

	//float* data = om.ptr<float>(0);
	//data[0] = Matlab::mxGetPr(om_new)[0];
	//data[1] = Matlab::mxGetPr(om_new)[1];
	//data[2] = Matlab::mxGetPr(om_new)[2];
	//Rodrigues(om, R);
	//
	//data = T.ptr<float>(0);
	//data[0] = Matlab::mxGetPr(T_new)[0];
	//data[1] = Matlab::mxGetPr(T_new)[1];
	//data[2] = Matlab::mxGetPr(T_new)[2];
	//
	//for (int i = 0; i < 3; ++i){
	//	float * ldata = KK_left.ptr<float>(i);
	//	float * rdata = KK_right.ptr<float>(i);
	//	for (int j = 0; j < 3; ++j){
	//		ldata[j] = Matlab::mxGetPr(KK_left_new)[3 * j + i];
	//		rdata[j] = Matlab::mxGetPr(KK_right_new)[3 * j + i];
	//		cout << ldata[j] << endl;
	//	}
	//	cout << endl;
	//}

	//proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
	//proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

	//R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T.convertTo(proj2.col(3), CV_32FC1);

	////proj1 = KK_left * proj1;
	////proj2 = KK_right * proj2;
	//KK_left.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
	//KK_right.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	//proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);
	//proj2.col(3) = Mat::zeros(3, 1, CV_32FC1);

	//vector<Point2f>& pt1 = pWorld_->xl;
	//vector<Point2f>& pt2 = pWorld_->xr;

	//Mat aaa;
	//triangulatePoints(proj1, proj2, pt1, pt2, aaa);

	//for (int i = 0; i < aaa.cols; ++i){
	//	cout << aaa.ptr<float>(i)[0] << ", " << aaa.ptr<float>(i)[1] << ", " << aaa.ptr<float>(i)[2] << ", " << aaa.ptr<float>(i)[3] << endl;
	//	cout << pWorld_->pointL[i].x << ", " << pWorld_->pointL[i].y << ", " << pWorld_->pointL[i].z << ", " << endl;
	//}

	//waitKey();
}

void TriangulationLayer::ComputeWorld(double lx, double ly, double rx, double ry,
	double& xx, double& yy, double& zz){

	// 如果之前没有打开Matlab，则打开Matlab
	if (!m_Ep){
		OpenMatlab();
	}
	const int pointCount = 1;
	pWorld_->pointL.resize(pointCount);
	pWorld_->pointR.resize(pointCount);

	// 将左右图像对应点坐标读入Matlab
	Matlab::mxArray* leftPoint = Matlab::mxCreateDoubleMatrix(2, pointCount, Matlab::mxREAL);
	Matlab::mxArray* rightPoint = Matlab::mxCreateDoubleMatrix(2, pointCount, Matlab::mxREAL);
	//for (int i = 0; i < pointCount; ++i){
		Matlab::mxGetPr(leftPoint)[0] = lx;
		Matlab::mxGetPr(leftPoint)[1] = ly;
		Matlab::mxGetPr(rightPoint)[0] = rx;
		Matlab::mxGetPr(rightPoint)[1] = ry;
	//}
	// 调用工具箱计算三维坐标
	Matlab::engEvalString(m_Ep, "clear xL, clear xR");
	Matlab::engPutVariable(m_Ep, "xL", leftPoint);
	Matlab::engPutVariable(m_Ep, "xR", rightPoint);
	if (pWorld_->rectified){
		Matlab::engEvalString(m_Ep, "[XL, XR] = stereo_triangulation(xL, xR, om_new, T_new, fc_left_new, cc_left_new, kc_left_new, alpha_c_left_new, fc_right_new, cc_right_new, kc_right_new, alpha_c_right_new); ");
	}
	else{
		Matlab::engEvalString(m_Ep, "[XL, XR] = stereo_triangulation(xL, xR, om, T, fc_left, cc_left, kc_left, alpha_c_left, fc_right, cc_right, kc_right, alpha_c_right); ");
	}

	// 从Matlab读出左右相机坐标系下的三维坐标
	Matlab::mxArray* xL = Matlab::engGetVariable(m_Ep, "XL");
	Matlab::mxArray* xR = Matlab::engGetVariable(m_Ep, "XR");

	CHECK_NOTNULL(xL);
	CHECK_NOTNULL(xR);

	//for (int i = 0; i < pointCount; ++i){
		xx = Matlab::mxGetPr(xL)[0];
		yy = Matlab::mxGetPr(xL)[1];
		zz = Matlab::mxGetPr(xL)[2];
		
	//}

	// 调用Matlab的Figure窗口显示三维点云
	/*if (__show){
		Matlab::engEvalString(m_Ep, "figure(1), scatter3(XL(1, :), XL(2, :), XL(3, :));");
		HWND hw = ::FindWindow(NULL, "right Adaboost");
		if (hw != NULL){
			::SetForegroundWindow(hw);
		}
	}*/

	Matlab::mxDestroyArray(leftPoint);
	Matlab::mxDestroyArray(rightPoint);

}

// 打开Matlab并加载工具箱和数据
void TriangulationLayer::OpenMatlab(){
	if (!(m_Ep = Matlab::engOpen(nullptr))){
		LOG(FATAL) << "engOpen failed!";
		return;
	}
	char matlab_cmd[256];
	char current_path[256];
	string current;
	getcwd(current_path, 256);
	current = current_path;
	sprintf(matlab_cmd, "cd %s", current.c_str());
	Matlab::engEvalString(m_Ep, matlab_cmd);
	Matlab::engSetVisible(m_Ep, isMatlabVisible);
	sprintf(matlab_cmd, "addpath(\'%s')", toolbox_dir.c_str());
	Matlab::engEvalString(m_Ep, matlab_cmd);
	sprintf(matlab_cmd, "cd %s", calibmat_dir.c_str());
	Matlab::engEvalString(m_Ep, matlab_cmd);
	Matlab::engEvalString(m_Ep, "clear");
	Matlab::engEvalString(m_Ep, "load('Calib_Results_stereo.mat')");
	Matlab::engEvalString(m_Ep, "load('Calib_Results_stereo_rectified.mat')");
}

// 关闭Matlab
void TriangulationLayer::CloseMatlab(){
	Matlab::engClose(m_Ep);
}

}
