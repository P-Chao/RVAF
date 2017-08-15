#include "IAEstimateLayer.h"

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

	if (layer.mxmul_param().has_col0() && layer.mxmul_param().has_col1() && layer.mxmul_param().has_col2()){
		string col[3];
		col[0] = layer.sacia_param().mxmul_param().col0();
		col[1] = layer.sacia_param().mxmul_param().col1();
		col[2] = layer.sacia_param().mxmul_param().col2();
		stringstream ss;
		for (int i = 0; i < 2; ++i){
			ss.clear();
			ss << col[i];
			for (int j = 0; j < 3; ++j){
				ss >> M[i][j];
			}
		}
		LOG(INFO) << "Matrix Opened From Proto.";
	}
	else if (layer.sacia_param().mxmul_param().has_filename()){
		string filename = layer.sacia_param().mxmul_param().filename();
		FILE *fp = fopen(filename.c_str(), "rb+");
		CHECK_NOTNULL(fp);
		fread(&M[0], 4, 4, fp);
		fread(&M[1], 4, 4, fp);
		fread(&M[2], 4, 4, fp);
		fclose(fp);
		LOG(INFO) << "Matrix Opened From File " << filename;
	}
	else{
		LOG(ERROR) << "No Input Matrix, Create Identify Matrix!";
		M[0][0] = 1; M[0][1] = 0; M[0][2] = 0; M[0][3] = 0;
		M[0][0] = 0; M[0][1] = 1; M[0][2] = 0; M[0][3] = 0;
		M[0][0] = 0; M[0][1] = 0; M[0][2] = 1; M[0][3] = 0;
	}
}

IAEstimateLayer::~IAEstimateLayer()
{
}

bool IAEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){



	return true;
}

}
