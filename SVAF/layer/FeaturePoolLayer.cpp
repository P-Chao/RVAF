#include "FeaturePoolLayer.h"
#include <pcl/registration/icp.h>
#include <map>

using namespace pcl;

namespace svaf{

FeaturePoolLayer::FeaturePoolLayer(LayerParameter& layer) : StereoLayer(layer)
{
}


FeaturePoolLayer::~FeaturePoolLayer()
{
}

bool FeaturePoolLayer::Run(vector<Block>& images, vector<Block>& disp,
	LayerParameter& layer, void* param){
	return RunForFeaturePool(images, disp, layer, param);
}

bool FeaturePoolLayer::RunForFeaturePool(vector<Block>& images, vector<Block>& disp,
	LayerParameter& layer, void* param){
	pWorld_ = (World *)param;
	CHECK_GE(images.size(), 2) << "need pair images.";
	CHECK_NOTNULL(images[0].pMatch);

	int matchcount = 0;
	for (int i = 0; i < images[0].ptidx.size(); ++i){
		if (images[0].ptidx[i] >= 0){
			matchcount++;
		}
	}

	if (matchcount < 8){
		LOG(ERROR) << "No Enough Match Points!";
		LOG(ERROR) << "\nLoop Cut Short\n";
		return false;
	}

	if (featpool_.size() == 0){
		GenerateFeaturePool(images);
	}
	else{
		ExcuteEularMatch(images);
		LOG(INFO) << "Matched Point Pairs: " << pWorld_->matchpt0.size();

		if (__show || __save)
			DisplayResult(images, disp);
		if (pWorld_->matchpt0.size() < 4){
			LOG(ERROR) << "Point is not enough.";
			return false;
		} else{
			SolveRT(pWorld_->matchpt0, pWorld_->matchpt2);
			SolveRT(pWorld_->matchpt0, pWorld_->matchpt1);
		}
	}
	
	return true;
}

void FeaturePoolLayer::GenerateFeaturePool(vector<Block>& images){
	Block &image0 = images[0];
	Block &image1 = *images[0].pMatch;

	int j = 0;
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}
		Mat RT(3, 4, CV_32FC1);
		Feat feat_;
		feat_.imgIndex = left.size();
		feat_.refIndex = 0;
		feat_.despl = image0.despciptors[i];
		feat_.despr = image1.despciptors[image0.ptidx[i]];
		
		feat_.RT = RT.clone();
		feat_.pointL = pWorld_->pointL[j];
		 
		// used to display
		//feat_.left = image0.image.clone();
		//feat_.right = image1.image.clone();
		feat_.pointl = image0.points[i];
		feat_.pointr = image1.points[image0.ptidx[i]];

		featpool_.push_back(feat_);
		++j;
	}

	left.push_back(image0.image.clone());
	right.push_back(image1.image.clone());
}

bool FeaturePoolLayer::ExcuteEularMatch(vector<Block>& images){
	Block &image0 = images[0];
	Block &image1 = *images[0].pMatch;

	int count = 0;
	float d0, d1, dist;
	unsigned int match;

	const unsigned int desp_length = images[0].despciptors[0].size();

	int l_l_count = 0;
	int l_r_count = 0;
	int r_l_count = 0;
	int r_r_count = 0;

	l_l_idx.clear();
	l_r_idx.clear();
	r_l_idx.clear();
	r_r_idx.clear();

	vector<Point3f> refpt;
	vector<Point3f> curpt;

	// L - L
	l_l_idx.resize(image0.ptidx.size(), -1);
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}

		d0 = d1 = FLT_MAX;
		for (int j = 0; j < featpool_.size(); ++j){
			dist = 0.0f;
			for (int k = 0; k < desp_length; ++k){
				float diff = image0.despciptors[i][k] - featpool_[j].despl[k];
				dist += diff * diff;
			}
			dist = sqrt(dist);

			if (dist < d0){
				d1 = d0;
				d0 = dist;
				match = j;
			} else if (dist < d1){
				d1 = dist;
			}
		}
		
		if (d0 / d1 < 0.65){
			l_l_idx[i] = match;
			l_l_count++;
		}
	}

	// L - R
	l_r_idx.resize(image0.ptidx.size(), -1);
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}

		d0 = d1 = FLT_MAX;
		for (int j = 0; j < featpool_.size(); ++j){
			dist = 0.0f;
			for (int k = 0; k < desp_length; ++k){
				float diff = image0.despciptors[i][k] - featpool_[j].despr[k];
				dist += diff * diff;
			}
			dist = sqrt(dist);

			if (dist < d0){
				d1 = d0;
				d0 = dist;
				match = j;
			} else if (dist < d1){
				d1 = dist;
			}
		}

		if (d0 / d1 < 0.65){
			l_r_idx[i] = match;
			l_r_count++;
		}
	}

	// R - L
	r_l_idx.resize(image0.ptidx.size(), -1);
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}

		d0 = d1 = FLT_MAX;
		for (int j = 0; j < featpool_.size(); ++j){
			dist = 0.0f;
			for (int k = 0; k < desp_length; ++k){
				float diff = image1.despciptors[image0.ptidx[i]][k] - featpool_[j].despl[k];
				dist += diff * diff;
			}
			dist = sqrt(dist);

			if (dist < d0){
				d1 = d0;
				d0 = dist;
				match = j;
			} else if(dist < d1){
				d1 = dist;
			}
		}

		if (d0 / d1 < 0.65){
			r_l_idx[i] = match;
			r_l_count++;
		}
	}

	// R - R 
	r_r_idx.resize(image0.ptidx.size(), -1);
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}

		d0 = d1 = FLT_MAX;
		for (int j = 0; j < featpool_.size(); ++j){
			dist = 0.0f;
			for (int k = 0; k < desp_length; ++k){
				float diff = image1.despciptors[image0.ptidx[i]][k] - featpool_[j].despr[k];
				dist += diff * diff;
			}
			dist = sqrt(dist);

			if (dist < d0){
				d1 = d0;
				d0 = dist;
				match = j;
			} else if (dist < d1){
				d1 = dist;
			}
		}

		if (d0 / d1 < 0.65){
			r_r_idx[i] = match;
			r_r_count++;
		}
	}

	// 2^4
	pWorld_->matchpt0.clear();
	pWorld_->matchpt1.clear();
	pWorld_->matchpt2.clear();

	int j = 0;
	vector<int> match_cnt(image0.ptidx.size(), 0);
	for (int i = 0; i < image0.ptidx.size(); ++i){
		map<int, int> match_id;
		if (l_l_idx[i] >= 0){
			match_cnt[i]++;
			auto iter = match_id.find(l_l_idx[i]);
			if (iter != match_id.end()){
				match_id[l_l_idx[i]]++;
			} else{
				match_id[l_l_idx[i]] = 1;
			}
		}
		if (l_r_idx[i] >= 0){
			match_cnt[i]++;
			auto iter = match_id.find(l_r_idx[i]);
			if (iter != match_id.end()){
				match_id[l_r_idx[i]]++;
			} else{
				match_id[l_r_idx[i]] = 1;
			}
		}
		if (r_l_idx[i] >= 0){
			match_cnt[i]++;
			auto iter = match_id.find(r_l_idx[i]);
			if (iter != match_id.end()){
				match_id[r_l_idx[i]]++;
			} else{
				match_id[r_l_idx[i]] = 1;
			}
		}
		if (r_r_idx[i] >= 0){
			match_cnt[i]++;
			auto iter = match_id.find(r_r_idx[i]);
			if (iter != match_id.end()){
				match_id[r_r_idx[i]]++;
			} else{
				match_id[r_r_idx[i]] = 1;
			}
		}

		// select
		//vector<int> idx;
		int sel_id = -1, sel_count = 0;
		if (match_cnt[i] > 0){
			for (auto it : match_id){
				if (it.second > sel_count){
					sel_id = it.first;
					sel_count = it.second;
				}
			}
		}

		// filter
		if (match_cnt[i] < 2 || ((float)sel_count / (float)match_cnt[i] <= 0.5)){
			l_l_idx[i] = -1;
			l_r_idx[i] = -1;
			r_l_idx[i] = -1;
			r_r_idx[i] = -1;
		} else{
			// decide which point, save select
			//curpt.push_back(pWorld_->pointL[j]);
			//refpt.push_back(featpool_[sel_id].pointL);
			pWorld_->matchpt1.push_back(pWorld_->pointL[j]);
			pWorld_->matchpt0.push_back(featpool_[sel_id].pointL);
			pWorld_->matchpt2.push_back(pWorld_->xl[j]);
		}

		if (image0.ptidx[i] >= 0){
			j++;
		}
	}
	//pWorld_->matchpt0 = refpt;
	//pWorld_->matchpt1 = curpt;

	return true;
}

void FeaturePoolLayer::ClearFeaturePool(){
	featpool_.clear();
}

//Feat FeaturePoolLayer::SearchFeaturePool(vector<float>& ldesp, vector<float>& rdesp){
	// search ldesp
	// search rdesp
//}

void FeaturePoolLayer::DisplayResult(vector<Block>& images, vector<Block>& disp){
	Block &image0 = images[0];
	Block &image1 = *images[0].pMatch;

	const int cols = (cv::max)(image0.image.cols, left[0].cols) + (cv::max)(image1.image.cols, right[0].cols);
	const int rows = (cv::max)(image0.image.rows, image1.image.rows) + (cv::max)(left[0].rows, right[0].rows);

	CHECK_EQ(image0.image.type(), image1.image.type());
	CHECK_EQ(image0.image.type(), left[0].type());
	CHECK_EQ(image1.image.type(), right[0].type());

	Mat mat0 = image0.image.clone();
	Mat mat1 = image1.image.clone();
	Mat mat2 = left[0].clone();
	Mat mat3 = right[0].clone();

	Mat disp_img(rows, cols, image0.image.type());

	const Rect roi0((cv::max)(image0.image.cols, left[0].cols) - image0.image.cols, 0, image0.image.cols, image0.image.rows);
	const Rect roi1((cv::max)(image0.image.cols, left[0].cols), 0, image1.image.cols, image1.image.rows);
	const Rect roi2((cv::max)(image0.image.cols, left[0].cols) - left[0].cols, (cv::max)(image0.image.rows, image1.image.rows), left[0].cols, left[0].rows);
	const Rect roi3((cv::max)(image0.image.cols, left[0].cols), (cv::max)(image0.image.rows, image1.image.rows), right[0].cols, right[0].rows);

	mat0.copyTo(disp_img(roi0));
	mat1.copyTo(disp_img(roi1));
	mat2.copyTo(disp_img(roi2));
	mat3.copyTo(disp_img(roi3));

	Point2f pt0, pt1, pt2, pt3;
	for (int i = 0; i < image0.ptidx.size(); ++i){
		// tl - tr
		if (image0.ptidx[i] >= 0){
			pt0 = image0.points[i];
			pt1 = image1.points[image0.ptidx[i]];
			pt0.x += roi0.x; pt0.y += roi0.y;
			pt1.x += roi1.x; pt1.y += roi1.y;
			line(disp_img, pt0, pt1, Scalar(255, 255, 255));
		}
		
		// tl - b1
		if (l_l_idx[i] >= 0){
			pt0 = image0.points[i];
			pt2 = featpool_[l_l_idx[i]].pointl;
			pt0.x += roi0.x; pt0.y += roi0.y;
			pt2.x += roi2.x; pt2.y += roi2.y;
			line(disp_img, pt0, pt2, Scalar(255, 128, 0));
		}

		// t1 - br
		if (l_r_idx[i] >= 0){
			pt0 = image0.points[i];
			pt3 = featpool_[l_r_idx[i]].pointr;
			pt0.x += roi0.x; pt0.y += roi0.y;
			pt3.x += roi3.x; pt3.y += roi3.y;
			line(disp_img, pt0, pt3, Scalar(255, 255, 128));
		}

		// tr - b1
		if (r_l_idx[i] >= 0){
			pt1 = image1.points[image0.ptidx[i]];
			pt2 = featpool_[r_l_idx[i]].pointl;
			pt1.x += roi1.x; pt1.y += roi1.y;
			pt2.x += roi2.x; pt2.y += roi2.y;
			line(disp_img, pt1, pt2, Scalar(0, 128, 255));
		}

		// tr - br
		if (r_r_idx[i] >= 0){
			pt1 = image1.points[image0.ptidx[i]];
			pt3 = featpool_[r_r_idx[i]].pointr;
			pt1.x += roi1.x; pt1.y += roi1.y;
			pt3.x += roi3.x; pt3.y += roi3.y;
			line(disp_img, pt1, pt3, Scalar(128, 255, 128));
		}
	}

	// bl - br
	for (int i = 0; i < featpool_.size(); ++i){
		pt2 = featpool_[i].pointl;
		pt3 = featpool_[i].pointr;
		pt2.x += roi2.x; pt2.y += roi2.y;
		pt3.x += roi3.x; pt3.y += roi3.y;
		line(disp_img, pt2, pt3, Scalar(255, 0, 128));
	}

	disp.push_back(Block("FeaturePool Match", disp_img, __show, __save));
}

int FeaturePoolLayer::SearchFeaturePool(vector<float>& ldesp, vector<float>& rdesp){
	// search ldesp
	int count = 0;
	float d0, d1, dist;
	unsigned int match;
	const int desp_length = ldesp.size();
	for (int i = 0; i < featpool_.size(); ++i){

	}

	// search rdesp
	for (int i = 0; i < featpool_.size(); ++i){

	}

	return 0;
}

void FeaturePoolLayer::StoreToFeaturePool(){

}

Mat FeaturePoolLayer::SolveRT(vector<Point3f>& ops, vector<Point3f>& ips){
	CHECK_EQ(ops.size(), ips.size()) << "ops and ips should have save size.";

	Mat ip(ips), op(ops);
	Mat R, out_inliers;
	estimateAffine3D(ip, op, R, out_inliers);
	for (int i = 0; i < R.rows; ++i){
		for (int j = 0; j < R.cols; ++j){
			printf("%f\t", R.ptr<double>(i)[j]);
		}
		printf("\n");
	}

	// use pcl icp
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_in->width = ips.size();
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (int i = 0; i < ips.size(); ++i){
		cloud_in->points[i].x = ips[i].x;
		cloud_in->points[i].y = ips[i].y;
		cloud_in->points[i].z = ips[i].z;
	}
	
	cloud_out->width = ops.size();
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->points.resize(cloud_out->width * cloud_out->height);
	for (int i = 0; i < ops.size(); ++i){
		cloud_out->points[i].x = ops[i].x;
		cloud_out->points[i].y = ops[i].y;
		cloud_out->points[i].z = ops[i].z;
	}

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	Mat RT(3, 4, CV_32FC1);
	return R;
}

Mat FeaturePoolLayer::SolveRT(vector<Point3f>& ops, vector<Point2f>& ips){
	CHECK_EQ(ops.size(), ips.size()) << "ops and ips should have save size.";

	Mat op(ops);
	Mat ip(ips);
	Mat rvec(3, 1, CV_64FC1);
	Mat tvec(3, 1, CV_64FC1);
	Mat R;

	double _cm[9] = {1372, 0, 120.7515,
					 0,	1272, 214.0282,
					 0, 0,	  1};
	Mat camMatrix(3, 3, CV_64FC1, _cm);

	double _dc[] = {0, 0, 0, 0};

	solvePnP(op, ip, camMatrix, Mat(1, 4, CV_64FC1, _dc),
		rvec, tvec, false);
	Rodrigues(rvec, R);

	for (int i = 0; i < R.rows; ++i){
		for (int j = 0; j < R.cols; ++j){
			printf("%f\t", R.ptr<double>(i)[j]);
		}
		printf("%f\t", tvec.ptr<double>(i)[0]);
		printf("  %f\n", rvec.ptr<double>(i)[0]);
	}
	printf("\n");

	solvePnPRansac(op, ip, camMatrix, Mat(1, 4, CV_64FC1, _dc),
		rvec, tvec, false);
	Rodrigues(rvec, R);

	for (int i = 0; i < R.rows; ++i){
		for (int j = 0; j < R.cols; ++j){
			printf("%f\t", R.ptr<double>(i)[j]);
		}
		printf("%f\t", tvec.ptr<double>(i)[0]);
		printf("  %f\n", rvec.ptr<double>(i)[0]);
	}

	vector<CvPoint3D32f> modelPoints;
	vector<CvPoint2D32f> imagePoints;
	for (int i = 0; i < ops.size(); ++i){
		modelPoints.push_back(cvPoint3D32f(ops[i].x, ops[i].y, ops[i].z));
		imagePoints.push_back(cvPoint2D32f(ips[i].x, ips[i].y));
	}

	CvPOSITObject* positObject = cvCreatePOSITObject(&modelPoints[0], modelPoints.size());

	
	float rotation_matrix[9];
	float translation_vector[3];
	CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f);

	cvPOSIT(positObject, &imagePoints[0], 1372, criteria, rotation_matrix, translation_vector);
	R = Mat(3, 3, CV_64FC1, rotation_matrix);
	Rodrigues(R, rvec);
	tvec = Mat(3, 1, CV_64FC1, translation_vector);

	for (int i = 0; i < R.rows; ++i){
		for (int j = 0; j < R.cols; ++j){
			printf("%f\t", R.ptr<float>(i)[j]);
		}
		printf("%f\t", tvec.ptr<float>(i)[0]);
		printf("  %f\n", rvec.ptr<float>(i)[0]);
	}

	//pose_dementhon(ops, ips, tvec, R);

	//Mat RT(3, 4, CV_32FC1);
	//Rodrigues(R, rvec);

	/*for (int i = 0; i < R.rows; ++i){
		for (int j = 0; j < R.cols; ++j){
			printf("%f\t", R.ptr<double>(i)[j]);
		}
		printf("%f\t", tvec.ptr<double>(i)[0]);
		printf("  %f\n", rvec.ptr<double>(i)[0]);
	}*/
	

	return Mat(3, 3, CV_32FC1);
}

}

