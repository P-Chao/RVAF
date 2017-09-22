#pragma once

#include <opencv2\opencv.hpp>

#include "Param.h"
#include "Figures.h"
#include <windows.h>

using namespace std;
using namespace cv;

namespace svaf{

class Layer;

template<typename T>
struct _Node{
	T		name;
	Layer*	layer;
	void*	param;
	_Node*	next;
	_Node(T str):name(str), next(NULL), layer(NULL), param(NULL){}
};
typedef _Node<string> Node;

typedef struct _World{
	bool		rectified;
	float		x, y, z, a, b, c; // 抓取点，和抓取方向，世界坐标

	vector<Point2f> xl, xr;
	vector<Point3f> pointL, pointR;
	vector<Point3f> pointW; // Left Camera -> world

	vector<Point3f> matchpt1;
	vector<Point3f> matchpt0;
	vector<Point2f> matchpt2;
} World;

typedef struct _Block{
	string	name;
	Mat		image;
	Rect	roi;
	
	vector<Point2f>	points;
	vector<float>	points_sc;
	vector<vector<float>>	despciptors;

	vector<KeyPoint>	keypoint;
	Mat					descriptors;
	vector<DMatch>		matches;

	_Block*	pMatch;
	vector<int>		ptidx;
	vector<Point3f>	point3d;

	bool isSave;
	bool isShow;

	_Block(string str, Mat& mat, bool isshow = true, bool issave = false) 
		:name(str), image(mat), isShow(isshow), isSave(issave), pMatch(NULL),
		roi(Rect(0, 0, mat.cols, mat.rows)){}
	_Block() : roi(Rect(0, 0, 0, 0)), isShow(true), isSave(false), pMatch(NULL){}
} Block;

class Circuit
{
public:
	explicit Circuit(SvafTask&, bool);
	~Circuit();

	static string time_id_;

protected:
	void Build();
	void Run();
	void RunStep();
	void InitStep();
	void EndStep();
	bool Disp();
	void Analysis();
	bool ReciveCmd();
	void SendData();

protected:
	cv::VideoCapture cap_[2];

	bool			useMapping_;
	HANDLE			c_fileMapping_;//cmd
	HANDLE			c_mutex_;
	LPTSTR			c_pMsg_;
	HANDLE			d_fileMapping_;//data
	HANDLE			d_mutex_;
	LPTSTR			d_pMsg_;

private:
	int			pause_ms_;
	Param		layers_;
	Block		block_;
	SvafTask	svaf_;
	Node*		linklist_;
	vector<Block>	images_;  // always clear begin frame;
	vector<Block>	disp_;

	World		world_;
	unsigned __int64 id_;
	static Figures<float> sout_;

};

}
