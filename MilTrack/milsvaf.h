#include "common.h"
#include <opencv2\opencv.hpp>

using namespace cv;

namespace pc{

void miltrack_frame(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void miltrack_firstframe(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void adatrack_frame(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void adatrack_firstframe(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
typedef  void(*TrackFun)(Mat&, pc::Rect&, TrackParam&, FeatureParam&);

void miltrack_firstframe_sync(Mat&, pc::Rect&, TrackParam&, FeatureParam&,
	Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void miltrack_frame_sync(Mat&, pc::Rect&, TrackParam&, FeatureParam&,
	Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void miltrack_firstframe_sync_mc(Mat&, pc::Rect&, TrackParam&, FeatureParam&,
	Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void miltrack_frame_sync_mc(Mat&, pc::Rect&, TrackParam&, FeatureParam&,
	Mat&, pc::Rect&, TrackParam&, FeatureParam&);
typedef	 void(*BinoTrackFun)(Mat&, pc::Rect&, TrackParam&, FeatureParam&,
	Mat&, pc::Rect&, TrackParam&, FeatureParam&);

void on_mouse(int, int, int, int, void *);

}
