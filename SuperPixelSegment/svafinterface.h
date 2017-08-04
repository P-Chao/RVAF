#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

namespace pc{

Mat SuperPixelSegment(Mat& image, int K = 400, int M = 10, bool optint = true, bool saveseg = false, string segname = "./supix.seg");
bool EadpMatch(Mat& left, Mat& right, Mat& l_disp, Mat& r_disp, Mat& check, Mat& fill,
	int dlength = 24, int factor = 2560, int guildmr = 1, int dispmr = 1, 
	float sg = -25.0, float sc = 25.5, float r1 = 10, float r2 = 500,
	string prefix = "./eadp", bool savetxt = false);
bool SgmMatch(Mat& left, Mat& right, Mat& l_disp, Mat& r_disp, Mat& check, Mat& fill,
	int dlength = 24, int factor = 2560, int dispmr = 1, float r1 = 10, float r2 = 500,
	string prefix = "./sgm", bool savetxt = false);

}
