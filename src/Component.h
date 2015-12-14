#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class Component
{
public:
	Component(std::vector<Point2d> data);
	~Component();
	std::vector<Point2d> points;
	float mean, variance, median;
	int minx, miny, maxx, maxy;
	void getStats(Mat *SWTimage);
};
