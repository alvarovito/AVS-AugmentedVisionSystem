#include "Component.h"

#define PI 3.14159265

Component::Component(std::vector<Point2d> data)
{
	points = data;
	stats = false;
}


Component::~Component()
{
}

void Component::getStats(Mat * SWTimage){
	std::vector<float> temp;
	temp.reserve(this->points.size());
	this->variance = 0;
	this->mean = 0;
	this->minx = 1000000;
	this->miny = 1000000;
	this->maxx = 0;
	this->maxy = 0;
	for (std::vector<Point2d>::iterator it = this->points.begin(); it != this->points.end(); it++){
		float stroke = SWTimage->at<float>(it->y, it->x);
		this->mean += stroke;
		temp.push_back(stroke);
		this->minx = min(minx, (int)it->x);
		this->miny = min(miny, (int)it->y);
		this->maxx = max(maxx, (int)it->x);
		this->maxy = max(maxy, (int)it->y);
	}
	mean = mean / ((float)points.size());
	for (std::vector<float>::const_iterator it = temp.begin(); it != temp.end(); it++) {
		variance += (*it - mean) * (*it - mean);
	}
	variance = variance / ((float)points.size());
	std::sort(temp.begin(), temp.end());
	median = temp[temp.size() / 2];
	stats = true;
}

int Component::getWidth(){
	assert(stats);
	return (maxx - minx);
}

int Component::isValid(){
	assert(stats);
	/*int width = (int)(maxx - minx + 1);
	int height = (int)(maxy - miny + 1);*/
	if (variance > 0.5 * mean){
		return 0;
	}
	float length = (float)(maxx - minx + 1);
	float width = (float)(maxy - miny + 1);

	// check font height
	if (width > 60) {
		return 0;
	}
	if (points.size() < 10) {
		return 0;
	}

	float area = length * width;
	float rminx = (float)minx;
	float rmaxx = (float)maxx;
	float rminy = (float)miny;
	float rmaxy = (float)maxy;
	// compute the rotated bounding box
	float increment = 1. / 36.;
	for (float theta = increment * PI; theta<PI / 2.0; theta += increment * PI) {
		float xmin, xmax, ymin, ymax, xtemp, ytemp, ltemp, wtemp;
		xmin = 1000000;
		ymin = 1000000;
		xmax = 0;
		ymax = 0;
		for (unsigned int i = 0; i < points.size(); i++) {
			xtemp = points[i].x * cos(theta) + points[i].y * -sin(theta);
			ytemp = points[i].x * sin(theta) + points[i].y * cos(theta);
			xmin = std::min(xtemp, xmin);
			xmax = std::max(xtemp, xmax);
			ymin = std::min(ytemp, ymin);
			ymax = std::max(ytemp, ymax);
		}
		ltemp = xmax - xmin + 1;
		wtemp = ymax - ymin + 1;
		if (ltemp*wtemp < area) {
			area = ltemp*wtemp;
			length = ltemp;
			width = wtemp;
		}
	}
	// check if the aspect ratio is between 1/10 and 10
	if (length / width < 1. / 10. || length / width > 10.) {
		return 0;
	}

	return 1;
}