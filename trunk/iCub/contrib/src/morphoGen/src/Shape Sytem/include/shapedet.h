#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "TLdetector.hpp"

class ShapeDet
{
private:
	CTLdetector detector;
	int method;
	std::string te_file1;
	std::string te_file2;
public:
	bool train();
	int shapedet(cv::Mat& img, std::vector<DetObj>& detObj);
};


