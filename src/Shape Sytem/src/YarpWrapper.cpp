
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "yarp/os/all.h"
#include "yarp/sig/all.h"
#include "iostream"
#include "shapedet.h"

using namespace yarp::os;
using namespace yarp::sig;

int main(char** argv, int argc)
{
	Network network;

	BufferedPort<ImageOf<PixelRgb>>* imageInputPort = new BufferedPort<ImageOf<PixelRgb>>();
	BufferedPort<Bottle>* bbOutputPort = new BufferedPort<Bottle>();

	imageInputPort->open("/img:i");
	bbOutputPort->open("/bb:o");

	network.connect("/cam1:o", "/img:i");

	ShapeDet shapeDet;
	shapeDet.train();

	while(true)
	{
		ImageOf<PixelRgb> *img = imageInputPort->read(true);

		cv::Mat cvMatImage(img->height(), img->width(), CV_8UC3);
		unsigned char* ptr = img->getRawImage();
		memcpy(cvMatImage.data, ptr, cvMatImage.cols * cvMatImage.rows * 3);

		cv::cvtColor(cvMatImage, cvMatImage, CV_RGB2BGR);

		std::cout<<cvMatImage.rows<<" "<<cvMatImage.cols<<" "<<cvMatImage.channels()<<std::endl;
		/*cv::Mat grayImage(img->height(), img->width(), CV_8U);
		cv::cvtColor(cvMatImage, grayImage, CV_RGB2GRAY);
		cv::Canny(grayImage,grayImage,60, 180);
		cv::imshow("received image", cvMatImage); 
		cv::waitKey(0);*/
		std::vector<DetObj> objects;
		shapeDet.shapedet(cvMatImage, objects);

		Bottle output = bbOutputPort->prepare();
		for (int i = 0; i < objects.size(); i++)
		{
			output.addInt(objects[i].box_tight.x);
			output.addInt(objects[i].box_tight.y);
			output.addInt(objects[i].box_tight.width);
			output.addInt(objects[i].box_tight.height);
			output.addInt(objects[i].id_label);
		}
		bbOutputPort->write();
		cv::waitKey(30);
	}

	return 0;
}