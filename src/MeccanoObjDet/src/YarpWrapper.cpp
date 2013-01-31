
#undef __GXX_EXPERIMENTAL_CXX0X__
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "yarp/os/all.h"
#include "yarp/sig/all.h"
#include "TLdetector.hpp"
#include <iostream>
#include <fstream>

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

	string para_yml_file = "data/para_cmp8toys.yml";
	/////////////////////////////////////////////////
	// STEP1: initiate
	/////////////////////////////////////////////////
	bool flag;
	CTLdetector detector;
	flag = detector.initiate(para_yml_file);
	if (!flag)		return 0;

	/////////////////////////////////////////////////
	// STEP2: train
	/////////////////////////////////////////////////
	flag = detector.train();
	if (!flag)		return 0;

	/////////////////////////////////////////////////
	// STEP3: detect
	/////////////////////////////////////////////////
	int key = 0;
	cv::Mat cvMatImage;
	std::cout<<"***Detecting..."<<std::endl;
	while(key != 27)
	{
		ImageOf<PixelRgb> *img = imageInputPort->read(true);
		cvMatImage.create(img->height(), img->width(), CV_8UC3);
		unsigned char* ptr = img->getRawImage();
		memcpy(cvMatImage.data, ptr, cvMatImage.cols * cvMatImage.rows * 3);
		cv::cvtColor(cvMatImage, cvMatImage, CV_RGB2BGR);

		detector.detect(cvMatImage);
		detector.showDetObjs(cvMatImage,Scalar(0,255,0),Scalar(255,255,255),480);
		//detector.dispDetObjs();
		std::vector<DetObj> objects;
		objects = detector.getDetObjs();

		Bottle output = bbOutputPort->prepare();
		for (int i = 0; i < objects.size(); i++)
		{
			output.addInt(objects[i].box_tight.x);
			output.addInt(objects[i].box_tight.y);
			output.addInt(objects[i].box_tight.width);
			output.addInt(objects[i].box_tight.height);
			output.addInt(objects[i].id_label);
			//if want to know the object name: detector.all_obj_cls[objects[i].id_label]
		}

		// free the memory of the objects.
		for (int i=0;i<objects.size();i++){
			objects[i].mat_edge_NN_tr.release();
		}
		objects.clear();
		bbOutputPort->write();
		key = cv::waitKey(100);
	}
	cvMatImage.release();
	std::cout<<"***Done."<<std::endl;
	return 0;
}

