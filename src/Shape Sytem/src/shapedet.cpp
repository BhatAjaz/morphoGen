//============================================================================
// Name        : shapedet.cpp
// Author      : cai
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#undef __GXX_EXPERIMENTAL_CXX0X__
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "TLdetector.hpp"
#include "shapedet.h"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

bool ShapeDet::train()
{
	method = TABLE_GROUPPT;//TABLE_PLAIN;//
	//string tr_yml_file = "/mnt/datagrid/personal/caihongp/darwin/S36_toy04/tiny_tr_data.yml";
	//string tr_bin_file = "/mnt/datagrid/personal/caihongp/darwin/S36_toy04/tiny_tr_data.bin";
	//string te_dir = "/mnt/datagrid/darwin/images/hongping/toy04/test/";
	string tr_yml_file = "data/tr_data_cmp8toys.yml";
	string tr_bin_file = "data/tr_data_cmp8toys.bin";
	string te_dir = "data/";
	string te_file1 = te_dir + "test_0%d.jpg";
	string te_file2 = te_dir + "test_%d.jpg";

	//////////////////////////////////////
	// train
	//////////////////////////////////////
	std::ifstream ifile(tr_bin_file.c_str());
	if (!ifile.good()) // binary file doesn't exist
		detector.transferTrainFileYml2Bin(tr_yml_file, tr_bin_file);
	bool flag = detector.train(tr_bin_file,method);
	return flag;
}

int ShapeDet::shapedet(cv::Mat& img, std::vector<DetObj>& detObj) 
{
	bool flag = false;

	flag = detector.detect(img,method);
	if (!flag) return 0;
	detector.showDetObjs(img,Scalar(0,255,0),Scalar(255,255,255),480);
	//waitKey(10);

	detObj = detector.getDetObjs();

	return 0;
}
//#pragma omp parallel for
//#pragma omp barrier
