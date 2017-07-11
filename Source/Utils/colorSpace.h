#pragma once
#include <iostream>
#include "cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Utils/util.h"

#define Max_Pix 255.0
#define Min_Pix 0.0
#define Inf_Num 0.00001

using namespace std;
using namespace cv;

//! Notification : image as a 3 channels matrix in OpenCV has the R,G,B index as 2, 1, 0
//! besides, we stock converted L,a,b channel in index of matrix as 0, 1, 2
//! RGB -> Lab : [0 256] of int give a range of L[-3.64672,9.59555] a[-2.21439,1.98422] b[-0.470511,0.467668]
//! besides, only the range of channel L is extended with the step refining

namespace ColorSpace
{
	//! ===== (uchar) RGB <---> (double) RGB
	Mat uRGB2dRGB(Mat srcImage);
	Mat dRGB2uRGB(Mat srcImage);

	//! ===== RGB <---> YCbCr
	Mat RGB2YCbCr(Mat srcImage);   //data type: uchar/double->double
	Mat YCbCr2RGB(Mat srcImage);   //data type: double->uchar
	Mat RGB2YCbCr(const Mat &srcImage, const vector<int> &roiIndexs);
	Mat YCbCr2RGB(const Mat &srcImage, const vector<int> &roiIndexs);
	Vec3i YCbCr2BGR(double Y, double Cb, double Cr);
	Vec3d RGB2YCbCr(uchar R, uchar G, uchar B);
};

