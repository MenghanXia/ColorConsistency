#pragma once
#include <iostream>
#include <fstream>
#include <math.h>
#include "cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "QuadProg/QuadProg.h"
#include "correspondence.h"
#include "Utils/colorSpace.h"
#include "Utils/util.h"

using namespace std;
using namespace cv;

#define PARAM_NUM 6
#define SAMPLE_FREQ_NUM 15          //! used only for the final step : pixel-mapping (the more, the more smooth curve)
#define BKGRNDPIX 255


//! NOTICE: imgNo. is the ID of each image, which is labeled as the order in 'filePathList'
//! NOTICE: imgIndex is the position index in the relevant member vector, such as '_imageInforList', '_filePathList'
class ToneUnifier
{
public:
	ToneUnifier(vector<string> filePathList)
	{
		_filePathList = filePathList;
		_imgNum = filePathList.size();
	};

public:
	//! old API : text file
	void unifyMultiTones();
	//! initialization for data structure
	void initialization();
	//! extract key correspondences of histograms
	void buildCorrespondings();
	//! set control points of mapping curve
	void setCtrlPoints(const Mat &image, int imgIndex);
	//! computing interpolation coefficients for sample values
	void figureInterpolation();
	//! Notice: image index modification during call 'setFixedImages'
	//! before: all the image are recorded in the order of in '_filePathList' [_filePathList, _imageInforList]
	//! after : resort to keep reference images in the first few positions [_imageInforList, _filePathList, _savePathList]
	void setFixedImages(vector<int> fixedImgNos);
	//! global optimization : f(x) = (x'Hx + c'x)/2   s.t. Ax>=b  Bx=d
	void quadraticProgramming(int channelNo, Vec2d curvatureRange);

	//! update each image by mapping curves
	void applyColorRemappingforImages(bool needIndividuals, bool applyRemapping);
	void updateImagebyRemapping(Mat &YccImage, int imgIndex);
	vector<Point2d> interpSamplesbyBSpline(vector<Point2d> ctrlPoints, int freqNum, int channelNo);
	inline double interpValuebyLinear(double xi, const vector<Point2d> &samples);

	//! tool functions
	void getHistogramPercentileValues(const Mat &YCbCrImage, int imgIndex);
	void getMajorGradientComponents(const Mat &YCbCrImage, int imgIndex);
	double calInterpCoeff(double xi, double x1, double x2, double x3);
	int findImageIndex(int imgNo);
	double measureError(int channelNo);
	void outputEvaluationReport();
	void compareColorDiff(int channelNo, double &rme0, double &rme1);
	
private:
	int _imgNum;
	Size _imgSize;                     //! the warped image size (all the images)
	clock_t _stime, _etime;
	vector<int> _fixedImgList;         //! record no. of reference images
	vector<string> _filePathList;      //! modified as resorted order
	vector<string> _savePathList;      //! as resorted order
	
	vector<Utils::ImageParam> _imageInforList;    //! in the order of reference images in the first few position
};