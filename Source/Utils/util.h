#pragma once
#include <iostream>
#include <fstream>
#include <math.h>
#include <Windows.h>
#include "cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

namespace Utils
{
	// ********** global variable list ********** //
	static string baseDir = "C:/Users/Richard/Desktop/Release/Data/";

	// ********** data struct list ********** //
	struct InterpInfo         //! record the NO. of involved ctrl-point for interpolation
	{                         //! and the interpolating coefficient
		int ctrlNos[3];
		double coeffs[3];
	};

	struct Adjacency
	{
		int neigbNo;
		vector<double> keyGVLists[3];         //! key gray values of channels
		vector<InterpInfo> interpCoeffs[3];   //! just for recording the interpolating parameter

		//! for other models
		double meanVs[3];
	};

	//! pixel intensity pair in luminance channel
	struct IntensityPair
	{
		int val0, val1;
		InterpInfo interp0, interp1;
	};

	//! property of each image
	struct ImageParam
	{
		int imgNo;
		vector<int> ROIIndexList; 
		vector<Point2d> ctrlPoints[3];     //! control-points of the mapping curve (3 channels)
		vector<Adjacency> neigbList;       //! matched keys of histogram with adjacent images
		vector<IntensityPair> gradMajors;  //! major gradient composition in luminance channel
		Vec2b gradminmaxVals;              //! intensity range of major gradient components (v0,v1)
		IntensityPair valuRange;           //! intensity range [low, up] of this image in luminance channel
	};

	// ********** tool function list ********** //
	vector<string> get_filelist(string foldname);
	//! detect roi regions
	void findBinaryROIMask(Mat &image, vector<int> &roiIndexs);
	//! extract pixels in overlaps
	vector<double>* extractROIPixels(const Mat &yccImage, const vector<int> &roiIndexs);

	//! for assessment
	double calStructuralDiffs(const Mat &srcImage, const Mat &dstImage, const vector<int> &roiIndexs);
	void getGradientOrientationMaps(Mat grayImage1, vector<double> &gradOrienVec1, Mat grayImage2, vector<double> &gradOrienVec2, const vector<int> &roiIndexs);

	//! warning: there is a maximum overlap pixel amount :25000000 [this can be reset freely]
	//! intersection of two mask pixel index set
	vector<int> intersectROIsPro(const vector<int> &roiIndexs1, const vector<int> &roiIndexs2);
	vector<int> intersectROIs(const vector<int> &roiIndexs1, const vector<int> &roiIndexs2);

	template<class InputIterator1,class InputIterator2,class OutputIterator>  
	OutputIterator set_intersection(InputIterator1 first1,InputIterator1 last1,InputIterator2 first2,InputIterator2 last2,OutputIterator result)  
	{  
		while (first1!=last1 && first2!=last2)
		{  
			if (*first1<*first2)           
				first1++;             
			else if (*first2<*first1)              
				first2++;         
			else  
			{     
				*result=*first1;  
				first1++;  
				first2++;         
				result++;  
			}         
		}     
		return result;    
	};
}
