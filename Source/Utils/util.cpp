#include "util.h"

using namespace Utils;

vector<string> Utils::get_filelist(string foldname)
{
	string foldName_copy = foldname;
	foldname += "/*.*";
	const char * mystr=foldname.c_str();
	vector<string> flist;
	string lineStr;
	vector<string> extendName;
	extendName.push_back("jpg");
	extendName.push_back("JPG");
	extendName.push_back("bmp");
	extendName.push_back("png");
	extendName.push_back("tif");

	HANDLE file;
	WIN32_FIND_DATA fileData;
	char line[1024];
	wchar_t fn[1000];
	mbstowcs(fn,mystr,999);
	file = FindFirstFile(fn, &fileData);
	FindNextFile(file, &fileData);
	while(FindNextFile(file, &fileData))
	{
		wcstombs(line,(const wchar_t*)fileData.cFileName,259);
		lineStr = line;
		for (int i = 0; i < 5; i ++)       //ÅÅ³ý·ÇÍ¼ÏñÎÄ¼þ
		{
			if (lineStr.find(extendName[i]) < 999)
			{
				lineStr = foldName_copy + "/" + lineStr;
				flist.push_back(lineStr);
				break;
			}
		}	
	}
	return flist;
}


void Utils::findBinaryROIMask(Mat &image, vector<int> &roiIndexs)
{
	int rows = image.rows, cols = image.cols;
//	Mat maski(rows, cols, CV_8UC1, Scalar(0));
	uchar* imgPtr = (uchar*)image.data;
//	uchar* mskPtr = (uchar*)maski.data;
	if (image.channels() == 4)
	{
		for (int i = 0; i < rows; i ++)
		{
			for (int j = 0; j < cols; j ++)
			{
				if (imgPtr[4*(i*cols+j)+3] != 0)  //! alpha channel
				{
//					mskPtr[i*cols+j] = 255;
					roiIndexs.push_back(i*cols+j);
				}
			}
		}
		return;
	}
	for (int i = 0; i < rows; i ++)
	{
		for (int j = 0; j < cols; j ++)
		{
			if (imgPtr[3*(i*cols+j)+0] == 0 && 
				imgPtr[3*(i*cols+j)+1] == 0 && 
				imgPtr[3*(i*cols+j)+2] == 0)    //! r, g, b channel
			{
				continue;
			}
//			mskPtr[i*cols+j] = 255;
			roiIndexs.push_back(i*cols+j);
		}
	}
/*	static int no = 0;
	char name[512];
	sprintf(name, "Mask/ROImask%03d.png", no++);
	string savePath = Utils::baseDir + string(name);
	imwrite(savePath, maski);*/
}


vector<int> Utils::intersectROIs(const vector<int> &roiIndexs1, const vector<int> &roiIndexs2)
{
	multiset<int> iset1(begin(roiIndexs1),end(roiIndexs1));  
	multiset<int> iset2(begin(roiIndexs2),end(roiIndexs2));  
	vector<int> intersection(25000000);
	auto iter = Utils::set_intersection(iset1.begin(),iset1.end(),iset2.begin(),iset2.end(), intersection.begin());
	intersection.resize(iter-intersection.begin());
	//! assumption : the index in this two input vector are assigned in ascending order
	return intersection;
}


vector<int> Utils::intersectROIsPro(const vector<int> &roiIndexs1, const vector<int> &roiIndexs2)
{
	int index0 = min(roiIndexs1[0],roiIndexs2[0]);
	int index1 = max(roiIndexs1[roiIndexs1.size()-1],roiIndexs2[roiIndexs2.size()-1]);
	vector<int> tempVect;
	//! initialization
	for (int i = 0; i <= index1-index0; i ++)
	{
		tempVect.push_back(0);
	}
	//! voting
	for (int i = 0; i < roiIndexs1.size(); i ++)
	{
		tempVect[roiIndexs1[i]-index0] ++;
	}
	for (int i = 0; i < roiIndexs2.size(); i ++)
	{
		tempVect[roiIndexs2[i]-index0] ++;
	}
	//! pick out overlap
	vector<int> intersection;
	for (int i = 0; i < tempVect.size(); i ++)
	{
		if (tempVect[i] == 2)
		{
			intersection.push_back(index0+i);
		}
	}
	return intersection;
}


vector<double>* Utils::extractROIPixels(const Mat &yccImage, const vector<int> &roiIndexs)
{
	int rows = yccImage.rows, cols = yccImage.cols;
	double *imgPtr = (double*)yccImage.data;
	//! store pixel values of each channel in each row
	vector<double>* ROIPixels = new vector<double>[3];
	for (int i = 0; i < roiIndexs.size(); i ++)
	{
		int index = roiIndexs[i];
		ROIPixels[0].push_back(imgPtr[3*(index)+0]);
		ROIPixels[1].push_back(imgPtr[3*(index)+1]);
		ROIPixels[2].push_back(imgPtr[3*(index)+2]);
	}
	return ROIPixels;
}


double Utils::calStructuralDiffs(const Mat &srcImage, const Mat &dstImage, const vector<int> &roiIndexs)
{
	int rows = srcImage.rows, cols = srcImage.cols;
	vector<double> gradOrienVec1, gradOrienVec2;
	getGradientOrientationMaps(srcImage, gradOrienVec1, dstImage, gradOrienVec2, roiIndexs);
	//! structural difference
	int num = 0;
	double sdErr = 0;
	for (int i = 0; i < gradOrienVec1.size(); i ++)
	{
		double orien1 = gradOrienVec1[i];
		double orien2 = gradOrienVec2[i];
		sdErr += fabs(orien1-orien2);
		num ++;
	}
	sdErr = sdErr/num;
	return sdErr;
}


void Utils::getGradientOrientationMaps(Mat grayImage1, vector<double> &gradOrienVec1, Mat grayImage2, vector<double> &gradOrienVec2, const vector<int> &roiIndexs)
{
	int rows = grayImage1.rows, cols = grayImage1.cols;
	uchar *srcPtr = (uchar*)grayImage1.data;
	uchar *dstPtr = (uchar*)grayImage2.data;
	for (int i = 0; i < roiIndexs.size(); i ++)
	{
		int index = roiIndexs[i];
		int val1 = srcPtr[index];
		int val2 = dstPtr[index];
		//! calculate the (r,c) of current roi pixel
		int r = index/cols, c = index%cols;
		if (r == rows-1 || c == cols-1)
		{
			continue;
		}
		int neighx1 = srcPtr[r*cols+c+1], neighy1 = srcPtr[(r+1)*cols+c];
		int neighx2 = dstPtr[r*cols+c+1], neighy2 = dstPtr[(r+1)*cols+c];
		if (neighx1 == 0 || neighy1 == 0)
		{
			continue;
		}
		double gx1 = val1-neighx1, gy1 = val1-neighy1;
		double orien1 = atan2l(gy1, gx1);
		gradOrienVec1.push_back(orien1);
		double gx2 = val2-neighx2, gy2 = val2-neighy2;
		double orien2 = atan2l(gy2, gx2);
		gradOrienVec2.push_back(orien2);
	}
}
