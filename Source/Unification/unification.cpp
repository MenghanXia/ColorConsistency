#include "unification.h"

using namespace std;
using namespace Utils;
using namespace ColorSpace;

#define LAMBDA 3    //! [1.0*M/m] balancing coefficient of quality energy term

void ToneUnifier::unifyMultiTones()
{
	//! load adjacent relations and detect mask
	initialization();
	_stime = clock();
	//! color matching between images in the overlap (set control points)
	buildCorrespondings();
	//! figure interpolating coefficient for each key points
	figureInterpolation();

	vector<int> fixedImgNos;   //! reference images, keep fixed in optimization
//	fixedImgNos.push_back(5); //fixedImgNos.push_back(4);
	//! resort image information list
	setFixedImages(fixedImgNos);

	Vec2d curvatures[3];      //! domain of curve slop setting in each channel
	for (int t = 0; t < 3; t ++)
	{
		curvatures[t][0] = 0.3;
		curvatures[t][1] = 5;
	}
	curvatures[0][0] = 0.3;
	for (int t = 0; t < 3; t ++)
	{
		cout<<"-Quadratic Programming for channel "<<t<<endl;
		quadraticProgramming(t, curvatures[t]);
		cout<<"-Done!"<<endl;
	}
	_etime = clock();

	//! color correction (needEachWarp should be true to facilitate the following evaluation)
	bool needEachWarp = true, applyRemapping = true;
	applyColorRemappingforImages(needEachWarp, applyRemapping);
	//! output evaluation report
	outputEvaluationReport();
}


void ToneUnifier::initialization()
{
	cout<<"-Initializing ..."<<endl;
	string paramPath = Utils::baseDir + "Cache/ParamsPro.txt";
	ifstream fin;
	fin.open(paramPath.c_str(), ios::in);
	if (!fin.is_open())
	{
		cout<<"File not found!\n";
		exit(0);
	}
	for (int i = 0; i < _imgNum; i ++)
	{
		Utils::ImageParam bar;
		fin>>bar.imgNo;
		int neigbNum = 0;
		fin>>neigbNum;
		for (int j = 0; j < neigbNum; j ++)
		{
			Utils::Adjacency neigbBar;
			fin>>neigbBar.neigbNo;

			bar.neigbList.push_back(neigbBar);
		}
		_imageInforList.push_back(bar);
	}
	fin.close();
	//! extract roi mask
	cout<<"-find ROI mask ..."<<endl;
	for (int i = 0; i < _imgNum; i ++)
	{
		Mat image = imread(_filePathList[i], -1);
		if (image.channels() == 1)
		{
			cout<<"Your input are gray images!"<<endl;
			exit(1);
		}
		if (i == 0)
		{
			_imgSize.width = image.cols;
			_imgSize.height = image.rows;
		}
		Utils::findBinaryROIMask(image, _imageInforList[i].ROIIndexList);
		cout<<i<<"  ";
		if ((i+1)%10 == 0)
		{
			cout<<endl;
		}
	}
	cout<<"-Done!"<<endl;
}


void ToneUnifier::buildCorrespondings()
{
	cout<<"-Building correspondences between images ..."<<endl;
	//! in the order of imgIndex, currently imgIndex = imgNo.
	for (int i = 0; i < _imgNum; i ++)
	{
		int curNo = _imageInforList[i].imgNo;
		cout<<"-KeyGV Registration of image:"<<curNo<<"  >>  ";
		Mat curImage = imread(_filePathList[curNo]);
		//! covert to Lab color space
		Mat curImagef = RGB2YCbCr(curImage, _imageInforList[i].ROIIndexList);
		//! set control-points in every channel
		setCtrlPoints(curImagef, i);
		if (i == _imgNum-1)  //! just to make "setCtrlPoints()" available to the last image
		{
			break;
		}
		vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
		//! finding valid neighbor images
		for (int j = 0; j < curNeigbList.size(); j ++)
		{
			int adjNo = curNeigbList[j].neigbNo;
			if (adjNo < curNo)
			{
				continue;
			}
			Mat adjImage = imread(_filePathList[adjNo]);
			//! covert to Lab color space
			Mat adjImagef = RGB2YCbCr(adjImage, _imageInforList[adjNo].ROIIndexList);
			vector<int> overlapIndexs = Utils::intersectROIsPro(_imageInforList[i].ROIIndexList, _imageInforList[adjNo].ROIIndexList);
			if (0)
			{
				int cols = curImage.cols;
				for (int p = 0; p < overlapIndexs.size(); p ++)
				{
					int r = overlapIndexs[p]/cols, c = overlapIndexs[p]%cols;
					circle(curImage, Point2i(c,r), 1, Scalar(255,0,0), -1);
				}
				string path = Utils::baseDir + "overlap.png";
				imwrite(path, curImage);
			}
			vector<double> *ROIPixels1 = Utils::extractROIPixels(curImagef, overlapIndexs);
			vector<double> *ROIPixels2 = Utils::extractROIPixels(adjImagef, overlapIndexs);
			vector<Adjacency> neigNeigbList = _imageInforList[adjNo].neigbList;
			//! locating the index of current image in the neighbor list
			int k = 0; //! element index
			for (k = 0; k < neigNeigbList.size(); k ++)
			{
				if (neigNeigbList[k].neigbNo == curNo)
				{
					break;
				}
			}
			for (int t = 0; t < 3; t ++)
			{
				vector<double> keyGVs1, keyGVs2;
				EntryMatch::grabHistkeyPtPairsbyProbi(ROIPixels1[t], ROIPixels2[t], keyGVs1, keyGVs2);
				_imageInforList[curNo].neigbList[j].keyGVLists[t] = keyGVs1;
				_imageInforList[adjNo].neigbList[k].keyGVLists[t] = keyGVs2;
			}
			delete []ROIPixels1;
			delete []ROIPixels2;
		}
		cout<<"-Done!"<<endl;
	}
	cout<<"-Completed!"<<endl;
}


void ToneUnifier::setCtrlPoints(const Mat &image, int imgIndex)
{
	vector<int> roiIndexs = _imageInforList[imgIndex].ROIIndexList;
	double *imgData = (double*)image.data;
	double minV[3] = {999,999,999}, maxV[3] = {-999,-999,-999};
	//! find pixel value range
	for (int t  = 0; t < 3; t ++)
	{
		for (int i = 0; i < roiIndexs.size(); i ++)
		{
			double val = imgData[3*roiIndexs[i]+t];
			if (minV[t] > val)
			{
				minV[t] = val;
			}
			if (maxV[t] < val)
			{
				maxV[t] = val;
			}
		}
	}
	//! define control points
	for (int t = 0; t < 3; t ++)
	{
		vector<double> ctrlVals;
		double step = (maxV[t]-minV[t])/(PARAM_NUM-2);
		ctrlVals.push_back(minV[t]-step/2);
		for (int i = 1; i < PARAM_NUM-1; i ++)
		{
			ctrlVals.push_back(ctrlVals[0]+step*i);
		}
		ctrlVals.push_back(maxV[t]+step/2);
		//! for float precision
		ctrlVals[0] -= 1e-10;
		ctrlVals[PARAM_NUM-1] += 1e-10;

		for (int i = 0; i < PARAM_NUM; i ++)
		{
			double xi = ctrlVals[i];
			Point2d point(xi,xi);
			_imageInforList[imgIndex].ctrlPoints[t].push_back(point);
		}
	}
	//! extract intensity range key values in luminance channel
	getHistogramPercentileValues(image, imgIndex);
	//! extract major gradient components in luminance channel
	getMajorGradientComponents(image, imgIndex);
}


void ToneUnifier::figureInterpolation()
{
	cout<<"-figuring interpolation (based on ctrl-points) ..."<<endl;
	//! figure interpolating coefficient
	cout<<" for: correspondence values ..."<<endl;
	for (int i = 0; i < _imgNum; i ++)
	{
		for (int t = 0; t < 3; t ++)
		{
			vector<int> label;
			for (int k = 0; k < PARAM_NUM; k ++) {label.push_back(0);}
			vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
			vector<Point2d> ctrlPts = _imageInforList[i].ctrlPoints[t];
			for (int j = 0; j < curNeigbList.size(); j ++)
			{
				vector<double> keyGVs = curNeigbList[j].keyGVLists[t];
				vector<InterpInfo> interpList;
				for (int p = 0; p < keyGVs.size(); p ++)
				{
					//! locating the position of this key value
					InterpInfo interpV;
					bool isGot = false;
					for (int q = 1; q < PARAM_NUM-1; q ++)
					{
						double t0 = (ctrlPts[q-1].x+ctrlPts[q].x)/2.0;
						double t1 = (ctrlPts[q].x+ctrlPts[q+1].x)/2.0;
						if (t0 <= keyGVs[p] && keyGVs[p] <= t1)
						{
							double ti = calInterpCoeff(keyGVs[p], ctrlPts[q-1].x, ctrlPts[q].x, ctrlPts[q+1].x);
							interpV.ctrlNos[0] = q-1;
							interpV.ctrlNos[1] = q;
							interpV.ctrlNos[2] = q+1;
							interpV.coeffs[0] = (1-2*ti+ti*ti)/2;
							interpV.coeffs[1] = (1+2*ti-2*ti*ti)/2;
							interpV.coeffs[2] = ti*ti/2;

							label[q-1] ++;
							label[q] ++;
							label[q+1] ++;
							isGot = true;
							break;
						}
					}
					if (!isGot)
					{
						cout<<"Impossible: this keyGV is out of interpolating range! imgNo:"<<i<<" channel:"<<t<<" "<<j<<" "<<p<<endl;
						exit(1);
					}
					interpList.push_back(interpV);
				}
				_imageInforList[i].neigbList[j].interpCoeffs[t] = interpList;
			}
/*			cout<<"image "<<i<<" chino "<<t<<": ";
			for (int k = 0; k < PARAM_NUM; k ++)
			{
				if (label[k] == 0)
				{
					cout<<k<<" ";
				}
			}
			cout<<endl;*/
		}
	}
	//! get interpolation for major range nodes [luminance channel]
	cout<<" for: range key values ..."<<endl;
	for (int i = 0; i < _imgNum; i ++)
	{
		vector<Point2d> ctrlPts = _imageInforList[i].ctrlPoints[0];
		double keyVals[2] = {_imageInforList[i].valuRange.val0, _imageInforList[i].valuRange.val1};
		InterpInfo interpVs[2];
		bool isGot = false;
		for (int p = 0; p < 2; p ++)
		{
			for (int q = 1; q < PARAM_NUM-1; q ++)
			{
				double t0 = (ctrlPts[q-1].x+ctrlPts[q].x)/2.0;
				double t1 = (ctrlPts[q].x+ctrlPts[q+1].x)/2.0;
				if (t0 <= keyVals[p] && keyVals[p] <= t1)
				{
					double ti = calInterpCoeff(keyVals[p], ctrlPts[q-1].x, ctrlPts[q].x, ctrlPts[q+1].x);
					interpVs[p].ctrlNos[0] = q-1;
					interpVs[p].ctrlNos[1] = q;
					interpVs[p].ctrlNos[2] = q+1;
					interpVs[p].coeffs[0] = (1-2*ti+ti*ti)/2;
					interpVs[p].coeffs[1] = (1+2*ti-2*ti*ti)/2;
					interpVs[p].coeffs[2] = ti*ti/2;

					isGot = true;
					break;
				}
			}
			if (!isGot)
			{
				cout<<"Impossible: this keyGV is out of interpolating range!"<<endl;
				exit(1);
			}
		}
		_imageInforList[i].valuRange.interp0 = interpVs[0];
		_imageInforList[i].valuRange.interp1 = interpVs[1];
	}
	//! get interpolation for major range nodes [luminance channel]
	cout<<" for: major gradient pairs ..."<<endl;
	for (int i = 0; i < _imgNum; i ++)
	{
		vector<Point2d> ctrlPts = _imageInforList[i].ctrlPoints[0];
		int minVal = _imageInforList[i].gradminmaxVals[0];
		int maxVal = _imageInforList[i].gradminmaxVals[1];
		vector<InterpInfo> curLooktable;
		//! find interpolations for look-table values
		for (int v = minVal; v <= maxVal; v ++)
		{
			InterpInfo bat;
			bool isGot = false;
			for (int q = 1; q < PARAM_NUM-1; q ++)
			{
				double t0 = (ctrlPts[q-1].x+ctrlPts[q].x)/2.0;
				double t1 = (ctrlPts[q].x+ctrlPts[q+1].x)/2.0;
				if (t0 <= v && v <= t1)
				{
					double ti = calInterpCoeff(v, ctrlPts[q-1].x, ctrlPts[q].x, ctrlPts[q+1].x);
					bat.ctrlNos[0] = q-1;
					bat.ctrlNos[1] = q;
					bat.ctrlNos[2] = q+1;
					bat.coeffs[0] = (1-2*ti+ti*ti)/2;
					bat.coeffs[1] = (1+2*ti-2*ti*ti)/2;
					bat.coeffs[2] = ti*ti/2;

					isGot = true;
					break;
				}
			}
			if (!isGot)
			{
				cout<<"Impossible: this keyGV is out of interpolating range! "<<i<<endl;
				cout<<minVal<<" - "<<maxVal<<endl;
				double minV = (ctrlPts[0].x+ctrlPts[1].x)/2;
				double maxV = (ctrlPts[4].x+ctrlPts[5].x)/2;
				cout<<minV<<" - "<<maxV<<endl;
				exit(1);
			}
			curLooktable.push_back(bat);
		}
		//! fill interpolations for each gradient-pair
		vector<IntensityPair> &gradMajors = _imageInforList[i].gradMajors;
		//cout<<"num "<<gradMajors.size()<<endl;
		for (int j = 0; j < gradMajors.size(); j ++)  //! TBD : include repetitive computation
		{
			//! find interpolation by indexing position in the look-table
			gradMajors[j].interp0 = curLooktable[gradMajors[j].val0-minVal];
			gradMajors[j].interp1 = curLooktable[gradMajors[j].val1-minVal];
		}
	}
	cout<<"-Done!"<<endl;
}


void ToneUnifier::setFixedImages(vector<int> fixedImgNos)
{
	if (fixedImgNos.size() == 0)
	{
		cout<<"@ No reference image set!"<<endl;
		return;
	}
	_fixedImgList = fixedImgNos;    //! save to global variable
	vector<Utils::ImageParam> tempInforList;
	vector<string> tempFileList;
	cout<<"@ Reference image(s): ";
	//! resort the vector of image information list : input reference images first
	for (int i = 0; i < _fixedImgList.size(); i ++)
	{
		int no = fixedImgNos[i];
		cout<<no<<", ";
		tempInforList.push_back(_imageInforList[no]);
		tempFileList.push_back(_filePathList[no]);
	}
	cout<<endl;
	//! resort the vector of image information list : input the rest images then
	for (int i = 0; i < _imageInforList.size(); i ++)
	{
		bool isReference = false;
		for (int j = 0; j < _fixedImgList.size(); j ++)
		{
			if (i == _fixedImgList[j])
			{
				isReference = true;
				break;
			}
		}
		if (!isReference)
		{
			tempInforList.push_back(_imageInforList[i]);
			tempFileList.push_back(_filePathList[i]);
		}
	}
	//! update order
	_imageInforList = tempInforList;
	_filePathList = tempFileList;
}


void ToneUnifier::quadraticProgramming(int channelNo, Vec2d curvatureRange)
{
	int elemNum = (_imgNum-_fixedImgList.size())*PARAM_NUM;
	Mat_<double> H = Mat(elemNum, elemNum, CV_64FC1, Scalar(0));
	Mat_<double> c = Mat(elemNum, 1, CV_64FC1, Scalar(0));
	double *Hptr = (double*)H.data;
	//! balancing coefficient for gradient preservation and contrast respectively
	double alpha = 20*PARAM_NUM*LAMBDA;
	double beta = 2*PARAM_NUM*LAMBDA;
	//! fill elements for matrix of quadratic programming : f = 0.5x'Hx + c'x   s.t. Ax >= b
	//! H : from affinity term [minimize value difference between non-reference image pairs]
	int initIndex = _fixedImgList.size();
	for (int i = initIndex; i < _imgNum-1; i ++)
	{
		int curNo = _imageInforList[i].imgNo;
		vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
		//! finding valid neighbor images
		for (int j = 0; j < curNeigbList.size(); j ++)
		{
			int neigbIndex = findImageIndex(curNeigbList[j].neigbNo);
			//! to avoid dual consideration
			if (neigbIndex < i)
			{
				continue;
			}
			vector<InterpInfo> curInterps = curNeigbList[j].interpCoeffs[channelNo];
			vector<InterpInfo> neigbInterps;
			vector<Adjacency> neigbNeigbList = _imageInforList[neigbIndex].neigbList;
			//! locating the index of current image in the neighbor list
			for (int k = 0; k < neigbNeigbList.size(); k ++)
			{
				if (neigbNeigbList[k].neigbNo == curNo)
				{
					neigbInterps = neigbNeigbList[k].interpCoeffs[channelNo];
					break;
				}
			}
			int offset1 = (i-initIndex)*PARAM_NUM, offset2 = (neigbIndex-initIndex)*PARAM_NUM;
			for (int k = 0; k < curInterps.size(); k ++)
			{
				//! cur-image
				double A1 = curInterps[k].coeffs[0];
				double B1 = curInterps[k].coeffs[1];
				double C1 = curInterps[k].coeffs[2];
				int no1 = curInterps[k].ctrlNos[0] + offset1;

				//! neighbor-image
				double A2 = neigbInterps[k].coeffs[0];
				double B2 = neigbInterps[k].coeffs[1];
				double C2 = neigbInterps[k].coeffs[2];
				int no2 = neigbInterps[k].ctrlNos[0] + offset2;

				int index[6] = {no1,no1+1,no1+2, no2,no2+1,no2+2};
				double coeff[6] = {A1,B1,C1, -A2,-B2,-C2};
				for (int p = 0; p < 6; p ++)
				{
					for (int q = 0; q < 6; q ++)
					{
						Hptr[index[p]*elemNum+index[q]] += coeff[p]*coeff[q];
					}
				}
			}
		}
	}
	//! H, c : from affinity term [minimize value difference between reference and non-reference image pairs] 
	for (int i = 0; i < _fixedImgList.size(); i ++)
	{
		int curNo = _imageInforList[i].imgNo;
		vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
		//! finding valid neighbor images
		for (int j = 0; j < curNeigbList.size(); j ++)
		{
			int neigbIndex = findImageIndex(curNeigbList[j].neigbNo);
			//! to avoid consider reference images
			if (neigbIndex < initIndex)
			{
				continue;
			}
			//vector<InterpInfo> curInterps = curNeigbList[j].interpCoeffs[channelNo];
			vector<double> curKeyValues = curNeigbList[j].keyGVLists[channelNo];
			vector<InterpInfo> neigbInterps;
			vector<Adjacency> neigbNeigbList = _imageInforList[neigbIndex].neigbList;
			//! locating the index of current image in the neighbor list
			for (int k = 0; k < neigbNeigbList.size(); k ++)
			{
				if (neigbNeigbList[k].neigbNo == curNo)
				{
					neigbInterps = neigbNeigbList[k].interpCoeffs[channelNo];
					break;
				}
			}
			int offset2 = (neigbIndex-initIndex)*PARAM_NUM;
			for (int k = 0; k < curKeyValues.size(); k ++)
			{
				//! cur-image
				double refVal = curKeyValues[k];

				//! neighbor-image
				double A2 = neigbInterps[k].coeffs[0];
				double B2 = neigbInterps[k].coeffs[1];
				double C2 = neigbInterps[k].coeffs[2];
				int no2 = neigbInterps[k].ctrlNos[0] + offset2;

				int index[3] = {no2,no2+1,no2+2};
				double coeff[3] = {-A2,-B2,-C2};
				for (int p = 0; p < 3; p ++)
				{
					//! H part
					for (int q = 0; q < 3; q ++)
					{
						Hptr[index[p]*elemNum+index[q]] += coeff[p]*coeff[q];
					}
					//! c part (here the index has add offset already before)
					c(index[p]) += 2*coeff[p]*refVal;
				}
			}
		}
	}
	//! H, c : from soft-constraint term [regularization: not deviating from original status]
	for (int i = initIndex; i < _imgNum; i ++)
	{
		double lambda1 = 1*LAMBDA, lambda2 = 0.5*LAMBDA;
		vector<Point2d> curCtrls = _imageInforList[i].ctrlPoints[channelNo];
		int offset = (i-initIndex)*PARAM_NUM;
		int nos[2] = {0, PARAM_NUM-2};
		//! start and end points
		for (int k = 0; k < 2; k ++)
		{
			int j = nos[k];
			//! to H
			Hptr[(offset+j+0)*elemNum+(offset+j+0)] += lambda1/4;
			Hptr[(offset+j+1)*elemNum+(offset+j+1)] += lambda1/4;
			Hptr[(offset+j+0)*elemNum+(offset+j+1)] += lambda1/4;
			Hptr[(offset+j+1)*elemNum+(offset+j+0)] += lambda1/4;
			//! to c
			double val0 = (curCtrls[j].x+curCtrls[j+1].x)/2;
			c(offset+j+0) += -val0*lambda1;
			c(offset+j+1) += -val0*lambda1;
		}
		//! main part points
		for (int j = 1; j < PARAM_NUM-2; j ++)
		{
			//! to H
			Hptr[(offset+j+0)*elemNum+(offset+j+0)] += lambda2/4;
			Hptr[(offset+j+1)*elemNum+(offset+j+1)] += lambda2/4;
			Hptr[(offset+j+0)*elemNum+(offset+j+1)] += lambda2/4;
			Hptr[(offset+j+1)*elemNum+(offset+j+0)] += lambda2/4;
			//! to c
			double val0 = (curCtrls[j].x+curCtrls[j+1].x)/2;
			c(offset+j+0) += -val0*lambda2;
			c(offset+j+1) += -val0*lambda2;
		}
	}
	
	//! only feasible to luminance channel
	if (channelNo == 0)
	{
		//! H, c : from soft-constraint term [gradient preservation]
		for (int i = initIndex; i < _imgNum; i ++)
		{
			vector<IntensityPair> &gradMajors = _imageInforList[i].gradMajors;
			double normWeight = alpha/gradMajors.size();
			int offset = (i-initIndex)*PARAM_NUM;
			for (int j = 0; j < gradMajors.size(); j ++)
			{
				int refGrad = -(gradMajors[j].val0 - gradMajors[j].val1);
				//! gradient-intensity 1
				double A1 = gradMajors[j].interp0.coeffs[0];
				double B1 = gradMajors[j].interp0.coeffs[1];
				double C1 = gradMajors[j].interp0.coeffs[2];
				int no1 = gradMajors[j].interp0.ctrlNos[0] + offset;

				//! gradient-intensity 2
				double A2 = gradMajors[j].interp1.coeffs[0];
				double B2 = gradMajors[j].interp1.coeffs[1];
				double C2 = gradMajors[j].interp1.coeffs[2];
				int no2 = gradMajors[j].interp1.ctrlNos[0] + offset;

				int index[6] = {no1,no1+1,no1+2,no2,no2+1,no2+2};
				double coeff[6] = {A1,B1,C1,-A2,-B2,-C2};
				for (int p = 0; p < 6; p ++)
				{
					//! H part
					for (int q = 0; q < 6; q ++)
					{
						Hptr[index[p]*elemNum+index[q]] += normWeight*coeff[p]*coeff[q];
					}
					//! c part (here the index has add offset already before)
					c(index[p]) += normWeight*2*coeff[p]*refGrad;
				}
			}		
		}
		
		//! c : from soft-constraint term [extend dynamic range by add term "-(up - low)"]
		for (int i = initIndex; i < _imgNum; i ++)
		{
			vector<Point2d> curCtrls = _imageInforList[i].ctrlPoints[channelNo];
			int offset = (i-initIndex)*PARAM_NUM;
			InterpInfo lowParam =_imageInforList[i].valuRange.interp0;
			InterpInfo upParam = _imageInforList[i].valuRange.interp1;
			//! low value
			double A1 = lowParam.coeffs[0];
			double B1 = lowParam.coeffs[1];
			double C1 = lowParam.coeffs[2];
			int no1 = lowParam.ctrlNos[0];

			//! up value
			double A2 = upParam.coeffs[0];
			double B2 = upParam.coeffs[1];
			double C2 = upParam.coeffs[2];
			int no2 = upParam.ctrlNos[0];

			int index[6] = {no1,no1+1,no1+2, no2,no2+1,no2+2};
			double coeff[6] = {A1,B1,C1, -A2,-B2,-C2};
			for (int p = 0; p < 6; p ++)
			{
				c(offset+index[p]) += beta*coeff[p];
			}
		} 
	}
	H = 2*H;     //! just to fit the standard quadratic programming equation
	
	int condNum = (_imgNum-_fixedImgList.size())*2*(PARAM_NUM-1);
	Mat_<double> A = Mat(condNum, elemNum, CV_64FC1, Scalar(0));
	Mat_<double> b = Mat(condNum, 1, CV_64FC1, Scalar(0));
	double *Aptr = (double*)A.data; 
	//! A, b : from hard-constraint [domain settings of curve slop]
	double curvature0 = curvatureRange[0], curvature1 = curvatureRange[1];
	int n = 0;
	for (int i = initIndex; i < _imgNum; i ++)
	{
		vector<Point2d> curCtrls = _imageInforList[i].ctrlPoints[channelNo];
		double step = curCtrls[1].x-curCtrls[0].x;
		int offset = (i-initIndex)*PARAM_NUM;
		for (int j = 0; j < PARAM_NUM-1; j ++)
		{
			Aptr[n*elemNum+(offset+j+0)] = -1;
			Aptr[n*elemNum+(offset+j+1)] = 1;
			b(n) = curvature0*step;

			Aptr[(n+1)*elemNum+(offset+j+0)] = 1;
			Aptr[(n+1)*elemNum+(offset+j+1)] = -1;
			b(n+1) = -curvature1*step;

			n += 2;
		}
	}
	//! A, b : from hard-constraint [domain settings of curve value]
	// TBD

	//! solve quadratic programming equation
	Mat_<double> X = Mat(elemNum, 1, CV_64FC1);
	QP::solveQuadraProgram(H, c, A.t(), -b, X);
	
	//! save optimized results
	for (int i = initIndex; i < _imgNum; i ++)
	{
		vector<Point2d> &ctrlPts = _imageInforList[i].ctrlPoints[channelNo];
		for (int j = 0; j < ctrlPts.size(); j ++)
		{
			ctrlPts[j].y = X((i-initIndex)*PARAM_NUM+j);
		}
	}
}


void ToneUnifier::updateImagebyRemapping(Mat &YccImage, int imgIndex)
{
	vector<Point2d> sampleLists[3];
	for (int t = 0; t < 3; t ++)
	{
		sampleLists[t] = interpSamplesbyBSpline(_imageInforList[imgIndex].ctrlPoints[t], SAMPLE_FREQ_NUM, t);
	}

	int rows = YccImage.rows, cols = YccImage.cols;
	double* dataPtr = (double*)YccImage.data;
	vector<int> roiIndexs = _imageInforList[imgIndex].ROIIndexList;
	for (int i = 0; i < roiIndexs.size(); i ++)            
	{
		int index = roiIndexs[i];
		//! mapping for 3 channels
		for (int t = 0; t < 3; t ++)
		{
			double pixV = dataPtr[3*index+t];
			dataPtr[3*index+t] = interpValuebyLinear(pixV, sampleLists[t]);
		}
	} 
}


vector<Point2d> ToneUnifier::interpSamplesbyBSpline(vector<Point2d> ctrlPoints, int freqNum, int channelNo)
{
	vector<Point2d> samples;
	int num = ctrlPoints.size();
	for (int i = 1; i < num-1; i ++)
	{
		double x1 = ctrlPoints[i-1].x, y1 = ctrlPoints[i-1].y;
		double x2 = ctrlPoints[i].x,   y2 = ctrlPoints[i].y;
		double x3 = ctrlPoints[i+1].x, y3 = ctrlPoints[i+1].y;
		for (int j = 0; j < freqNum; j ++)
		{
			double t  = 1.0*j/freqNum;
			double t1 = 1-2*t+t*t;
			double t2 = 1+2*t-2*t*t;
			double t3 = t*t;
			double xPt = (t1*x1 + t2*x2 + t3*x3)/2.0;
			double yPt = (t1*y1 + t2*y2 + t3*y3)/2.0;
			//if (xPt < ctrlPoints[1].x || xPt > ctrlPoints[num-2].x)
			//{
			//	continue;
			//}
			samples.push_back(Point2d(xPt,yPt));
		}
	}
	return samples;
}


inline double ToneUnifier::interpValuebyLinear(double xi, const vector<Point2d> &samples)
{
	int num = samples.size();
	double yi = 0;
	if (xi <= samples[0].x)
	{
		yi = samples[0].y - (samples[0].x-xi)*(samples[1].y-samples[0].y)/(samples[1].x-samples[0].x);
		//yi = xi;
		//yi = exterpValuebyGamma(xi, samples);
	}
	else if (xi >= samples[num-1].x)
	{
		yi = samples[num-1].y + (xi-samples[num-1].x)*(samples[num-1].y-samples[num-2].y)/(samples[num-1].x-samples[num-2].x);
		//yi = xi;
		//yi = exterpValuebyGamma(xi, samples);
	}
	else
	{
		//! step 1 : location with bigger gap
		for (int i = 0; i < num; i += SAMPLE_FREQ_NUM)
		{
			int no = min(num-1, i+SAMPLE_FREQ_NUM);
			if (xi >= samples[i].x && xi < samples[no].x)
			{
				//! step 2 : location with smaller gap
				for (int j = i; j < no; j ++)
				{
					if (xi >= samples[j].x && xi < samples[j+1].x)
					{
						yi = ((xi-samples[j].x)*samples[j+1].y + (samples[j+1].x-xi)*samples[j].y)/(samples[j+1].x-samples[j].x);
						return yi;
					}
				}
			}
		}
	}
	return yi;
}


void ToneUnifier::applyColorRemappingforImages(bool needIndividuals, bool applyRemapping)
{
	int rows = _imgSize.height, cols = _imgSize.width;
	Mat baseImage(rows, cols, CV_8UC3, Scalar(BKGRNDPIX, BKGRNDPIX, BKGRNDPIX));
	uchar *basePtr = (uchar*)baseImage.data;
	//! warp images in the order of their ID, namely imgNo
	for (int i = 0; i < _imgNum; i ++)
	{
		cout<<"-Compositing image "<<i<<" ... "<<endl;
		int curIndex = findImageIndex(i);
		vector<int> roiIndexList = _imageInforList[curIndex].ROIIndexList;
		Mat curImage = imread(_filePathList[curIndex]);
		//! covert to YcrCb color space
		Mat curImagef = ColorSpace::RGB2YCbCr(curImage, roiIndexList);
		if (applyRemapping)
		{
			updateImagebyRemapping(curImagef, curIndex);
		}
		double* dataPtr = (double*)curImagef.data;
		Mat warpMap;
		if (needIndividuals)
		{
			warpMap = Mat(rows, cols, CV_8UC4, Scalar(BKGRNDPIX,BKGRNDPIX,BKGRNDPIX,0));
		}
		uchar *warpPtr = (uchar*)warpMap.data;
		for (int j = 0; j < roiIndexList.size(); j ++)
		{
			int index = roiIndexList[j];
			//! from YCbCr to RGB
			double Y = dataPtr[3*index+0];        //! Y
			double Cb = dataPtr[3*index+1]-127.5;   //! Cb-128
			double Cr = dataPtr[3*index+2]-127.5;   //! Cr-128
			int R = int(Y + 1.402*Cr);
			int G = int(Y - 0.3441*Cb - 0.7141*Cr);
			int B = int(Y + 1.7720*Cb);
			int Bi = min(255, max(0,B));
			int Gi = min(255, max(0,G));
			int Ri = min(255, max(0,R));

			basePtr[3*index+0] = Bi;
			basePtr[3*index+1] = Gi;
			basePtr[3*index+2] = Ri;

			if (needIndividuals)
			{
				//warpMap.at<Vec4b>(r,c)[3] = 255;      //! alpha channel
				//warpMap.at<Vec4b>(r,c)[0] = bgr[0];  //! B
				//warpMap.at<Vec4b>(r,c)[1] = bgr[1];  //! G
				//warpMap.at<Vec4b>(r,c)[2] = bgr[2];  //! R
				warpPtr[4*index+3] = 255;
				warpPtr[4*index+0] = Bi;
				warpPtr[4*index+1] = Gi;
				warpPtr[4*index+2] = Ri;
			}
		}
		if (needIndividuals)
		{
			char name[512];
			sprintf(name, "Results/R_IMG%03d.png", i);
			string savePath = Utils::baseDir + string(name);
			imwrite(savePath, warpMap);
		}
	}
	string path = Utils::baseDir + "mosaic.png";
	imwrite(path, baseImage);
	//! save in the order by imgIndex, the same as in '_filePathList'
	for (int i = 0; i < _imgNum; i ++)
	{
		int imgNo = _imageInforList[i].imgNo;
		char name[512];
		sprintf(name, "Results/R_IMG%03d.png", imgNo);
		string savePath = Utils::baseDir + string(name);
		_savePathList.push_back(savePath);
	}
	cout<<"-Done!"<<endl;
}


// ========================== Tool Functions

void ToneUnifier::getHistogramPercentileValues(const Mat &YCbCrImage, int imgIndex)
{
	vector<double> *ROIPixels = Utils::extractROIPixels(YCbCrImage, _imageInforList[imgIndex].ROIIndexList);
	double quantile = 0.1;
	Vec2d RangVs;
	EntryMatch::analyzeCDFQuatileValues(ROIPixels[0], quantile, RangVs);
	_imageInforList[imgIndex].valuRange.val0 = RangVs[0];
	_imageInforList[imgIndex].valuRange.val1 = RangVs[1];
}


void ToneUnifier::getMajorGradientComponents(const Mat &YCbCrImage, int imgIndex)
{
	int rows = YCbCrImage.rows, cols = YCbCrImage.cols;
	vector<int> roiIndexs = _imageInforList[imgIndex].ROIIndexList;
	double *dataPtr = (double*)YCbCrImage.data;
	Mat_<int> countMap = Mat(256, 256, CV_16UC1, Scalar(0));
	vector<Point2d> ctrlPoints = _imageInforList[imgIndex].ctrlPoints[0];    //! luminance channel
	int m = ctrlPoints.size();
	double minV = (ctrlPoints[0].x+ctrlPoints[1].x)/2;
	double maxV = (ctrlPoints[m-2].x+ctrlPoints[m-1].x)/2;
	for (int i = 0; i < roiIndexs.size(); i ++)
	{
		int index = roiIndexs[i];
		int val = dataPtr[3*index+0];     //! integrate to be a index
		//! calculate the (r,c) of current roi pixel
		int r = index/cols, c = index%cols;
		if (r == rows-1 || c == cols-1)
		{
			continue;
		}
		int neighx = dataPtr[3*(r*cols+c+1)+0];
		int neighy = dataPtr[3*((r+1)*cols+c)+0];
		//! avoid interpolation overflow
		if (neighx < minV || neighx > maxV || neighy < minV || neighy > maxV)
		{
			continue;
		}
		if (neighx == 0 || neighy == 0)     //! roi bounding pixels
		{
			continue;
		}
		countMap(val,neighx) ++;
		countMap(val,neighy) ++;
	}
	vector<IntensityPair> &gradMajors = _imageInforList[imgIndex].gradMajors;
	int minVal = 256, maxVal = 0;
	for (int i = 0; i < 255; i ++)
	{
		for (int j = i+1; j < 256; j ++)
		{
			if (i < minV || j > maxV)
			{
				continue;
			}
			if (countMap(i,j) > 200)       //! threshold: entry value
			{
				IntensityPair bar;
				bar.val0 = i;
				bar.val1 = j;
				gradMajors.push_back(bar);
				minVal = min(i,minVal);
				maxVal = max(j,maxVal);
			}
		}
	}
	_imageInforList[imgIndex].gradminmaxVals = Vec2b(minVal, maxVal);
}


double ToneUnifier::calInterpCoeff(double xi, double x1, double x2, double x3)
{
	//! equation : 2*xi = (1-2t+t^2)*x1 + (1+2t-t^2)*x2 + t^2*x3
	double a = x1-2*x2+x3;
	double b = -2*(x1-x2);
	double c = x1 + x2 - 2*xi;
	double tm = sqrt(b*b-4*a*c);
	double t1, t2;
	if (fabs(a) < 1e-10)
	{
		t1 = t2 = -c/b;
	}
	else
	{
		t1 = (tm-b)/(2*a);
		t2 = (-tm-b)/(2*a);
	}
	//! check required conditions
	if (t1 >= -1e-7 && t1 <= 1+1e-7)
	{
		return min(1, max(t1,0));
	}
	else if (t2 >= -1e-7 && t2 <= 1+1e-7)
	{
		return min(1, max(t2,0));
	}
	else
	{
		cout<<" Waring: no solution for this quadratic equation!"<<endl;
		return -1;
	}
}


int ToneUnifier::findImageIndex(int imgNo)
{
	for (int i = 0; i < _imgNum; i ++)
	{
		if (_imageInforList[i].imgNo == imgNo)
		{
			return i;
		}
	}
}


double ToneUnifier::measureError(int channelNo)
{
	int no = 0;
	double rme = 0;
	for (int i = 0; i < _imgNum-1; i ++)
	{
		int curNo = _imageInforList[i].imgNo;
		vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
		vector<Point2d> curCtrls = _imageInforList[curNo].ctrlPoints[channelNo];
		//! finding valid neighbor images
		for (int j = 0; j < curNeigbList.size(); j ++)
		{
			int neigNo = curNeigbList[j].neigbNo;
			//! curNo should be smaller than neigNo (matters for following equation)
			if (neigNo < curNo)
			{
				continue;
			}
			vector<InterpInfo> curInterps = curNeigbList[j].interpCoeffs[channelNo];
			vector<InterpInfo> neiInterps;
			vector<Adjacency> testNeigbList = _imageInforList[neigNo].neigbList;
			vector<Point2d> neiCtrls = _imageInforList[neigNo].ctrlPoints[channelNo];
			//! locating the index of current image in the neighbor list
			for (int k = 0; k < testNeigbList.size(); k ++)
			{
				if (testNeigbList[k].neigbNo == curNo)
				{
					neiInterps = testNeigbList[k].interpCoeffs[channelNo];
					break;
				}
			}
			for (int k = 0; k < curInterps.size(); k ++)
			{
				//! cur-image
				double A1 = curInterps[k].coeffs[0];
				double B1 = curInterps[k].coeffs[1];
				double C1 = curInterps[k].coeffs[2];
				int no1 = curInterps[k].ctrlNos[0];

				//! neig-image
				double A2 = neiInterps[k].coeffs[0];
				double B2 = neiInterps[k].coeffs[1];
				double C2 = neiInterps[k].coeffs[2];
				int no2 = neiInterps[k].ctrlNos[0];

				double val1 = A1*curCtrls[no1].y + B1*curCtrls[no1+1].y + C1*curCtrls[no1+2].y;
				double val2 = A2*neiCtrls[no2].y + B2*neiCtrls[no2+1].y + C2*neiCtrls[no2+2].y;

				rme += (val1-val2)*(val1-val2);
				no ++;
			}

		}
	}
	rme = sqrtf(rme/no);
	return rme;
}


void ToneUnifier::outputEvaluationReport()
{
	cout<<"# Making Evaluation Report ..."<<endl;
	//! color consistence
	Vec2d colorDiffs[3] = {0};
	for (int t = 0; t < 3; t ++)
	{
		compareColorDiff(t, colorDiffs[t][0], colorDiffs[t][1]);
	}

	//! gradient detail
	double aveGradDiff = 0;
	//! in the order of imgIndex
	for (int i = 0; i < _imgNum; i ++)
	{
		Mat srcImage = imread(_filePathList[i], 0);
		Mat dstImage = imread(_savePathList[i], 0);
		aveGradDiff += Utils::calStructuralDiffs(srcImage, dstImage, _imageInforList[i].ROIIndexList);
	}
	aveGradDiff = 10*aveGradDiff/_imgNum;
	double runTime = double(_etime-_stime)/CLOCKS_PER_SEC;
	string savePath = Utils::baseDir + "report.txt";
	ofstream fout;
	fout.open(savePath.c_str(), ios::out);
	fout<< fixed << setprecision(5);
	fout<<"Algorithm running time:"<<runTime<<" s"<<endl;
	fout<<"# Color Disparity"<<endl;
	fout<<"-original :"<<colorDiffs[0][0]<<" "<<colorDiffs[1][0]<<" "<<colorDiffs[2][0]<<"  mean:"<<(colorDiffs[0][0]+colorDiffs[1][0]+colorDiffs[2][0])/3<<endl;
	fout<<"-corrected:"<<colorDiffs[0][1]<<" "<<colorDiffs[1][1]<<" "<<colorDiffs[2][1]<<"  mean:"<<(colorDiffs[0][1]+colorDiffs[1][1]+colorDiffs[2][1])/3<<endl;
	fout<<"# Gradient Difference"<<endl;
	fout<<"-deviation:"<<aveGradDiff<<endl;
	fout.close();
	cout<<"-Done!"<<endl;
}


void ToneUnifier::compareColorDiff(int channelNo, double &rme0, double &rme1)
{
	int no = 0;
	rme0 = rme1 = 0;
	for (int i = 0; i < _imgNum-1; i ++)  //! by index order
	{
		int curNo = _imageInforList[i].imgNo;
		vector<Adjacency> curNeigbList = _imageInforList[i].neigbList;
		vector<Point2d> curCtrls = _imageInforList[i].ctrlPoints[channelNo];
		//! finding valid neighbor images
		for (int j = 0; j < curNeigbList.size(); j ++)
		{
			int neigIndex = findImageIndex(curNeigbList[j].neigbNo);
			//! curNo should be smaller than neigNo (matters for following equation)
			if (neigIndex < i)
			{
				continue;
			}
			vector<InterpInfo> curInterps = curNeigbList[j].interpCoeffs[channelNo];
			vector<InterpInfo> neiInterps;
			vector<double> curKeys = curNeigbList[j].keyGVLists[channelNo];
			vector<double> neiKeys;
			vector<Adjacency> testNeigbList = _imageInforList[neigIndex].neigbList;
			vector<Point2d> neiCtrls = _imageInforList[neigIndex].ctrlPoints[channelNo];
			//! locating the index of current image in the neighbor list
			for (int k = 0; k < testNeigbList.size(); k ++)
			{
				if (testNeigbList[k].neigbNo == curNo)
				{
					neiInterps = testNeigbList[k].interpCoeffs[channelNo];
					neiKeys = testNeigbList[k].keyGVLists[channelNo];
					break;
				}
			}
			for (int k = 0; k < curInterps.size(); k ++)
			{
				//! cur-image
				double A1 = curInterps[k].coeffs[0];
				double B1 = curInterps[k].coeffs[1];
				double C1 = curInterps[k].coeffs[2];
				int no1 = curInterps[k].ctrlNos[0];

				//! neig-image
				double A2 = neiInterps[k].coeffs[0];
				double B2 = neiInterps[k].coeffs[1];
				double C2 = neiInterps[k].coeffs[2];
				int no2 = neiInterps[k].ctrlNos[0];

				double val1 = A1*curCtrls[no1].y + B1*curCtrls[no1+1].y + C1*curCtrls[no1+2].y;
				double val2 = A2*neiCtrls[no2].y + B2*neiCtrls[no2+1].y + C2*neiCtrls[no2+2].y;

				rme0 += (curKeys[k]-neiKeys[k])*(curKeys[k]-neiKeys[k]);
				rme1 += (val1-val2)*(val1-val2);
				no ++;
			}
		}
	}
	rme0 = sqrtf(rme0/no);
	rme1 = sqrtf(rme1/no);
}
