#include "ColorSpace.h"


Mat ColorSpace::uRGB2dRGB(Mat srcImage)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage(row, col, CV_64FC3, Scalar(0,0,0));
	uchar *srcData = (uchar*)srcImage.data;
	double *dstData = (double*)dstImage.data;
	for(r = 0; r < row; ++r)
	{
		for(c = 0; c < col; ++c)
		{
			double R = srcData[3*(r*col+c)+2];    //! R
			double G = srcData[3*(r*col+c)+1];    //! G
			double B = srcData[3*(r*col+c)+0];    //! B

			dstData[3*(r*col+c)+0] = B;   //! B
			dstData[3*(r*col+c)+1] = G;   //! G
			dstData[3*(r*col+c)+2] = R;   //! R
		}
	}
	return dstImage;
}


Mat ColorSpace::dRGB2uRGB(Mat srcImage)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage;
	dstImage = Mat(row, col, CV_8UC3, Scalar(0,0,0));

	double *srcData = (double*)srcImage.data;
	uchar *dstData2;
	dstData2 = (uchar*)dstImage.data;

	for(r = 0; r < row; ++r)
	{
		for(c = 0; c < col; ++c)
		{
			double B = srcData[3*(r*col+c)+0];
			double G = srcData[3*(r*col+c)+1];
			double R = srcData[3*(r*col+c)+2];

			dstData2[3*(r*col+c)+0] = min(255, max(0,int(B)));
			dstData2[3*(r*col+c)+1] = min(255, max(0,int(G)));
			dstData2[3*(r*col+c)+2] = min(255, max(0,int(R)));
		}
	}
	return dstImage;
}


Mat ColorSpace::RGB2YCbCr(Mat srcImage)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage(row, col, CV_64FC3, Scalar(0,0,0));
	uchar *srcData = (uchar*)srcImage.data;
	double *dstData = (double*)dstImage.data;
	for(r = 0; r < row; ++r)
	{
		for(c = 0; c < col; ++c)
		{
			double R = srcData[3*(r*col+c)+2];    //! R
			double G = srcData[3*(r*col+c)+1];    //! G
			double B = srcData[3*(r*col+c)+0];    //! B
			double Y  = 0   + 0.299*R    + 0.587*G    + 0.114*B;
			double Cb = 127.5 - 0.168736*R - 0.331264*G + 0.5*B;
			double Cr = 127.5 + 0.5*R      - 0.418688*G - 0.081312*B;

			dstData[3*(r*col+c)+0] = Y;   //! Y
			dstData[3*(r*col+c)+1] = Cb;  //! Cb
			dstData[3*(r*col+c)+2] = Cr;  //! Cr
		}
	}
	return dstImage;
}


Mat ColorSpace::YCbCr2RGB(Mat srcImage)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage = Mat(row, col, CV_8UC3, Scalar(0,0,0));
	double *srcData = (double*)srcImage.data;
	uchar *dstData = (uchar*)dstImage.data;

	for(r = 0; r < row; ++r)
	{
		for(c = 0; c < col; ++c)
		{
			double Y = srcData[3*(r*col+c)+0];        //! Y
			double Cb = srcData[3*(r*col+c)+1]-127.5;   //! Cb-128
			double Cr = srcData[3*(r*col+c)+2]-127.5;   //! Cr-128

			double R = Y + 1.402*Cr;
			double G = Y - 0.3441*Cb - 0.7141*Cr;
			double B = Y + 1.7720*Cb;

			dstData[3*(r*col+c)+0] = min(255, max(0,int(B)));
			dstData[3*(r*col+c)+1] = min(255, max(0,int(G)));
			dstData[3*(r*col+c)+2] = min(255, max(0,int(R)));
		}
	}
	return dstImage;
}


Mat ColorSpace::RGB2YCbCr(const Mat &srcImage, const vector<int> &roiIndexs)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage(row, col, CV_64FC3, Scalar(0,0,0));
	uchar *srcData = (uchar*)srcImage.data;
	double *dstData = (double*)dstImage.data;
	for (int i = 0; i < roiIndexs.size(); i ++)
	{
		int index = roiIndexs[i];
		double R = srcData[3*index+2];    //! R
		double G = srcData[3*index+1];    //! G
		double B = srcData[3*index+0];    //! B
		double Y  = 0   + 0.299*R    + 0.587*G    + 0.114*B;
		double Cb = 127.5 - 0.168736*R - 0.331264*G + 0.5*B;
		double Cr = 127.5 + 0.5*R      - 0.418688*G - 0.081312*B;

		dstData[3*index+0] = Y;   //! Y
		dstData[3*index+1] = Cb;  //! Cb
		dstData[3*index+2] = Cr;  //! Cr
	}
	return dstImage;
}


Mat ColorSpace::YCbCr2RGB(const Mat &srcImage, const vector<int> &roiIndexs)
{
	int r, c;
	int row = srcImage.rows, col = srcImage.cols;
	Mat dstImage(row, col, CV_8UC3, Scalar(0,0,0));
	double *srcData = (double*)srcImage.data;
	uchar *dstData = (uchar*)dstImage.data;
	for (int i = 0; i < roiIndexs.size(); i ++)
	{
		int index = roiIndexs[i];
		double Y = srcData[3*index+0];        //! Y
		double Cb = srcData[3*index+1]-127.5;   //! Cb-128
		double Cr = srcData[3*index+2]-127.5;   //! Cr-128

		double R = Y + 1.402*Cr;
		double G = Y - 0.3441*Cb - 0.7141*Cr;
		double B = Y + 1.7720*Cb;

		dstData[3*index+0] = min(255, max(0,int(B)));
		dstData[3*index+1] = min(255, max(0,int(G)));
		dstData[3*index+2] = min(255, max(0,int(R)));
	}
	return dstImage;
}


Vec3d ColorSpace::RGB2YCbCr(uchar R, uchar G, uchar B)
{
	double Y  = 0   + 0.299*R    + 0.587*G    + 0.114*B;
	double Cb = 127.5 - 0.168736*R - 0.331264*G + 0.5*B;
	double Cr = 127.5 + 0.5*R - 0.418688*G - 0.081312*B;
	return Vec3d(Y,Cb,Cr);
}


Vec3i ColorSpace::YCbCr2BGR(double Y, double Cb, double Cr)
{
	Cb -= 127.5;
	Cr -= 127.5;
	double R = Y + 1.402*Cr;
	double G = Y - 0.3441*Cb - 0.7141*Cr;
	double B = Y + 1.7720*Cb;

	int Bi = min(255, max(0,int(B)));
	int Gi = min(255, max(0,int(G)));
	int Ri = min(255, max(0,int(R)));
	return Vec3i(Bi,Gi,Ri);
}
