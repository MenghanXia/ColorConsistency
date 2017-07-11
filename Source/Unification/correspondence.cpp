#include "correspondence.h"


void EntryMatch::analyzeCDFQuatileValues(vector<double> data, double quatile, Vec2d &qValues)
{
	int pixNum = data.size();
	int binNum = 256;
	int hist[256] = {0};
	double minGray = 999, maxGray = 0;
	for (int i = 0; i < pixNum; i ++)
	{
		double pix = data[i];
		if (minGray > pix)
		{
			minGray = pix;
		}
		if (maxGray < pix)
		{
			maxGray = pix;
		}
	}
	double binWidth = (maxGray-minGray)/binNum;
	//! count histogram of image
	for (int i = 0; i < pixNum; i ++)
	{
		double pix = data[i];
		int index = int((pix-minGray)/binWidth);
		index = min(binNum-1, max(0,index));
		hist[index] ++;
	}

	double cdf[256] = {0.0};
	//! convert to CDF from histogram
	cdf[0] = 1.0*hist[0]/pixNum;
	for (int i = 1; i < binNum; i ++)
	{
		cdf[i] = 1.0*hist[i]/pixNum + cdf[i-1];
	}
	cdf[binNum-1] = 1.0;       //! just for normalization

	//! search quantile values
	for (int i = 0; i < binNum-1; i ++)
	{
		if (cdf[i] <= quatile && cdf[i+1] > quatile)
		{
			qValues[0] = minGray + i*binWidth;
			break;
		}
	}
	for (int i = binNum-1; i > 1; i --)
	{
		if (cdf[i-1] < 1-quatile && cdf[i] >= 1-quatile)
		{
			qValues[1] = minGray + i*binWidth;
			break;
		}
	}
}


void EntryMatch::grabHistkeyPtPairsbyMatch(vector<double> data1, vector<double> data2, vector<double> &keyGVs1, vector<double> &keyGVs2)
{

}


void EntryMatch::grabHistkeyPtPairsbyProbi(vector<double> data1, vector<double> data2, vector<double> &keyGVs1, vector<double> &keyGVs2)
{
	int pixNum1 = data1.size(), pixNum2 = data2.size();
	int binNum = 300;
	int hist1[300] = {0}, hist2[300] = {0};
	int i, j;
	double minGray1 = 999, maxGray1 = 0;
	for (i = 0; i < pixNum1; i ++)
	{
		double pix = data1[i];
		if (pix < 0)
		{
			int kk = 0;
		}
		if (minGray1 > pix)
		{
			minGray1 = pix;
		}
		if (maxGray1 < pix)
		{
			maxGray1 = pix;
		}
	}
	double minGray2 = 999, maxGray2 = 0;
	for (i = 0; i < pixNum2; i ++)
	{
		double pix = data2[i];
		if (minGray2 > pix)
		{
			minGray2 = pix;
		}
		if (maxGray2 < pix)
		{
			maxGray2 = pix;
		}
	}
	double minGray = min(minGray1, minGray2);
	double maxGray = max(maxGray1, maxGray2);
	double binWidth = (maxGray-minGray)/binNum;

	//! count histogram of image 1
	for (i = 0; i < pixNum1; i ++)
	{
		double pix = data1[i];
		int index = int((pix-minGray)/binWidth);
		index = min(binNum-1, max(0,index));
		hist1[index] ++;
	}
	//! count histogram of image 2
	for (i = 0; i < pixNum2; i ++)
	{
		double pix = data2[i];
		int index = int((pix-minGray)/binWidth);
		index = min(binNum-1, max(0,index));
		hist2[index] ++;
	}

	double cdf1[300] = {0.0};
	double cdf2[300] = {0.0};
	//! convert to CDF from histogram
	cdf1[0] = 1.0*hist1[0]/pixNum1;
	cdf2[0] = 1.0*hist2[0]/pixNum2;
	for (i = 1; i < binNum; i ++)
	{
		cdf1[i] = 1.0*hist1[i]/pixNum1 + cdf1[i-1];
		cdf2[i] = 1.0*hist2[i]/pixNum2 + cdf2[i-1];
	}
	cdf1[binNum-1] = 1.0;       //! just for normalization
	cdf2[binNum-1] = 1.0;       //! just for normalization

	int ctrlPtNum = MATCH_NUM;
	//! set quadratic probability values
	double* quadraticVals = new double[ctrlPtNum];    //! 0.0 < range < 1.0
	double step = 1.0/(ctrlPtNum+1);   //! +2 : add meaningless 0.0 and 1.0
	for (i = 0; i < ctrlPtNum; i ++)     
	{
		quadraticVals[i] = (i+1)*step;
	}

	//! get histogram gray samples of quadratic probability
	for (i = 0; i < ctrlPtNum; i ++)
	{
		double p = quadraticVals[i];     //! current target probability
		//! finding corresponding bin index of histogram 1
		double binIndex1 = 0;           //! default value
		for (j = 0; j < binNum-1; j ++)
		{	
			//! consider case : cdf1[j+1] == cdf1[j]
			if (cdf1[j] <= p && p < cdf1[j+1])
			{
				if (cdf1[j+1] == cdf1[j])
				{
					binIndex1 = j;  //interpolating for a precise gray Value
				}
				else
				{
					binIndex1 = j+(p-cdf1[j])/(cdf1[j+1]-cdf1[j]);  //interpolating for a precise gray Value
				}
				break;
			}
		}
		keyGVs1.push_back(binIndex1);

		//! finding corresponding bin index of histogram 2
		double binIndex2 = 0;           //! default value
		for (j = 0; j < binNum-1; j ++)
		{	
			//! consider case : cdf1[j+1] == cdf1[j]
			if (cdf2[j] <= p && p < cdf2[j+1])
			{
				if (cdf2[j+1] == cdf2[j])
				{
					binIndex2 = j;  //interpolating for a precise gray Value
				}
				else
				{
					binIndex2 = j+(p-cdf2[j])/(cdf2[j+1]-cdf2[j]);  //interpolating for a precise gray Value
				}
				break;
			}
		}
		keyGVs2.push_back(binIndex2);
	}
	//! convert index of bin to real intensity values
	for (i = 0; i < ctrlPtNum; i ++)   //convert to the real gray intensity
	{
		keyGVs1[i] = binWidth*keyGVs1[i]+minGray;
		keyGVs2[i] = binWidth*keyGVs2[i]+minGray;
	}
	//! consider the problem from precise of the discretized bin's width
	keyGVs1[0] = max(keyGVs1[0],minGray1);
	keyGVs1[MATCH_NUM-1] = min(keyGVs1[MATCH_NUM-1],maxGray1);
	keyGVs2[0] = max(keyGVs2[0],minGray2);
	keyGVs2[MATCH_NUM-1] = min(keyGVs2[MATCH_NUM-1],maxGray2);

	if (0)
	{
		string filePath = Utils::baseDir + "Cache/temp/histMatchPts.txt";
		FILE *fp = fopen(filePath.c_str(), "w");
		for (i = 0; i < ctrlPtNum; i ++)
		{
			fprintf(fp, "%lf  %lf\n", keyGVs1[i], keyGVs2[i]);
		}
		fclose(fp);
	}

	delete []quadraticVals;
}
