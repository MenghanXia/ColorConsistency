#pragma once
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp> 
#include "opencv2/nonfree/nonfree.hpp"
#include "Utils/util.h"

using namespace std;
using namespace cv;

#define MATCH_NUM 16

// ============== NOTIFICATION ============== //
//! node is encoded from 1, 2, ..., n.        //
//! namely, the minimum node no is 1          //
// ===========================================//


class EntryMatch
{
public:
	EntryMatch(){};

public:
	//! only for : values range [0,255]
	static void analyzeCDFQuatileValues(vector<double> data, double quatile, Vec2d &qValues);

	static void grabHistkeyPtPairsbyMatch(vector<double> data1, vector<double> data2, vector<double> &keyGVs1, vector<double> &keyGVs2);
	static void grabHistkeyPtPairsbyProbi(vector<double> data1, vector<double> data2, vector<double> &keyGVs1, vector<double> &keyGVs2);
};
