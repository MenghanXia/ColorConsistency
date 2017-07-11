#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include "unification.h"

using namespace std;
using namespace cv;

void main()
 {
	cout<<"# Running CoColour ..."<<endl;
	clock_t start_time, end_time;
	start_time = clock();

	string dataDir = Utils::baseDir + "Images";
	vector<string> filePathList = Utils::get_filelist(dataDir);
	cout<<"-Loaded "<<filePathList.size()<<" files."<<endl;

	ToneUnifier unifier(filePathList);   //! ToneUnifier LinearModel LinearGamma  
	unifier.unifyMultiTones();

	end_time = clock();
	cout<<"# All done! Consumed "<<double(end_time-start_time)/CLOCKS_PER_SEC<<" seconds"<<endl;
}