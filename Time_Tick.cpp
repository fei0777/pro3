#include "Time_Tick.h"

//#include "HOG_SVM.h"
#include <windows.h>
#include <winbase.h>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace cv;

bool Time_Tick::debug = true;

Time_Tick::Time_Tick()
{
	t = (double)getTickCount();
}

Time_Tick::Time_Tick(string s)
{
	name = s;
	t = (double)getTickCount();
}

Time_Tick::~Time_Tick()
{
	t = (double)getTickCount() - t;
	if(debug)
		cout << name << " : " << t/1000 << "ms" << endl;
}

void Time_Tick::part()
{
	int tmp = (double)getTickCount() - t;
	if(debug)
		cout << name << " : " << tmp / 1000 << "ms" << endl;
}