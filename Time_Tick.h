#pragma once
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
using namespace std;
class Time_Tick
{
private:
	static bool debug;
	double t;
	string name;
public:
	Time_Tick();
	Time_Tick(string s);
	~Time_Tick();

	void part();
};

