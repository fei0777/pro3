#pragma once
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <math.h>

//#pragma comment(lib,"opencv_world310.lib") //加入链接库

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
//#include <ml.hpp>
//#include <face.hpp>
//#include <face/facerec.hpp>
#include <queue>
#include <thread>
#include <mutex>

using namespace std;
using namespace cv;
using namespace cv::ml;
//using namespace face;

#define MIN_WIDTH_LENGTH 30
#define MIN_HEIGHT_LENGTH 30

#define STORE_RECTS_NUM 5

class vedio_func
{
public:
	bool isLoad;
	string file;
	VideoCapture capture;
	int fps;
	long int frame_nums;

	//
	int store_rects_index;
	int store_rects_num;
	vector<vector<Rect>> store_rects;
	vector<vector<int>> store_flag;
	vector<vector<double>> store_probability;
	//用于多帧检测
	long int frame_i;
	map<int, map<Rect, map<string, double>>> frame_result;
	Mat last_mat;
	vector<Point> last_mat_points;

	/****************中间暂时没用****************/
	queue<Mat> input_mats;
	queue<vector<Point>> input_points;
	queue<Mat> output_mats;
	queue<vector<Rect>> output_rects;
	int frame_w_i;
	int frame_w_o;
	int frame_r_i;
	int frame_r_o;
	mutex frame_w_lock;
	mutex frame_r_lock;
	/********************************/




public:
	vedio_func();
	~vedio_func();

public:
	void loadVedio(const char* filename);
	void playVedio();
	void debug_playVedio();
	void playVedio(const char* filename);

	void init();
	//此函数只接受视频中连续的帧，否则没有效果
	void deal_mat(Mat& mat, vector<Rect>& rect_result);
	void calcu_pro_with_pre(Mat& mat, vector<Rect>& rect_result, vector<int>& rect_flag, vector<double>& rect_pro, vector<bool>& new_old);

	/****************中间暂时没用****************/
	void input_frame_w(Mat frame, int _frame_i);
	void output_frame_w(Mat& frame, int& _frame_o, int flag = 0);
	void input_frame_r(Mat frame, int _frame_i);
	void output_frame_r(Mat& frame, int& _frame_o);
	bool get_no_deal_frame(Mat& pre_frame, vector<Point>& pre_points, Mat& now_frame, vector<Point>& now_points);
	/********************************/

};

