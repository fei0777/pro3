#pragma once
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>  
#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  

using namespace std;
using namespace cv;

#define MAX_VALUE 0xffff
#define THRESH 50

#define NO_TYPE 0
#define SQUARE_TYPE 1
#define CROSS_TYPE 2
#define DIAMOND_TYPE 3

#define MAX_NUM_MOVE_OBJ 5

int get_mat_xy_value(Mat& src, int x, int y);

class Nodes
{
public:
	int x;
	int y;
	int value;
	int nums;
	int type;
	Nodes* _nodes;
	Nodes() :x(0), y(0) {
		type = NO_TYPE; nums = 0; _nodes = NULL;
	};
	Nodes(int _x, int _y) :x(_x), y(_y) {
		type = NO_TYPE; nums = 0; _nodes = NULL;
	};
	virtual void set_xy(int _x, int _y) {
		x = _x; y = _y;
	}
	void init_value(Mat& src) {
		value = get_mat_xy_value(src, x, y);
		for (int i = 0; i < nums; i++)
		{
			_nodes[i].value = get_mat_xy_value(src, _nodes[i].x, _nodes[i].y);
		}
	}
	//virtual void init_value(Mat& src);
	int get_min_dis_node(Nodes* nodes, int size)//返回index
	{
		int min_i = -1;
		int min_dis = MAX_VALUE;
		int min_self_dis = MAX_VALUE;
		for (int i = 0; i < size; i++)
		{
			if (nodes[i].type != this->type)
				continue;
			int s = 0;
			for (int j = 0; j < nodes[i].nums; j++)
			{
				if (nodes[i]._nodes[j].value < 0 || _nodes[j].value < 0)
					continue;
				int tmp = nodes[i]._nodes[j].value - _nodes[j].value;
				s += tmp * tmp;
			}
			//周围欧式距离差相等的情况下，选择中心距离差最小的点
			if (i == 0)
			{
				min_dis = s;
			}
			if (s == min_dis && min_i >= 0)
			{
				if (abs(value - nodes[i].value) < abs(value - nodes[min_i].value))
				{
					min_i = i;
					min_dis = s;
				}
			}
			else if (s < min_dis)
			{
				min_i = i;
				min_dis = s;
			}
		}
		return min_i;
	}
};
class Square_Nodes:public Nodes
{
public:
	void set_xy(int _x, int _y) {
		x = _x; y = _y;
		for (int i = 0; i < nums; i++)
		{
			_nodes[i].x = _x + i % 3;
			_nodes[i].y = _y + i / 3;
		}
	}
	Square_Nodes() :Nodes() {
		type = SQUARE_TYPE;
		nums = 9;
		_nodes = new Nodes[nums];
		set_xy(0, 0);
	};
	Square_Nodes(int _x, int _y) :Nodes(_x, _y) { 
		type = SQUARE_TYPE;
		nums = 9;
		_nodes = new Nodes[nums];
		set_xy(_x, _y);
	}
	~Square_Nodes() {
		delete[] _nodes;
	}
};
class Cross_Nodes :public Nodes
{
public:
	void set_xy(int _x, int _y) {
		x = _x; y = _y;
		_nodes[0].x = _x; _nodes[0].y = _y - 1;
		_nodes[1].x = _x - 1; _nodes[1].y = _y;
		_nodes[2].x = _x; _nodes[2].y = _y;
		_nodes[3].x = _x + 1; _nodes[3].y = _y;
		_nodes[4].x = _x; _nodes[4].y = _y + 1;
	}
	Cross_Nodes() :Nodes() {
		type = CROSS_TYPE;
		nums = 5;
		_nodes = new Nodes[nums];
		set_xy(0, 0);
	};
	Cross_Nodes(int _x, int _y) :Nodes(_x, _y) {
		type = CROSS_TYPE;
		nums = 5;
		_nodes = new Nodes[nums];
		set_xy(_x, _y);
	}
	~Cross_Nodes() {
		delete[] _nodes;
	}
};
class Diamond_Nodes :public Nodes
{
public:
	void set_xy(int _x, int _y) {
		x = _x; y = _y;
		_nodes[0].x = _x; _nodes[0].y = _y - 2;
		_nodes[1].x = _x - 1; _nodes[1].y = _y - 1;
		_nodes[2].x = _x; _nodes[2].y = _y - 1;
		_nodes[3].x = _x + 1; _nodes[3].y = _y - 1;
		_nodes[4].x = _x - 2; _nodes[4].y = _y;
		_nodes[5].x = _x - 1; _nodes[5].y = _y;
		_nodes[6].x = _x; _nodes[6].y = _y;
		_nodes[7].x = _x + 1; _nodes[7].y = _y;
		_nodes[8].x = _x + 2; _nodes[8].y = _y;
		_nodes[9].x = _x - 1; _nodes[9].y = _y + 1;
		_nodes[10].x = _x; _nodes[10].y = _y + 1;
		_nodes[11].x = _x + 1; _nodes[11].y = _y + 1;
		_nodes[12].x = _x; _nodes[12].y = _y + 2;
	}
	Diamond_Nodes() :Nodes() {
		type = DIAMOND_TYPE;
		nums = 13;
		_nodes = new Nodes[nums];
		set_xy(0, 0);
	};
	Diamond_Nodes(int _x, int _y) :Nodes(_x, _y) {
		type = DIAMOND_TYPE;
		nums = 13;
		_nodes = new Nodes[nums];
		set_xy(_x, _y);
	}
	~Diamond_Nodes() {
		delete[] _nodes;
	}
};

class node_func
{
public:
	static bool debug;

	static int erode_ele_size;
public:
	node_func();
	~node_func();

public:
	static void QuickSort(int *A, int left, int right);
	static void DrawPoint(Mat& src,vector<Point>& a);

	static double gaussrand(double V, double E);
	static void get_gaus_nums(int *a, int nums, int min_threshold, int max_threshold);
	static void get_gaus_nodes(Mat& src, vector<Point>& a, int flag = 0);//no_use_edge默认20

	static void get_Moravec_nodes(Mat& src, vector<Point>& a, int kSize = 5, int threshold = 0, int flag = 0);//no_use_edge默认20

	static void get_Harris_nodes(Mat& src, vector<Point>& a, int thresh = 0, int flag = 0);
	static void get_GoodFeature_nodes(Mat& src, Mat& ROI, vector<Point>& a, int g_maxCornerNumber=33, int flag = 0);

	//static int get_mat_xy_value(Mat& src, int x, int y);
	//static int get_mat_xy_value2(Mat& src, int x, int y);
	//下列函数使用点跟踪的方法
	static void get_min_distance_xy_tss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_xy_tss2(Mat& src, int value, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_xy_ntss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_xy_4ss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_xy_ds(Mat& src, int value, int x, int y, int& ret_x, int& ret_y);
	static void node_track(Mat& pre_src, Mat& next_src, vector<Point>& a, vector<Point>& b);

	//下列函数使用区域跟踪的方法
	static void get_min_distance_area_tss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_area_tss2(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_area_ntss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_area_4ss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_area_ds1(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void get_min_distance_area_ds2(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y);
	static void node_track2(Mat& pre_src, Mat& next_src, vector<Point>& a, vector<Point>& b);

	//获取两个点集中，移动方向一致的数量最多的点的移动方向
	static bool get_xd_yd_pp(vector<Point>& a, vector<Point>& b, int& xd, int& yd);
	//获取相邻两帧中真正的移动目标
	//pre_src:前一帧原图像
	//now_src:当前帧原图像
	//result:画框
	//dst:返回的差值图像
	//width_miss:左侧忽略宽度
	//height_miss:上册忽略宽度
	//xcache:计算差值的准点左右晃动
	//ycache:计算差值的准点上下晃动
	static void find_move_obj(Mat& pre_src, Mat& now_src, vector<Rect>& result, Mat& dst, int width_miss = 0, int height_miss = 0, int xcache = 0, int ycache = 0);


	//计算相邻帧的像素差，以二值图像形式输出
	//前两个参数需要为3通道图像，后一个参数需要为单通道图像,flag == 0 时，表示不用重新寻找points点
	static void calculate_edge(Mat& pre_src, Mat& now_src, Mat& dst, vector<Point>& pre_points, int width_miss = 0, int height_miss = 0, int flag = 0);

	static double check_points(vector<Point>& a);
	static int find_near_type(short** a, int i, int j, int width, int height, int pid);
	static void self_filter(Mat& gray_src, map<int, Rect>& result);
	static void contour_filter(Mat& gray_src, map<int, Rect>& result);
	static void get_move(Mat& gray_src, map<int, Rect>& result);

	static void merge_move_obj(map<int, Rect>& result);


	//src需要为灰度图像,src和a的长宽需要完全一致
	static void convert_mat_intArray(Mat& src, short** a);
	static void convert_intArray_mat(short** a, Mat& src);
	static void new_short_2d_array(short**& a, int width, int height);
	static void delete_short_2d_array(short**& a, int width, int height);
};

