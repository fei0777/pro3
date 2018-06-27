#include "node_func.h"
#include "Time_Tick.h"
#include "vedio_func.h"

bool node_func::debug = true;
int node_func::erode_ele_size = 0;
node_func::node_func()
{
}


node_func::~node_func()
{
}

void node_func::QuickSort(int *A, int left, int right)
{
	if (left >= right) return;
	int x = A[(left + right) >> 1], low = left, high = right;
	while (low < high)
	{
		while (A[low] < x)
			low++;
		while (A[high] > x)
			high--;
		if (low <= high)
		{
			int Temp = A[low];
			A[low] = A[high];
			A[high] = Temp;
			low++;
			high--;
		}
	}
	QuickSort(A, left, high);
	QuickSort(A, low, right);
}

void node_func::DrawPoint(Mat& src, vector<Point>& a)
{
	vector<Point>::iterator it = a.begin();
	for (; it != a.end(); it++)
	{
		int i = it->x;
		int j = it->y;
		circle(src, Point(i, j), 5, Scalar(0, 10, 255), 2, 8, 0);
	}
	
}

double node_func::gaussrand(double V, double E)
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	X = X*V + E;
	return X;
}
void node_func::get_gaus_nums(int *a, int nums, int min_threshold, int max_threshold)
{
	int E = max_threshold / 2;
	int V = E / 2;
	for (int i = 0; i < nums; )
	{
		int index = (int)gaussrand((double)V, (double)E);
		if (index > min_threshold && index < max_threshold)
		{
			a[i] = index;
		}
		else
		{
			continue;
		}
		i++;
	}
}
void node_func::get_gaus_nodes(Mat& src, vector<Point>& a, int flag)
{
	int height = src.rows;
	int width = src.cols;
	int x_edge = (int)(width*0.05 / 2);
	int y_edge = (int)(height*0.05 / 2);
	int index = x_edge;
	int num_x = 0;
	while (num_x <= 0)
	{
		num_x = width / index;
		index = index / 2;
		if (index == 0)
			break;
	}
	index = y_edge;
	int num_y = 0;
	while (num_y <= 0)
	{
		num_y = height / index;
		index = index / 2;
		if (index == 0)
			break;
	}
	if (num_x <= 0 || num_y <= 0)
	{//出错
		return;
	}
	int *x = new int[num_x];
	int *y = new int[num_y];
	//int no_use_edge = 20;

	get_gaus_nums(x, num_x, x_edge, width - 1 - x_edge);
	get_gaus_nums(y, num_y, y_edge, height - 1 - y_edge);
	QuickSort(x, 0, num_x - 1);
	QuickSort(y, 0, num_y - 1);

	if (!flag)
	{
		a.clear();
	}
	int last_x = -1;
	int last_y = -1;
	for (int i = 0; i < num_x; i++)
	{
		while (x[i] == last_x)
		{
			x[i]++;
		}
		for (int j = 0; j < num_y; j++)
		{
			int tmp = y[j];
			while (tmp == last_y)
			{
				tmp++;
			}
			a.push_back(Point(x[i], tmp));
			last_y = tmp;
		}
		last_x = x[i];
	}
}

void node_func::get_Moravec_nodes(Mat& src, vector<Point>& a, int kSize, int threshold, int flag)
{
	int height = src.rows;
	int width = src.cols;
	int index = 30;
	int nCount = 0;
	int r = kSize;
	Point *pPoint = new Point[height * width];

	if (!flag)
	{
		a.clear();
	}
	for (int i = r; i < src.rows - r; i++)
	{
		for (int j = r; j < src.cols - r; j++)
		{
			int wV1, wV2, wV3, wV4;
			wV1 = wV2 = wV3 = wV4 = 0;
			//计算水平方向窗内的兴趣值  
			for (int k = -r; k < r; k++)
				wV1 += (src.at<uchar>(i, j + k) - src.at<uchar>(i, j + k + 1))*
				(src.at<uchar>(i, j + k) - src.at<uchar>(i, j + k + 1));
			//计算垂直方向窗内的兴趣值  
			for (int k = -r; k < r; k++)
				wV2 += (src.at<uchar>(i + k, j) - src.at<uchar>(i + k + 1, j))*
				(src.at<uchar>(i + k, j) - src.at<uchar>(i + k + 1, j));
			//计算45方向窗内兴趣值  
			for (int k = -r; k < r; k++)
				wV3 += (src.at<uchar>(i + k, j + k) - src.at<uchar>(i + k + 1, j + k + 1))*
				(src.at<uchar>(i + k, j + k) - src.at<uchar>(i + k + 1, j + k + 1));
			//计算135方向窗内的兴趣值  
			for (int k = -r; k < r; k++)
				wV3 += (src.at<uchar>(i + k, j - k) - src.at<uchar>(i + k + 1, j - k - 1))*
				(src.at<uchar>(i + k, j - k) - src.at<uchar>(i + k + 1, j - k - 1));
			//选取其中的最小值作为该像素点的最终兴趣值  
			int value = min(min(wV1, wV2), min(wV3, wV4));
			//若兴趣值大于阈值，则将点的坐标存入数组中  
			if (value > threshold)
			{
				a.push_back(Point(j,i));
				//pPoint[nCount] = Point(j, i);
				//nCount++;
			}
		}
	}
}

void node_func::get_Harris_nodes(Mat& src, vector<Point>& a, int thresh, int flag)
{
	if (!flag)
	{
		a.clear();
	}
	Mat g_grayImage;
	cvtColor(src, g_grayImage, COLOR_BGR2GRAY);
	Mat dstImage;
	Mat normImage;
	Mat scaledImage;
	dstImage = Mat::zeros(src.size(), CV_32FC1);
	Mat src1 = src.clone();
	//进行角点检测  
	cornerHarris(g_grayImage, dstImage, 2, 3, 0.04, BORDER_DEFAULT);
	/*if (node_func::debug)
	{
		imshow("debug", dstImage);
		cvWaitKey(0);
	}*/
	//归一化与转换  
	normalize(dstImage, normImage, 0, 255, NORM_MINMAX, CV_32FC1);
	/*if (node_func::debug)
	{
		imshow("debug", normImage);
		cvWaitKey(0);
	}*/
	convertScaleAbs(normImage, scaledImage);
	/*if (node_func::debug)
	{
		imshow("debug", scaledImage);
		cvWaitKey(0);
	}*/

	for (int j = 0; j < normImage.rows; j++)//height
	{
		float* pData1 = normImage.ptr<float>(j);
		for (int i = 0; i < normImage.cols; i++)//width
		{
			if((int)pData1[i] > thresh)
			//((int)normImage.at<float>(j, i) > thresh)
			{
				a.push_back(Point(i, j));
			}
		}

	}
}

void node_func::get_GoodFeature_nodes(Mat& src, Mat& ROI, vector<Point>& a, int g_maxCornerNumber, int flag)
{
	if (!flag)
	{
		a.clear();
	}
	Mat g_grayImage;
	cvtColor(src, g_grayImage, COLOR_BGR2GRAY);
	if (g_maxCornerNumber <= 1)
	{
		g_maxCornerNumber = 1;
	}
	//vector<Point2f> corners;//用a替代
	double qualityLevel = 0.3;
	double minDistance = 10;
	int blockSize = 5;
	double k = 0.04;//使用Harris算法时使用，最好使用默认值0.04
	//Mat copy = src.clone();//可以用Mat()替代，表示检测全幅图像。此处用上面的ROI替代
	goodFeaturesToTrack(g_grayImage, a, g_maxCornerNumber, qualityLevel, minDistance, ROI, blockSize, true, k);
}

//横方向为列上限，x标定，竖方向为行上限，y标定
int get_mat_xy_value(Mat& src, int x, int y)
{
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return -1;
	/*if (x >= src.rows)
		return -1;*/
	uchar* pData = src.ptr<uchar>(y);
	return (int)pData[x];
	//return (int)src.at<float>(x, y);
}
//依次缩短步法的3步法
void node_func::get_min_distance_xy_tss2(Mat& src, int value, int x, int y, int& ret_x, int& ret_y)
{
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 8;
	int pre_x = x;
	int pre_y = y;
	while(pid >= 1)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		int v[9] = { 0 };
		int d[9] = { 0 };
		int min_i = -1;
		int min_value = MAX_VALUE;
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] >= 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
		}
		if (d[4] <= min_value)//中间不动特性
		{
			min_i = 4;
			min_value = d[4];
		}
		ret_x = _x[min_i];
		ret_y = _y[min_i];
		pre_x = ret_x;
		pre_y = ret_y;

		pid = pid / 2;
	}
}

//4重循环，3步法
void node_func::get_min_distance_xy_tss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y)
{
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 1;
	int pre_x = x;
	int pre_y = y;
	for(int i=0;i<4;i++)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		int v[9] = { 0 };
		int d[9] = { 0 };
		int min_i = -1;
		int min_value = MAX_VALUE;
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] >= 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
		}
		if (d[4] <= min_value)//中间不动特性
		{
			min_i = 4;
			min_value = d[4];
		}
		ret_x = _x[min_i];
		ret_y = _y[min_i];
		pre_x = ret_x;
		pre_y = ret_y;
	}
}

void node_func::get_min_distance_xy_ntss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y)
{
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 1;
	int pre_x = x;
	int pre_y = y;
	for (int i = 0; i < 4; i++)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		int v[9] = { 0 };
		int d[9] = { 0 };
		int min_i = -1;
		int min_value = MAX_VALUE;
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] >= 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
		}
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid * 4;
			_y[j] = pre_y + (j / 3 - 1) * pid * 4;
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] >= 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
		}
		if (d[4] <= min_value)//中间不动特性
		{
			min_i = 4;
			min_value = d[4];
		}
		ret_x = _x[min_i];
		ret_y = _y[min_i];
		pre_x = ret_x;
		pre_y = ret_y;
	}
}

//0   1   2
//3   4   5
//6   7   8
//
void node_func::get_min_distance_xy_4ss(Mat& src, int value, int x, int y, int& ret_x, int& ret_y)
{
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 3;
	int pre_x = x;
	int pre_y = y;
	for (int i = 0; i < 4; i++)
	{
		if (i == 3)
			pid = 1;

		int _x[9] = { 0 };
		int _y[9] = { 0 };
		int v[9] = { 0 };
		int d[9] = { 0 };
		int min_i = -1;
		int min_value = MAX_VALUE;
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] >= 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
		}
		if (d[4] <= min_value)//中间不动特性
		{
			min_i = 4;
			min_value = d[4];
		}
		ret_x = _x[min_i];
		ret_y = _y[min_i];
		pre_x = ret_x;
		pre_y = ret_y;
		/*if (min_i == 4)
			break;*/
	}
}

//      0
//   1     2
//3     4     5
//   6     7
//      8
//   0
//1  2  3
//   4
void node_func::get_min_distance_xy_ds(Mat& src, int value, int x, int y, int& ret_x, int& ret_y)
{
	int pre_x = x;
	int pre_y = y;
	int pid = 2;
	int _x[9] = { 0,-1,1,-2,0,2,-1,1,0 };
	int _y[9] = { -2,-1,-1,0,0,0,1,1,2 };
	int iterator_i = 0;
	do
	{
		int v[9] = { 0 };
		int d[9] = { 0 };
		int min_i = -1;
		int min_value = MAX_VALUE;
		for (int j = 0; j < 9; j++)
		{
			if (iterator_i == 0)
			{
				_x[j] += pre_x;
				_y[j] += pre_y;
			}
			v[j] = get_mat_xy_value(src, _x[j], _y[j]);
			d[j] = v[j] > 0 ? abs(v[j] - value) : MAX_VALUE;
			if (d[j] < min_value)
			{
				min_i = j;
				min_value = d[j];
			}
			if (pid == 1 && j >= 5)
			{
				break;
			}
		}
		if (pid == 2 && d[4] <= min_value)//中间不动特性
		{
			min_i = 4;
			min_value = d[4];
		}
		else if (pid == 1 && d[2] <= min_value)//中间不动特性
		{
			min_i = 2;
			min_value = d[2];
		}
		ret_x = _x[min_i];
		ret_y = _y[min_i];
		pre_x = ret_x;
		pre_y = ret_y;
		iterator_i++;

		if (min_i == 4)
		{
			iterator_i = 0;
			pid = 1;
			_x[0] = 0; _x[1] = -1; _x[2] = 0; _x[3] = 1; _x[4] = 0;
			_y[0] = -1; _y[1] = 0; _y[2] = 0; _y[3] = 0; _y[4] = 1;
		}
	} while (pid == 2 && iterator_i<4);
}




//使用点跟踪的方法
void node_func::node_track(Mat& pre_src, Mat& next_src, vector<Point>& a, vector<Point>& b)
{
	Mat pre_gray(pre_src.size(), CV_8U, Scalar(0));
	Mat next_gray(pre_src.size(), CV_8U, Scalar(0));
	cvtColor(pre_src, pre_gray, COLOR_BGR2GRAY);
	cvtColor(next_src, next_gray, COLOR_BGR2GRAY);
	vector<Point>::iterator it = a.begin();
	int i = 0;
	for (; it != a.end(); it++)
	{
		i++;//50有bug
		int x = it->x;
		int y = it->y;

		int ret_x = 0;
		int ret_y = 0;

		int value = get_mat_xy_value(pre_gray, x, y);

		get_min_distance_xy_tss(next_gray, value, x, y, ret_x, ret_y);

		b.push_back(Point(ret_x,ret_y));
	}
}

//效果相对较好，抖动2.5
//4次循环方形查找
void node_func::get_min_distance_area_tss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 1;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[9];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[9];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[9];
	}
	else
	{
		sn = new Nodes[9];
	}
	for (int i = 0; i < 4; i++)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		//Square_Nodes sn[9];
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			sn[j].set_xy(_x[j], _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, 9);
		if (min_i < 0)
		{
			continue;
		}
		pre_x = sn[min_i].x; pre_y = sn[min_i].y;
		ret_x = sn[min_i].x; ret_y = sn[min_i].y;
		
	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//效果一般，抖动5
//步数（宽度）依次减半的查找
void node_func::get_min_distance_area_tss2(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[9];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[9];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[9];
	}
	else
	{
		sn = new Nodes[9];
	}
	int pid = 8;
	for (; pid >= 1; pid = pid / 2)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		//Square_Nodes sn[9];
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			sn[j].set_xy(_x[j], _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, 9);
		if (min_i < 0)
		{
			continue;
		}
		pre_x = sn[min_i].x; pre_y = sn[min_i].y;
		ret_x = sn[min_i].x; ret_y = sn[min_i].y;
	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//效果相对较好，抖动2
//先找实心小方形，再找空心大方形
void node_func::get_min_distance_area_ntss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 1;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[9];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[9];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[9];
	}
	else
	{
		sn = new Nodes[9];
	}
	for (int i = 0; i < 4; i++)
	{
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		//Square_Nodes sn[9];

		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			sn[j].set_xy(_x[j], _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, 9);
		if (min_i >= 0)
		{
			pre_x = sn[min_i].x; pre_y = sn[min_i].y;
			ret_x = sn[min_i].x; ret_y = sn[min_i].y;
			continue;
		}

		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid * 4;
			_y[j] = pre_y + (j / 3 - 1) * pid * 4;
			sn[j].set_xy(_x[j], _y[j]);
			sn[j].init_value(src);
		}
		min_i = target_node.get_min_dis_node(sn, 9);
		if (min_i >= 0)
		{
			pre_x = sn[min_i].x; pre_y = sn[min_i].y;
			ret_x = sn[min_i].x; ret_y = sn[min_i].y;
			continue;
		}
	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//效果一般，抖动3
//4步查找法，方形宽度2 2 2 1
void node_func::get_min_distance_area_4ss(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 3;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[9];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[9];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[9];
	}
	else
	{
		sn = new Nodes[9];
	}
	for (int i = 0; i < 4; i++)
	{
		if (i == 3)
			pid = 1;
		int _x[9] = { 0 };
		int _y[9] = { 0 };
		//Square_Nodes sn[9];
		for (int j = 0; j < 9; j++)
		{
			_x[j] = pre_x + (j % 3 - 1) * pid;
			_y[j] = pre_y + (j / 3 - 1) * pid;
			sn[j].set_xy(_x[j], _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, 9);
		if (min_i < 0)
		{
			continue;
		}
		pre_x = sn[min_i].x; pre_y = sn[min_i].y;
		ret_x = sn[min_i].x; ret_y = sn[min_i].y;
	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//效果一般，抖动3
//3次循环空心大菱形，最后一次循环实心十字形
void node_func::get_min_distance_area_ds1(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 2;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	int xy_size = 9;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[xy_size];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[xy_size];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[xy_size];
	}
	else
	{
		sn = new Nodes[xy_size];
	}
	int _x[9] = { 0,-1,1,-2,0,2,-1,1,0 };
	int _y[9] = { -2,-1,-1,0,0,0,1,1,2 };
	for (int i = 0; i < 4; i++)
	{
		if (i == 3)
		{
			pid = 1;
			xy_size = 5;
			_x[0] = 0; _x[1] = -1; _x[2] = 0; _x[3] = 1; _x[4] = 0;
			_y[0] = -1; _y[1] = 0; _y[2] = 0; _y[3] = 0; _y[4] = 1;
		}
		//Square_Nodes sn[9];
		for (int j = 0; j < xy_size; j++)
		{
			//_x[j] = pre_x + (j % 3 - 1) * pid;
			//_y[j] = pre_y + (j / 3 - 1) * pid;
			sn[j].set_xy(pre_x + _x[j], pre_y + _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, xy_size);
		if (min_i < 0)
		{
			continue;
		}
		pre_x = sn[min_i].x; pre_y = sn[min_i].y;
		ret_x = sn[min_i].x; ret_y = sn[min_i].y;

	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//效果一般，抖动3
//4次循环实心大菱形
void node_func::get_min_distance_area_ds2(Mat& src, Nodes& target_node, int x, int y, int& ret_x, int& ret_y)
{
	//默认target_node已经进行value初始化init_value,因为src为后一帧图像数据，无法取点
	if (x < 0 || x >= src.cols || y < 0 || y >= src.rows)
		return;
	int pid = 2;
	int pre_x = x;
	int pre_y = y;
	Nodes * sn = NULL;
	int xy_size = 13;
	if (target_node.type == SQUARE_TYPE)
	{
		sn = new Square_Nodes[xy_size];
	}
	else if (target_node.type == CROSS_TYPE)
	{
		sn = new Cross_Nodes[xy_size];
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		sn = new Diamond_Nodes[xy_size];
	}
	else
	{
		sn = new Nodes[xy_size];
	}
	for (int i = 0; i < 4; i++)
	{
		int _x[13] = { 0,-1,0,1,-2,-1,0,1,2,-1,0,1,0 };
		int _y[13] = { -2,-1,-1,-1,0,0,0,0,0,1,1,1,2 };
		//Square_Nodes sn[xy_size];
		for (int j = 0; j < xy_size; j++)
		{
			/*_x[j] = pre_x + _x[j];
			_y[j] = pre_y + _y[j];
			sn[j].set_xy(_x[j], _y[j]);*/
			sn[j].set_xy(pre_x + _x[j], pre_y + _y[j]);
			sn[j].init_value(src);
		}
		int min_i = target_node.get_min_dis_node(sn, xy_size);
		if (min_i < 0)
		{
			continue;
		}
		pre_x = sn[min_i].x; pre_y = sn[min_i].y;
		ret_x = sn[min_i].x; ret_y = sn[min_i].y;

	}
	if (target_node.type == SQUARE_TYPE)
	{
		delete[]((Square_Nodes*)sn);
	}
	else if (target_node.type == CROSS_TYPE)
	{
		delete[]((Cross_Nodes*)sn);
	}
	else if (target_node.type == DIAMOND_TYPE)
	{
		delete[]((Diamond_Nodes*)sn);
	}
	else
	{
		delete[]sn;
	}
}

//使用区域跟踪的方法
void node_func::node_track2(Mat& pre_src, Mat& next_src, vector<Point>& a, vector<Point>& b)
{
	Mat pre_gray(pre_src.size(), CV_8U, Scalar(0));
	Mat next_gray(pre_src.size(), CV_8U, Scalar(0));
	cvtColor(pre_src, pre_gray, COLOR_BGR2GRAY);
	cvtColor(next_src, next_gray, COLOR_BGR2GRAY);
	vector<Point>::iterator it = a.begin();
	int i = 0;
	for (; it != a.end(); it++)
	{
		i++;//50有bug
		int x = it->x;
		int y = it->y;

		int ret_x = -1;
		int ret_y = -1;

		Diamond_Nodes sn(x, y);
		sn.init_value(pre_gray);

		get_min_distance_area_ntss(next_gray, sn, x, y, ret_x, ret_y);

		//if(ret_x >= 0 && ret_y >= 0)
			b.push_back(Point(ret_x, ret_y));
	}
}

//获取两个点集中，移动方向一致的数量最多的点的移动方向
bool node_func::get_xd_yd_pp(vector<Point>& a, vector<Point>& b, int& xd, int& yd)
{
	if (a.size() != b.size())
		return false;
	if (a.size() == 0)
		return false;

	map<int, map<int, int>> x_y_n;
	int Size = a.size();
	for (int i = 0; i < Size; i++)
	{
		/*if (debug)
		{
			printf("(%d,%d) -> (%d,%d)\n", a[i].x, a[i].y, b[i].x, b[i].y);
		}*/
		if (b[i].x < 0 || b[i].y < 0)
			continue;
		int xd_i = b[i].x - a[i].x;
		int yd_i = b[i].y - a[i].y;
		x_y_n[xd_i][yd_i]++;
	}

	int x_max, y_max;
	int max_num = 0;
	map<int, map<int, int>>::iterator it = x_y_n.begin();
	for (; it != x_y_n.end(); it++)
	{
		map<int, int>::iterator itt = it->second.begin();
		for (; itt != it->second.end(); itt++)
		{
			if (itt->second > max_num)
			{
				max_num = itt->second;
				x_max = it->first;
				y_max = itt->first;
			}
		}
	}

	xd = x_max;
	yd = y_max;

	return true;
}

//获取相邻两帧中真正的移动目标//前两个参数需要为3通道图像
void node_func::find_move_obj(Mat& pre_src, Mat& now_src, vector<Rect>& result, Mat& dst, int width_miss, int height_miss, int xcache, int ycache)
{
	if (pre_src.cols != now_src.cols)
		return;
	if (pre_src.rows != now_src.rows)
		return;

	Time_Tick* tt;
	int width = now_src.cols;
	int height = now_src.rows;
	Mat gray_pre(pre_src.size(), CV_8U, Scalar(0));
	Mat gray_now(now_src.size(), CV_8U, Scalar(0));
	cvtColor(pre_src, gray_pre, COLOR_BGR2GRAY);
	cvtColor(now_src, gray_now, COLOR_BGR2GRAY);
	//Mat dst(now_src.size(), CV_8U, Scalar(0));
	tt = new Time_Tick("find_move_obj");
	if (xcache == 0 && ycache == 0 && width_miss == 0 && height_miss == 0)
	{
		//此操作无太多时间浪费平均0.5ms
		absdiff(gray_now, gray_pre, dst);
	}
	else
	{
		//下列方法平均耗时650ms
		short** pre_a;
		short** now_a;
		new_short_2d_array(pre_a, width, height);
		new_short_2d_array(now_a, width, height);
		convert_mat_intArray(gray_pre, pre_a);
		convert_mat_intArray(gray_now, now_a);
		tt->part();
		for (int i = height_miss; i < height; i++)
		{
			uchar* pData = dst.ptr<uchar>(i);
			for (int j = width_miss; j < width; j++)
			{
				int pre_v = pre_a[i][j];
				int num_v = (xcache * 2 + 1)*(ycache * 2 + 1);
				int *v = new int[num_v];
				memset(v, 0, sizeof(int)*num_v);
				int num_i = 0;
				int min_dis = MAX_VALUE;
				for (int m = (-1)*ycache; m <= ycache; m++)
				{
					if (m + i < 0 || m + i >= height)
						continue;
					for (int n = (-1)*xcache; n <= xcache; n++)
					{
						if (n + j < 0 || n + j >= width)
							continue;
						v[num_i] = now_a[m + i][n + j];
						if (v[num_i] < 0)
						{
							num_i++; continue;
						}
						else
						{
							int tmp = abs(v[num_i] - pre_v);
							if (tmp < min_dis)
							{
								min_dis = tmp;
							}
						}
					}
				}
				delete[]v;
				pData[j] = min_dis < THRESH ? 0 : min_dis;
			}
		}
		delete_short_2d_array(pre_a, width, height);
		delete_short_2d_array(now_a, width, height);
		//下列方法平均耗时900ms
		/*for (int i = height_miss; i < height; i++)
		{
			uchar* pData = dst.ptr<uchar>(i);
			for (int j = width_miss; j < width; j++)
			{
				int pre_v = get_mat_xy_value(gray_pre, j, i);
				int num_v = (xcache * 2 + 1)*(ycache * 2 + 1);
				int *v = new int[num_v];
				memset(v, 0, sizeof(int)*num_v);
				int num_i = 0;
				int min_dis = MAX_VALUE;
				for (int m = (-1)*ycache; m <= ycache; m++)
				{
					for (int n = (-1)*xcache; n <= xcache; n++)
					{
						v[num_i] = get_mat_xy_value(gray_now, n+j, m + i);
						if (v[num_i] < 0)
						{
							num_i++; continue;
						}
						else
						{
							int tmp = abs(v[num_i] - pre_v);
							if (tmp < min_dis)
							{
								min_dis = tmp;
							}
						}
					}
				}
				delete[]v;
				pData[j] = min_dis < THRESH ? 0 : min_dis;
			}
		}*/
	}
	delete tt;
}

//前两个参数需要为3通道图像，后一个参数需要为单通道图像,flag为1则表示重新选取角点
void node_func::calculate_edge(Mat& pre_src, Mat& now_src, Mat& dst, vector<Point>& pre_points, int width_miss, int height_miss, int flag)
{
	if (pre_src.cols != now_src.cols)
		return;
	if (pre_src.rows != now_src.rows)
		return;
	if (pre_src.cols != dst.cols)
		return;
	if (pre_src.rows != dst.rows)
		return;

	vector<Point> points2;

	Rect roi(width_miss, height_miss, pre_src.cols - width_miss, pre_src.rows - height_miss);//感兴趣的框（x,y,w,h)
	if (flag)
	{
		//此操作过于浪费时间,大约500ms
		Mat srcROI = Mat::zeros(pre_src.size(),CV_8UC1);
		srcROI(roi).setTo(255);
		node_func::get_GoodFeature_nodes(pre_src, srcROI, pre_points, 100);//重新寻找角点
		//node_func::get_GoodFeature_nodes(pre_src, Mat(), pre_points, 100);//重新寻找角点


		/*Mat test(pre_src, roi);
		node_func::get_Harris_nodes(test, pre_points, 160);*/

		return;
	}
	node_func::node_track2(pre_src, now_src, pre_points, points2);
	int xd = 0;
	int yd = 0;
	if (!get_xd_yd_pp(pre_points, points2, xd, yd))
	{
		//return;
		//do nothing;
	}
	if (debug)
	{
		//printf("(%d,%d)\n", xd, yd);
	}
	erode_ele_size = max(abs(xd) , abs(yd));
	int width = now_src.cols;
	int height = now_src.rows;
	Mat gray_pre(pre_src.size(), CV_8U, Scalar(0));
	Mat gray_now(now_src.size(), CV_8U, Scalar(0));
	cvtColor(pre_src, gray_pre, COLOR_BGR2GRAY);
	cvtColor(now_src, gray_now, COLOR_BGR2GRAY);

	//Time_Tick *tt = new Time_Tick("way1");
	//way 1
	/*************************************************/
	int swidth = dst.cols - width_miss;
	int sheight = dst.rows - height_miss;
	//此操作无太多时间浪费,平均0.5ms
	int a[8] = { 0 };
	int b[8] = { 0 };
	if (xd > 0)//右移
	{
		a[2] = width_miss + xd;
		a[3] = dst.cols;
		b[2] = width_miss;
		b[3] = dst.cols - xd;
		a[6] = width_miss; a[7] = dst.cols - xd;
		b[6] = width_miss; b[7] = dst.cols - xd;
	}
	else//左移
	{
		a[2] = width_miss;
		a[3] = dst.cols + xd;
		b[2] = width_miss - xd;
		b[3] = dst.cols;
		a[6] = width_miss - xd; a[7] = dst.cols;
		b[6] = width_miss - xd; b[7] = dst.cols;
	}
	if (yd > 0)//下移
	{
		a[0] = height_miss + yd;
		a[1] = dst.rows;
		b[0] = height_miss;
		b[1] = dst.rows - yd;
		a[4] = height_miss; a[5] = dst.rows - yd;
		b[4] = height_miss; b[5] = dst.rows - yd;
	}
	else//上移
	{
		a[0] = height_miss;
		a[1] = dst.rows + yd;
		b[0] = height_miss - yd;
		b[1] = dst.rows;
		a[4] = height_miss - yd; a[5] = dst.rows;
		b[4] = height_miss - yd; b[5] = dst.rows;
	}
	if (a[0] > height || a[2] > width || a[1] < 0 || a[3] < 0 || a[0] > a[1] || a[2] > a[3])
	{
		//错误判定
	}
	else
	{
		Mat dstROI(dst, Rect(width_miss, height_miss, swidth, sheight));
		Mat s1(height, width, CV_8UC1);
		Mat s2(height, width, CV_8UC1);
		//Range(左包含，右不包含)
		gray_now(Range(a[0], a[1]), Range(a[2], a[3])).copyTo(s1(Range(a[4], a[5]), Range(a[6], a[7])));
		gray_pre(Range(b[0], b[1]), Range(b[2], b[3])).copyTo(s2(Range(b[4], b[5]), Range(b[6], b[7])));
		absdiff(s1, s2, dstROI);
		threshold(dstROI, dstROI, THRESH, 0.0, THRESH_TOZERO);
		dstROI.copyTo(dst);
	}
	pre_points.swap(points2);
	/*************************************************/

	//way 2
	/*************************************************/
	//if (xd == 0 && yd == 0)//表示镜头未移动
	//{
	//	//Mat dstROI = Mat::zeros(dst.rows - height_miss, dst.cols - width_miss, CV_8UC1);
	//	//dstROI(roi).setTo(0);
	//	Mat dstROI(dst, Rect(width_miss, height_miss, dst.cols - width_miss, dst.rows - height_miss));
	//	//此操作无太多时间浪费,平均0.5ms
	//	Mat s1 = gray_now(Range(height_miss, dst.rows), Range(width_miss, dst.cols));
	//	Mat s2 = gray_pre(Range(height_miss, dst.rows), Range(width_miss, dst.cols));
	//	absdiff(s1, s2, dstROI);
	//}

	////下列操作平均20ms
	//for (int i = height_miss; i < height; i++)
	//{
	//	if (i - yd < 0 || i - yd >= height)
	//		continue;
	//	uchar* pData1 = gray_pre.ptr<uchar>(i - yd);
	//	uchar* pData2 = gray_now.ptr<uchar>(i);
	//	uchar* pData = dst.ptr<uchar>(i);
	//	for (int j = width_miss; j < width; j++)
	//	{
	//		if (j - xd < 0 || j - xd >= width)
	//			continue;
	//		int tmp = abs(pData1[j - xd] - pData2[j]);
	//		pData[j] = tmp < THRESH ? 0 : tmp;
	//	}
	//}
	//pre_points.swap(points2);
	/*************************************************/

	/****************************/
	//测试程序
	/*imshow("test", dst);
	cvWaitKey(1);*/
	/****************************/
	//delete tt;
}

double node_func::check_points(vector<Point>& a)
{
	if (a.size() < 10)
	{
		return 1.0;
	}
	int error_size = 0;
	vector<Point>::iterator it = a.begin();
	for (; it != a.end(); it++)
	{
		if (it->x < 0 || it->y < 0)
		{
			error_size++;
		}
	}
	return error_size * 1.0 / a.size();
}

int node_func::find_near_type(short** a, int i, int j, int width, int height, int pid)
{
	for (int m = i - pid; m < i + 1; m++)
	{
		if (m < 0)
			continue;
		for (int n = j - pid; n <= j + pid; n++)
		{
			if (n < 0 || n >= width)
				continue;
			if (m == i && n == j)
				break;
			if (a[m][n] > 0)
				return a[m][n];
		}
	}
	return -1;
}
//过滤掉孤立的噪点，并把集中的噪点统计，由矩形框框出
void node_func::self_filter(Mat& gray_src, map<int, Rect>& result)
{
	if (gray_src.channels() != 1)
		return;
	int height = gray_src.rows;
	int width = gray_src.cols;
	short** t = NULL;
	short** tmp = NULL;
	//Time_Tick* t2 = new Time_Tick("convert");
	new_short_2d_array(t, width, height);
	new_short_2d_array(tmp, width, height);
	convert_mat_intArray(gray_src, t);
	//delete t2;

	//添加自定义的过滤规则
	int class_num = 0;
	map<int, int> type_num;
	int return_thresh = 0;
	int return_thresh_num = 10000;
	//map<int, vector<Point>> type_points;//way2 需要
	//下列两行way 1 需要
	map<int, Point> type_lu;
	map<int, Point> type_rd;
	//下列操作在20ms左右
	//添加type_lu和type_rd检测以后。到20-150ms，镜头晃动时会达到450ms
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (t[i][j] == 0)
			{
				tmp[i][j] = 0;
			}
			else
			{
				int new_class = class_num + 1;
				int class_tmp = find_near_type(tmp, i, j, width, height, 2);
				if (class_tmp > 0)
				{
					new_class = class_tmp;
				}
				else
				{
					class_num = class_num + 1;
					new_class = class_num + 1;
					type_lu[new_class].x = j;
					type_lu[new_class].y = i;
					type_rd[new_class].x = j;
					type_rd[new_class].y = i;
				}
				tmp[i][j] = new_class;
				type_num[new_class]++;
				if (type_num[new_class] > 20)
					return_thresh++;
				if (return_thresh > return_thresh_num)
					return;
				//type_points[new_class].push_back(Point(j, i));//存的是坐标,way2 需要
				//way 1:
				if (type_lu[new_class].x > j)
					type_lu[new_class].x = j;
				if (type_lu[new_class].y > i)
					type_lu[new_class].y = i;
				if (type_rd[new_class].x < j)
					type_rd[new_class].x = j;
				if (type_rd[new_class].y < i)
					type_rd[new_class].y = i;
			}
		}
	}

	if (type_num.size() > 0 && type_num.size()<100)
	{
		/*******************************way 2*/
		/*map<int, vector<Point>>::iterator itt = type_points.begin();
		for (; itt != type_points.end(); itt++)
		{
			int type_i = itt->first;
			if (type_num[type_i] < 20)
				continue;
			vector<Point>::iterator itp = itt->second.begin();
			for (; itp != itt->second.end(); itp++)
			{
				int _x = itp->x;
				int _y = itp->y;
				if (type_lu[type_i].x == 0 && type_lu[type_i].y == 0)
				{
					type_lu[type_i].x = _x;
					type_lu[type_i].y = _y;
				}
				else
				{
					type_lu[type_i].x = type_lu[type_i].x < _x ? type_lu[type_i].x : _x;
					type_lu[type_i].y = type_lu[type_i].y < _y ? type_lu[type_i].y : _y;
				}
				if (type_rd[type_i].x == 0 && type_rd[type_i].y == 0)
				{
					type_rd[type_i].x = _x;
					type_rd[type_i].y = _y;
				}
				else
				{
					type_rd[type_i].x = type_rd[type_i].x > _x ? type_rd[type_i].x : _x;
					type_rd[type_i].y = type_rd[type_i].y > _y ? type_rd[type_i].y : _y;
				}
			}
		}*/
		//下列操作在60-100ms
		/*******************************way3*/
		//Time_Tick* tt = new Time_Tick("循环1");
		//map<int, Point> type_lu;
		//map<int, Point> type_rd;
		//for (int i = 0; i < height; i++)
		//{
		//	for (int j = 0; j < width; j++)
		//	{
		//		int type_i = tmp[i][j];
		//		if (type_i != 0)
		//		{
		//			int type_i_size = type_num[type_i];
		//			if (type_i_size < 10)//连续像素块小于20个则忽略
		//			{
		//				t[i][j] = 0;
		//			}
		//			else//连续像素块大于20个则先至为白色
		//			{
		//				t[i][j] = 255;
		//				//获取每个像素块儿的左上角type_lu及右下角type_rd的位置
		//				if (type_lu[type_i].x == 0 && type_lu[type_i].y == 0)
		//				{
		//					type_lu[type_i].x = j;
		//					type_lu[type_i].y = i;
		//				}
		//				else
		//				{
		//					type_lu[type_i].x = type_lu[type_i].x < j ? type_lu[type_i].x : j;
		//					type_lu[type_i].y = type_lu[type_i].y < i ? type_lu[type_i].y : i;
		//				}
		//				if (type_rd[type_i].x == 0 && type_rd[type_i].y == 0)
		//				{
		//					type_rd[type_i].x = j;
		//					type_rd[type_i].y = i;
		//				}
		//				else
		//				{
		//					type_rd[type_i].x = type_rd[type_i].x > j ? type_rd[type_i].x : j;
		//					type_rd[type_i].y = type_rd[type_i].y > i ? type_rd[type_i].y : i;
		//				}
		//			}
		//		}
		//	}
		//}
		//delete tt;
		//下列操作不超过1ms
		map<int, Point>::iterator it = type_lu.begin();
		for (; it != type_lu.end(); it++)
		{
			int type_i = it->first;
			if (type_num[type_i] < 20)
				continue;
			result[type_i].x = it->second.x;
			result[type_i].y = it->second.y;

			result[type_i].width = type_rd[type_i].x - it->second.x;
			result[type_i].height = type_rd[type_i].y - it->second.y;
		}
	}
	

	convert_intArray_mat(t, gray_src);
	delete_short_2d_array(tmp, width, height);
	delete_short_2d_array(t, width, height);
}

void node_func::contour_filter(Mat& gray_src, map<int, Rect>& result)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(gray_src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(contours[i]);
		if (contours[i].size() < 20)
			continue;
		if (rect.size.height <= 0 || rect.size.width <= 0)
			continue;
		Rect rect_i;
		rect_i.x = rect.center.x;
		rect_i.y = rect.center.y;
		rect_i.height = rect.size.height;
		rect_i.width = rect.size.width;

		result[i] = rect_i;
	}
	return;
}

void node_func::get_move(Mat& gray_src, map<int, Rect>& result)
{
	if (gray_src.channels() != 1)
		return;
	//二值化操作
	medianBlur(gray_src, gray_src, 5);//效果较好
	Mat Binary_dst = gray_src.clone();
	Mat tmp_dst = gray_src.clone();
	threshold(tmp_dst, Binary_dst, 30, 255.0, CV_THRESH_BINARY);

	//去噪
	//GaussianBlur(Binary_dst, dst, Size(5,5), 0, 0);//高斯滤波,效果一般
	//blur(Binary_dst, dst, Size(3, 3), Point(-1, -1));//不知道什么滤波,效果不好
	//medianBlur(Binary_dst, dst, 3);//效果较好
	//imshow("edge", dst);

	//膨胀或腐蚀
	int erode_type;
	int erode_ele_type = 0;
	//int erode_ele_size = 3;//定义成全局变量，根据抖动的情况自动修改腐蚀系数值
	if (erode_ele_size >= 3)
		return;
	erode_ele_size += 2;
	int dilate_ele_size = 3;
	switch (erode_ele_type)
	{
	case 0:
		erode_type = MORPH_RECT;
		break;
	case 1:
		erode_type = MORPH_CROSS;
		break;
	case 2:
		erode_type = MORPH_ELLIPSE;
		break;
	default:
		erode_type = MORPH_RECT;
		break;
	}
	Mat ele = getStructuringElement(erode_type, Size(2 * erode_ele_size + 1, 2 * erode_ele_size + 1),
		Point(erode_ele_size, erode_ele_size));
	Mat ele1 = getStructuringElement(erode_type, Size(2 * dilate_ele_size + 1, 2 * dilate_ele_size + 1),
		Point(dilate_ele_size, dilate_ele_size));
	//腐蚀
	erode(Binary_dst, tmp_dst, ele);
	//上面所有操作不超过10ms

	//imshow("gray_src", gray_src);
	//imshow("Binary_dst", Binary_dst);
	//cvWaitKey(1);

	//self_filter：90-150ms之间
	self_filter(gray_src, result); 
	//contour_filter(gray_src, result);
	//merge_move_obj操作一般不超过5ms,有时候会耗时20ms
	merge_move_obj(result);

	/*********************将矩形框放大后再合并一次***********************/
	map<int, Rect> rects;
	rects.swap(result);
	map<int, Rect>::iterator it = rects.begin();
	//cout << "Rect Size:" << rects.size() << endl;
	//按一定比例扩大矩形框
	for (; it != rects.end(); it++)
	{
		//过小的不进行扩充
		if (it->second.width <= MIN_WIDTH_LENGTH || it->second.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
			continue;
		Rect rect_i = it->second;
		float bit = 0.4;
		rect_i.x = rect_i.x - rect_i.width*bit >= 0 ? rect_i.x - rect_i.width*bit : 0;
		rect_i.y = rect_i.y - rect_i.height*bit >= 0 ? rect_i.y - rect_i.height*bit : 0;
		rect_i.width = rect_i.width*bit * 2 + rect_i.width + rect_i.x >= gray_src.cols ? gray_src.cols - rect_i.x - 1 : rect_i.width*bit * 2 + rect_i.width;
		rect_i.height = rect_i.height*bit * 2 + rect_i.height + rect_i.y >= gray_src.rows ? gray_src.rows - rect_i.y - 1 : rect_i.height*bit * 2 + rect_i.height;
		if (rect_i.width <= 0 || rect_i.height <= 0)
			continue;
		result[it->first] = rect_i;
	}
	//扩大以后再次合并
	merge_move_obj(result);
	/********************************************************************/

	//dilate(gray_src, tmp_dst, ele1);
}

void node_func::merge_move_obj(map<int, Rect>& result)
{
	int pid = 0;
	int size_tmp = result.size();
	while (result.size() > MAX_NUM_MOVE_OBJ)
	//for (int i = 0; i < 10 && result.size() > 0; i++)
	{
		size_tmp = result.size();
		map<int, Rect> tmp;
		map<int, int> type_state;
		result.swap(tmp);
		map<int, Rect>::iterator it = tmp.begin();
		for (; it != tmp.end(); it++)
		{
			map<int, Rect>::iterator jt = tmp.begin();
			if (type_state[it->first] > 0)
				continue;
			for (; jt != tmp.end(); jt++)
			{
				if (type_state[jt->first] > 0)
					continue;
				if (it->first == jt->first)//同一个则不合并
					continue;
				int tmp_x = it->second.x - jt->second.x;
				int tmp_y = it->second.y - jt->second.y;
				int tmp_width = 0;
				int tmp_height = 0;
				if (tmp_x < 0)
				{
					tmp_width = it->second.width;
				}
				else
				{
					tmp_width = jt->second.width;
				}
				if (tmp_y < 0)
				{
					tmp_height = it->second.height;
				}
				else
				{
					tmp_height = jt->second.height;
				}
				tmp_x = abs(tmp_x);
				tmp_y = abs(tmp_y);
				if (tmp_x <= tmp_width + pid && tmp_y <= tmp_height + pid )//有交叉的条件
				{
					Rect tmp_r;//合并后的新矩形
					if (it->second.x < jt->second.x)
					{
						tmp_r.x = it->second.x;
						tmp_r.width = jt->second.x - it->second.x + jt->second.width;
					}
					else
					{
						tmp_r.x = jt->second.x;
						tmp_r.width = it->second.x - jt->second.x + it->second.width;
					}
					if (it->second.y < jt->second.y)
					{
						tmp_r.y = it->second.y;
						tmp_r.height = jt->second.y - it->second.y + jt->second.height;
					}
					else
					{
						tmp_r.y = jt->second.y;
						tmp_r.height = it->second.y - jt->second.y + it->second.height;
					}
					result[it->first] = tmp_r;
					size_tmp++;

					type_state[it->first] = 1;
					type_state[jt->first] = 1;

					break;
				}
			}//jt  end
			if (type_state[it->first] <= 0)
			{
				type_state[it->first] = 1;
				Rect tmp_r = it->second;
				result[it->first] = tmp_r;
			}
		}
		tmp.clear();
		if (size_tmp > result.size() && result.size() != 0)
		{
			size_tmp = result.size();
		}
		else
		{
			pid += 5;
		}
	}
}

void node_func::convert_mat_intArray(Mat& src, short** a)
{
	int height = src.rows;
	int width = src.cols;
	for (int i = 0; i < height; i++)
	{
		uchar* pData = src.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
		{
			a[i][j] = pData[j];
		}
	}
}
void node_func::convert_intArray_mat(short** a, Mat& src)
{
	int height = src.rows;
	int width = src.cols;
	for (int i = 0; i < height; i++)
	{
		uchar* pData = src.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
		{
			pData[j] = a[i][j];
		}
	}
}
void node_func::new_short_2d_array(short**& a, int width, int height)
{
	a = new short*[height];
	for (int j = 0; j < height; j++)
	{
		a[j] = new short[width];
	}
}
void node_func::delete_short_2d_array(short**& a, int width, int height)
{
	for (int j = height - 1; j >= 0; j--)
	{
		delete[](a[j]);
	}
	delete[]a;
}