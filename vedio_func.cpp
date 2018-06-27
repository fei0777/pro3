#include "vedio_func.h"
#include "node_func.h"
#include "Time_Tick.h"


vedio_func::vedio_func()
{
	init();
}


vedio_func::~vedio_func()
{
}


void vedio_func::loadVedio(const char* filename)
{
	if (isLoad)
	{
		capture.release();
		frame_i = 0;
	}
	if (strlen(filename) <= 0)
		return;
	if (!capture.open(filename))
		return;

	isLoad = true;
	//file = filename;
	//Mat frame;
	fps = (int)capture.get(CV_CAP_PROP_FPS);
	frame_nums = (int)capture.get(CV_CAP_PROP_FRAME_COUNT);
}
void vedio_func::playVedio()
{
	Mat frame;
	Mat last_frame;
	vector<Point> points;
	//long int frame_i = 0;
	while (vedio_func::isLoad)
	{
		capture >> frame;
		frame_i++;
		if (frame.empty())
		{
			capture.release();
			return;
		}
		Mat frame1 = frame.clone();
		//if (frame_i % 30 == 1)
		if (frame_i  == 1)
		{
			//node_func::get_Harris_nodes(frame1, points, 165);//第三个参数为阈值
			node_func::get_GoodFeature_nodes(frame1, Mat(), points, 100);//第三个参数限制焦点数目
		}
		else
		{
			vector<Point> points2;
			vector<vector<int>> directions;
			node_func::node_track2(last_frame, frame, points, points2);
			points.swap(points2);
			points2.clear();
			//points = points2;
		}
		
		if (points.size() < 3000)
		{
			node_func::DrawPoint(frame1, points);
			cout << "right: " << points.size() << endl;
		}
		else
		{
			cout <<"error: "<< points.size() << endl;
		}
		
		imshow(file.c_str(), frame1);
		char c = cvWaitKey(1);
		if (c == 27)
			break;
		last_frame = frame.clone();
	}
}
void vedio_func::debug_playVedio()
{
	Mat frame;
	Mat last_frame;
	vector<Point> points;
	//long int frame_i = 0;
	vector<Point> pre_points;
	while (vedio_func::isLoad)
	{
		capture >> frame;
		if (frame.empty())
		{
			capture.release();
			return;
		}
		frame_i++;
		vector<Rect> result;
		Mat dst(frame.size(), CV_8U, Scalar(0));
		if (frame_i == 1)
		{
			last_frame = frame.clone();
		}
		if (frame_i % 60 == 1)
		{
			//flag == 1时为400ms
			node_func::calculate_edge(last_frame, frame, dst, pre_points, 0, 70, 1);//若此处flag设为0，则完全关闭镜头晃动检测
		}
		else
		{
			//flag == 0时为50ms
			node_func::calculate_edge(last_frame, frame, dst, pre_points, 0, 70, 0);
		}
		if (frame_i > 30000)
		{
			frame_i = 0;
		}
		last_frame = frame.clone();

		if (node_func::debug)
		{
			node_func::DrawPoint(frame, pre_points);
		}

		//对结果进行去燥，膨胀腐蚀等操作
		map<int, Rect> rects;
		//get_move:100-350ms之间，需要优化
		Time_Tick* tt = new Time_Tick("get_move");
		node_func::get_move(dst, rects);
		delete tt;

		//把矩形画在原帧上
		map<int, Rect>::iterator it = rects.begin();
		//cout << "Rect Size:" << rects.size() << endl;
		for (; it != rects.end(); it++)
		{
			if (it->second.width <= MIN_WIDTH_LENGTH || it->second.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
				continue;
			Rect rect_i = it->second;
			float bit = 0.4;
			rect_i.x = rect_i.x - rect_i.width*bit >= 0 ? rect_i.x - rect_i.width*bit : 0;
			rect_i.y = rect_i.y - rect_i.height*bit >= 0 ? rect_i.y - rect_i.height*bit : 0;
			rect_i.width = rect_i.width*bit * 2 + rect_i.width + rect_i.x >= frame.cols ? frame.cols - rect_i.x - 1 : rect_i.width*bit * 2 + rect_i.width;
			rect_i.height = rect_i.height*bit * 2 + rect_i.height + rect_i.y >= frame.rows ? frame.rows - rect_i.y - 1 : rect_i.height*bit * 2 + rect_i.height;
			if (rect_i.width <= 0 || rect_i.height <= 0)
				continue;
			rectangle(frame, rect_i, Scalar(0, 255, 0), 3, 1, 0);
		}
		imshow(file.c_str(), frame);
		char c = cvWaitKey(1);
		if (c == 27)
			break;
		imshow("edge", dst);
		c = cvWaitKey(1);
		if (c == 27)
			break;
	}
}
void vedio_func::playVedio(const char* filename)
{
	loadVedio(filename);
	if (node_func::debug)
	{
		debug_playVedio();
	}
	else
	{
		playVedio();
	}
}

void vedio_func::init()
{
	isLoad = false;
	if (capture.isOpened())
	{
		capture.release();
	}

	frame_i = 0;
	store_rects_index = -1;
	store_rects_num = 0;
	while (input_points.size() > 0)
	{
		input_points.pop();
	}
	while (output_mats.size() > 0)
	{
		output_mats.pop();
	}
	while (output_rects.size() > 0)
	{
		output_rects.pop();
	}
	/*****************中间暂时没用****************/
	frame_w_i = 0;
	frame_w_o = 0;
	frame_r_i = 0;
	frame_r_o = 0;
	/*********************************/
}
//此函数只接受视频中连续的帧，否则没有效果
void vedio_func::deal_mat(Mat& mat, vector<Rect>& rect_result)
{
	//现在有10ms左右的丢失，不知道损耗在哪里
	if (!isLoad)
	{
		if (mat.empty())
			return;
		frame_i++;
		vector<Rect> result;
		Mat dst(mat.size(), CV_8U, Scalar(0));
		if (frame_i == 1)
		{
			last_mat = mat.clone();
		}
		//calculate_edge:当flag=1时，last_mat_points不需要输入，直接作为输出
		//当flag=0时，last_mat_points需要输入，函数结束后，值更新为输出
		//Time_Tick* t1 = new Time_Tick("calculate_edge");	
		if (node_func::check_points(last_mat_points) > 0.2)
		{
			//flag == 1时为400ms
			node_func::calculate_edge(last_mat, mat, dst, last_mat_points, 0, 70, 1);//若此处flag设为0，则完全关闭镜头晃动检测
		}
		else
		{
			//flag == 0时为50ms
			node_func::calculate_edge(last_mat, mat, dst, last_mat_points, 0, 70, 0);
		}
		//delete t1;

		if (frame_i > 30000)
		{
			frame_i = 0;
		}
		last_mat = mat.clone();

		//Time_Tick* t3 = new Time_Tick("DrawPoint");
		if (node_func::debug)
		{
			//node_func::DrawPoint(mat, last_mat_points);
			//imshow("dst", dst);
			//cvWaitKey(1);
		}
		//delete t3;

		//对结果进行去燥，膨胀腐蚀等操作
		map<int, Rect> rects;
		//get_move:100-350ms之间，需要优化
		//Time_Tick* t2 = new Time_Tick("get_move");
		node_func::get_move(dst, rects);
		//delete t2;

		map<int, Rect>::iterator it = rects.begin();
		//cout << "Rect Size:" << rects.size() << endl;
		//按一定比例扩大矩形框
		for (; it != rects.end(); it++)
		{
			rect_result.push_back(it->second);
		}

		////把矩形画在原帧上
		//map<int, Rect>::iterator it = rects.begin();
		////cout << "Rect Size:" << rects.size() << endl;
		//for (; it != rects.end(); it++)
		//{
		//	if (it->second.width <= MIN_WIDTH_LENGTH || it->second.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
		//		continue;
		//	Rect rect_i = it->second;
		//	float bit = 0.4;
		//	rect_i.x = rect_i.x - rect_i.width*bit >= 0 ? rect_i.x - rect_i.width*bit : 0;
		//	rect_i.y = rect_i.y - rect_i.height*bit >= 0 ? rect_i.y - rect_i.height*bit : 0;
		//	rect_i.width = rect_i.width*bit * 2 + rect_i.width + rect_i.x >= mat.cols ? mat.cols - rect_i.x - 1 : rect_i.width*bit * 2 + rect_i.width;
		//	rect_i.height = rect_i.height*bit * 2 + rect_i.height + rect_i.y >= mat.rows ? mat.rows - rect_i.y - 1 : rect_i.height*bit * 2 + rect_i.height;
		//	if (rect_i.width <= 0 || rect_i.height <= 0)
		//		continue;
		//	rect_result.push_back(rect_i);
		//	//rectangle(mat, rect_i, Scalar(0, 255, 0), 3, 1, 0);
		//}
	}
}

//此函数如果需要使用，则必须一直使用，不能中间使用，否则无效
//参数解释:
//rect_result:当前帧的可疑区域提取
//rect_flag:当前帧的可疑区域提取以后的识别结果
//rect_pro:当前帧的可疑区域提取结果的所属概率
//new_old:表示当前矩形是新产生的(true)，还是前帧的矩形(false)
void vedio_func::calcu_pro_with_pre(Mat& mat, vector<Rect>& rect_result, vector<int>& rect_flag, vector<double>& rect_pro, vector<bool>& new_old)
{
	if (rect_result.size() != rect_flag.size() || rect_flag.size() != rect_pro.size())
		return;

	//int store_rects_index;
	//int store_rects_num;
	//vector<vector<Rect>> store_rects;
	//vector<vector<int>> store_flag;
	//vector<vector<double>> store_probability;*/
	int width = mat.cols;
	int height = mat.rows;
	vector<Rect> result_tmp;
	vector<int> flag_tmp;
	vector<double> pro_tmp;

	vector<bool> old_find;
	if (store_rects_index >= 0)
	{
		int index = store_rects_index;
		for (int j = 0; j < store_rects[index].size(); j++)
		{
			old_find.push_back(false);
		}
	}
	//result_tmp.swap(rect_result);
	//rect_result.clear();
	int length = rect_result.size();
	for (int i = 0; i < length; i++)
	{
		Rect ri = rect_result[i];
		int fi = rect_flag[i];
		double pi = rect_pro[i];

		bool isfind = false;
		if (store_rects_index >= 0)
		{
			int index = store_rects_index;
			for (int j = 0; j < store_rects[index].size(); j++)
			{
				Rect rj = store_rects[index][j];
				int fj = store_flag[index][j];
				double pj = store_probability[index][j];

				if (old_find[j] == true)
					continue;
				if (rj.x <= 0 || rj.y <= 0 || rj.x + rj.width + 1 >= width || rj.y + rj.height + 1 >= height)
				{
					old_find[j] = true;
					continue;
				}

				int new_width = 0;
				int new_height = 0;
				int tmp_x = ri.x - rj.x;
				int tmp_y = ri.y - rj.y;
				int tmp_width = 0;
				int tmp_height = 0;
				int pid = 0;
				if (tmp_x < 0)
				{
					tmp_width = ri.width;
					new_width = ri.width + tmp_x;
				}
				else
				{
					tmp_width = rj.width;
					new_width = ri.width - tmp_x;
				}
				if (tmp_y < 0)
				{
					tmp_height = ri.height;
					new_height = ri.height + tmp_y;
				}
				else
				{
					tmp_height = rj.height;
					new_height = ri.height - tmp_y;
				}
				//tmp_x = abs(tmp_x);
				//tmp_y = abs(tmp_y);
				if (new_width + pid > 0 && new_height + pid > 0)//有交叉的条件
				{
					isfind = true;

					int flag = 1;
					if (fi != fj)
						flag = -1;

					//新的完全包含旧的
					if (ri.x < rj.x && ri.width > rj.width + abs(tmp_x) && ri.y < rj.y && ri.height > ri.height + abs(tmp_y))
					{
						if (rj.width*rj.height*1.0 / (ri.width*ri.height) < 0.5)//如果新的太大，只用旧的
						{
							if (old_find[j] == true)
								continue;
							result_tmp.push_back(rj);
							flag_tmp.push_back(fi == fj ? fj : (pi > pj ? fi : fj));
							pro_tmp.push_back(pj * 0.7 + pi * 0.3 * flag);
							new_old.push_back(true);
						}
						else
						{
							if (ri.width <= MIN_WIDTH_LENGTH || ri.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
								continue;
							result_tmp.push_back(ri);
							flag_tmp.push_back(fi == fj ? fj : (pi > pj ? fi : fj));
							pro_tmp.push_back(pj * 0.7 + pi * 0.3 * flag);
							new_old.push_back(true);
						}
					}
					//旧的完全包含新的
					else if (ri.x > rj.x && ri.width + abs(tmp_x) < rj.width && ri.y > rj.y && ri.height + abs(tmp_y) < ri.height)
					{
						if (ri.width*ri.height*1.0 / (rj.width*rj.height) < 0.5)//如果新的太小，只用旧的
						{
							if (old_find[j] == true)
								continue;
							result_tmp.push_back(rj);
							flag_tmp.push_back(fi == fj ? fj : (pi > pj ? fi : fj));
							pro_tmp.push_back(pj * 0.7 + pi * 0.3 * flag);
							new_old.push_back(true);
						}
						else
						{
							if (ri.width <= MIN_WIDTH_LENGTH || ri.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
								continue;
							result_tmp.push_back(ri);
							flag_tmp.push_back(fi == fj ? fj : (pi > pj ? fi : fj));
							pro_tmp.push_back(pj * 0.7 + pi * 0.3 * flag);
							new_old.push_back(true);
						}
					}
					else
					{
						if (ri.width <= MIN_WIDTH_LENGTH || ri.height <= MIN_HEIGHT_LENGTH)//长宽过短的矩形删除
							continue;
						result_tmp.push_back(ri);
						flag_tmp.push_back(fi == fj ? fj : (pi > pj ? fi : fj));
						pro_tmp.push_back(pj * 0.7 + pi * 0.3 * flag);
						new_old.push_back(true);
						//else if((new_height * new_width) * 1.0 / (ri.width * ri.height) > 0.8)
					}
					old_find[j] = true;
				}//交叉判定结束
			}//内层循环结束
		}
		if (isfind == false)
		{
			new_old.push_back(true);
			result_tmp.push_back(ri);
			flag_tmp.push_back(fi);
			pro_tmp.push_back(pi);
		}
	}//外层循环结束
	if (store_rects_index >= 0)
	{
		int index = store_rects_index;
		for (int j = 0; j < store_rects[index].size(); j++)
		{
			if (old_find[j] == false)
			{
				Rect rj = store_rects[index][j];
				int fj = store_flag[index][j];
				double pj = store_probability[index][j];
				new_old.push_back(false);
				result_tmp.push_back(rj);
				flag_tmp.push_back(fj);
				pro_tmp.push_back(pj);
			}
		}
	}
	if (store_rects_index >= 0)
	{
		result_tmp.swap(rect_result);
		flag_tmp.swap(rect_flag);
		pro_tmp.swap(rect_pro);
	}
	store_rects_index = (store_rects_index + 1) % STORE_RECTS_NUM;
	if (store_rects_num < STORE_RECTS_NUM)
	{
		store_rects.push_back(rect_result);
		store_flag.push_back(rect_flag);
		store_probability.push_back(rect_pro);
		/*store_rects[store_rects_index].swap(rect_result);
		store_flag[store_rects_index].swap(rect_flag);
		store_probability[store_rects_index].swap(rect_pro);*/
		store_rects_num++;
	}
	else
	{
		store_rects[store_rects_index] = rect_result;
		store_flag[store_rects_index] = rect_flag;
		store_probability[store_rects_index] = rect_pro;
	}
}

/*****************中间暂时没用****************/
void vedio_func::input_frame_w(Mat frame, int _frame_i)
{
	while (!frame_w_lock.try_lock())
	{

	}
	frame_w_i++;
	input_mats.push(frame);

	frame_w_lock.unlock();
}
//flag==1表示，不从队列中删除
void vedio_func::output_frame_w(Mat& frame, int& _frame_o, int flag)
{
	if (flag == 1)
	{
		_frame_o = frame_w_o + 1;
		frame = input_mats.front();
		return;
	}
	while (!frame_w_lock.try_lock())
	{

	}
	frame_w_o++;
	frame = input_mats.front();
	_frame_o = frame_w_o;
	input_mats.pop();

	frame_w_lock.unlock();
}

void vedio_func::input_frame_r(Mat frame, int _frame_i)
{
	while (!frame_r_lock.try_lock())
	{

	}
	frame_r_i++;
	output_mats.push(frame);

	frame_r_lock.unlock();
}
void vedio_func::output_frame_r(Mat& frame, int& _frame_o)
{
	while (!frame_r_lock.try_lock())
	{

	}
	frame_r_o++;
	frame = output_mats.front();
	_frame_o = frame_w_o;
	output_mats.pop();

	frame_r_lock.unlock();
}


bool vedio_func::get_no_deal_frame(Mat& pre_frame, vector<Point>& pre_points, Mat& now_frame, vector<Point>& now_points)
{
	int _pre_index;
	int _now_index;
	output_frame_w(pre_frame, _pre_index);
	output_frame_w(now_frame, _now_index, 1);

	Mat dst(now_frame.size(), CV_8U, Scalar(0));
	if (_pre_index % 60 == 1)
	{
		//把边缘结果存入dst, 前一帧的角点存入pre_points
		node_func::calculate_edge(pre_frame, now_frame, dst, pre_points, 0, 70, 1);//若此处flag设为0，则完全关闭镜头晃动检测
	}
	else
	{
		node_func::calculate_edge(pre_frame, now_frame, dst, pre_points, 0, 70, 0);
	}
	now_points.swap(pre_points);
	//对结果进行去燥，膨胀腐蚀等操作
	map<int, Rect> rects;
	//get_move:100-350ms之间，需要优化
	Time_Tick* tt = new Time_Tick("get_move");
	node_func::get_move(dst, rects);
	delete tt;

	return true;
}
/*********************************/

