#pragma once

#include <caffe/caffe.hpp>

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
//#include <opencv2\xfeatures2d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#endif  // USE_OPENCV
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <time.h> 
#include <stdio.h>

#include "caffe/common.hpp"  
#include "caffe/layers/input_layer.hpp"  
#include "caffe/layers/inner_product_layer.hpp"  
#include "caffe/layers/dropout_layer.hpp"  
#include "caffe/layers/conv_layer.hpp"  
#include "caffe/layers/relu_layer.hpp"  

#include "caffe/layers/pooling_layer.hpp"  
#include "caffe/layers/lrn_layer.hpp"  
#include "caffe/layers/softmax_layer.hpp"  


namespace caffe
{

	extern INSTANTIATE_CLASS(InputLayer);
	extern INSTANTIATE_CLASS(InnerProductLayer);
	extern INSTANTIATE_CLASS(DropoutLayer);
	extern INSTANTIATE_CLASS(ConvolutionLayer);
	REGISTER_LAYER_CLASS(Convolution);
	extern INSTANTIATE_CLASS(ReLULayer);
	REGISTER_LAYER_CLASS(ReLU);
	extern INSTANTIATE_CLASS(PoolingLayer);
	REGISTER_LAYER_CLASS(Pooling);
	extern INSTANTIATE_CLASS(LRNLayer);
	REGISTER_LAYER_CLASS(LRN);
	extern INSTANTIATE_CLASS(SoftmaxLayer);
	REGISTER_LAYER_CLASS(Softmax);

}


#define RA 0.2
using namespace caffe;  // NOLINT(build/namespaces)
using std::string;

using namespace cv;
using namespace std;

typedef std::pair<string, float> Prediction;

class Classifier
{
public:
	Classifier(const string& model_file,
		const string& trained_file,
		const string& mean_file,
		const vector<string>& label_files);
	//static void onMouse(int event, int x, int y, int, void*);
	std::vector<vector<Prediction>> Classify(const cv::Mat& img, int N = 5);

private:
	void SetMean(const string& mean_file);

	std::vector<vector<float>> Predict(const cv::Mat& img);

	void WrapInputLayer(std::vector<cv::Mat>* input_channels);

	void Preprocess(const cv::Mat& img,
		std::vector<cv::Mat>* input_channels);

private:
	class boost::shared_ptr<Net<float>> net_;
	cv::Size input_geometry_;
	int num_channels_;
	cv::Mat mean_;
	std::vector<vector<string>> labels_; //multi
};
