#ifndef _GRURUN_HPP_
#define _GRURUN_HPP_
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // for cv::cvtColor
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#include <opencv2/dnn.hpp>
#include <iostream> // for std::cerr
#include <fstream>  // for std::ofstream
#include <thread>
#include <chrono>
#include <atomic>
class RoadShapeClassifier {
public:
    static constexpr int kSeqLen = 64;
    static constexpr int kInputDim = 5;
    static constexpr int kHiddenDim = 16;
    static constexpr int kNumClasses = 6;

    // 类别名称（可根据需要保留）
    const char* labels[6] = {
        "zhixian", "shizi", "daodazuohuandao",
        "jinruzuohuandao", "daodayouhuandao", "jinruyouhuandao"
    };

    RoadShapeClassifier();
    ~RoadShapeClassifier();

    bool init();   // 手写推理无需初始化，但保留接口兼容性

    std::vector<float> run(const std::vector<cv::Point>& points);

private:
    // 快速激活函数
    static inline float sigmoid(float x);
    static inline float tanh(float x);

    // 预处理：将 64 个点转换为模型输入特征
    void preprocess(const std::vector<cv::Point>& points, float feat[][kInputDim]);
};
#endif