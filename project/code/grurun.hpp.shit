#ifndef _GRURUN_HPP_
#define _GRURUN_HPP_

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

class RoadShapeClassifier {
public:
    static constexpr int kSeqLen = 64;
    static constexpr int kInputDim = 3;   // 仅使用 dx, dy, ddir_x
    static constexpr int kHiddenDim = 8;  // 隐藏层维度降为 8
    static constexpr int kNumClasses = 6;

    const char* labels[6] = {
        "zhixian", "shizi", "daodazuohuandao",
        "jinruzuohuandao", "daodayouhuandao", "jinruyouhuandao"
    };

    RoadShapeClassifier();
    ~RoadShapeClassifier();

    bool init();

    std::vector<float> run(const std::vector<cv::Point>& points);

private:
    static inline float sigmoid(float x);
    static inline float tanh(float x);

    // 预处理：将 64 个点转换为 (64, 3) 特征
    void preprocess(const std::vector<cv::Point>& points, float feat[][kInputDim]);
};

#endif