#ifndef _LSTMRUN_HPP_
#define _LSTMRUN_HPP_

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// LSTM 版推理（input_dim=4, hidden_dim=8），接口与 grurun.hpp 完全一致
// 类名保持 RoadShapeClassifier，可无缝替换进 SoC 主程序
class RoadShapeClassifier {
public:
    static constexpr int kSeqLen = 64;
    static constexpr int kInputDim = 4;   // dx, dy, ddir_x, ddir_y
    static constexpr int kHiddenDim = 8;
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

    // 预处理：将 64 个点转换为 (64, 4) 特征
    void preprocess(const std::vector<cv::Point>& points, float feat[][kInputDim]);
};

#endif
