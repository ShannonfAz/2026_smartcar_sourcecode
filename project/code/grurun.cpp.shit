#include "grurun.hpp"
#include "gru_weights.h"   // 由 export_weights.py 生成（input_dim=3, hidden_dim=8）
#include <cmath>

// ======================== 预处理统计量 ========================
// 从新训练的 feat_stats.npz 中提取（维度：3）
// 请用 Python 读取后替换下面的值：
//     import numpy as np
//     data = np.load('feat_stats.npz')
//     print(data['mean'], data['std'])
static const float FEAT_MEAN[5] = { 1.26657093f, 0.00101598f, 0.00324430f };
static const float FEAT_STD[5]  = { 6.82215166f, 9.76642132f, 0.32148936f };

// ======================== 激活函数 ========================
inline float RoadShapeClassifier::sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}
inline float RoadShapeClassifier::tanh(float x) {
    return std::tanh(x);
}

RoadShapeClassifier::RoadShapeClassifier()  = default;
RoadShapeClassifier::~RoadShapeClassifier() = default;

bool RoadShapeClassifier::init() {
    return true;
}

// ======================== 推理主函数 ========================
std::vector<float> RoadShapeClassifier::run(const std::vector<cv::Point>& points) {
    if (points.size() != kSeqLen) return {};

    float features[kSeqLen][kInputDim];
    preprocess(points, features);

    // ---------- 1. GRU 前向传播 ----------
    float h[kHiddenDim] = {0};
    float all_h[kSeqLen][kHiddenDim];

    for (int t = 0; t < kSeqLen; ++t) {
        float gate_x[3 * kHiddenDim];
        float gate_h[3 * kHiddenDim];

        // W_ih * x_t + b_ih
        for (int i = 0; i < 3 * kHiddenDim; ++i) {
            float sum = b_ih[i];
            for (int j = 0; j < kInputDim; ++j) {
                sum += features[t][j] * W_ih[i * kInputDim + j];
            }
            gate_x[i] = sum;
        }

        // W_hh * h + b_hh
        for (int i = 0; i < 3 * kHiddenDim; ++i) {
            float sum = b_hh[i];
            for (int j = 0; j < kHiddenDim; ++j) {
                sum += h[j] * W_hh[i * kHiddenDim + j];
            }
            gate_h[i] = sum;
        }

        // 门控运算
        for (int i = 0; i < kHiddenDim; ++i) {
            float r = sigmoid(gate_x[i] + gate_h[i]);
            float z = sigmoid(gate_x[i + kHiddenDim] + gate_h[i + kHiddenDim]);
            float n = tanh(gate_x[i + 2 * kHiddenDim] + r * gate_h[i + 2 * kHiddenDim]);
            h[i] = (1.0f - z) * n + z * h[i];
        }

        // 保存当前时间步输出（用于平均池化）
        for (int i = 0; i < kHiddenDim; ++i) {
            all_h[t][i] = h[i];
        }
    }

    // ---------- 2. 平均池化 ----------
    float feat[kHiddenDim] = {0};
    for (int i = 0; i < kHiddenDim; ++i) {
        for (int t = 0; t < kSeqLen; ++t) {
            feat[i] += all_h[t][i];
        }
        feat[i] /= kSeqLen;
    }

    // ---------- 3. LayerNorm ----------
    float mean = 0.0f, var = 0.0f;
    for (int i = 0; i < kHiddenDim; ++i) mean += feat[i];
    mean /= kHiddenDim;
    for (int i = 0; i < kHiddenDim; ++i) {
        float diff = feat[i] - mean;
        var += diff * diff;
    }
    var = var / kHiddenDim + 1e-5f;
    float inv_std = 1.0f / std::sqrt(var);
    for (int i = 0; i < kHiddenDim; ++i) {
        feat[i] = (feat[i] - mean) * inv_std;
        feat[i] = feat[i] * ln_gamma[i] + ln_beta[i];
    }

    // ---------- 4. 全连接层 ----------
    float logits[kNumClasses] = {0};
    for (int i = 0; i < kNumClasses; ++i) {
        for (int j = 0; j < kHiddenDim; ++j) {
            logits[i] += feat[j] * fc_weight[i * kHiddenDim + j];
        }
        logits[i] += fc_bias[i];
    }

    // ---------- 5. Softmax ----------
    float max_logit = logits[0];
    for (int i = 1; i < kNumClasses; ++i) {
        if (logits[i] > max_logit) max_logit = logits[i];
    }
    float sum = 0.0f;
    for (int i = 0; i < kNumClasses; ++i) {
        logits[i] = std::exp(logits[i] - max_logit);
        sum += logits[i];
    }
    std::vector<float> probs(kNumClasses);
    for (int i = 0; i < kNumClasses; ++i) {
        probs[i] = logits[i] / sum;
    }
    return probs;
}

// ======================== 预处理 ========================
void RoadShapeClassifier::preprocess(const std::vector<cv::Point>& points,
                                     float feat[][kInputDim]) {
    // 1. 位移 (dx, dy)
    float diff[kSeqLen][2];
    for (int i = 0; i < kSeqLen - 1; ++i) {
        diff[i][0] = static_cast<float>(points[i+1].x - points[i].x);
        diff[i][1] = static_cast<float>(points[i+1].y - points[i].y);
    }
    diff[kSeqLen-1][0] = diff[kSeqLen-2][0];
    diff[kSeqLen-1][1] = diff[kSeqLen-2][1];

    // 2. 步长和方向向量
    float norm[kSeqLen];
    float dir_vec[kSeqLen][2];
    for (int i = 0; i < kSeqLen; ++i) {
        float len = std::sqrt(diff[i][0]*diff[i][0] + diff[i][1]*diff[i][1]) + 1e-8f;
        norm[i] = len;
        dir_vec[i][0] = diff[i][0] / len;
        dir_vec[i][1] = diff[i][1] / len;
    }

    // 3. 方向变化 (ddir_x)，只取 x 分量
    float ddir_x[kSeqLen];
    for (int i = 0; i < kSeqLen - 1; ++i) {
        ddir_x[i] = dir_vec[i+1][0] - dir_vec[i][0];
    }
    ddir_x[kSeqLen-1] = ddir_x[kSeqLen-2];

    // 4. Z-score 归一化并填入特征（仅 3 维：dx, dy, ddir_x）
    for (int i = 0; i < kSeqLen; ++i) {
        feat[i][0] = (diff[i][0] - FEAT_MEAN[0]) / FEAT_STD[0];
        feat[i][1] = (diff[i][1] - FEAT_MEAN[1]) / FEAT_STD[1];
        feat[i][2] = (ddir_x[i] - FEAT_MEAN[2]) / FEAT_STD[2];
    }
}