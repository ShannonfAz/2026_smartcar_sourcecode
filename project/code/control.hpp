#ifndef __control_hpp__
#define __control_hpp__
#include "zf_common_headfile.hpp"
#include "byd_math.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

class gyropid {
private:
    // PID 系数
    float kp;
    float ki;
    float kd;

    // 状态变量
    float integral;      // 积分累加项
    float prev_error;    // 上一次误差，用于微分计算

    // 输出与积分限幅
    float out_min;       // 输出下限
    float out_max;       // 输出上限
    float integral_limit;// 积分项限制（抗积分饱和）

public:
    // 构造函数，可带参数设置 PID 系数和限幅
    gyropid(float p = 1.0f, float i = 10.0f, float d = 5.0f,
        float min_out = 0.0f, float max_out = 2000.0f,
        float int_limit = 100.0f);

    // 执行一次 PID 计算并输出 PWM
    // 参数 target_speed：目标转速
    int16 run(float target_gyro,int gyroz);

    // 重置积分器和微分前次误差（例如启动或切换模式时调用）
    void reset();
};






// class pid {
// private:
//     // PID 系数
//     float kp;
//     float ki;
//     float kd;

//     // 状态变量
//     float integral;      // 积分累加项
//     float prev_error;    // 上一次误差，用于微分计算

//     // 输出与积分限幅
//     float out_min;       // 输出下限
//     float out_max;       // 输出上限
//     float integral_limit;// 积分项限制（抗积分饱和）

// public:
//     // 构造函数，可带参数设置 PID 系数和限幅
//     pid(float p = 1.0f, float i = 10.0f, float d = 5.0f,
//         float min_out = 0.0f, float max_out = 2000.0f,
//         float int_limit = 100.0f);

//     // 执行一次 PID 计算并输出 PWM
//     // 参数 target_speed：目标转速
//     uint16 run(float target_speed,int encoder_speed);

//     // 重置积分器和微分前次误差（例如启动或切换模式时调用）
//     void reset();
// };

class control{
    private:
    //pid pidl,pidr;
    public:
    std::string show;
    float maxl,maxr;
    float lsp = 1000,rsp = 1000;
    float ld = 0,rd = 0;
    float elsl = 0,ersl = 0;
    control();
    void run(float l_speed,float r_speed);
};



#endif