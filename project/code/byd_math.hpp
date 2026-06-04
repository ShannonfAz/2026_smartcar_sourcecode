#ifndef __BYD_MATH_HPP__
#define __BYD_MATH_HPP__
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
namespace byd_math{
float cos(cv::Point2f A,cv::Point2f B,cv::Point2f C);
float angle(cv::Point2f A,cv::Point2f B);
float AOBangle(const cv::Point2f& A, const cv::Point2f& B);
float AB_to_C(cv::Point2f A,cv::Point2f B,cv::Point2f C);  
float k(cv::Point2f A,cv::Point2f B);
float b(cv::Point2f A,float k);
float buxian(float y,float k,float b);
cv::Point IPMPoint(std::vector<std::vector<float>> H,cv::Point input);
std::vector<int> json_output_place(const std::vector<int>& a);




class kdtree{
private:
    int MAXN = 200010;
    // struct node {
    //     double x, y;
    // };
    void maintain(int x);
public:
    std::vector<int> d, lc, rc;
    std::vector<double> L, R, D, U;
    std::vector<cv::Point> s;
    kdtree(int n);
    double ans = 2e18;
    double dist(int a, int b);
    double f(int a, int b);
    double f(const cv::Point& coord, int box_idx) const;
    void nearest_rec(const cv::Point& target, int l, int r,int& best_idx, double& best_dist) const;
    int build(int l, int r);
    std::size_t nearest_index(const cv::Point& coord) const;
};
};

#endif