#ifndef __IMGPROCESS_NEW_HPP__
#define __IMGPROCESS_NEW_HPP__

#include "zf_common_headfile.hpp"
#include "byd_math.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
using i64 = long long;



class imgprocess{

private:
std::set<int> has_enter;//at(x).at(y)
static std::vector<std::vector<float>> H;


static std::vector<cv::Point> linyu;

unsigned short *screen_base;

class balinyu_point{
private:
public:
cv::Point2f coord;
float dir;//统一：IPM前：0:6点钟 2:9点钟 4:12点钟 6:3点钟；IPM后：逆时针增长的不写pi的弧度制，十二点钟为0
float change = 0;
bool useful = 0;
};
struct rnmb{
    float x;
    int index;
};


cv::Mat gray;
cv::Mat rgb;
cv::Mat bin;

int status;
/*0:正常 -1:出界 -2:斑马线
* 1:十字(下两点可见)
* 2:十字(下两点不可见)
*/
i64 licheng = 0;
int if_huandao = 0;
int bottom_mid_x;

class line_enum{
public:
std::vector<balinyu_point> lineenum;
std::vector<std::pair<cv::Point,i64>> DP_point;//左点右index
std::vector<bool> DP_Point_useful;
std::vector<std::pair<cv::Point,i64>> Pruning_point;
std::vector<std::pair<float,bool>> branch;//左为lstart+rstart/2到边中点的方向，右为是否启用
// std::vector<int> Pruning_Point_index;
std::vector<cv::RotatedRect> Ellipses;
std::vector<float> qzh;
balinyu_point at(std::size_t __n);
void clear();
void push_back(const imgprocess::balinyu_point &__x);
std::size_t size();
float sum(std::size_t from,std::size_t to);
float avg(std::size_t from,std::size_t to);
float cos(std::size_t A,std::size_t B,std::size_t C);
void IPM();

void approxPolyDP();
void RHT_Ellipse();
void reflashDP();
void drawDP();
void drawPP();
void branchfilter(cv::Point mdp);//把深度<10的分支删掉

};


line_enum l_line_enum;
line_enum r_line_enum;
line_enum all_line_enum;





std::vector<rnmb> firstlinel;
std::vector<rnmb> firstliner;

int zcbltrue = 159;
int zcblmin = 239;
void longest();



int bottom_mid();
/*左：坐标，右：0为左1为右
*/

cv::Point midpoint;



cv::Point balinyu_start_l(int input);
cv::Point balinyu_start_r(int input);
uchar balinyu_linyu(cv::Point coord_input,int dir);
cv::Point balinyu_next_point(cv::Point coord_input,int dir);
cv::Point balinyu_suanzi(cv::Point coord_input,bool l_r,bool if_draw = 1);
void balinyu(cv::Point l_start,cv::Point r_start);
void firstline_l(bool if_draw = 1);
void firstline_r(bool if_draw = 1);
int endy;
bool if_time = 0;























int nowtype = 0;
bool if_cross = 1;
int lhd = 0;//0默认 1前拐点 2 上拐点
int rhd = 0;
i64 ban_hd_time = 0;
i64 hd_enter_time = 0;
i64 max_hd_length = 100000;
std::pair<cv::Point,cv::Point> lline = {cv::Point(0,239),cv::Point(10,0)},rline = lline = {cv::Point(319,239),cv::Point(309,0)};




void filter();
void typesort();
void pruning();
void switchbranch(int type = 0);//-1选最左，0选最上，1选最右
void pushbackline(std::vector<cv::Point>& line, cv::Point start, cv::Point end);
void splitline(int type = 0);
void fnlmidline(int type = 0);

int grurun();


void cross_circle(bool if_draw = 1);
int buxian();
void cross1();
void cross2();
bool huandaol();
bool huandaor();
int out_of_runway();
int banmaxian();
void cross114();
public:




// std::vector<float> finallinel;
// std::vector<float> finalliner;
std::vector<cv::Point> finalline;
std::vector<cv::Point> finallinel;
std::vector<cv::Point> finalliner;
std::vector<cv::Point> finalmidline;
std::vector<int> midline;
std::vector<cv::Point2f> midline_point;
imgprocess();
~imgprocess();
int run(cv::Mat rgb_input,cv::Mat gray_input);
    
};



#endif