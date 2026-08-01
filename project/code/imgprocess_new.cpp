#include "imgprocess_new.hpp"
/*输入待处理图片，一张灰一张彩
*/


#define TIME_IT_NS(code)                                                \
    do {                                                                \
        auto _start = std::chrono::high_resolution_clock::now();        \
        code;                                                           \
        auto _end   = std::chrono::high_resolution_clock::now();        \
        auto _dur   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count(); \
        std::cout << "time: " << _dur << " ns" << std::endl;            \
    } while(0)



imgprocess::imgprocess(){
    screen_base = ips200.returnscreenbase();
    std::cout << "图像处理初始化完成" << std::endl;

}
imgprocess::~imgprocess(){

}


//在这里写单应性矩阵
std::vector<std::vector<float>> imgprocess::H = {
    {1.203540334247222, 1.819732982951531, -32.03321046330151},
    {0, 3.722308642201908, 0.7006166549438853},
    {0, 0.01140143137106444, 1.002145988814093}
};

/*(-1,-1)( 0,-1)( 1,-1)
* (-1, 0)( 0, 0)( 1, 0)
* (-1, 1)( 0, 1)( 1, 1)
*/
std::vector<cv::Point> imgprocess::linyu = {
    cv::Point(0,1),
    cv::Point(-1,1),
    cv::Point(-1,0),
    cv::Point(-1,-1),
    cv::Point(0,-1),
    cv::Point(1,-1),
    cv::Point(1,0),
    cv::Point(1,1)
};

imgprocess::balinyu_point imgprocess::line_enum::at(std::size_t __n){
    return imgprocess::line_enum::lineenum.at(__n);
}
void imgprocess::line_enum::clear(){
    lineenum.clear();
    DP_point.clear();
}
void imgprocess::line_enum::push_back(const imgprocess::balinyu_point &__x){
    lineenum.push_back(__x);
    if(!qzh.empty()){
        qzh.push_back(qzh.back() + lineenum.back().dir);
    }else{
        qzh.push_back(lineenum.back().dir);
    }
}
std::size_t imgprocess::line_enum::size(){
    return lineenum.size();
}
float imgprocess::line_enum::sum(std::size_t from,std::size_t to){
    if(to >= qzh.size()){
        std::cerr << "操你妈,溢出了" << std::endl << "详细信息:输入:" << from << "," << to << std::endl;
        return -1;
    }
    if(from == 0){
        return qzh.at(to);
    }else{
        return qzh.at(to) - qzh.at(from-1);
    }
}
float imgprocess::line_enum::avg(std::size_t from,std::size_t to){
    return (float)sum(from,to)/(float)(to-from+1.0);
}
float imgprocess::line_enum::cos(std::size_t A,std::size_t B,std::size_t C){
    return byd_math::cos(lineenum.at(A).coord,lineenum.at(B).coord,lineenum.at(C).coord);
}
void imgprocess::line_enum::IPM(){
    cv::Point2f temp,tempnext;
    for(auto& rnm:lineenum){
        temp.x = (rnm.coord.x * H.at(0).at(0) + rnm.coord.y * H.at(0).at(1) + H.at(0).at(2))
        /(rnm.coord.x * H.at(2).at(0) + rnm.coord.y * H.at(2).at(1) + H.at(2).at(2));
        temp.y = (rnm.coord.x * H.at(1).at(0) + rnm.coord.y * H.at(1).at(1) + H.at(1).at(2))
        /(rnm.coord.x * H.at(2).at(0) + rnm.coord.y * H.at(2).at(1) + H.at(2).at(2));
        tempnext = rnm.coord + (cv::Point2f)linyu.at(rnm.dir);
        tempnext.x = (tempnext.x * H.at(0).at(0) + tempnext.y * H.at(0).at(1) + H.at(0).at(2))
        /(tempnext.x * H.at(2).at(0) + tempnext.y * H.at(2).at(1) + H.at(2).at(2));
        tempnext.y = (tempnext.x * H.at(1).at(0) + tempnext.y * H.at(1).at(1) + H.at(1).at(2))
        /(tempnext.x * H.at(2).at(0) + tempnext.y * H.at(2).at(1) + H.at(2).at(2));
        rnm.dir = byd_math::angle(temp,tempnext);
        rnm.coord = temp;
    }
    for(std::size_t i = 0;i < lineenum.size();i++){
        if(i){
            qzh.at(i) = qzh.at(i-1);
            qzh.at(i) += lineenum.at(i).dir;
            lineenum.at(i).change = lineenum.at(i).dir - lineenum.at(i-1).dir;
        }else{
            qzh.at(0) = lineenum.at(0).dir;
            lineenum.at(0).change = 0;
        }
    }
}








int imgprocess::run(cv::Mat rgb_input,cv::Mat gray_input){
    finalline.clear();
    finallinel.clear();
    finalliner.clear();
    finalmidline.clear();
    // finallinel.assign(240,-1);
    // finalliner.assign(240,320);
    midline.assign(240,159);
    midline_point.clear();
    status = 0;
    l_line_enum.clear();
    r_line_enum.clear();
    all_line_enum.clear();
    firstlinel.assign(240,{-1,0});
    firstliner.assign(240,{320,0});
    rgb = rgb_input;
    gray = gray_input;
    cv::Mat show,bin1;
    //图像处理
    // std::cout << "hb";
    // TIME_IT_NS();
    cv::threshold(gray,bin,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    // cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::erode(bin, bin, kernel);
	//cv::dilate(bin1, bin, kernel1);
    // bin = bin1;
    cv::rectangle(bin,cv::Rect(cv::Point(0,0),cv::Point(UVC_WIDTH-1,UVC_HEIGHT-1)),cv::Scalar(0), 2, 8, 0);
    cv::line(bin,cv::Point(159,0),cv::Point(0,20),0,3,8,0);
    cv::line(bin,cv::Point(160,0),cv::Point(319,20),0,3,8,0);
    cv::resize(bin,show,cv::Size(240,180));
    cv::cvtColor(show,show,cv::COLOR_GRAY2BGR565);
    // ips200.displayimage_rgb565(reinterpret_cast<uint16*>(show.ptr(0)), 240, 180);
    //ips200.clear();
    //出线，斑马线
    // out_of_runway();
    // status = out_of_runway();
    // status = banmaxian();

    // if(status < 0)return -1;

    longest();

    bottom_mid_x = zcbltrue;



    // bottom_mid_x = bottom_mid();
    if(bottom_mid_x == 0)return 0;
    has_enter.clear();
    cv::Point bsl = balinyu_start_l(bottom_mid_x),bsr = balinyu_start_r(bottom_mid_x);
    midpoint = (bsl + bsr)/2;

    balinyu(bsl,bsr);
    // std::cout << 1 << std::endl;
    if(all_line_enum.size() < 30)return -2;
    all_line_enum.IPM();
    midpoint = (all_line_enum.lineenum.at(0).coord+all_line_enum.lineenum.back().coord)/2;
    std::vector<balinyu_point> templineenum;
    templineenum.push_back(all_line_enum.at(0));
    for(std::size_t i = 1;i < all_line_enum.size()-1;i++){
        cv::Point temppoint = all_line_enum.at(i-1).coord+all_line_enum.at(i).coord+all_line_enum.at(i+1).coord;
        temppoint.x /= 3;
        temppoint.y /= 3;
        templineenum.push_back({temppoint,all_line_enum.at(i).dir,all_line_enum.at(i).change,all_line_enum.at(i).useful});
    }
    templineenum.push_back(all_line_enum.lineenum.back());
    all_line_enum.lineenum = templineenum;


    // for(auto rnm:all_line_enum.lineenum){
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.coord.x*3/4)),std::min(239,std::max(0,(int)rnm.coord.y*3/4)),RGB565_BLUE);
    // }
    // std::cout << 2 << std::endl;
    all_line_enum.approxPolyDP();
    // std::cout << 3 << std::endl;
    if(all_line_enum.DP_point.size() < 4)return -2;
    // std::cout << 4 << std::endl;

    // std::cout << 5 << std::endl;
    filter();
    // std::cout << 6 << std::endl;
    // int type = lstmrun();
    // ips200.show_string(0,180,"                             ");
    // ips200.show_string(0,180,classifier.labels[type]);
    if(key_3.get_level() == 0){
        system_delay_ms(100);
        if(key_3.get_level() == 0){

            using json = nlohmann::json;
            json root;
            std::ifstream ifs("/home/root/byd_2026_8_1.json");
            if (ifs.is_open()) {
                try {
                    ifs >> root;
                } catch (...) {
                    root = json::array();  // 解析失败则覆盖为空数组
                }
                ifs.close();
            } else {
                root = json::array();      // 文件不存在则新建空数组
            }
            json points;
            std::vector<int> a,b;
            for(auto rnm:all_line_enum.DP_point){
                a.push_back(rnm.second);
            }
            b = byd_math::json_output_place(a);
            // for(auto rnm:a)std::cout << rnm << " ";
            // std::cout << std::endl;
            // for(auto rnm:b)std::cout << rnm << " ";
            // std::cout << std::endl;
            for(auto rnm:b){
                points.push_back({std::lround(all_line_enum.lineenum.at(rnm).coord.x),std::lround(all_line_enum.lineenum.at(rnm).coord.y)});
            }
            json sample = {
                {"point",points},
                {"label","zhixian"}
            };
            /*
            lable:
            simple
            cross
            circle
            */
            /*
            lable:
            zhixian
            shizi
            daodazuohuandao//左环岛初见部分，拐点在左下方
            jinruzuohuandao//左环岛前方部分，拐点在左上方
            likaizuohuandao//左环岛离开部分，拐点在右下方
            daodayouhuandao
            jinruyouhuandao
            likaiyouhuandao
            */
            root.push_back(sample);
            std::ofstream ofs("/home/root/byd_2026_7_31.json");
            ofs << root.dump(4);
            ofs.close();
        }
    }
    // std::cout << std::endl;
    // for(auto rnm:all_line_enum.DP_point){
    //     std::cout << rnm.first << std::endl;
    // }
    //all_line_enum.drawDP();


    lstmrun();
    typesort();

    // ips200.show_int(0,196,lhd,3);
    // ips200.show_int(50,196,rhd,3);
    // ips200.show_int(0,212,nowtype,3);
    // ips200.show_int(0,228,0,3);
    // nowtype = 0;
    // all_line_enum.reflashDP();
    // all_line_enum.drawDP();
    if(all_line_enum.DP_point.size() < 4)return -2;
    pruning();
    // ips200.show_int(0,228,1,3);
    // all_line_enum.drawPP();
    if(all_line_enum.Pruning_point.size() < 2)return -2;
    all_line_enum.branchfilter(midpoint);
    // std::cout << std::endl;
    // for(auto rnm:all_line_enum.branch){
    //     std::cout << rnm.first << "," << rnm.second << std::endl;
    // }
    // ips200.show_int(0,228,2,3);
    switchbranch(nowtype);
    if(finalline.empty()){
        // ips200.show_string(0,228,"byd1");
        return -2;
    }
    // ips200.show_int(0,228,3,3);
    splitline(nowtype);
    // ips200.show_int(0,228,4,3);
    if(finallinel.empty() || finalliner.empty()){
        // ips200.show_string(0,228,"byd2");
        return -2;
    }
    // for(auto rnm:finallinel){
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.x*3/4)),std::min(239,std::max(0,(int)rnm.y*3/4)),RGB565_BLUE);
    // }
    // for(auto rnm:finalliner){
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.x*3/4)),std::min(239,std::max(0,(int)rnm.y*3/4)),RGB565_RED);
    // }
    fnlmidline(nowtype);
    // ips200.show_int(0,228,finalmidline.back().x,3);
    // ips200.show_int(40,228,finalmidline.back().y,3);
    // ips200.draw_line(std::min(239,std::max(0,(int)finalmidline.back().x*3/4)),0,std::min(239,std::max(0,(int)finalmidline.back().x*3/4)),179,RGB565_BROWN);
    // ips200.draw_line(0,std::min(179,std::max(0,(int)finalmidline.back().y*3/4)),239,std::min(179,std::max(0,(int)finalmidline.back().y*3/4)),RGB565_BROWN);
    // for(auto rnm:finalmidline){
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.x*3/4)),std::min(239,std::max(0,(int)rnm.y*3/4)),RGB565_BROWN);
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.x*3/4 - 1)),std::min(239,std::max(0,(int)rnm.y*3/4)),RGB565_BROWN);
    //     ips200.draw_point(std::min(239,std::max(0,(int)rnm.x*3/4 + 1)),std::min(239,std::max(0,(int)rnm.y*3/4)),RGB565_BROWN);
    // }


    return 0;
    
}

int imgprocess::lstmrun(){
    auto _start = std::chrono::high_resolution_clock::now();
    std::vector<int> a,b;
    for(auto rnm:all_line_enum.DP_point){
        a.push_back(rnm.second);
    }
    b = byd_math::json_output_place(a);
    std::vector<cv::Point> points;
    for(auto rnm:b){
        points.push_back({static_cast<int>(std::round(all_line_enum.lineenum.at(rnm).coord.x)),static_cast<int>(std::round(all_line_enum.lineenum.at(rnm).coord.y))});
    }
    std::vector<float> probs = classifier.run(points);
    if (probs.empty()) {
        std::cout << "推理失败" << std::endl;
        return -1;
    }
    float maxprob = 0.0;
    int type = 0;
    for (int i = 0; i < 6; ++i) {
        //std::cout << classifier.labels[i] << ":" << probs[i] << std::endl;
        if(probs[i] > maxprob){
            maxprob = probs[i];
            type = i;
        }
    }
    std::cout << classifier.labels[type] << ':' << maxprob << std::endl;
    auto _end   = std::chrono::high_resolution_clock::now();
    auto _dur   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
    //std::cout << "runtime: " << _dur << " ns" << std::endl;
    return type;
}























int imgprocess::out_of_runway(){
    int count = 0;
    for(int i = 234;i < 237;i++){
        for(int j = 0;j < 320;j++){
            if(bin.at<uchar>(i,j) >= 1)count++;
        }
    }
    if(count < 64)return -1;
    return 0;
}
int imgprocess::banmaxian(){
    int banma = 0, i, black = 0, white, banmacount = 0, region = 0, j;
    for (j = 160;j <= 200;j++)
	{
		for (i = 60;i <= 260;i++)
		{
			if (bin.at<uchar>(j,i) == 255)
			{
				white = 1;
			}
			else
			{
				white = 0;
			}
			if (white != black)
			{
				black = white;
				banmacount++;
			}
			if (banmacount > 30)
			{
				banmacount = 0;
				region++;
			}
		}
	}
	if (region > 10)
	{
		banma = -2;
	}
	return banma;
}

void imgprocess::longest() {
	std::vector<int> zcbl(320, 239);
	zcbltrue = (319) / 2;
	zcblmin = 240;
	for (int i = 0;i < 320;i+=5) {
		for (int j = 234;j >= 0;j--) {
			zcbl.at(i) = j;
			if (bin.at<uchar>(j, i) == 0)break;
		}
		if (zcbl.at(i) < zcblmin) {
			zcbltrue = i;
			zcblmin = zcbl.at(i);
		}
	}
}











int imgprocess::bottom_mid(){
    std::pair<int,int> temp111;
    std::vector<std::pair<int,int>> byd111;
    for(int i = 0;i < UVC_WIDTH-1;i++){
        if(!bin.at<uchar>(UVC_HEIGHT-6,i)&&bin.at<uchar>(UVC_HEIGHT-6,i+1)){
            temp111.first = i;
        }
        if(bin.at<uchar>(UVC_HEIGHT-6,i)&&!bin.at<uchar>(UVC_HEIGHT-6,i+1)){
            temp111.second = i+1;
            byd111.push_back(temp111);
        }
    }
    int ans = 160,max = -1;
    if(byd111.empty())return 160;
    for(auto rnm:byd111){
        if(rnm.second-rnm.first > max){
            ans = (rnm.second+rnm.first) >> 1;
            max = rnm.second - rnm.first;
        }
    }
    return ans;
    // int add255 = 0,count = 0;
    // for(int i = 0;i < UVC_WIDTH;i++)if(bin.at<uchar>(UVC_HEIGHT-6,i)){
    //     add255+=i;
    //     count++;
    // }
    // if(count == 0)return 160;
    // return add255/count;
}
/*注意，这一八邻域是在黑端找黑白跳变
*/
cv::Point imgprocess::balinyu_start_l(int input){
    while(input--){
        if(!bin.at<uchar>(UVC_HEIGHT-6,input))return cv::Point(input,UVC_HEIGHT-6);
    }
    return cv::Point(-1,-1);
}
cv::Point imgprocess::balinyu_start_r(int input){
    while(input++){
        if(!bin.at<uchar>(UVC_HEIGHT-6,input))return cv::Point(input,UVC_HEIGHT-6);
    }
    return cv::Point(-1,-1);
}
uchar imgprocess::balinyu_linyu(cv::Point coord_input,int dir){
    cv::Point temp = coord_input + linyu.at((dir+8)%8);
    cv::Point temp1 = cv::Point(temp.y,temp.x);
    return bin.at<uchar>(temp1.x,temp1.y);
}
cv::Point imgprocess::balinyu_next_point(cv::Point coord_input,int dir){
    return coord_input + linyu.at((dir+8)%8);
}
cv::Point imgprocess::balinyu_suanzi(cv::Point coord_input,bool l_r,bool if_draw){
    balinyu_point temp;
    if(if_draw){
        //ips200.draw_point(coord_input.x*3/4,coord_input.y*3/4,RGB565_BLUE);
    }
    for(int i = 0;i < 8;i++){
        if(!balinyu_linyu(coord_input,i)&&balinyu_linyu(coord_input,i+1)){
            if(has_enter.find(balinyu_next_point(coord_input,i).y*UVC_WIDTH+balinyu_next_point(coord_input,i).x)!=has_enter.end())continue;
            temp.coord = coord_input;
            temp.dir = i;
            if(all_line_enum.lineenum.empty()){
                if(coord_input.x >= 4)temp.useful = 1;
            }else{
                temp.useful = all_line_enum.lineenum.back().useful;
                if(coord_input.x >= 4)temp.useful = 1;
            }
            all_line_enum.push_back(temp);
            has_enter.insert(temp.coord.y*UVC_WIDTH + temp.coord.x);
            // std::cout << coord_input.x << "," << coord_input.y << "|" << i << std::endl;
            endy = std::min(endy,coord_input.y);
            return balinyu_next_point(coord_input,i);
        }
    }
    return cv::Point(-1,-1);
}
void imgprocess::balinyu(cv::Point l_start,cv::Point r_start){
    lline.first.x = std::max(0,(l_start-cv::Point(10,0)).x);
    lline.first.y = (l_start-cv::Point(10,0)).y;
    rline.first.x = std::min(319,(r_start+cv::Point(10,0)).x);
    rline.first.y = (r_start+cv::Point(10,0)).y;
    lline.second = cv::Point(10,0);
    rline.second = cv::Point(309,0);
    cv::line(bin,lline.first,cv::Point(10,0),0,3,8,0);
    cv::line(bin,rline.first,cv::Point(309,0),0,3,8,0);
    // ips200.draw_line(lline.first.x*3/4,lline.first.y*3/4,lline.second.x*3/4,lline.second.y*3/4,RGB565_BLACK);
    // ips200.draw_line(rline.first.x*3/4,rline.first.y*3/4,rline.second.x*3/4,rline.second.y*3/4,RGB565_BLACK);
    lline.first = byd_math::IPMPoint(H,lline.first);
    lline.second = byd_math::IPMPoint(H,lline.second);
    rline.first = byd_math::IPMPoint(H,rline.first);
    rline.second = byd_math::IPMPoint(H,rline.second);
    // ips200.draw_line(lline.first.x*3/4,lline.first.y*3/4,lline.second.x*3/4,lline.second.y*3/4,RGB565_YELLOW);
    // ips200.draw_line(rline.first.x*3/4,rline.first.y*3/4,rline.second.x*3/4,rline.second.y*3/4,RGB565_YELLOW);





    endy = UVC_HEIGHT - 5;
    cv::Point next_point;
    //cv::Point next_point_l,next_point_r;
    next_point = l_start;
    has_enter.insert(r_start.y*UVC_WIDTH + r_start.x);
    // next_point_l = l_start;
    // next_point_r = r_start;

    // std::cout << l_start.x << " " << l_start.y << " " << r_start.x << " " << r_start.y << std::endl;
    if(l_start.x <= 0){
        return;
    }
    // std::cout << "l" << std::endl << 
    next_point = balinyu_suanzi(l_start,0);
    if(next_point == cv::Point(-1,-1)){
        return;
    }

    // next_point_r = balinyu_suanzi(r_start,1);
    // if(next_point_r == cv::Point(-1,-1)){
    //     return;
    // }
    l_start = next_point;
    // r_start = next_point_r;
    // std::cout << "blyloop";
    //while(fabs(l_start.x-r_start.x)+fabs(l_start.y-r_start.y) > 5)
    while(1){
        next_point = balinyu_suanzi(l_start,0);
        if(next_point == cv::Point(-1,-1)){
            // std::cout << "bydl" << std::endl;
            return;
        }
        l_start = next_point;
        if(has_enter.find(l_start.y*UVC_WIDTH+l_start.x)!=has_enter.end()){
            // std::cout << "bydl" << l_start.x << " " << l_start.y << std::endl;
            // std::cout << (bool)bin.at<uchar>(l_start.y-1,l_start.x-1) << " " << (bool)bin.at<uchar>(l_start.y-1,l_start.x) << " " << (bool)bin.at<uchar>(l_start.y-1,l_start.x+1) << std::endl;
            // std::cout << (bool)bin.at<uchar>(l_start.y,l_start.x-1) << " " << (bool)bin.at<uchar>(l_start.y,l_start.x) << " " << (bool)bin.at<uchar>(l_start.y,l_start.x+1) << std::endl;
            // std::cout << (bool)bin.at<uchar>(l_start.y+1,l_start.x-1) << " " << (bool)bin.at<uchar>(l_start.y+1,l_start.x) << " " << (bool)bin.at<uchar>(l_start.y+1,l_start.x+1) << std::endl;
            return;//重复进入同一个点就滚蛋，在八邻域中，于同一边界上这不可能在左右相遇前出现
        }
        // if(l_start.y < zcblmin+40){
        //     break;
        //     //if(l_start.y > l_line_enum.lineenum.back().coord.y)break;
        // }
    }
    //std::cout << "end" << std::endl;
    // std::cout << 121.52 << std::endl;
}

void imgprocess::filter(){
    // for(int i = 1;i < all_line_enum.DP_point.size() - 1;i++){
    //     if(cv::norm(all_line_enum.DP_point.at(i+1).first - all_line_enum.DP_point.at(i).first) < 8.0){
    //         all_line_enum.DP_Point_useful.at(i) = 0;
    //     }
    // }
    // all_line_enum.reflashDP();
    for(int i = 0;i < (int)all_line_enum.DP_point.size() - 3;i++){
        if(cv::norm(all_line_enum.DP_point.at(i+3).first - all_line_enum.DP_point.at(i).first) < 16.0 &&
            std::max(
                byd_math::AB_to_C(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+3).first,all_line_enum.DP_point.at(i+1).first),
                byd_math::AB_to_C(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+3).first,all_line_enum.DP_point.at(i+2).first))
                > 3
            ){

            all_line_enum.DP_Point_useful.at(i+1) = 0;
            all_line_enum.DP_Point_useful.at(i+2) = 0;
            i += 2;
        }else if(cv::norm(all_line_enum.DP_point.at(i+2).first - all_line_enum.DP_point.at(i).first) < 16.0 &&
            byd_math::AB_to_C(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+2).first,all_line_enum.DP_point.at(i+1).first)
            > 3
            ){
            all_line_enum.DP_Point_useful.at(i+1) = 0;
            i++;
        }
        // }else if(cv::norm(all_line_enum.DP_point.at(i+3).first - all_line_enum.DP_point.at(i).first) < 16.0){
        //     all_line_enum.DP_Point_useful.at(i+1) = 0;
        //     all_line_enum.DP_Point_useful.at(i+2) = 0;
        //     i+=2;
        // }else if(cv::norm(all_line_enum.DP_point.at(i+2).first - all_line_enum.DP_point.at(i).first) < 16.0){
        //     all_line_enum.DP_Point_useful.at(i+1) = 0;
        //     i++;
        // }
    }
    all_line_enum.reflashDP();
    // all_line_enum.drawDP();
    std::vector<std::pair<cv::Point, i64>> newdp;
    newdp.push_back(all_line_enum.DP_point.at(0));
    for(int i = 1;i < all_line_enum.DP_point.size();i++){
        if(all_line_enum.DP_point.at(i).second - all_line_enum.DP_point.at(i-1).second > 100){
            int num = (all_line_enum.DP_point.at(i).second - all_line_enum.DP_point.at(i-1).second)/100;
            for(int j = 1;j <= num;j++){
                newdp.push_back({all_line_enum.lineenum.at(
                    all_line_enum.DP_point.at(i-1).second + 
                    (all_line_enum.DP_point.at(i).second - all_line_enum.DP_point.at(i-1).second)*j/(num + 1)
                ).coord,all_line_enum.DP_point.at(i-1).second + 
                (all_line_enum.DP_point.at(i).second - all_line_enum.DP_point.at(i-1).second)*j/(num + 1)
                });
            }
        }
        newdp.push_back(all_line_enum.DP_point.at(i));
    }
    all_line_enum.DP_point = newdp;
}

void imgprocess::typesort(){
    bool u_l = 0,l_d = 0,r_u = 0,d_r = 0;
    for(int i = 1;i < all_line_enum.DP_point.size()-1;i++){
        cv::Point2f A = all_line_enum.DP_point.at(i).first - all_line_enum.DP_point.at(i-1).first;
        cv::Point2f B = all_line_enum.DP_point.at(i+1).first - all_line_enum.DP_point.at(i).first;
        A.y = -A.y;
        B.y = -B.y;
        float angle = byd_math::AOBangle(A,B);
        if(angle > 0.4){
            if(std::abs(byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first)) < 0.25 && 
            (byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first) > 0.25 &&
            byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first) < 0.75
            )
            ){
                u_l = 1;
            }else if((byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first) > 0.25 &&
            byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first) < 0.75
            ) &&
            std::abs(byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first)) > 0.75
            ){
                l_d = 1;
            }else if((byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first) < -0.25 &&
            byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first) > -0.75
            )&&std::abs(byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first)) < 0.25){
                r_u = 1;
            }else if(std::abs(byd_math::angle(all_line_enum.DP_point.at(i-1).first,all_line_enum.DP_point.at(i).first)) > 0.75&&
            (byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first) < -0.25 &&
            byd_math::angle(all_line_enum.DP_point.at(i).first,all_line_enum.DP_point.at(i+1).first) > -0.75)
            ){
                d_r = 1;
            }
        }
    }
    // ips200.show_int(0,180,u_l,1);
    // ips200.show_int(20,180,r_u,1);
    // ips200.show_int(40,180,d_r,1);
    // ips200.show_int(60,180,l_d,1);
    //std::cout << ban_hd_time << "|" << encoder_l_count+encoder_r_count << std::endl;
    if(u_l && l_d){
        lhd = 0;
        rhd = 0;
        ban_hd_time = encoder_l_count + encoder_r_count + 5000;
        nowtype = 0;
        return;
    }
    if(r_u && d_r){
        lhd = 0;
        rhd = 0;
        ban_hd_time = encoder_l_count + encoder_r_count + 5000;
        nowtype = 0;
        return;
    }
    if(encoder_l_count+encoder_r_count > ban_hd_time){
        if(u_l && !d_r && !l_d && lhd != 2 && rhd != 2){
            if(lstmrun() == 2){
                hd_enter_time = encoder_l_count + encoder_r_count + 20000;
                if(lhd == 0){
                    lhd = 1;
                }
            }
            rhd = 0;
            nowtype = 0;
            return;
        }
        else if(!u_l && !r_u && l_d && lhd != 2 && rhd != 2){
            if(lstmrun() == 4){
                hd_enter_time = encoder_l_count + encoder_r_count + 20000;
                if(rhd == 0){
                    rhd = 1;
                }
            }
            lhd = 0;
            nowtype = 0;
            return;
        }
        if(nowtype == 0){
            if(encoder_l_count+encoder_r_count <= hd_enter_time){
                if(lhd == 1 && r_u ){
                    if(lstmrun() == 3){
                        lhd = 2;
                        nowtype = 0;
                        //ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                        //std::cout << "byd0" << std::endl;
                        return;
                    }
                }
                else if(rhd == 1 && d_r ){
                    if(lstmrun() == 5){
                        rhd = 2;
                        nowtype = 0;
                        //ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                        //std::cout << "byd1" << std::endl;
                        return;
                    }
                }
                else return;
            }else{
                if(lhd == 2){
                    nowtype = -1;
                    ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                    max_hd_length = encoder_l_count + encoder_r_count + 100000;
                    return;
                }
                else if(rhd == 2){
                    nowtype = 1;
                    ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                    max_hd_length = encoder_l_count + encoder_r_count + 100000;
                    return;
                }else{
                    lhd = 0;
                    rhd = 0;
                    nowtype = 0;
                    ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                }
            }
        }else{
            if(lhd == 2 && r_u && !u_l && !d_r && !l_d){
                if(lstmrun() == 3){
                    lhd = 0;
                    nowtype = 0;
                    ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                    //std::cout << "byd2" << std::endl;
                    return;
                }
            }
            else if(rhd == 2 && d_r && !u_l && !r_u && !l_d){
                if(lstmrun() == 5){
                    rhd = 0;
                    nowtype = 0;
                    ban_hd_time = encoder_l_count + encoder_r_count + 25000;
                    //std::cout << "byd3" << std::endl;
                    return;
                }
            }else if(encoder_l_count + encoder_r_count > max_hd_length){
                lhd = 0;
                rhd = 0;
                nowtype = 0;
                return;
            }
        }
    }
}












// void imgprocess::pruning(){//ver.0
//     all_line_enum.Pruning_point.clear();
//     all_line_enum.Pruning_point.push_back({all_line_enum.DP_point.at(0).first,0});
//     for(int i = 1;i < all_line_enum.DP_point.size()-1;i++){
//         cv::Point2f A = all_line_enum.DP_point.at(i).first - all_line_enum.DP_point.at(i-1).first;
//         cv::Point2f B = all_line_enum.DP_point.at(i+1).first - all_line_enum.DP_point.at(i).first;
//         A.y = -A.y;
//         B.y = -B.y;
//         float angle = byd_math::AOBangle(A,B);
//         if(angle > 0.4){
//             // std::cout << all_line_enum.DP_point.at(i-1).first << all_line_enum.DP_point.at(i).first << all_line_enum.DP_point.at(i+1).first << std::endl;
//             all_line_enum.Pruning_point.push_back({all_line_enum.DP_point.at(i).first,i});
//         }
//     }
//     all_line_enum.Pruning_point.push_back({all_line_enum.DP_point.back().first,all_line_enum.DP_point.size()-1});
//     // all_line_enum.drawPP();
//     if(all_line_enum.Pruning_point.size() > 4){
//         std::vector<std::pair<cv::Point, i64>> temppp;
//         temppp.push_back(all_line_enum.Pruning_point.at(0));
//         temppp.push_back(all_line_enum.Pruning_point.at(1));
//         for(int i = 2;i < all_line_enum.Pruning_point.size()-2;i++){
//             if(all_line_enum.Pruning_point.at(i).second - all_line_enum.Pruning_point.at(i-1).second == 1 && 
//             all_line_enum.Pruning_point.at(i+1).second - all_line_enum.Pruning_point.at(i).second > 1){
//                 if(std::abs(byd_math::AOBangle(all_line_enum.Pruning_point.at(i).first-all_line_enum.Pruning_point.at(i-1).first,
//                 all_line_enum.Pruning_point.at(i+1).first-all_line_enum.Pruning_point.at(i).first))>0.5)continue;
//             }else if(all_line_enum.Pruning_point.at(i).second - all_line_enum.Pruning_point.at(i-1).second >= 1 && 
//             all_line_enum.Pruning_point.at(i+1).second - all_line_enum.Pruning_point.at(i).second == 1){
//                 if(std::abs(byd_math::AOBangle(all_line_enum.Pruning_point.at(i).first-all_line_enum.Pruning_point.at(i-1).first,
//                 all_line_enum.Pruning_point.at(i+1).first-all_line_enum.Pruning_point.at(i).first))>0.5)continue;
//             }
//             temppp.push_back(all_line_enum.Pruning_point.at(i));
//         }
//         temppp.push_back(all_line_enum.Pruning_point.at(all_line_enum.Pruning_point.size()-2));
//         temppp.push_back(all_line_enum.Pruning_point.back());
//         all_line_enum.Pruning_point = temppp;
//     }

// }
void imgprocess::pruning(){//ver.1
    all_line_enum.Pruning_point.clear();

    /*
    思考：是否可以直接开始挑出所有<135°的内拐角，然后直接合
    中止;
    思考：没必要筛点，将所有拐点全部导入，直接剪枝
    分析：
    逻辑：单次处理多步，顺时针搜索，如果找到内角<135°的点，直接删
    保护措施：删除这个点后，i++，再由for i++，防止删断
    逻辑：删点时，确保左右不同时useful
    逻辑：pp index差=dp index差 则该点无用

    */
    //重写：
    for(int i = 0;i < all_line_enum.DP_point.size();i++){//复制dp_point，记录原index
        all_line_enum.Pruning_point.push_back({all_line_enum.DP_point.at(i).first,i});
    }
    while(1){
        //std::vector<bool> ppuseful(all_line_enum.Pruning_point.size(),1);
        std::vector<std::pair<cv::Point, i64>> newpp;
        newpp.clear();
        newpp.push_back(all_line_enum.Pruning_point.at(0));
        for(int i = 1;i < all_line_enum.Pruning_point.size()-1;i++){
            if(all_line_enum.Pruning_point.at(i).second - all_line_enum.Pruning_point.at(i-1).second == 1 
            || all_line_enum.Pruning_point.at(i+1).second - all_line_enum.Pruning_point.at(i).second == 1){
                cv::Point2f A = all_line_enum.Pruning_point.at(i).first - all_line_enum.Pruning_point.at(i-1).first;
                cv::Point2f B = all_line_enum.Pruning_point.at(i+1).first - all_line_enum.Pruning_point.at(i).first;
                A.y = -A.y;
                B.y = -B.y;
                float angle = byd_math::AOBangle(A,B);
                if(angle <= -0.25 || angle >= 0.75){
                    i++;
                    if(i < all_line_enum.Pruning_point.size()-1)newpp.push_back(all_line_enum.Pruning_point.at(i));
                    continue;
                }else{
                    newpp.push_back(all_line_enum.Pruning_point.at(i));
                }
            }else{
                cv::Point2f A = all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i).second).first - all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i).second-1).first;
                cv::Point2f B = all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i).second+1).first - all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i).second).first;
                A.y = -A.y;
                B.y = -B.y;
                float angle = byd_math::AOBangle(A,B);
                if(angle < 0.05){
                    i++;
                    if(i < all_line_enum.Pruning_point.size()-1)newpp.push_back(all_line_enum.Pruning_point.at(i));
                    continue;
                }else{
                    newpp.push_back(all_line_enum.Pruning_point.at(i));
                }

            }
        }
        newpp.push_back(all_line_enum.Pruning_point.back());
        if(newpp.size() == all_line_enum.Pruning_point.size())break;
        else{
            all_line_enum.Pruning_point = newpp;
        }
    }
    






}

void imgprocess::line_enum::branchfilter(cv::Point mdp){
    std::pair<cv::Point,cv::Point> start_arrow = {cv::Point(-1,-1),cv::Point(-1,-1)},dir_arrow = {cv::Point(-1,-1),cv::Point(-1,-1)};
    branch.assign(Pruning_point.size()-1,{0,1});
    // float start_angle = 0.0;
    // if(branch.size() >= 3){
    //     start_angle = (byd_math::angle(Pruning_point.at(0).first,Pruning_point.at(1).first) + 
    //                   byd_math::angle(Pruning_point.back().first,Pruning_point.at(Pruning_point.size()-2).first))/2.0;
    // }
    for(int i = 0;i < Pruning_point.size()-1;i++){
        
        //branch.at(i).first = (Pruning_point.at(i+1).first - Pruning_point.at(i).first).dot((Pruning_point.at(i+1).first + Pruning_point.at(i).first)/2.0 - mdp);
        //byd_math::angle(Pruning_point.at(i).first,Pruning_point.at(i+1).first) + 0.5 - start_angle;
        // branch.at(i).first = byd_math::angle(mdp,Pruning_point.at(i).first) - start_angle;
        // if(std::abs(branch.at(i).first) < std::abs(byd_math::angle(mdp,Pruning_point.at(i+1).first) - start_angle))branch.at(i).first = byd_math::angle(mdp,Pruning_point.at(i+1).first) - start_angle;
        float max = 0.0;
        for(int j = Pruning_point.at(i).second+1;j < Pruning_point.at(i+1).second;j++){
            max = std::max(max,byd_math::AB_to_C(Pruning_point.at(i).first,Pruning_point.at(i+1).first,DP_point.at(j).first));
        }
        if(max < 24.0){
            branch.at(i).second = 0;
        }
    }
    if(branch.at(0).second == 0){
        start_arrow.first = Pruning_point.at(1).first;
    }else start_arrow.first = Pruning_point.at(0).first;
    if(branch.back().second == 0){
        start_arrow.second = Pruning_point.at(Pruning_point.size()-2).first;
    }else start_arrow.second = Pruning_point.back().first;
    for(int i = 0;i < Pruning_point.size()-1;i++){
        dir_arrow.first = Pruning_point.at(i).first;
        dir_arrow.second = Pruning_point.at(i+1).first;
        branch.at(i).first = ((dir_arrow.first + dir_arrow.second)/2 - (start_arrow.first + start_arrow.second)/2).dot(start_arrow.second - start_arrow.first);
    }




    // branch.assign(Pruning_point.size()-1,{0,1});
    // float start_angle = 0.0;
    // if(branch.size() >= 3){
    //     start_angle = (byd_math::angle(Pruning_point.at(0).first,Pruning_point.at(1).first) + 
    //                   byd_math::angle(Pruning_point.back().first,Pruning_point.at(Pruning_point.size()-2).first))/2.0;
    // }
    // for(int i = 0;i < Pruning_point.size()-1;i++){
        
    //     branch.at(i).first = (Pruning_point.at(i+1).first - Pruning_point.at(i).first).dot((Pruning_point.at(i+1).first + Pruning_point.at(i).first)/2.0 - mdp);
    //     //byd_math::angle(Pruning_point.at(i).first,Pruning_point.at(i+1).first) + 0.5 - start_angle;
    //     // branch.at(i).first = byd_math::angle(mdp,Pruning_point.at(i).first) - start_angle;
    //     // if(std::abs(branch.at(i).first) < std::abs(byd_math::angle(mdp,Pruning_point.at(i+1).first) - start_angle))branch.at(i).first = byd_math::angle(mdp,Pruning_point.at(i+1).first) - start_angle;
    //     float max = 0.0;
    //     for(int j = Pruning_point.at(i).second+1;j < Pruning_point.at(i+1).second;j++){
    //         max = std::max(max,byd_math::AB_to_C(Pruning_point.at(i).first,Pruning_point.at(i+1).first,DP_point.at(j).first));
    //     }
    //     if(max < 16.0){
    //         branch.at(i).second = 0;
    //     }
    // }
}

void imgprocess::switchbranch(int type){//傻逼，你的当务之急是改成选拐点
    int index = -1;
    float min = 114514.0;

    switch (type)
    {
    case -1:
        for(int i = 0;i < all_line_enum.branch.size();i++){
            if(all_line_enum.branch.at(i).second){
                index = i;
                break;
            }
        }
        break;
    case 0:
        for(int i = 0;i < all_line_enum.branch.size();i++){
            if(all_line_enum.branch.at(i).second && std::abs(all_line_enum.branch.at(i).first) < min){
                index = i;
                min = std::abs(all_line_enum.branch.at(i).first);
            }
        }
        break;
    case 1:
        for(int i = all_line_enum.branch.size()-1;i >= 0;i--){
            if(all_line_enum.branch.at(i).second){
                index = i;
                break;
            }
        }
        break;
    default:
        std::cout << "fuck you" << std::endl;
        break;
    }
    if(index == -1)return;
    // std::cout << std::endl << index << std::endl;
    for(int i = 0;i < all_line_enum.branch.size();i++){
        if(i != index){
            pushbackline(finalline,all_line_enum.Pruning_point.at(i).first,all_line_enum.Pruning_point.at(i+1).first);
        }else{
            for(int j = all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i).second).second;
                j < std::min(all_line_enum.DP_point.at(all_line_enum.Pruning_point.at(i+1).second).second,(i64)all_line_enum.lineenum.size());
                j++){
                if(finalline.empty()){
                    finalline.push_back(all_line_enum.lineenum.at(j).coord);
                }else if(finalline.back() != (cv::Point)all_line_enum.lineenum.at(j).coord){
                    finalline.push_back(all_line_enum.lineenum.at(j).coord);
                }
            }
        }
    }
}

void imgprocess::pushbackline(std::vector<cv::Point>& line, cv::Point start, cv::Point end)
{
    int x = start.x;
    int y = start.y;

    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int sx = (start.x < end.x) ? 1 : -1;
    int sy = (start.y < end.y) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if(line.empty()){
            line.push_back(cv::Point(x, y));
        }else if(line.back() != cv::Point(x, y)){
            line.push_back(cv::Point(x, y));
        }

        if (x == end.x && y == end.y)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

void imgprocess::splitline(int type){
    if(type == -1){
        float min = 114514.0;
        int indexl = -1,indexr = -1;
        for(int i = 0;i < finalline.size();i++){
            if(cv::norm(finalline.at(i)-cv::Point(0,0)) < min){
                indexl = i;indexr = i;
                min = cv::norm(finalline.at(i)-cv::Point(0,0));
            }
        }
        if(indexl == -1 || indexr == -1){
            return;
        }
        while(byd_math::AB_to_C(lline.first,lline.second,finalline.at(indexl)) < 8){
            indexl--;
            if(indexl <= 0)break;
        }
        while(byd_math::AB_to_C(rline.first,rline.second,finalline.at(indexr)) < 8){
            indexr++;
            if(indexr >= finalline.size()-1)break;
        }
        for(int i = 0;i <= indexl;i++){
            finallinel.push_back(finalline.at(i));
        }
        pushbackline(finalliner,all_line_enum.lineenum.back().coord,finalline.at(indexr));
        // for(int i = finalline.size()-1;i >= indexr;i--){
        //     finalliner.push_back(finalline.at(i));
        // }
    }else if(type == 0){
        float max = 0.0;
        int indexl = -1,indexr = -1;
        for(int i = 0;i < finalline.size();i++){
            if(cv::norm(finalline.at(i)-midpoint) > max){
                indexl = i;indexr = i;
                max = cv::norm(finalline.at(i)-midpoint);
            }
        }
        if(indexl == -1 || indexr == -1){
            return;
        }
        while(byd_math::AB_to_C(lline.first,lline.second,finalline.at(indexl)) < 8){
            indexl--;
            if(indexl <= 0)break;
        }
        while(byd_math::AB_to_C(rline.first,rline.second,finalline.at(indexr)) < 8){
            indexr++;
            if(indexr >= finalline.size()-1)break;
        }
        for(int i = 0;i <= indexl;i++){
            finallinel.push_back(finalline.at(i));
        }
        for(int i = finalline.size()-1;i >= indexr;i--){
            finalliner.push_back(finalline.at(i));
        }
    }else if(type == 1){
        float min = 114514.0;
        int indexl = -1,indexr = -1;
        for(int i = 0;i < finalline.size();i++){
            if(cv::norm(finalline.at(i)-cv::Point(319,0)) < min){
                indexl = i;indexr = i;
                min = cv::norm(finalline.at(i)-cv::Point(319,0));
            }
        }
        if(indexl == -1 || indexr == -1){
            return;
        }
        while(byd_math::AB_to_C(lline.first,lline.second,finalline.at(indexl)) < 8){
            indexl--;
            if(indexl <= 0)break;
        }
        while(byd_math::AB_to_C(rline.first,rline.second,finalline.at(indexr)) < 8){
            indexr++;
            if(indexr >= finalline.size()-1)break;
        }
        pushbackline(finallinel,all_line_enum.lineenum.at(0).coord,finalline.at(indexl));
        // for(int i = 0;i <= indexl;i++){
        //     finallinel.push_back(finalline.at(i));
        // }
        for(int i = finalline.size()-1;i >= indexr;i--){
            finalliner.push_back(finalline.at(i));
        }
    }else{
        std::cout << "fuck you" << std::endl;
        return;
    }
    // float max = 0.0;
    // int index = -1;
    // for(int i = 0;i < finalline.size();i++){
    //     if(cv::norm(finalline.at(i)-midpoint) > max){
    //         index = i;
    //         max = cv::norm(finalline.at(i)-midpoint);
    //     }
    // }
    // if(index == -1){
    //     return;
    // }
    // for(int i = 0;i <= index;i++){
    //     finallinel.push_back(finalline.at(i));
    // }
    // for(int i = finalline.size()-1;i > index;i--){
    //     finalliner.push_back(finalline.at(i));
    // }
}

void imgprocess::fnlmidline(int type){
    //type = 0;
    finalmidline.clear();
    if(type == -1){
        // finalline.push_back(finallinel.at(0));
        // for(auto rnm:finallinel){
        //     if(rnm.x < finalline.at(0).x){
        //         finalline.at(0) = rnm;
        //     }
        // }
        // finalline.at(0).y -= 10;
        // if(finalline.at(0).y < 20)finalline.at(0).y = 20;
        if(finallinel.back() != finalliner.back()){
            finalmidline.push_back((finallinel.back() + finalliner.back())/2);
        }else{
            finalmidline.push_back(finallinel.back() + (midpoint-finallinel.back())/3);
        }
        if(finalmidline.back().x > 100)finalmidline.back().x = 100;
        //finalmidline.at(0).y -= (finalmidline.at(0).y)/5;
        //std::cout << finallinel.back() << "|" << (midpoint-finallinel.back())/3 << "|" << finalmidline.at(0) << std::endl;
    }else if(type == 0){
        bool if_l = 0;
        int yl = std::max(finallinel.back().y,std::min(finallinel.at(0).y,finallinel.back().y + (finallinel.at(0).y-finallinel.back().y)*2/3));
        int yr = std::max(finalliner.back().y,std::min(finalliner.at(0).y,finalliner.back().y + (finalliner.at(0).y-finalliner.back().y)*2/3));
        cv::Point templ,tempr;
        for(auto rnm:finallinel){
            templ = rnm;
            if(rnm.y <= yl){
                templ = rnm;
                // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_RED);
                break;
            }
        }
        for(auto rnm:finalliner){
            tempr = rnm;
            if(rnm.y <= yr){
                tempr = rnm;
                // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_BLUE);
                break;
            }
        }
        // cv::Point temppoint = (templ + tempr)/2.0,tempstart = (all_line_enum.lineenum.at(0).coord + all_line_enum.lineenum.back().coord)/2.0,fnlpoint = temppoint*2-tempstart;
        // fnlpoint.x += (temppoint.x - tempstart.x)*2.5/(tempstart.y - temppoint.y);
        // finalmidline.push_back(fnlpoint);
        finalmidline.push_back((templ + tempr)/2.0);
        if(finalmidline.back().x <= 159.5)if_l = 1;
        yl = std::max(finallinel.back().y,std::min(finallinel.at(0).y,finallinel.back().y + (finallinel.at(0).y-finallinel.back().y)/3));
        yr = std::max(finalliner.back().y,std::min(finalliner.at(0).y,finalliner.back().y + (finalliner.at(0).y-finalliner.back().y)/3));
        for(auto rnm:finallinel){
            templ = rnm;
            if(rnm.y <= yl){
                templ = rnm;
                // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_RED);
                break;
            }
        }
        for(auto rnm:finalliner){
            tempr = rnm;
            if(rnm.y <= yr){
                tempr = rnm;
                // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_BLUE);
                break;
            }
        }
        if(if_l){
            if(((templ + tempr)/2.0).x < finalmidline.back().x)finalmidline.back() = (templ + tempr)/2.0;
        }else{
            if(((templ + tempr)/2.0).x > finalmidline.back().x)finalmidline.back() = (templ + tempr)/2.0;            
        }
        // if(finallinel.back() != finalliner.back()){
        //     finalmidline.push_back((finallinel.back() + finalliner.back())/2);
        // }else{
        //     int yl = std::max(finallinel.back().y,std::min(finallinel.at(0).y,finallinel.back().y + (finallinel.at(0).y-finallinel.back().y)/3));
        //     int yr = std::max(finalliner.back().y,std::min(finalliner.at(0).y,finalliner.back().y + (finalliner.at(0).y-finalliner.back().y)/3));
        //     cv::Point templ,tempr;
        //     for(auto rnm:finallinel){
        //             templ = rnm;
        //         if(rnm.y <= yl){
        //             templ = rnm;
        //             // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_RED);
        //             break;
        //         }
        //     }
        //     for(auto rnm:finalliner){
        //             tempr = rnm;
        //         if(rnm.y <= yr){
        //             tempr = rnm;
        //             // ips200.draw_line(rnm.x*3/4-1,rnm.y*3/4,rnm.x*3/4+1,rnm.y*3/4,RGB565_BLUE);
        //             break;
        //         }
        //     }
        //     finalmidline.push_back((templ + tempr)/2);
        // }

    }else if(type == 1){
        // finalline.push_back(finalliner.at(0));
        // for(auto rnm:finalliner){
        //     if(rnm.x > finalline.at(0).x){
        //         finalline.at(0) = rnm;
        //     }
        // }
        // finalline.at(0).y -= 10;
        // if(finalline.at(0).y < 20)finalline.at(0).y = 20;
        if(finallinel.back() != finalliner.back()){
            finalmidline.push_back((finallinel.back() + finalliner.back())/2);
        }else{
            finalmidline.push_back(finalliner.back() + (midpoint-finalliner.back())/3);
        }
        if(finalmidline.back().x < 220)finalmidline.back().x = 220;
        //finalmidline.at(0).y -= (finalmidline.at(0).y)/5;
    }else{
        std::cout << "fuck you" << std::endl;
        return;
    }


}

void imgprocess::fnlcircle(){
    /*
    点到圆心的平方实质为距离的平方
    设圆心为cv::Point(159.5,0)
    当前点为cv::Point(nowmidpoint.x,239-nowmidpoint.y+camlen)
    则距离平方为
    */
}
// cv::Point nowmidpoint = cv::Point(-1,-1);
// cv::Point carmidpoint = cv::Point(159.5,0);
// int camlen = 40;
// int lj = 31;
// float msp = 110;
// float P = 4.5;
// void pit_callback1(void)
// {
//     //std::cout << encoder_l_count+encoder_r_count << std::endl;
//     if(nowmidpoint.x >= 0 && nowmidpoint.y >= 0){
//         cv::Point realmidpoint = cv::Point(nowmidpoint.x,239-nowmidpoint.y+camlen);
//         cv::Point realdir = realmidpoint - carmidpoint;
//         float r = (cv::norm(realdir)/2.0)/(realdir.x/cv::norm(realdir));
//         if(std::abs(r)<1)return;
//         float finalsp = msp * (1.0-0.3*std::abs(realdir.x/cv::norm(realdir)));
//         // if(imu_acc_z > -3500)finalsp /= 2.0;
//         // if(imu_acc_z < -4500)finalsp *= 2.0;
//         float lsp = finalsp*(1.0+(lj/(2.0*r))*P);
//         float rsp = finalsp*(1.0-(lj/(2.0*r))*P);
//         lsp -= (imu_gyro_z / finalsp)*1.4;
//         rsp += (imu_gyro_z / finalsp)*1.4;





































void imgprocess::line_enum::approxPolyDP(){
    DP_point.clear();

    if(lineenum.size() < 30)return;
    DP_point.push_back({lineenum.at(0).coord,0});
    DP_point.push_back({lineenum.back().coord,lineenum.size()-1});
    std::vector<std::pair<cv::Point,i64>> newDPpoint;
    while(1){
        newDPpoint.clear();
        for(size_t i = 0;i < DP_point.size() - 1;i++){
            float D_MAX = 0.0;
            float place = -1;
            for(i64 j = DP_point.at(i).second + 1;j < DP_point.at(i+1).second;j++){
                if(byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord) > D_MAX){
                    place = j;
                    D_MAX = byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord);
                }
            }
            if(D_MAX >= 20/*cv::norm(DP_point.at(i).first-DP_point.at(i+1).first)*0.1*/){
                newDPpoint.push_back({lineenum.at(place).coord,place});
            }
        }
        if(newDPpoint.empty())break;
        else{
            for(auto niumo:newDPpoint)DP_point.push_back(niumo);
            std::sort(DP_point.begin(),DP_point.end(),[](const std::pair<cv::Point,i64>& a,const std::pair<cv::Point,i64>& b){
                return a.second < b.second;
            });
        }
    }
    while(1){
        newDPpoint.clear();
        for(size_t i = 0;i < DP_point.size() - 1;i++){
            float D_MAX = 0.0;
            float place = -1;
            for(i64 j = DP_point.at(i).second + 1;j < DP_point.at(i+1).second;j++){
                if(byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord) > D_MAX){
                    place = j;
                    D_MAX = byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord);
                }
            }
            if(D_MAX >= 10/*cv::norm(DP_point.at(i).first-DP_point.at(i+1).first)*0.1*/){
                newDPpoint.push_back({lineenum.at(place).coord,place});
            }
        }
        if(newDPpoint.empty())break;
        else{
            for(auto niumo:newDPpoint)DP_point.push_back(niumo);
            std::sort(DP_point.begin(),DP_point.end(),[](const std::pair<cv::Point,i64>& a,const std::pair<cv::Point,i64>& b){
                return a.second < b.second;
            });
        }
    }
    while(1){
        newDPpoint.clear();
        for(size_t i = 0;i < DP_point.size() - 1;i++){
            float D_MAX = 0.0;
            float place = -1;
            for(i64 j = DP_point.at(i).second + 1;j < DP_point.at(i+1).second;j++){
                if(byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord) > D_MAX){
                    place = j;
                    D_MAX = byd_math::AB_to_C(DP_point.at(i).first,DP_point.at(i+1).first,lineenum.at(j).coord);
                }
            }
            if(D_MAX >= 7.5/*cv::norm(DP_point.at(i).first-DP_point.at(i+1).first)*0.1*/){
                newDPpoint.push_back({lineenum.at(place).coord,place});
            }
        }
        if(newDPpoint.empty())break;
        else{
            for(auto niumo:newDPpoint)DP_point.push_back(niumo);
            std::sort(DP_point.begin(),DP_point.end(),[](const std::pair<cv::Point,i64>& a,const std::pair<cv::Point,i64>& b){
                return a.second < b.second;
            });
        }
    }
    // drawDP();
    DP_Point_useful.assign(DP_point.size(),1);
    // std::cout << DP_point.size() << std::endl;
    // newDPpoint.clear();
    // for(size_t i = 1;i < DP_point.)
}
void imgprocess::line_enum::reflashDP(){
    std::vector<std::pair<cv::Point,i64>> tempDP_point;
    for(int i = 0;i < DP_Point_useful.size();i++){
        if(DP_Point_useful.at(i))tempDP_point.push_back(DP_point.at(i));
    }
    DP_point = tempDP_point;
    DP_Point_useful.assign(DP_point.size(),1);
}
void imgprocess::line_enum::drawDP(){
    for(int i = 0;i < DP_point.size()-1;i++){
        ips200.draw_line(DP_point.at(i).first.x*3/4,DP_point.at(i).first.y*3/4,DP_point.at(i+1).first.x*3/4,DP_point.at(i+1).first.y*3/4,RGB565_PURPLE);
        ips200.draw_point(DP_point.at(i).first.x*3/4,DP_point.at(i).first.y*3/4,RGB565_CYAN);
        ips200.draw_point(DP_point.at(i).first.x*3/4 + 1,DP_point.at(i).first.y*3/4 + 1,RGB565_CYAN);
        ips200.draw_point(DP_point.at(i).first.x*3/4 + 1,DP_point.at(i).first.y*3/4 - 1,RGB565_CYAN);
        ips200.draw_point(DP_point.at(i).first.x*3/4 - 1,DP_point.at(i).first.y*3/4 + 1,RGB565_CYAN);
        ips200.draw_point(DP_point.at(i).first.x*3/4 - 1,DP_point.at(i).first.y*3/4 - 1,RGB565_CYAN);
    }
}
void imgprocess::line_enum::drawPP(){
    for(int i = 0;i < Pruning_point.size()-1;i++){
        ips200.draw_line(Pruning_point.at(i).first.x*3/4,Pruning_point.at(i).first.y*3/4,Pruning_point.at(i+1).first.x*3/4,Pruning_point.at(i+1).first.y*3/4,RGB565_RED);
        ips200.draw_point(Pruning_point.at(i).first.x*3/4,Pruning_point.at(i).first.y*3/4,RGB565_CYAN);
        ips200.draw_point(Pruning_point.at(i).first.x*3/4 + 1,Pruning_point.at(i).first.y*3/4 + 1,RGB565_CYAN);
        ips200.draw_point(Pruning_point.at(i).first.x*3/4 + 1,Pruning_point.at(i).first.y*3/4 - 1,RGB565_CYAN);
        ips200.draw_point(Pruning_point.at(i).first.x*3/4 - 1,Pruning_point.at(i).first.y*3/4 + 1,RGB565_CYAN);
        ips200.draw_point(Pruning_point.at(i).first.x*3/4 - 1,Pruning_point.at(i).first.y*3/4 - 1,RGB565_CYAN);
    }
}
