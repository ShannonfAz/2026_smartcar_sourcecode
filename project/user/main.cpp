
/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.hpp"
#include "imgprocess_new.hpp"
#include "control.hpp"
using i64 = long long;


#define TIME_IT_NS(code)                                                \
    do {                                                                \
        auto _start = std::chrono::high_resolution_clock::now();        \
        code;                                                           \
        auto _end   = std::chrono::high_resolution_clock::now();        \
        auto _dur   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count(); \
        std::cout << "time: " << _dur << " ns" << std::endl;            \
    } while(0)










#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1

#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2

#define ENCODER_DIR_1_PATH           ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH           ZF_ENCODER_DIR_2


zf_driver_pit pit_timer;
zf_driver_pit pit_timer1;
zf_driver_pit pit_timer2;
struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;
zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);
zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);
zf_driver_gpio  key_4(ZF_GPIO_KEY_4, O_RDWR);
zf_driver_gpio  key_3(ZF_GPIO_KEY_3, O_RDWR);
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max) 
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)

zf_device_ips200 ips200;
zf_device_uvc uvc_dev;
zf_device_imu imu_dev;
zf_driver_adc battery_adc(ADC_CH7_PATH);
control ctrl;
int midline = 160;


void sigint_handler(int signum) //ctrl+c退出时干的
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}
void cleanup()//退出时干的
{
    pit_timer.stop();
    printf("程序异常退出，执行清理操作\n");
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
}
int encoder_l_speed = 100,encoder_r_speed = 100;
i64 encoder_l_count = 0,encoder_r_count = 0;
std::vector<int> els_temp(4,0),ers_temp(4,0);


int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;


void pit_callback(void)
{
    // els_temp[0]=els_temp[1];
    // els_temp[1]=els_temp[2];
    // els_temp[2]=els_temp[3];
    // els_temp[3]=encoder_dir_1.get_count();
    // encoder_dir_1.clear_count();
    // ers_temp[0]=ers_temp[1];
    // ers_temp[1]=ers_temp[2];
    // ers_temp[2]=ers_temp[3];
    // ers_temp[3]=-encoder_dir_2.get_count();
    // encoder_dir_2.clear_count();

    // encoder_l_speed = els_temp[0]+els_temp[1]+els_temp[2]+els_temp[3];
    // encoder_l_speed >>= 2;

    // encoder_r_speed = ers_temp[0]+ers_temp[1]+ers_temp[2]+ers_temp[3];
    // encoder_r_speed >>= 2;
    imu_acc_x = imu_dev.get_acc_x();
    imu_acc_y = imu_dev.get_acc_y();
    imu_acc_z = imu_dev.get_acc_z();

    imu_gyro_x = imu_dev.get_gyro_x();
    imu_gyro_y = imu_dev.get_gyro_y();


    //只有这个有用，向右为正
    imu_gyro_z = imu_dev.get_gyro_z();



    encoder_l_speed = encoder_dir_1.get_count();
    encoder_r_speed = -encoder_dir_2.get_count();
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();

    encoder_l_count += std::abs(encoder_l_speed);
    encoder_r_count += std::abs(encoder_r_speed);
    // std::cout << encoder_r_speed << std::endl;
}

int16 csp = 0,startspeedl = 1200,startspeedr = 1200;
gyropid pidgyro(3.0,0.1,0,-500,500,100);
int temp114514;
float kp = 2,ki = 1.0,kd = 1.0;
float kp1 = 2,ki1 = 1.0,kd1 = 1.0;
cv::Point nowmidpoint = cv::Point(-1,-1);
cv::Point carmidpoint = cv::Point(159.5,0);
int camlen = 40;
int lj = 31;
float msp = 110;
float P = 4.5;
void pit_callback1(void)
{
    //std::cout << encoder_l_count+encoder_r_count << std::endl;
    if(nowmidpoint.x >= 0 && nowmidpoint.y >= 0){
        cv::Point realmidpoint = cv::Point(nowmidpoint.x,239-nowmidpoint.y+camlen);
        cv::Point realdir = realmidpoint - carmidpoint;
        float r = (cv::norm(realdir)/2.0)/(realdir.x/cv::norm(realdir));
        if(std::abs(r)<1)return;
        float finalsp = msp * (1.0-0.3*std::abs(realdir.x/cv::norm(realdir)));
        // if(imu_acc_z > -3500)finalsp /= 2.0;
        // if(imu_acc_z < -4500)finalsp *= 2.0;
        float lsp = finalsp*(1.0+(lj/(2.0*r))*P);
        float rsp = finalsp*(1.0-(lj/(2.0*r))*P);
        lsp -= (imu_gyro_z / finalsp)*1.4;
        rsp += (imu_gyro_z / finalsp)*1.4;
        // ips200.show_float(0,244,lsp,3,3);
        // ips200.show_float(64,244,rsp,3,3);
        // std::cout << lsp << "|" << rsp << std::endl;
        ctrl.run(lsp,rsp);
    }


}
float adc_scale = 0.0;
void pit_callback2(void){

    ips200.show_float(0,304,battery_adc.convert() * adc_scale * 0.011,2,6);
}

RoadShapeClassifier classifier;
int main(int, char**) 
{
	std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    adc_scale = battery_adc.get_scale();
    encoder_l_count = 0;
    encoder_r_count = 0;
    imgprocess imgpcs;
    if (!classifier.init()) {
        std::cout << "模型初始化失败" << std::endl;
    }else{
        std::cout << "模型初始化完成" << std::endl;
    }
    {
        using json = nlohmann::json;
        std::ifstream ifs("/home/root/byd_2026_5_24.json");
        json data;
        try {
            data = json::parse(ifs);
        } catch (const std::exception& e) {
            std::cerr << "JSON 解析错误: " << e.what() << std::endl;
            return 1;
        }
        if (!data.is_array()) {
            std::cerr << "JSON 顶层不是数组" << std::endl;
            return 1;
        }
        int total = 0, correct = 0;
        for (const auto& item : data) {
            // 读取标签
            std::string true_label = item["label"].get<std::string>();

            // 读取点序列（64个点）
            auto pts_json = item["point"];
            if (pts_json.size() != 64) {
                std::cerr << "跳过样本，点数量不为64" << std::endl;
                continue;
            }

            std::vector<cv::Point> points;
            points.reserve(64);
            for (const auto& p : pts_json) {
                points.emplace_back(
                    static_cast<int>(p[0].get<float>()),
                    static_cast<int>(p[1].get<float>())
                );
            }

            // 执行推理
            std::vector<float> probs = classifier.run(points);
            if (probs.empty()) {
                std::cerr << "推理失败" << std::endl;
                continue;
            }

            // 输出结果
            std::cout << "真实标签: " << true_label << std::endl;
            std::cout << "预测概率:" << std::endl;
            for (int i = 0; i < 6; ++i) {
                printf("  %-20s: %.4f\n", classifier.labels[i], probs[i]);
            }
            std::cout << std::endl;

            total++;
            // 计算预测标签
            int pred_idx = std::max_element(probs.begin(), probs.end()) - probs.begin();
            if (classifier.labels[pred_idx] == true_label) {
                correct++;
            }
        }
        if (total > 0) {
            float acc = 100.0f * correct / total;
            printf("\n总样本: %d  正确: %d  准确率: %.2f%%\n", total, correct, acc);
        } else {
            std::cout << "没有有效样本" << std::endl;
        }
    }
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    i64 licheng = 0;
	ips200.init(FB_PATH);
    imu_dev.init();
	if(uvc_dev.init(UVC_PATH) < 0){
		std::cerr << "你妈的摄像头插了吗？" << std::endl;
        return -1;
    }
    cv::Mat rgb;
    cv::Mat gray(cv::Size(320,240),CV_8UC1);
    cv::Mat rgb_show;
    std::vector<int> midlinepoint(240);
    int status = 0;

    int if_huandao = 0;
    drv8701e_dir_1.set_level(0);
    drv8701e_dir_2.set_level(1);

    pit_timer.init_ms(10, pit_callback);
    pit_timer1.init_ms(10, pit_callback1);
    pit_timer2.init_ms(1000, pit_callback2);
    auto _start = std::chrono::high_resolution_clock::now();
    auto _end   = std::chrono::high_resolution_clock::now();
    auto _dur   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
    // auto _start1 = std::chrono::high_resolution_clock::now();
    // auto _end1   = std::chrono::high_resolution_clock::now();
    // auto _dur1   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
    while(1){
        //std::cout << encoder_l_count+encoder_r_count << std::endl;
        _end   = std::chrono::high_resolution_clock::now();
        _dur   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
        // std::cout << "runtime: " << _dur << " ns" << std::endl;  
        _start = std::chrono::high_resolution_clock::now();

        status = 0;
        //while(uvc_dev.wait_image_refresh());
            //         _end1   = std::chrono::high_resolution_clock::now();
            // _dur1   = std::chrono::duration_cast<std::chrono::nanoseconds>(_end1 - _start).count();
            // std::cout << "pretime: " << _dur1 << " ns" << std::endl;
        if(uvc_dev.wait_image_refresh() == 0){
            rgb = uvc_dev.get_frame_mjpg();
            for(int i = 0;i < 320;i++){
                for(int j = 0;j < 240;j++){
                    gray.at<uchar>(j,i) = std::min(rgb.at<cv::Vec3b>(j,i)[1],rgb.at<cv::Vec3b>(j,i)[2]);
                }
            }
            // cv::Mat r,g;
            // cv::extractChannel(rgb,r,2);
            // cv::extractChannel(rgb,g,1);
            //cv::cvtColor(rgb,gray,cv::COLOR_BGR2GRAY);
            // cv::Mat reshaped = rgb.reshape(1, rgb.rows * rgb.cols);
            // cv::reduce(reshaped, gray, 1, cv::REDUCE_MIN);
            // gray = gray.reshape(1, rgb.rows);
            // cv::min(r,g,gray);
            // gray = r;
            //cv::extractChannel(rgb,gray,2);
            // uint16* rgb_image = uvc_dev.get_rgb_image_ptr();

            // if(NULL != rgb_image){
            //     // cv::resize(rgb,rgb_show,cv::Size(240,180));
            //     // cv::cvtColor(rgb_show,rgb_show,cv::COLOR_BGR2BGR565);
            //     // ips200.displayimage_rgb565(reinterpret_cast<uint16*>(rgb_show.ptr(0)), 240, 180);

            // }
            //std::cout << "dfptime";
            //TIME_IT_NS(ips200.draw_point(1,1,RGB565_BLACK););
            //TIME_IT_NS();

            status = imgpcs.run(rgb,gray);
            //nowmidpoint = cv::Point(192,192);
            if(!imgpcs.finalmidline.empty()){
                nowmidpoint = imgpcs.finalmidline.at(0);
                //nowmidpoint = cv::Point(192,192);
            }
            
            //std::cout << 1 << std::endl;
            if(key_4.get_level() == 0){
                system_delay_ms(100);
                if(key_4.get_level() == 0){
                    cleanup();
                    return 0;
                }
            }
            //std::cout << 1 << std::endl;
            if(status < 0){
                // cleanup();
            }
            // if(status == 1){//左环岛
            //     ips200.show_string(0,32,"lhd1");
            //     if(licheng == 0){
            //         licheng = (encoder_l_count + encoder_r_count)/2;
            //     }else{
            //         if((encoder_l_count + encoder_r_count)/2 - licheng > 5000){
            //             status = 0;
            //             licheng = 0;
            //         }
            //     }
            // }
            // if(status == 2){//右环岛
            //     ips200.show_string(0,32,"rhd1");
            //     if(licheng == 0){
            //         licheng = (encoder_l_count + encoder_r_count)/2;
            //     }else{
            //         if((encoder_l_count + encoder_r_count)/2 - licheng > 5000){
            //             status = 0;
            //             licheng = 0;
            //         }
            //     }
            // }


            
            // for(int i = 0;i < 240;i++){
            //     midlinepoint.at(i) = (int)((imgpcs.finallinel.at(i) + imgpcs.finalliner.at(i))/2);
            //     //ips200.draw_point(midlinepoint.at(i)*3/4,i*3/4,RGB565_PURPLE);
            // }
            // //midline = 150;
            // midline = (imgpcs.finallinel.at(160) + imgpcs.finalliner.at(180))/2.0;






            // std::cout << "midline" << midline << std::endl;

            //control((imgpcs.finallinel.at(160) + imgpcs.finalliner.at(160))/2.0);


        }
        system_delay_ms(15);
        // ips200.clear();
    }
}