#include "control.hpp"

gyropid::gyropid(float p, float i, float d,
         float min_out, float max_out, float int_limit)
    : kp(p), ki(i), kd(d),
      integral(0.0f),
      prev_error(0.0f),
      out_min(min_out),
      out_max(max_out),
      integral_limit(int_limit)
{
    // 确保限幅值逻辑正确
    if (out_min > out_max) {
        float tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }
    if (integral_limit < 0.0f) {
        integral_limit = -integral_limit;
    }
}

int16 gyropid::run(float target_gyro,int gyroz)
{
    // 1. 计算误差
    float error = target_gyro - gyroz;

    // 2. 积分项（累加并限幅）
    integral += error;
    if (integral > integral_limit) {
        integral = integral_limit;
    } else if (integral < -integral_limit) {
        integral = -integral_limit;
    }

    // 3. 微分项（误差的差分）
    float derivative = error - prev_error;

    // 4. PID 输出
    float output = kp * error + ki * integral + kd * derivative;

    // 5. 输出限幅
    if (output > out_max) {
        output = out_max;
    } else if (output < out_min) {
        output = out_min;
    }

    // 6. 保存本次误差供下次微分使用
    prev_error = error;

    // 7. 将占空比值写入硬件（值范围 0-10000）
    //drv8701e_pwm_1.set_duty(static_cast<int>(output));
    std::cout << output << std::endl;
    return static_cast<uint16>(output);
}
void gyropid::reset()
{
    integral   = 0.0f;
    prev_error = 0.0f;
}













// pid::pid(float p, float i, float d,
//          float min_out, float max_out, float int_limit)
//     : kp(p), ki(i), kd(d),
//       integral(0.0f),
//       prev_error(0.0f),
//       out_min(min_out),
//       out_max(max_out),
//       integral_limit(int_limit)
// {
//     // 确保限幅值逻辑正确
//     if (out_min > out_max) {
//         float tmp = out_min;
//         out_min = out_max;
//         out_max = tmp;
//     }
//     if (integral_limit < 0.0f) {
//         integral_limit = -integral_limit;
//     }
// }
// uint16 pid::run(float target_speed,int encoder_speed)
// {
//     // 1. 计算误差
//     float error = target_speed - encoder_speed;

//     // 2. 积分项（累加并限幅）
//     integral += error;
//     if (integral > integral_limit) {
//         integral = integral_limit;
//     } else if (integral < -integral_limit) {
//         integral = -integral_limit;
//     }

//     // 3. 微分项（误差的差分）
//     float derivative = error - prev_error;

//     // 4. PID 输出
//     float output = kp * error + ki * integral + kd * derivative;

//     // 5. 输出限幅
//     if (output > out_max) {
//         output = out_max;
//     } else if (output < out_min) {
//         output = out_min;
//     }

//     // 6. 保存本次误差供下次微分使用
//     prev_error = error;

//     // 7. 将占空比值写入硬件（值范围 0-10000）
//     //drv8701e_pwm_1.set_duty(static_cast<int>(output));
//     // std::cout << encoder_r_speed << std::endl;
//     return static_cast<uint16>(output);
// }
// void pid::reset()
// {
//     integral   = 0.0f;
//     prev_error = 0.0f;
// }


control::control(){
    maxl = 9999,maxr = 9999;
    elsl = encoder_l_speed;ersl = encoder_r_speed;

}
void control::run(float l_speed,float r_speed){
    // std::cout << encoder_l_speed << '\n' << encoder_r_speed << std::endl;
    // drv8701e_pwm_1.set_duty(pidl.run(160,encoder_l_speed));
    //drv8701e_pwm_2.set_duty(1500);//pidr.run(160,encoder_r_speed));
    // int fx;
    // if(midline - 160 >= 0)fx = 1;
    // else fx = -1;
    //speed /= 160.0;
    ld = encoder_l_speed - elsl;
    rd = encoder_r_speed - ersl;
    elsl = encoder_l_speed;ersl = encoder_r_speed;
    // midline = 160;
    // int l_speed = speed + fx*std::min(speed,(float)(std::pow(std::abs(midline - 160),1.0)*0.1)) - imu_gyro_z*0.01;
    // int r_speed = speed - fx*std::min(speed,(float)(std::pow(std::abs(midline - 160),1.0)*0.1)) + imu_gyro_z*0.01;
    // std::cout << l_speed << " " << r_speed << std::endl;
    lsp += (l_speed - encoder_l_speed)*8.0 - ld*30.0;//std::max(-100.0f,std::min(100.0f,ld));
    //maxl = std::min(9999,(int)std::abs(l_speed - encoder_l_speed)*100 + 2000);
    lsp = std::min(maxl,lsp);
    lsp = std::max(-maxl,lsp);
    rsp += (r_speed - encoder_r_speed)*8.0 - rd*30.0;//std::max(-100.0f,std::min(100.0f,rd));
    //maxr = std::min(9999,(int)std::abs(r_speed - encoder_r_speed)*100 + 2000);
    rsp = std::min(maxr,rsp);
    rsp = std::max(-maxr,rsp);
    // lsp = 2000;
    // rsp = 0;
    if(lsp < 0){
        drv8701e_dir_1.set_level(1);
    }else{
        drv8701e_dir_1.set_level(0);
    }
    if(rsp < 0){
        drv8701e_dir_2.set_level(0);
    }else{
        drv8701e_dir_2.set_level(1);
    }
    // show.clear();
    // for(int i = 0;i < 161;i++)show += ' ';
    // show.at(std::min(160,std::abs((int)(encoder_l_speed)/2))) = '+';
    // show.at(std::min(160,std::abs((int)(encoder_r_speed)/2))) = 'x';
    // std::cout << show << maxl << maxr << std::endl;
    // // std::cout << "-------------------------" << std::endl;
    // // std::cout << "speed" << encoder_l_speed << " " << encoder_r_speed << " " << std::endl;
    // // std::cout << "gotospeed" << l_speed << " " << r_speed << std::endl;
    // // std::cout << "+=" << (l_speed - encoder_l_speed)/2.0 - std::max(-100.0f,std::min(100.0f,ld*ld)) << (r_speed - encoder_r_speed)/2.0 - std::max(-100.0f,std::min(100.0f,rd*rd)) << std::endl;
    // // std::cout << "d" << ld << " " << rd << std::endl;  
    // // std::cout << "output" << lsp << " " << rsp << std::endl;
    // // std::cout << "-------------------------" << std::endl;
    drv8701e_pwm_1.set_duty(std::abs((int)lsp));
    drv8701e_pwm_2.set_duty(std::abs((int)rsp));
    // // drv8701e_pwm_1.set_duty(std::max(0LL,(long long)(500-10*(160-midline))));
    // // drv8701e_pwm_2.set_duty(std::min(2000LL,(long long)(500+10*(160-midline))));
}