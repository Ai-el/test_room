#include "foc_ctrl.h"


#include <stdio.h>
#include <math.h>

// 定义电机参数
#define R 1.0   // 电阻
#define L 0.1   // 电感
#define Ke 0.01 // 电动势常数

// 定义 PI 值
#define PI 3.14159265359

// 定义控制相关参数
#define Ts 0.001  // 控制周期
#define Kp_d 0.1   // d 轴电流控制增益
#define Ki_d 0.01  // d 轴电流控制积分增益
#define Kp_q 0.1   // q 轴电流控制增益
#define Ki_q 0.01  // q 轴电流控制积分增益

// 定义电流变量
double id_ref = 1.0;  // d 轴电流给定值
double iq_ref = 0.0;  // q 轴电流给定值
double id = 0.0;      // d 轴电流反馈值
double iq = 0.0;      // q 轴电流反馈值

// 定义 Clarke 变换
void clarke_transform(double ia, double ib, double *i_alpha, double *i_beta) {
    *i_alpha = ia;
    *i_beta = (2.0 * ib - ia) / sqrt(3.0);
}

// 定义 Park 变换
void park_transform(double theta, double i_alpha, double i_beta, double *id, double *iq) {
    *id = i_alpha * cos(theta) + i_beta * sin(theta);
    *iq = -i_alpha * sin(theta) + i_beta * cos(theta);
}

// FOC 控制
void FOC_control(double id_ref, double iq_ref, double id, double iq, double *vd, double *vq) {
    static double id_sum = 0.0;
    static double iq_sum = 0.0;
    
    double error_d = id_ref - id;
    double error_q = iq_ref - iq;
    
    id_sum += error_d * Ts;
    iq_sum += error_q * Ts;
    
    *vd = Kp_d * error_d + Ki_d * id_sum;
    *vq = Kp_q * error_q + Ki_q * iq_sum;
}

int REV_I() {
    // 模拟电流反馈值
    double ia = 1.0;
    double ib = 0.5;
    
    // Clarke 变换
    double i_alpha, i_beta;
    clarke_transform(ia, ib, &i_alpha, &i_beta);
    
    // Park 变换
    double theta = 0; // 电机角度
    double id, iq;
    park_transform(theta, i_alpha, i_beta, &id, &iq);
    
    // FOC 控制
    double vd, vq;
    FOC_control(id_ref, iq_ref, id, iq, &vd, &vq);
    
    printf("输出d轴电压: %f\n", vd);
    printf("输出q轴电压: %f\n", vq);
    
    return 0;
}



#define PWM_PERIOD 1000 // PWM周期（总时间）
#define PI 3.1415926

// SVM算法-输入alpha和beta的值，输出三个相位的PWM占空比
void svm(double alpha, double beta, int* pwm_a, int* pwm_b, int* pwm_c)
{
    double u_alpha, u_beta;
    double t_temp; // 临时参数

    // 计算空间矢量的长度
    double u_max = sqrt(alpha*alpha + beta*beta); 

    // 计算空间矢量的角度（相对于alpha轴）
    double theta = atan2(beta, alpha);

    // 根据空间矢量的长度u和角度theta，计算占空比
    if (u_max <= 0.5*PWM_PERIOD)
    {
        // 占空比小于等于0.5时，采用三角波调制
        t_temp = PWM_PERIOD * (PI / 3 - theta) / PI;
        *pwm_a = (int)(PWM_PERIOD/2 - t_temp);
        *pwm_b = (int)(PWM_PERIOD/2 + cos(PI/3) * t_temp);
        *pwm_c = (int)(PWM_PERIOD/2 + cos(PI/3) * t_temp);
    }
    else
    {
        // 占空比大于0.5时，采用定步进调制
        t_temp = alpha / u_max;
        *pwm_a = (int)(PWM_PERIOD/2 - t_temp*PWM_PERIOD/2);
        t_temp = (beta - alpha*sin(PI/3)) / u_max;
        *pwm_b = (int)(PWM_PERIOD/2 - t_temp*PWM_PERIOD/2);
        *pwm_c = PWM_PERIOD - *pwm_a - *pwm_b;
    }
}

int FOC_out()
{
    // 示例输入的alpha和beta值
    double alpha = 0.6;
    double beta = 0.8;

    // 输出的三个相位的PWM占空比
    int pwm_a, pwm_b, pwm_c;

    // 调用SVM函数，实现PWM占空比计算
    svm(alpha, beta, &pwm_a, &pwm_b, &pwm_c);

    // 输出结果
    printf("PWM A: %d\n", pwm_a);
    printf("PWM B: %d\n", pwm_b);
    printf("PWM C: %d\n", pwm_c);

    return 0;
}