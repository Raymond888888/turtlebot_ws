#ifndef PID_H_
#define PID_H_

#include <stdint.h>

enum PID_Mode_e { PID_POSITION = 0, PID_DELTA, PID_COMP_POSITION };

#pragma pack(1)
typedef struct PID_config_t {
    float KP;
    float KI;
    float KD;
    float KP_fine;
    float range_rough;
    float range_fine;
    float error_max;
    float outputMax;
    float compensation;
    float error_preload;
    enum PID_Mode_e PID_Mode;
} pid_config;

typedef struct PID_t {
    struct PID_config_t config;
    float error[3];
    float error_sum;
    float fdb;
    float ref;
    float output;
    float output_unlimited; // 经outputMax限制前的原始输出
    float error_delta;
} pid;
#pragma pack()


void PID_Init(struct PID_t* pid, struct PID_config_t* config) { pid->config = *config; }

void PID_Calc(struct PID_t* pid) {
    pid->error[2] = pid->error[1];        //上上次误差
    pid->error[1] = pid->error[0];        //上次误差
    pid->error[0] = pid->ref - pid->fdb;  //本次误差

    if (pid->config.PID_Mode == PID_POSITION)  //位置式PID
    {
        pid->error_sum += pid->error[0];  //积分上限判断
        if (pid->error_sum > pid->config.error_max) pid->error_sum = pid->config.error_max;
        if (pid->error_sum < -pid->config.error_max) pid->error_sum = -pid->config.error_max;

        pid->output = pid->config.KP * pid->error[0] + pid->config.KI * pid->error_sum + pid->config.KD * (pid->error[0] - pid->error[1]);
    }

    else if (pid->config.PID_Mode == PID_DELTA)  //增量式PID
    {
        pid->output += pid->config.KP * (pid->error[0] - pid->error[1]) + pid->config.KI * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->config.KI * pid->error[0];
    }

    else if (pid->config.PID_Mode == PID_COMP_POSITION)  // PI分离模式 用于摩擦轮和拨弹
    {
        // pid->error_delta = pid->error[0] - pid->error[1];
        // if (pid->error[0] > pid->config.range_rough)  // bangbang
        // {
        //     pid->output = pid->config.outputMax;
        //     pid->error_sum = 0;
        // } else if (pid->error[0] < -pid->config.range_rough)  // bangbang
        // {
        //     pid->output = -pid->config.outputMax;
        //     pid->error_sum = 0;
        // } else if (fabsf(pid->error[0]) < pid->config.range_fine)  //细调
        // {
        //     pid->error_sum += pid->error[0];  //积分上限判断
        //     if (pid->error_sum > pid->config.error_max) pid->error_sum = pid->config.error_max;
        //     if (pid->error_sum < -pid->config.error_max) pid->error_sum = -pid->config.error_max;
        //     pid->output = pid->config.KP_fine * pid->error[0] + pid->config.KI * pid->error_sum + pid->config.KD * pid->error_delta;
        // } else  //粗调
        // {
        //     pid->output = pid->config.KP * (pid->error[0] + fsgn(pid->error[0]) * pid->config.compensation) + pid->config.KD * pid->error_delta;
        //     pid->error_sum = fsgn(pid->error[0]) * pid->config.error_preload;
        // }
    }

    /*----- 输出上限 -----*/
    pid->output_unlimited = pid->output;
    if (pid->output > pid->config.outputMax) pid->output = pid->config.outputMax;
    if (pid->output < -pid->config.outputMax) pid->output = -pid->config.outputMax;
}

void PID_SetConfig_Pos(struct PID_config_t* obj, float kp, float ki, float kd, float errormax, float outputmax) {
    obj->PID_Mode = PID_POSITION;
    obj->KP = kp;
    obj->KI = ki;
    obj->KD = kd;
    obj->error_max = errormax;
    obj->outputMax = outputmax;
}

// PI分离
void PID_SetConfig_Comp(struct PID_config_t* obj, float kp_rough, float kp_fine, float ki, float kd, float rangerough, float rangefine, float compensation, float errorpreload, float errormax,
                        float outputmax) {
    obj->PID_Mode = PID_COMP_POSITION;
    obj->KP = kp_rough;
    obj->KP_fine = kp_fine;
    obj->KI = ki;
    obj->KD = kd;
    obj->range_rough = rangerough;
    obj->range_fine = rangefine;
    obj->compensation = compensation;   //位置环补偿
    obj->error_preload = errorpreload;  //位置环积分预载
    obj->error_max = errormax;
    obj->outputMax = outputmax;
}
#endif
