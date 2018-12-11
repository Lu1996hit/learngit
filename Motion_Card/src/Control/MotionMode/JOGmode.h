/*
 *        File: JOGmode.h
 *     Version:
 * Description:
 *
 *  Created on: 2018年4月18日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_JOGMODE_H_
#define SRC_CONTROL_MOTIONMODE_JOGMODE_H_

typedef struct
{
    double acc;         //点位运动的加速度。正数，单位：pulse/ms^2
    double dec;         //点位运动的减速度。正数，单位：pulse/ms^2
    double smooth;      //平滑系数。取值范围：[0，1)。平滑系数的数值越大，加速度过程越稳定
}TJogPrm;


typedef struct
{
    long t[3];                  // t0 t1 t2 运行时间
    double pos[3];              // 每一段的初始长度上
    double turnVel[4];          // 每一段的转折速度上（用在平滑化曲线处理）
    double realstartVel;        // 实际起跳速度
    double realacc;             // 实际加速度
    double downVel;             // 减速时的初始速度，转折时候的速度

    long  objPos;               // 目标位置(PP中的最大速度，Jog中的目标速度)
    double objVel;              // 目标速度。单位：pulse/ms。
    double acc;                 // 点位运动时间的加速度。正数，单位：pulse/ms^2。
    double dec;                 // 点位运动的减速度。正数，单位：pulse/ms^2。
    double startVel;            // 起跳速度。正数，单位：pulse/ms。
    double endVel;              // 设定速度。单位：pulse/ms。
    short  smoothTime;          // 平滑时间。正参数，取值范围：[0，50]，单位ms。
    double smooth;              // 平滑系数。取值范围：[0，1)。
}PP_JOG_KERNEL;


extern void Run_JOGmode(int axis);
extern void Decouple_JOGmode();
extern ERROR_CODE Prep_JOGmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_JOGMODE_H_ */
