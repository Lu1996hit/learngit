/*
 *        File: PPmode.h
 *     Version:
 * Description: 点位模式
 *
 *  Created on: 2018年4月13日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_PPMODE_H_
#define SRC_CONTROL_MOTIONMODE_PPMODE_H_

//typedef struct
//{
//	//此处顺序禁止改动,但是可以注释掉
//	double acc;						//点位运动时间的加速度。正数，单位：pulse/ms^2。											// 2
//	double dec;						//点位运动的减速度。正数，单位：pulse/ms^2。												// 2
//	double velStart;				//起跳时间。正数，单位：pulse/ms。														// 2
//	short  smoothTime;				//平滑时间。正参数，取值范围：[0，50]，单位ms。平滑时间的数值越大，加减速过程越平稳。		// 1
//}TTrapPrm;

typedef struct
{
    long t[3];                  // t0 t1 t2 运行时间
//    double pos[3];              // 每一段的初始长度上
    double turnVel[4];          // 每一段的转折速度上（用在平滑化曲线处理）
    double realstartVel;        // 实际起跳速度
//    double realacc;             // 实际加速度
//    double downVel;             // 减速时的初始速度，转折时候的速度

    long  objPos;               // 目标位置(PP中的最大速度，Jog中的目标速度)
    double objVel;              // 目标速度。单位：pulse/ms。
    double acc;                 // 点位运动时间的加速度。正数，单位：pulse/ms^2。
    double dec;                 // 点位运动的减速度。正数，单位：pulse/ms^2。
    double startVel;            // 起跳速度。正数，单位：pulse/ms。
    double endVel;              // 设定速度。单位：pulse/ms。
    short  smoothTime;          // 平滑时间。正参数，取值范围：[0，50]，单位ms。
}PP_KERNEL;


extern void Init_PP_JOG_Kernel();			//初始化
extern void Run_PPmode();					//中断中运行
extern void Decouple_PPmode();				//解码
extern ERROR_CODE Prep_PPmode(int axis);	//准备

#endif /* SRC_CONTROL_MOTIONMODE_PPMODE_H_ */
