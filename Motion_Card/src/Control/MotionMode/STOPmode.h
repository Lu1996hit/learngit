/*
 *        File: STOPmode.h
 *     Version:
 * Description: 实现轴的停止
 *
 *  Created on: 2018年8月23日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_STOPMODE_H_
#define SRC_CONTROL_MOTIONMODE_STOPMODE_H_


void Init_Stop_Kernel();			//初始化停止模式
void Decouple_STOPmode();			//解码
ERROR_CODE Prep_STOPmode(int axis);
void Run_STOPmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_STOPMODE_H_ */
