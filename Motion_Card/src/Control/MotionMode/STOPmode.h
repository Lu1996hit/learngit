/*
 *        File: STOPmode.h
 *     Version:
 * Description: ʵ�����ֹͣ
 *
 *  Created on: 2018��8��23��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_STOPMODE_H_
#define SRC_CONTROL_MOTIONMODE_STOPMODE_H_


void Init_Stop_Kernel();			//��ʼ��ֹͣģʽ
void Decouple_STOPmode();			//����
ERROR_CODE Prep_STOPmode(int axis);
void Run_STOPmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_STOPMODE_H_ */
