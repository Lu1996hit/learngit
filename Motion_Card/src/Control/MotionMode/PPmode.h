/*
 *        File: PPmode.h
 *     Version:
 * Description: ��λģʽ
 *
 *  Created on: 2018��4��13��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_PPMODE_H_
#define SRC_CONTROL_MOTIONMODE_PPMODE_H_

//typedef struct
//{
//	//�˴�˳���ֹ�Ķ�,���ǿ���ע�͵�
//	double acc;						//��λ�˶�ʱ��ļ��ٶȡ���������λ��pulse/ms^2��											// 2
//	double dec;						//��λ�˶��ļ��ٶȡ���������λ��pulse/ms^2��												// 2
//	double velStart;				//����ʱ�䡣��������λ��pulse/ms��														// 2
//	short  smoothTime;				//ƽ��ʱ�䡣��������ȡֵ��Χ��[0��50]����λms��ƽ��ʱ�����ֵԽ�󣬼Ӽ��ٹ���Խƽ�ȡ�		// 1
//}TTrapPrm;

typedef struct
{
    long t[3];                  // t0 t1 t2 ����ʱ��
//    double pos[3];              // ÿһ�εĳ�ʼ������
    double turnVel[4];          // ÿһ�ε�ת���ٶ��ϣ�����ƽ�������ߴ���
    double realstartVel;        // ʵ�������ٶ�
//    double realacc;             // ʵ�ʼ��ٶ�
//    double downVel;             // ����ʱ�ĳ�ʼ�ٶȣ�ת��ʱ����ٶ�

    long  objPos;               // Ŀ��λ��(PP�е�����ٶȣ�Jog�е�Ŀ���ٶ�)
    double objVel;              // Ŀ���ٶȡ���λ��pulse/ms��
    double acc;                 // ��λ�˶�ʱ��ļ��ٶȡ���������λ��pulse/ms^2��
    double dec;                 // ��λ�˶��ļ��ٶȡ���������λ��pulse/ms^2��
    double startVel;            // �����ٶȡ���������λ��pulse/ms��
    double endVel;              // �趨�ٶȡ���λ��pulse/ms��
    short  smoothTime;          // ƽ��ʱ�䡣��������ȡֵ��Χ��[0��50]����λms��
}PP_KERNEL;


extern void Init_PP_JOG_Kernel();			//��ʼ��
extern void Run_PPmode();					//�ж�������
extern void Decouple_PPmode();				//����
extern ERROR_CODE Prep_PPmode(int axis);	//׼��

#endif /* SRC_CONTROL_MOTIONMODE_PPMODE_H_ */
