/*
 *        File: JOGmode.h
 *     Version:
 * Description:
 *
 *  Created on: 2018��4��18��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_JOGMODE_H_
#define SRC_CONTROL_MOTIONMODE_JOGMODE_H_

typedef struct
{
    double acc;         //��λ�˶��ļ��ٶȡ���������λ��pulse/ms^2
    double dec;         //��λ�˶��ļ��ٶȡ���������λ��pulse/ms^2
    double smooth;      //ƽ��ϵ����ȡֵ��Χ��[0��1)��ƽ��ϵ������ֵԽ�󣬼��ٶȹ���Խ�ȶ�
}TJogPrm;


typedef struct
{
    long t[3];                  // t0 t1 t2 ����ʱ��
    double pos[3];              // ÿһ�εĳ�ʼ������
    double turnVel[4];          // ÿһ�ε�ת���ٶ��ϣ�����ƽ�������ߴ���
    double realstartVel;        // ʵ�������ٶ�
    double realacc;             // ʵ�ʼ��ٶ�
    double downVel;             // ����ʱ�ĳ�ʼ�ٶȣ�ת��ʱ����ٶ�

    long  objPos;               // Ŀ��λ��(PP�е�����ٶȣ�Jog�е�Ŀ���ٶ�)
    double objVel;              // Ŀ���ٶȡ���λ��pulse/ms��
    double acc;                 // ��λ�˶�ʱ��ļ��ٶȡ���������λ��pulse/ms^2��
    double dec;                 // ��λ�˶��ļ��ٶȡ���������λ��pulse/ms^2��
    double startVel;            // �����ٶȡ���������λ��pulse/ms��
    double endVel;              // �趨�ٶȡ���λ��pulse/ms��
    short  smoothTime;          // ƽ��ʱ�䡣��������ȡֵ��Χ��[0��50]����λms��
    double smooth;              // ƽ��ϵ����ȡֵ��Χ��[0��1)��
}PP_JOG_KERNEL;


extern void Run_JOGmode(int axis);
extern void Decouple_JOGmode();
extern ERROR_CODE Prep_JOGmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_JOGMODE_H_ */
