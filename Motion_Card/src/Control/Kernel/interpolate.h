/*
 * 		  File: interpolate.h
 *     Version:
 * Description: 
 * 
 *  Created on: 2018��3��15��
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_INTERPOLATE_H_
#define SRC_CONTROL_INTERPOLATE_H_

#include "mymotors.h"


//�岹��������
//�������޸�
//#define TIME 5000			//0.1ms
#define TIME 10000			//0.1ms  ��Ƶ����

/*
 * �ں�״̬ö��
 */
typedef enum {
	IDLE_STA = 0,		//����״̬
	UP_STA ,			//ֱ�߲岹��ʱ����
	STEADY_STA ,		//ֱ�߲岹��ʱ����
	DOWN_STA , 			//ֱ�߲岹��ʱ����
	CIRCLE_STA ,		//Բ�ܲ岹��ʱ����
	UNCONTROL_STA ,		//����ĳЩ���ɿص��ǲ��ɿ�ʱ����ȷ����״̬
	GOHOME_STA,			//�����˶���״̬

}INTERPOLATE_COMMAND_TYPE;

/*
 * �����ʱ��ȡsin���㻹��ȡcos����
 */
typedef enum {
	COS_MODE = 0,
	SIN_MODE ,
}SIN_COS_COMMAND_TYPE;

/*
 * �岹�ں˽ṹ��
 * ������������state
 * ��һ�����״̬����ʾ�ѵ�ǰ��������Ϊĳ�ֹ���
 * �ڶ������״̬����ʾ��ǰ���������е�״̬���������IDLE״̬����ǰ��˵�����˶���
 */
typedef struct
{
	MOTORS_STA axsta;			//��ĳ�ʼ״̬		Ԥ�������У��������״̬
	MOTORS_STA kersta;			//�ں����е�״̬

	int flag;                   // �Ƿ���Ҫ���¹滮�ı�־ 1����Ҫ���¹滮 0������Ҫ���¹滮 �滮�����Ҫ��0
    int step;                   // ״̬��־λ����������ٶ����߹滮Ϊ���ɶ�
    long count;                 // ����ִ�м�������ÿ 0.1 ms ����1

    int axis;                   // ��ǰ���ָ��
    int dir;                    // ���з���     1������        0������

    long nowPos;                // ��ǰλ��
    //long aimPos;              // ��ǰָ���µ�Ŀ��λ��
    double realPos;             // ��һ������ʵλ��

    //����dir�����޸ĳɣ��������˶�ʱ��dir = 1��ʾ�����˶���dir = 2��ʾ�����˶������ᾲֹʱ��dir = 0��
    //����ͨ���ں˵�״̬���жϵ�ǰ�����ǲ��������˶�������Ҫ���˶���״̬�ڼ�¼��dir��

    double nowacc;              // ��ǰ���ٶ�
    double nowVel;              // ��ǰ�ٶȡ���λ��pulse/ms��
    double lastVel;             // ��һʱ���ٶ�
}KernelPrm;

extern KernelPrm kernel[AXISNUM];

long Approximate(double value);				//��������Ĵ���
extern void Init_Kernel();
extern void Run_Kernel();


#endif /* SRC_CONTROL_INTERPOLATE_H_ */
