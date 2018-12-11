/*
 *        File: Crdmode.h
 *     Version:
 * Description:
 *
 *  Created on: 2018��7��20��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_CRDMODE_H_
#define SRC_CONTROL_MOTIONMODE_CRDMODE_H_

typedef struct CrdPrm
{
	short dimension;					//1 * 1 = 1
	short axisIndex[4];					//1 * 4 = 4
	double synVelMax;					//2 * 1 = 2
	double synAccMax;					//2 * 1 = 2
	short evenTime;						//1 * 1 = 1
	short setOriginFlag;				//1 * 1 = 1
	long originPos[4];					//2 * 4 = 8
										//sum = 1 + 4 + 2 + 2 + 1 + 1 + 8 = 19 + 1 = 20
}TCrdPrm;

/*
 * dimension������ϵ��ά����ȡֵ��Χ��[2,4]��
 * axisIndex[4]������ϵ��滮����ӳ���ϵ��axisIndex[0..3]��Ӧ�滮��XYZA��ȡֵ��Χ[0,7]
 * synVelMax��������ϵ�����ϳ��ٶȡ�����û�������岹�ε�ʱ�������õ�Ŀ���ٶȴ����˸��ٶȣ���
 * �ᱻ����Ϊ���ٶȡ�ȡֵ��Χ��(0,32767)����λ��pulse/ms��
 * synAccMax��������ϵ�����ϳɼ��ٶȡ�����û�������岹�ε�ʱ�������õļ��ٶȴ����˸ü��ٶȣ���
 * ���ᱻ���Ƴɸü��ٶȡ�ȡֵ��Χ��(0,32767)����λ��pulse/ms^2��
 * evenTime��ÿ���岹�ε���С���ٶ�ʱ�䡣ȡֵ��Χ��[0,32767)����λ��ms��
 * setOriginFlag����ʾ�Ƿ���Ҫָ������ϵ��ԭ������Ĺ滮λ�ã��ò������Է����û����������ڻ�������
 * ϵ�ļӹ�����ϵ��0������Ҫָ��ԭ������ֵ��������ϵ��ԭ���ڵ�ǰ�滮λ���ϡ�1����Ҫָ��ԭ������ֵ��
 * ����ϵ��ԭ����originPosָ���Ĺ滮λ���ϡ�
 * originPos[4]��ָ��������ϵԭ��Ĺ滮λ��ֵ��
 */

extern void Init_Crd_Kernel();
extern void Decouple_CRDmode();
extern void Run_CRDmode(int axis);
extern ERROR_CODE Prep_CRDmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_CRDMODE_H_ */
