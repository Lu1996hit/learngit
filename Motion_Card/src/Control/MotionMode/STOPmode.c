/*
 *        File: STOPmode.c
 *     Version:
 * Description: ����ʵ�����ֹͣ��ֹͣ�����е��˶��з������ȼ�����ߵ�
 *
 *  Created on: 2018��8��23��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

/*
 * ֱ���˶��ϵ�ֹͣ��
 * Բ��ģʽ�ϵ�ֹͣ�������Ҫ�ر�ע�⣬ֹͣ��ʱ�򣬹켣ҲҪ����Բ����
 */


#include "system.h"
#include "taskComm.h"
#include <Kernel/interpolate.h>

#include "STOPmode.h"


/*
 * ��ǰʹ�õ�һЩ����
 */
typedef struct
{
	unsigned int type;					//ֹͣ����
										// 0�� ƽ��ֹͣ
										// 1�� ��ͣ
	double decSmoothStop;				//ƽ��ֹͣ���ٶ�
	double decAbruptStop;				//��ͣ���ٶ�
	double start_pos;					//��ʾλ��
	double end_pos;						//�յ�λ��
	double real_acc;					//ʵ�ʼ��ٶȣ���������
	double start_vel;					//��ʼ�ٶ�
	long t;								//����ʱ��
	MOTORS_STA last_state;				//��һ����״̬
}STOP_KERNEL;

static STOP_KERNEL stopkernel[AXISNUM];

/*
 * ��ʼ������ģʽ
 * ��ֵ��д��һЩĬ�ϵĲ���
 */
void Init_Stop_Kernel()
{
	int count;
	STOP_KERNEL* pStopKer;
	pStopKer = stopkernel;
	for(count = 0; count < AXISNUM; count ++)
	{
		pStopKer->type = 0;				//Ĭ������Ϊƽ��ֹͣ
		pStopKer->decSmoothStop = 0.01;
		pStopKer->decAbruptStop = 0.1;
		pStopKer->start_pos = 0;
		pStopKer->end_pos = 0;
		pStopKer->real_acc = 0;
		pStopKer->last_state = MOTORS_STA_IDLE;
		pStopKer ++;
	}
	return;
}

/*
 * ����ƽ��ֹͣ���ٶȺͼ�ͣ���ٶ�
 */
static ERROR_CODE SetStopDec()
{
	int axis;
	STOP_KERNEL* pStopKer;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			pStopKer = &stopkernel[axis];
			memcpy(&(pStopKer->decSmoothStop), &cmd.prm[3], sizeof(double)*2);

			return RTN_SUCC;
		}
	}
	return RTN_SUCC;
}

/*
 * ��ȡƽ��ֹͣ���ٶȺͼ�ͣ���ٶ�
 */
static ERROR_CODE GetStopDec()
{
	int axis;
	STOP_KERNEL* pStopKer;

	unsigned int *pBuf = cmd.buf;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			pStopKer = &stopkernel[axis];
			memcpy(pBuf, &(pStopKer->decSmoothStop), sizeof(double)*2);
			cmd.buflen += sizeof(double)*2;
			return RTN_SUCC;
		}
	}
	return RTN_SUCC;
}

/*
 * ֹͣһ�����߶����Ĺ滮�˶���ֹͣ����ϵ�˶�
 *
 */
static ERROR_CODE Stop()
{
	int axis;
	KernelPrm *pKer;
	STOP_KERNEL *pStopKer;
	for(axis = 0; axis < AXISNUM; axis ++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			pKer = &kernel[axis];
			pStopKer = &stopkernel[axis];

			if((cmd.prm[3] >> axis) & 0x01)
			{
				pStopKer->type = 1;
			}
			else
			{
				pStopKer->type = 0;
			}

			pKer->flag = 1;				//��־λ��1����ʼ����
			//�����ظ���ֹͣ
			if(pStopKer->last_state == MOTORS_STA_STOPMODE)
				break;
			pStopKer->last_state = pKer->kersta;
			pKer->axsta = MOTORS_STA_STOPMODE;
			pKer->kersta = MOTORS_STA_STOPMODE;			//������ɺ���ΪSTOPMODE
			pKer->step = 0;
			pKer->count = 0;
		}
	}
	return RTN_SUCC;
}

/*
 * �滮ֹͣ�˶�
 * 1.������ֱ��һ�����µ�ֹͣ��������ϵ������µ�ֹͣ
 */
ERROR_CODE Prep_STOPmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	STOP_KERNEL *pStopKer = &stopkernel[axis];
	//����Ƿ���Ҫ���¹滮
	if(pKer->flag == 0)
		return RTN_SUCC;

	switch(pStopKer->last_state)
	{
	/*
	 * ����ģʽ�²���Ҫ����ֹͣ��ֱ���˳�����
	 */
	case MOTORS_STA_IDLE:
	{
		pKer->flag = 0;
		pKer->axsta = MOTORS_STA_IDLE;
		pKer->kersta = MOTORS_STA_IDLE;
		break;
	}

	//���������ģʽ����ֱ���˶���Ӧ�ĵ�����򵥵�ֱ��ֹͣ
	case MOTORS_STA_PPMODE:
	case MOTORS_STA_JOGMODE:
	case MOTORS_STA_PTMODE:
	{
		if(pKer->kersta == MOTORS_STA_IDLE)			//û�����˶���ֱ���˳��Ϳ�����
		{
			pKer->axsta = MOTORS_STA_IDLE;
			pKer->flag = 0;
			return RTN_SUCC;
		}

		double t;
		double acc;			//���ɼ��ٶ�
		double distance;

		if(pStopKer->type == 0)
		{
			acc = pStopKer->decSmoothStop;
		}
		else
		{
			acc = pStopKer->decAbruptStop;
		}

		pStopKer->start_pos = pKer->realPos;

		if(pKer->nowVel >= 0)
		{
			pKer->dir = 1;			//���򣬼��ٶ�Ϊ����
			t = pKer->nowVel / acc;
			distance = 0.5*acc*t*t;
			pStopKer->end_pos = pStopKer->start_pos + distance;
			pStopKer->real_acc = -acc;
		}
		else
		{
			pKer->dir = 0;			//���򣬼��ٶ�Ϊ����
			t = -pKer->nowVel / acc;
			distance = 0.5*acc*t*t;
			pStopKer->end_pos = pStopKer->start_pos - distance;
			pStopKer->real_acc = acc;
		}

		pStopKer->start_vel = pKer->nowVel;
		pStopKer->t = Approximate(t * 10);
		pKer->flag = 0;
		pKer->count = 0;
		pKer->step = 0;
		break;
	}
	//����������ϵ�µ�ֹͣ,�ⲿ�ַŵ�����ϵģʽ�µ�������
	case MOTORS_STA_CRDMODE:
	case MOTORS_STA_CRDAUX:
	{
		break;
	}
	case MOTORS_STA_STOPMODE:
	{
		pKer->flag = 0;
		break;
	}
	default: 						break;
	}
	return RTN_SUCC;
}

/*
 * ����ָ��
 */
void Decouple_STOPmode()
{
	switch(cmd.type)
	{
		case CMD_SET_STOP_DEC:			cmd.rtn = SetStopDec();					break;
		case CMD_GET_STOP_DEC:			cmd.rtn = GetStopDec();					break;
		case CMD_STOP_MOVE:				cmd.rtn = Stop();						break;
		default:						cmd.rtn = RTN_INVALID_COMMAND;			break;
	}
}

/*
 * ֹͣģʽ�˶�����
 */
void Run_STOPmode(int axis)
{
	long pos;
	PVAT_S pvat;
	KernelPrm *pKer = &kernel[axis];
	STOP_KERNEL *pStopKer = &stopkernel[axis];

	//δ��������ݣ�ֱ������
	if(pKer->flag == 1)
		return;

	//return;
	switch(pStopKer->last_state)
	{
	case MOTORS_STA_IDLE:		//��������²�������������
	{
		break;
	}

	//���������ģʽ����ֱ���˶���Ӧ�ĵ�����򵥵�ֱ��ֹͣ
	case MOTORS_STA_PPMODE:
	case MOTORS_STA_JOGMODE:
	case MOTORS_STA_PTMODE:
	{
		pKer->count ++;
		if(pKer->count >= pStopKer->t)		//���һ��
		{
			pKer->nowVel = 0;
			pKer->nowacc = 0;
			pKer->count = 0;
			pKer->kersta = MOTORS_STA_IDLE;
			pKer->axsta = MOTORS_STA_IDLE;
			pKer->realPos = pStopKer->end_pos;
		}
		else
		{

			pKer->nowacc = pStopKer->real_acc;
			pKer->nowVel = pKer->nowVel + (pStopKer->real_acc) * 0.1;
			pKer->realPos = pStopKer->start_pos + pStopKer->start_vel*pKer->count*0.1 + pStopKer->real_acc*pKer->count*pKer->count*0.005;
		}

		pos = Approximate(pKer->realPos);
		pvat.aim_pos = pos;
		pvat.start_acc = 0;
		pvat.start_vel = (pos - pKer->nowPos)*10000;
		pvat.min_period = TIME;
		M_SetPvat(axis, &pvat);
		pKer->nowPos = pvat.aim_pos;
		break;
	}
	//����������ϵ�µ�ֹͣ,�ⲿ�ַŵ�����ϵģʽ�µ�������
	case MOTORS_STA_CRDMODE:
	case MOTORS_STA_CRDAUX:
	{
		break;
	}
	case MOTORS_STA_STOPMODE:
	{
		pKer->flag = 0;
		break;
	}
	default: 						break;
	}

	return;
}
