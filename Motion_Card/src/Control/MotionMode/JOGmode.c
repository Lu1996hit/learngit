/*
 *        File: JOGmode.c
 *     Version:
 * Description: Jog�˶�ģʽ
 * 				��Jog�˶�ģʽ�£�������Զ�������Ŀ���ٶȡ����ٶȡ����ٶȡ�ƽ
 * 				��ϵ�����˶��������ܹ������˶���ֹͣ��
 *
 *  Created on: 2018��4��18��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "system.h"
#include "taskComm.h"
#include "interpolate.h"
#include "JOGmode.h"

extern PP_JOG_KERNEL pjkernel[AXISNUM];

/*
 * �趨ָ����ΪJOGģʽ
 */
static ERROR_CODE PrfJog()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].kersta == MOTORS_STA_IDLE)		//��鵱ǰ���Ƿ�������
				kernel[axis].axsta = MOTORS_STA_JOGMODE;
			else
				return RTN_ERROR;
		}
	}
	return RTN_SUCC;
}

/*
 * ����Jog�˶�ģʽ�µ��˶�����
 */
static ERROR_CODE SetJogPrm()
{
	int axis;
	KernelPrm *pKer;
	PP_JOG_KERNEL *pPJ;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			pKer = &kernel[axis];
			if(pKer->axsta != MOTORS_STA_JOGMODE)
				return RTN_ERROR;

			pPJ = &pjkernel[axis];
			memcpy(&pPJ->acc,p,2);
			p += 2;
			memcpy(&pPJ->dec,p,2);
			p += 2;
			memcpy(&pPJ->smooth,p,2);
			p += 2;
			pKer->flag = 1;
		}
	}
	return RTN_SUCC;
}

static ERROR_CODE GetJogPrm()
{
	int axis;
	PP_JOG_KERNEL *pPJ;
	unsigned int *p = cmd.buf;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].axsta != MOTORS_STA_JOGMODE)
				return RTN_ERROR;

			pPJ = &pjkernel[axis];
			memcpy(p,&pPJ->acc,2);
			p += 2;
			memcpy(p,&pPJ->dec,2);
			p += 2;
			memcpy(p,&pPJ->smooth,2);
			p += 2;
			cmd.buflen += 6;
		}
	}
	return RTN_SUCC;
}

/*
 * �滮Jog�˶�ģʽ
 */
ERROR_CODE Prep_JOGmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	PP_JOG_KERNEL *pPJ = &pjkernel[axis];

	//����Ƿ���Ҫ���¹滮
	if(pKer->flag == 0)		// == 0������Ҫ���¹滮
		return RTN_SUCC;

	double t_temp;
	double pos_temp;
	//�����ٶȷ���
	if(pPJ->objVel > pKer->nowVel)				//��Ҫ����
	{
		pPJ->realacc = pPJ->acc;
		t_temp = (pPJ->objVel - pKer->nowVel)/pPJ->acc;
	}
	else if(pPJ->objVel < pKer->nowVel)			//��Ҫ����
	{
		pPJ->realacc = -pPJ->dec;
		t_temp = (pKer->nowVel - pPJ->objVel)/pPJ->dec;
	}
	else
	{
		pKer->flag = 0;
		return RTN_SUCC;						//��ǰ�ٶȵ���Ŀ���ٶȣ�����Ҫ���¹滮
	}
	pPJ->realstartVel = pKer->nowVel;
	pPJ->t[0] = Approximate(t_temp*10);			//��Ҫ���ٵ�ʱ��
	pPJ->pos[0] = pKer->nowPos;
	//Pos end = Pos start + (1/2)*a*t^2 + Vel start*t
	pos_temp = pPJ->pos[0] + pKer->nowVel * t_temp + 0.5 * pPJ->realacc * t_temp * t_temp;
	pPJ->pos[1] = Approximate(pos_temp);		//���ٽ���ʱ�ľ���
	pKer->step = 0;
	pKer->count = 0;
	pKer->flag = 0;
	return RTN_SUCC;
}

/*
 * ����ָ��
 */
void Decouple_JOGmode()
{
	switch(cmd.type)
	{
		case CMD_JOG_MODE:		cmd.rtn = PrfJog();						break;
		case CMD_JOG_SETPRM:	cmd.rtn = SetJogPrm();					break;
		case CMD_JOG_GETPRM:	cmd.rtn = GetJogPrm();					break;
		default: 				cmd.rtn = RTN_INVALID_COMMAND;			break;
	}
}

/*
 * Jogģʽ���к���
 * �����ж�������
 */
void Run_JOGmode(int axis)
{
	long pos;
	PVAT_S pvat;
	KernelPrm *pKer = &kernel[axis];
	PP_JOG_KERNEL *pPJ = &pjkernel[axis];
	if(pKer->flag == 1)
		return;
	switch(pKer->step)
	{
		case 0:
		{
			if(pPJ->t[0] != 0)
			{
				pKer->count ++;
				if(pKer->count >= pPJ->t[0])		//���һ��
				{
					pKer->realPos = pPJ->pos[1];
					pKer->count = 0;
					pKer->step = 1;
				}
				else
				{
					pKer->realPos = pPJ->pos[0] + (0.005)*pPJ->realacc*pKer->count*pKer->count + 0.1*pPJ->realstartVel*pKer->count;
				}
				pKer->nowVel = pPJ->realacc*pKer->count*0.1 + pPJ->realstartVel;
				pKer->nowacc = pPJ->realacc;
				break;
			}
			else
			{
				pKer->count = 0;
				pKer->step = 1;			//ת����һ��
			}
		}
		case 1:
		{
			pKer->count ++;
			pKer->nowacc = pPJ->realacc;
			pKer->nowVel = pPJ->objVel;
			pKer->realPos += pKer->nowVel*0.1;
		}
	}
	if(pKer->nowVel >= 0)
		pKer->dir = 1;
	else
		pKer->dir = 0;

	pos = Approximate(pKer->realPos);
	pvat.aim_pos = pos;
	//pKer->aimPos = pvat.aim_pos;
	pvat.start_acc = 0;
	pvat.start_vel = (pos - pKer->nowPos)*10000;
	pvat.min_period = TIME;
	M_SetPvat(axis, &pvat);
	pKer->nowPos = pvat.aim_pos;
}
