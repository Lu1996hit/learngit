/*
 *        File: JOGmode.c
 *     Version:
 * Description: Jog运动模式
 * 				在Jog运动模式下，各轴可以独立设置目标速度、加速度、减速度、平
 * 				滑系数等运动参数，能够独立运动或停止。
 *
 *  Created on: 2018年4月18日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "system.h"
#include "taskComm.h"
#include "interpolate.h"
#include "JOGmode.h"

extern PP_JOG_KERNEL pjkernel[AXISNUM];

/*
 * 设定指定轴为JOG模式
 */
static ERROR_CODE PrfJog()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].kersta == MOTORS_STA_IDLE)		//检查当前轴是否在运行
				kernel[axis].axsta = MOTORS_STA_JOGMODE;
			else
				return RTN_ERROR;
		}
	}
	return RTN_SUCC;
}

/*
 * 设置Jog运动模式下的运动参数
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
 * 规划Jog运动模式
 */
ERROR_CODE Prep_JOGmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	PP_JOG_KERNEL *pPJ = &pjkernel[axis];

	//检查是否需要重新规划
	if(pKer->flag == 0)		// == 0：不需要重新规划
		return RTN_SUCC;

	double t_temp;
	double pos_temp;
	//检查加速度方向
	if(pPJ->objVel > pKer->nowVel)				//需要加速
	{
		pPJ->realacc = pPJ->acc;
		t_temp = (pPJ->objVel - pKer->nowVel)/pPJ->acc;
	}
	else if(pPJ->objVel < pKer->nowVel)			//需要减速
	{
		pPJ->realacc = -pPJ->dec;
		t_temp = (pKer->nowVel - pPJ->objVel)/pPJ->dec;
	}
	else
	{
		pKer->flag = 0;
		return RTN_SUCC;						//当前速度等于目标速度，不需要重新规划
	}
	pPJ->realstartVel = pKer->nowVel;
	pPJ->t[0] = Approximate(t_temp*10);			//需要加速的时间
	pPJ->pos[0] = pKer->nowPos;
	//Pos end = Pos start + (1/2)*a*t^2 + Vel start*t
	pos_temp = pPJ->pos[0] + pKer->nowVel * t_temp + 0.5 * pPJ->realacc * t_temp * t_temp;
	pPJ->pos[1] = Approximate(pos_temp);		//加速结束时的距离
	pKer->step = 0;
	pKer->count = 0;
	pKer->flag = 0;
	return RTN_SUCC;
}

/*
 * 解码指令
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
 * Jog模式运行函数
 * 放在中断中运行
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
				if(pKer->count >= pPJ->t[0])		//最后一次
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
				pKer->step = 1;			//转跳下一步
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
