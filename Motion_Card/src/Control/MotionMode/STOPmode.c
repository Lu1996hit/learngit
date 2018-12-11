/*
 *        File: STOPmode.c
 *     Version:
 * Description: 用于实现轴的停止，停止在所有的运动中发的优先级是最高的
 *
 *  Created on: 2018年8月23日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

/*
 * 直线运动上的停止，
 * 圆弧模式上的停止，这个需要特别注意，停止的时候，轨迹也要按照圆弧走
 */


#include "system.h"
#include "taskComm.h"
#include <Kernel/interpolate.h>

#include "STOPmode.h"


/*
 * 当前使用的一些参数
 */
typedef struct
{
	unsigned int type;					//停止种类
										// 0： 平滑停止
										// 1： 急停
	double decSmoothStop;				//平滑停止减速度
	double decAbruptStop;				//急停减速度
	double start_pos;					//启示位置
	double end_pos;						//终点位置
	double real_acc;					//实际加速度，区分正负
	double start_vel;					//开始速度
	long t;								//减速时间
	MOTORS_STA last_state;				//上一步的状态
}STOP_KERNEL;

static STOP_KERNEL stopkernel[AXISNUM];

/*
 * 初始化减速模式
 * 赋值，写上一些默认的参数
 */
void Init_Stop_Kernel()
{
	int count;
	STOP_KERNEL* pStopKer;
	pStopKer = stopkernel;
	for(count = 0; count < AXISNUM; count ++)
	{
		pStopKer->type = 0;				//默认设置为平滑停止
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
 * 设置平滑停止减速度和急停减速度
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
 * 读取平滑停止减速度和急停减速度
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
 * 停止一个或者多个轴的规划运动，停止坐标系运动
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

			pKer->flag = 1;				//标志位置1，开始处理。
			//避免重复的停止
			if(pStopKer->last_state == MOTORS_STA_STOPMODE)
				break;
			pStopKer->last_state = pKer->kersta;
			pKer->axsta = MOTORS_STA_STOPMODE;
			pKer->kersta = MOTORS_STA_STOPMODE;			//处理完成后置为STOPMODE
			pKer->step = 0;
			pKer->count = 0;
		}
	}
	return RTN_SUCC;
}

/*
 * 规划停止运动
 * 1.区分是直接一个轴下的停止还是坐标系等情况下的停止
 */
ERROR_CODE Prep_STOPmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	STOP_KERNEL *pStopKer = &stopkernel[axis];
	//检查是否需要重新规划
	if(pKer->flag == 0)
		return RTN_SUCC;

	switch(pStopKer->last_state)
	{
	/*
	 * 这种模式下不需要运行停止，直接退出即可
	 */
	case MOTORS_STA_IDLE:
	{
		pKer->flag = 0;
		pKer->axsta = MOTORS_STA_IDLE;
		pKer->kersta = MOTORS_STA_IDLE;
		break;
	}

	//下面的三种模式都是直线运动对应的的是最简单的直接停止
	case MOTORS_STA_PPMODE:
	case MOTORS_STA_JOGMODE:
	case MOTORS_STA_PTMODE:
	{
		if(pKer->kersta == MOTORS_STA_IDLE)			//没有在运动，直接退出就可以了
		{
			pKer->axsta = MOTORS_STA_IDLE;
			pKer->flag = 0;
			return RTN_SUCC;
		}

		double t;
		double acc;			//生成加速度
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
			pKer->dir = 1;			//正向，加速度为负数
			t = pKer->nowVel / acc;
			distance = 0.5*acc*t*t;
			pStopKer->end_pos = pStopKer->start_pos + distance;
			pStopKer->real_acc = -acc;
		}
		else
		{
			pKer->dir = 0;			//反向，加速度为正数
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
	//耦合组成坐标系下的停止,这部分放到坐标系模式下单独处理
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
 * 解码指令
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
 * 停止模式运动函数
 */
void Run_STOPmode(int axis)
{
	long pos;
	PVAT_S pvat;
	KernelPrm *pKer = &kernel[axis];
	STOP_KERNEL *pStopKer = &stopkernel[axis];

	//未处理好数据，直接跳出
	if(pKer->flag == 1)
		return;

	//return;
	switch(pStopKer->last_state)
	{
	case MOTORS_STA_IDLE:		//正常情况下不会进入这个部分
	{
		break;
	}

	//下面的三种模式都是直线运动对应的的是最简单的直接停止
	case MOTORS_STA_PPMODE:
	case MOTORS_STA_JOGMODE:
	case MOTORS_STA_PTMODE:
	{
		pKer->count ++;
		if(pKer->count >= pStopKer->t)		//最后一次
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
	//耦合组成坐标系下的停止,这部分放到坐标系模式下单独处理
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
