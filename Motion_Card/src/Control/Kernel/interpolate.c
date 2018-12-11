/*
 * 		  File: interpolate.c
 *     Version:
 * Description:
 *
 *  Created on: 2018年3月15日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#include "my_project.h"
#include "interpolate.h"


KernelPrm kernel[AXISNUM];


/*
 * 四舍五入的代码
 */
long Approximate(double value)
{
	if(value >= 0)
		value += 0.5;
	else
		value -= 0.5;
	return (long)value;		//强制转换
//	return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}

/*
 * 初始化内核
 */
void Init_Kernel()
{
	int axis;
	KernelPrm *p;
	p = kernel;
	for(axis = 0;axis < AXISNUM; p++,axis++)
	{
		p->axsta = MOTORS_STA_IDLE;
		p->kersta = MOTORS_STA_IDLE;
		p->flag = 0;
		p->step = 0;
		p->count = 0;
		p->nowPos = 0;
		p->realPos = 0;
		p->nowacc = 0;
		p->nowVel = 0;
		p->axis = axis;
	}
}

/*
 * 清除数据用的
 */
static void Kernel_IDLE(KernelPrm *p)
{
	p->nowVel = 0;
	p->nowacc = 0;
}

/*
 * 内核运行程序
 */
void Run_Kernel()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		switch(kernel[axis].kersta)
		{
			case MOTORS_STA_IDLE:			Kernel_IDLE(&kernel[axis]);		break;
			case MOTORS_STA_PPMODE:			Run_PPmode(axis);				break;
			case MOTORS_STA_JOGMODE:		Run_JOGmode(axis);				break;
			case MOTORS_STA_PTMODE:			Run_PTmode(axis);				break;
			case MOTORS_STA_GEARMODE:										break;
			case MOTORS_STA_FOLLOWMODE:										break;
			case MOTORS_STA_CRDMODE:		Run_CRDmode(axis);				break;
			case MOTORS_STA_STOPMODE:		Run_STOPmode(axis);				break;
			case MOTORS_STA_ERRORMODE:										break;
			default:														break;
		}
	}
}
