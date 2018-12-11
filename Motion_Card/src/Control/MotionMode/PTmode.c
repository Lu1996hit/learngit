/*
 * 		  File: PTmode.c
 *     Version:
 * Description: PT模式
 * 				PT模式使用一系列“位置、时间”数据点描述速度规划，用户需要将速度曲线分割
 * 				成若干段。PT模式的数据段要求用户输入每段所需的时间和位置点。
 * 				PT模式包含普通段，匀速段，停止段
 * 				动态模式下，串联FIFO1和FIFO2，并且FIFO大小为1024，组成2048大小的超大
 * 				FIFO使用。
 * 				静态模式下，自由选择使用的FIFO，大小固定为1024，不支持循环计数
 *
 *				注：
 * 				一：在描述一次完整的PT运动时，第一段的起点位置和时间被假定为0。压入控制
 * 				器的数据为位置点，即相对于第1段是起点的绝对值，而不是每段位移长度。位置
 * 				的单位是脉冲（pulse），时间单位是毫秒（ms）。
 * 
 *  Created on: 2018年3月15日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#include "system.h"
#include <Kernel/interpolate.h>
#include "taskComm.h"
#include "PTmode.h"

#define FIFO_CHANNEL_NUM	2			//FIFO个数
#define FIFO_MAX_LENGTH		1024		//每个FIFO最大的个数

/*
 * 定义FIFO使用模式
 */
#define PT_MODE_STATIC		0		//静态模式
#define PT_MODE_DYNAMIC		1		//动态模式
/*
 * 数据段类型宏定义
 */
#define PT_SEGMENT_NORMAL	0		//普通段
#define PT_SEGMENT_EVEN		1		//匀速段
#define PT_SEGMENT_STOP		2		//减速段

typedef struct
{
	//以下顺序禁止修改
	long pos;			//位置
	long time;			//时间
	int type;			//数据段类型		普通段 匀速段 减速段
}PT_FIFO;

//八轴，每个轴对应两个FIFO，每个FIFO最大支持1024个
PT_FIFO ptfifo[AXISNUM][FIFO_CHANNEL_NUM][FIFO_MAX_LENGTH];
#pragma DATA_SECTION(ptfifo,"EXTRAM_DATA");

typedef struct
{
	int mode;			//静态模式或者动态模式
	int fifo;			//静态模式下，记录当前正在使用的FIFO
	int nowpoint;		//当前FIFO使用的指针
	int fpoint1;		//FIFO1中的数据个数
	int fpoint2;		//FIFO2中的数据个数
	long loop;			//循环次数
	long nowloop;		//当前的循环次数

	long zeropos;		//当前PT模式的原点的实际位置
	long startpos;		//起始时的位置
	long objpos;		//目标位置
	long runtime;		//运行时间
	double startVel;	//起始速度
	double endVel;		//结束速度
	double acc;			//加速度
}PT_KERNEL;
static PT_KERNEL ptkernel[AXISNUM];

/*
 * 初始化PT_KERNEL
 */
void Init_PT_Kernel()
{
	int axis;
	PT_KERNEL *pPt = ptkernel;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		pPt->fifo = 0;
		pPt->nowpoint = 0;
		pPt->fpoint1 = 0;						//FIFO1中数据清零
		pPt->fpoint2 = 0;						//FIFO2中数据清零
		pPt->loop = 1;							//初始化为1，默认只执行1次
		pPt->nowloop = 0;
		pPt ++;
	}
}

/*
 * 设定指定轴为PT运动模式
 * 同时必须要设置静态或动态模式
 */
static ERROR_CODE PrfPt()
{
	int axis;
	PT_KERNEL *pPt;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].kersta == MOTORS_STA_IDLE)		//检查当前轴是否在运行
			{
				pPt = &ptkernel[axis];
				kernel[axis].axsta = MOTORS_STA_PTMODE;
				pPt->mode = *p;							//设置静态或者动态模式
				if(pPt->mode == PT_MODE_STATIC)
				{
					/*
					 * Insert Code
					 * 加入初始化参数的代码，但是目前来看用不上
					 */
				}
				else if(pPt->mode == PT_MODE_DYNAMIC)
				{
					/*
					 * Insert Code
					 */
				}
				else
					return RTN_ERROR;
				p ++;
			}
			else
				return RTN_ERROR;
		}
	}
	return RTN_SUCC;
}

/*
 * 查询PT运动模式指定FIFO的剩余空间
 * 动态模式下指定FIFO无效，但是可以查询剩余的空间
 */
static ERROR_CODE PtSpace()
{
	int axis;
	PT_KERNEL *pPt;
	unsigned int *pp = &cmd.prm[3];
	unsigned int *pb = cmd.buf;
	int space;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].axsta != MOTORS_STA_PTMODE)
				return RTN_ERROR;

			pPt = &ptkernel[axis];
			if(pPt->mode == PT_MODE_STATIC)				//静态模式下处理
			{
				if(*pp == 0)								//通道0
				{
					space = FIFO_MAX_LENGTH - pPt->fpoint1;
				}
				else if(*pp == 1)						//通道1
				{
					space = FIFO_MAX_LENGTH - pPt->fpoint2;
				}
				else
					return RTN_ERROR;
			}
			else if(pPt->mode == PT_MODE_DYNAMIC)		//动态模式下处理
			{
				if(pPt->fpoint1 >= pPt->nowpoint)		//存的序号大于当前使用的序号
				{
					space = 2*FIFO_MAX_LENGTH - pPt->fpoint1 + pPt->nowpoint - 1;
				}
				else									//当前使用的需要大于存的需要
				{
					space = pPt->nowpoint - pPt->fpoint1 - 1;
				}
			}
			else
				return RTN_ERROR;
			memcpy(pb,&space,1);
			pp ++;
			pb ++;
			cmd.buflen += 1;
		}
	}
	return RTN_SUCC;
}

/*
 * 向PT运动模式指定FIFO中的数据
 * 动态模式下指定FIFO无效
 */
static ERROR_CODE PtData()
{
	int axis;
	PT_KERNEL *pPt = ptkernel;
	PT_FIFO *pFifo;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].axsta != MOTORS_STA_PTMODE)
				return RTN_ERROR;

			pPt = &ptkernel[axis];
			if(pPt->mode == PT_MODE_STATIC)				//静态模式下处理
			{
				if(*(p+5) == 0)							//FIFO0
				{
					if(pPt->fpoint1 >= FIFO_MAX_LENGTH)
						return RTN_NO_SPACE;
					pFifo = ptfifo[axis][0] + pPt->fpoint1;
					pPt->fpoint1 ++;
				}
				else if(*(p+5) == 1)					//FIFO2
				{
					if(pPt->fpoint2 >= FIFO_MAX_LENGTH)
						return RTN_NO_SPACE;
					pFifo = ptfifo[axis][1] + pPt->fpoint2;
					pPt->fpoint2 ++;
				}
				else
					return RTN_ERROR;
			}
			else if(pPt->mode == PT_MODE_DYNAMIC)		//动态模式下处理
			{
				if((pPt->fpoint1+1)%(FIFO_MAX_LENGTH*2) == pPt->nowpoint)
					return RTN_NO_SPACE;
				pFifo = ptfifo[axis][0] + pPt->fpoint1;
				pPt->fpoint1 ++;
				if(pPt->fpoint1 >= 2*FIFO_MAX_LENGTH)
					pPt->fpoint1 = 0;
			}
			else
				return RTN_ERROR;
			memcpy(pFifo,p,5);							// no fifo data
			p += 6;										// long pos + long time + short type + short fifo
			cmd.buflen += 1;
		}
	}
		return RTN_SUCC;
}

/*
 * 清除PT运动模式指定FIFO中的数据
 * 运动状态下该指令无效
 * 运动模式下该指令无效
 */
static ERROR_CODE PtClear()
{
	int axis;
	KernelPrm *pKer;
	PT_KERNEL *pPt;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			pKer = &kernel[axis];
			if(pKer->axsta != MOTORS_STA_PTMODE)
				return RTN_ERROR;

			pPt = &ptkernel[axis];

			if(pPt->mode == PT_MODE_STATIC)				//静态模式下处理
			{
				if(pKer->kersta == MOTORS_STA_IDLE)		//没有在运行
				{
					if(*p == 0)
					{
						pPt->fpoint1 = 0;
					}
					else if(*p == 1)
					{
						pPt->fpoint2 = 0;
					}
					else
						return RTN_ERROR;
				}
				else if(pKer->kersta == MOTORS_STA_PTMODE)
				{
					if(*p == 0 && pPt->fifo == 1)
					{
						pPt->fpoint1 = 0;
					}
					else if(*p == 1 && pPt->fifo == 0)
					{
						pPt->fpoint2 = 0;
					}
					else
						return RTN_ERROR;
				}
				else
					return RTN_ERROR;
				pPt->nowpoint = 0;
				p ++;
			}
			else if(pPt->mode == PT_MODE_DYNAMIC)		//动态模式下处理
			{
				if(pKer->kersta == MOTORS_STA_IDLE)
				{
					pPt->fpoint1 = 0;					//总指针清零
					pPt->nowpoint = 0;					//当前指针清零
					p ++;
				}
				else
					return RTN_ERROR;
			}
			else
				return RTN_ERROR;
		}
	}
	return RTN_SUCC;
}

/*
 * 设置PT运动模式循环执行的次数
 * 动态模式下该指令无效
 * loop为非负数。如果需要无限循环，设置为0
 */
static ERROR_CODE SetPtLoop()
{
	int axis;
	PT_KERNEL *pPt;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].axsta != MOTORS_STA_PTMODE)
				return RTN_ERROR;
			//动态模式，返回指令无效
			pPt = &ptkernel[axis];
			if(pPt->mode == PT_MODE_DYNAMIC)
				return RTN_INVALID_COMMAND;

			memcpy(&pPt->loop,p,2);
			p += 2;
		}
	}
	return RTN_SUCC;
}

/*
 * 查询PT运动模式循环执行的次数
 * 动态模式下该指令无效
 */
static ERROR_CODE GetPtLoop()
{
	int axis;
	PT_KERNEL *pPt;
	unsigned int *p = cmd.buf;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			if(kernel[axis].axsta != MOTORS_STA_PTMODE)
				return RTN_ERROR;
			//动态模式，返回指令无效
			pPt = &ptkernel[axis];
			if(pPt->mode == PT_MODE_DYNAMIC)
				return RTN_INVALID_COMMAND;

			memcpy(p,&pPt->loop,2);
			p += 2;
			cmd.buflen += 2;
		}
	}
	return RTN_SUCC;
}

/*
 * 规划PT
 * 需要快速准确的找到需要规划的参数是哪个
 */
ERROR_CODE Prep_PTmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	PT_KERNEL *pPt = &ptkernel[axis];
	PT_FIFO *pFifo;
	//检查是否需要重新规划
	if(pKer->flag == 0)
		return RTN_SUCC;
	//循环次数到达极限
	if(pPt->nowloop >= pPt->loop && pPt->loop != 0) 		//非零，一直循环下去，或者没有到达循环
		return RTN_ERROR;

	if(pPt->mode == PT_MODE_STATIC)		//静态模式下处理流程
	{
		if(pPt->fifo == 0)
			pFifo = ptfifo[axis][0];
		else
			pFifo = ptfifo[axis][1];

	}
	else								//动态模式下处理流程
	{
		pFifo = ptfifo[axis][0];
	}
	pFifo += pPt->nowpoint;
	pPt->runtime = pFifo->time;

	long deltaPos;
	double evenVel;
	pPt->objpos = pPt->zeropos + pFifo->pos;
	pPt->startpos = pKer->nowPos;			//当前段的起始位置
	deltaPos = pPt->objpos - pPt->startpos;
	evenVel = deltaPos / (pPt->runtime+0.0);//转换成浮点数进行计算

	switch(pFifo->type)
	{
		case PT_SEGMENT_NORMAL:
		{
			pPt->startVel = pPt->endVel;
			pPt->endVel = 2 * evenVel - pPt->startVel;
			pPt->acc = (pPt->endVel - pPt->startVel)/pPt->runtime;
			break;
		}
		case PT_SEGMENT_EVEN:
		{
			pPt->startVel = evenVel;
			pPt->endVel = evenVel;
			pPt->acc = 0;
			break;
		}
		case PT_SEGMENT_STOP:
		{
			pPt->startVel = 2 * evenVel;
			pPt->endVel = 0;
			pPt->acc = -pPt->startVel/pPt->runtime;
			break;
		}
		default: return RTN_ERROR;
	}

	pPt->runtime = pPt->runtime * 10;							//时间刻度的变换
	pKer->flag = 0;
	pKer->flag = 1;
	pKer->flag = 0;
	//pKer->aimPos = pPt->objpos;
	return RTN_SUCC;
}

/*
 * 开启PT运动
 */
static ERROR_CODE Open_PTmode(KernelPrm *pKer)
{
	pKer->kersta = MOTORS_STA_PTMODE;
	pKer->step = 0;
	pKer->count = 0;
	pKer->flag = 1;			//标志位置1
	return RTN_SUCC;
}

/*
 * 关闭PT运动
 */
static ERROR_CODE Exit_PTmode(KernelPrm *pKer)
{
	pKer->flag = 0;
	pKer->kersta = MOTORS_STA_IDLE;
	return RTN_SUCC;
}

/*
 * 启动PT运动
 * 动态模式下指定FIFO无效
 */
static ERROR_CODE PtStart()
{
//	ERROR_CODE rtn;
	int axis;
	PT_KERNEL *pPt;
	KernelPrm *pKer;
	unsigned int *p = &cmd.prm[3];
	for(axis = 0; axis < AXISNUM; axis++)
	{
		if((cmd.mark >> axis) & 0x01)			//使能状态
		{
			pPt = &ptkernel[axis];
			pKer = &kernel[axis];

			/*
			 * 判断是否可以进入PT模式
			 * 1.是否设置了PT模式
			 * 2.缓存区是否有数据
			 */
			if(kernel[axis].axsta == MOTORS_STA_PTMODE)
			{
				if(pPt->mode == PT_MODE_STATIC)
				{
					if((*p >> axis) & 0x01)			// == 1
					{
						if(pPt->fpoint2 == 0)
							return RTN_ERROR;
						pPt->fifo = 1;
					}
					else							// == 0
					{
						if(pPt->fpoint1 == 0)
							return RTN_ERROR;
						pPt->fifo = 0;
					}
				}
				else
				{
					if(pPt->fpoint1 == 0)			//没有数据
						return RTN_ERROR;
					if((*p >> axis) & 0x01)			//== 1
					{
						pPt->fifo = 1;
					}
					else
					{
						pPt->fifo = 0;
					}
				}

				Open_PTmode(&kernel[axis]);

				pPt->nowloop = 0;				//当前圈数清零
				pPt->nowpoint = 0;				//当前起始指针零
				pPt->endVel = 0;				//如果最开始是普通段，可以通过这个设置第一段初速度
				pPt->zeropos = pKer->nowPos;	//设置PT模式是初始的位置
			}
			else
				return RTN_ERROR;
		}
//		else									//关闭状态		暂时不使用
//		{
//			if(pKer->state == MOTORS_STA_PTMODE)		//正在运行时的状态
//			{
//
//			}
//		}
	}
	return RTN_SUCC;
}

/*
 * 解码指令
 */
void Decouple_PTmode()
{
	switch(cmd.type)
	{
		case CMD_PT_MODE:				cmd.rtn = PrfPt();						break;
		case CMD_PT_SPACE:				cmd.rtn = PtSpace();					break;
		case CMD_PT_DATA:				cmd.rtn = PtData();						break;
		case CMD_PT_CLEAR:				cmd.rtn = PtClear();					break;
		case CMD_PT_SET_LOOP:			cmd.rtn = SetPtLoop();					break;
		case CMD_PT_GET_LOOP:			cmd.rtn = GetPtLoop();					break;
		case CMD_PT_START:				cmd.rtn = PtStart();					break;
		default:						cmd.rtn = RTN_INVALID_COMMAND;			break;
	}
}

/*
 * PT运动模式运行函数
 * 放在中断中运行
 * 需要判断是否结束PT模式
 * 需要控制循环次数的累加和开始时候的位置
 */
int  PTCOUNT = 0;
void Run_PTmode(int axis)
{
	int overflag = 0;
	long pos;
	PVAT_S pvat;
	KernelPrm *pKer = &kernel[axis];
	PT_KERNEL *pPt = &ptkernel[axis];
	if(pKer->flag == 1)		//数据没有处理完成，直接跳出
	{
		PTCOUNT ++;			//测试使用，用于判断DSP的处理能力
		return;
	}
	else
	{
		pKer->count ++;
		if(pKer->count >= pPt->runtime)			//最后一次
		{
			pKer->realPos = pPt->objpos;
			pKer->nowVel = pPt->endVel;
			pKer->count = 0;
			pKer->flag = 1;						//重新准备
			pPt->nowpoint ++;
			overflag = 1;
		}
		else
		{
			pKer->realPos = pPt->startpos + pKer->count*pPt->acc*pKer->count*0.005 + pPt->startVel*pKer->count*0.1;
			pKer->nowVel = pPt->startVel + pPt->acc * pKer->count * 0.1;
		}
		pKer->nowacc = pPt->acc;

		if(pKer->nowVel >= 0)
			pKer->dir = 1;
		else
			pKer->dir = 0;

		pos = Approximate(pKer->realPos);
		pvat.aim_pos = pos;
		pvat.start_acc = 0;
		pvat.start_vel = (pos - pKer->nowPos)*10000;
		pvat.min_period = TIME;
		M_SetPvat(axis, &pvat);
		pKer->nowPos = pvat.aim_pos;

		if(overflag == 1)
		{
			/*
			 * 判断循环和退出
			 * 区分动态模式和静态模式
			 */
			if(pPt->mode == PT_MODE_DYNAMIC)		//动态模式下处理
			{
				if(pPt->nowpoint >= 2*FIFO_MAX_LENGTH)
					pPt->nowpoint = 0;
				//判断是否需要结束动态模式
				if(pPt->nowpoint == pPt->fpoint1)	//两个指针相等，退出
				{
					Exit_PTmode(&kernel[axis]);
					pPt->nowpoint = 0;				//指针清零
					pPt->fpoint1 = 0;				//指针清零
				}
			}
			else if(pPt->mode == PT_MODE_STATIC)	//静态模式下处理
			{
				/*
				 * 区分有限循环和无限循环的情况
				 */
				if(pPt->loop == 0)					//无限循环的情况
				{/**************************************************************/
					/*
					 * 判断使用的是什么缓冲区
					 * 判断当前缓冲区是否用完
					 * 如果用完
					 * 重新标定原点
					 * 指针清零
					 * 进入下一次循环
					 */
					if(pPt->fifo == 0)
					{
						if(pPt->nowpoint >= pPt->fpoint1)	//溢出，进入下一次循环
						{
							pPt->zeropos = pKer->nowPos;
							pPt->nowpoint = 0;
						}
					}
					else
					{
						if(pPt->nowpoint >= pPt->fpoint2)
						{
							pPt->zeropos = pKer->nowPos;
							pPt->nowpoint = 0;
						}
					}

				}
				else								//有限循环次数
				{/**************************************************************/
					if(pPt->fifo == 0)
					{
						if(pPt->nowpoint >= pPt->fpoint1)	//是否溢出，进入下一次循环
						{
							pPt->nowloop ++;
							pPt->nowpoint = 0;
							pPt->zeropos = pKer->nowPos;
							pKer->flag = 1;
						}
						if(pPt->nowloop >= pPt->loop)		//是否结束循环
						{
							Exit_PTmode(&kernel[axis]);
							pPt->nowpoint = 0;
						}
					}
					else
					{
						if(pPt->nowpoint >= pPt->fpoint2)
						{
							pPt->nowloop ++;
							pPt->nowpoint = 0;
							pPt->zeropos = pKer->nowPos;
							pKer->flag = 1;
						}
						if(pPt->nowloop >= pPt->loop)
						{
							Exit_PTmode(&kernel[axis]);
							pPt->nowpoint = 0;
						}
					}
				}
			}
		}
		return;
	}
}
