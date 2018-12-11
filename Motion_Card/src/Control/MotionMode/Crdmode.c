/*
 *        File: Crdmode.c
 *     Version:
 * Description: 插补模式
 * 				（1）直线插补和圆弧插补
 * 				（2）两个坐标系同时运行插补运动
 * 				（3）每个坐标系具有两个缓冲区
 * 				（4）每个缓冲区具有延时和数字量输出等功能
 * 				（5）具有前瞻预处理功能
 * 				（6）超越保佑，永无Bug
 *
 *  Created on: 2018年7月20日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include <stdlib.h>
#include <string.h>
#include "system.h"
#include "taskComm.h"
#include "interpolate.h"
#include "Crdmode.h"
#include "CrdmodeAux.h"
#include <math.h>

#define	M_2PI		6.283185307179586

/*
 * 统一一下参数传递的格式，才crd.prm[]中：
 * crd.prm[0]:存储的是tpye			//指令种类
 * crd.prm[1]:存储的是mark			//坐标系编号
 * crd.prm[2]:存储的是序列号
 * crd.prm[3]:存储的是fifo的值		//缓冲区FIFO的编号，如果当前的指针和fifo有关
 */

/*
 * 指向与坐标系方向相关联的轴的内核
 */
KernelPrm* pKernelC[CRDNUM][4];		//坐标系2使用

//每个轴对应的坐标系
static int crdindex[AXISNUM];

/*
 * 定义坐标系
 */
TCrdPrm crdprm[CRDNUM];							//一共有两个坐标系

/*
 * 定义缓冲区
 */
CMDCRD crdfifo[CRDNUM][CRDFIFONUM][CRDFIFOLEN];			//缓冲区大小
#pragma DATA_SECTION(crdfifo, "EXTRAM_DATA");

/*
 * 该结构体中用于记录每个轴的指令缓冲区的信息
 */
typedef struct
{
	int fifo[CRDFIFONUM];						//两个坐标系，每个坐标系有两个FIFO
												//用于记录当前FIFO的使用情况
	int nowfifo[CRDFIFONUM];					//当前的使用的FIFO中的指令的个数
	int runstate;								//记录主FIFO当前的运动情况
												//0：未开始运动时
												//1：使用0号fifo开始运动
												//2：从主FIFO中暂停
												//3：使用1号fifo开始运动
	double staVel;								//起始速度
	double nowVel;								//当前坐标系的合成速度
	double nowAcc;								//当前坐标系的合成加速度
	long helpPos[4];							//辅助参数，记录当前指令压栈后，轴的位置。
	long zeroPos[4];							//原点的位置，在原始坐标系中的位置
	long pausePos[4];							//暂停时候的位置

	double r;									//圆弧半径
	double ac,bc;								//圆弧圆心

	long startPos[4];							//每个坐标系的起始的当前指令下的起始的位置
	long time[3];								//三段时间，加速 匀速 减速的时间
	double endPos[3];							//三段时间，每段时间对应的最终的位置
	double conver[4];							//用于直线中的位置换算

	CMDCRD *pCmdCrd;							//指向当前运行的指令的指针
	//这些参数可是设置成长时间有效
	//不参与创建坐标系时候的清零
	double decSmoothStop;						//平滑停止加速度
	double decAbruptStop;						//紧急停止加速度
	TCrdPrm *pCrdPrm;							//指针，指向坐标系参数
}CRD_KERNEL;

static CRD_KERNEL crdkernel[CRDNUM];
/*
 * 初始化参数
 */
void Init_Crd_Kernel()
{
	int count;
	for(count = 0; count < 4; count++)
	{
		pKernelC[0][count] = NULL;								//指向轴的指针清零
		pKernelC[1][count] = NULL;
	}
	memset(crdprm, 0, sizeof(TCrdPrm)*2);
	for(count = 0; count < CRDNUM; count++)
	{
		crdkernel[count].decSmoothStop = 0.01;
		crdkernel[count].decAbruptStop = 0.1;				//设置初始值
		memset(&crdkernel[count], 0, sizeof(CRD_KERNEL)-sizeof(double)*3);
		crdkernel[count].pCrdPrm = &crdprm[count];
	}
}

/*
 * 坐标系是否依然成立，有没有被修改掉
 * index：表示坐标系
 */
static ERROR_CODE CrdWithal(int index)
{
	TCrdPrm* pCrd;
	KernelPrm* pKernel;
	if(index == 0)				//1号坐标系
	{
		pCrd = &crdprm[0];
		pKernel = pKernelC[0][0];
	}
	else if(index == 1)			//2号坐标系
	{
		pCrd = &crdprm[1];
		pKernel = pKernelC[1][0];
	}
	else						//输入的index有误
	{
		return RTN_ERROR;
	}

	if(pCrd == NULL)			//为空，返回错误
	{
		return RTN_ERROR;
	}

	if(pCrd->dimension < 2)		//坐标系中没有数值，返回错误
	{
		return RTN_ERROR;
	}
	int count;
	for(count = 0;count < pCrd->dimension; count++)
	{
		if(pKernel == NULL)
			return RTN_ERROR;
		pKernel ++;
	}
	return RTN_SUCC;
}

/*
 * 开始运行坐标系模式
 */
static ERROR_CODE Open_Crdmode(int crd, int fifo)
{
	int count;
	TCrdPrm *pCrdPrm;
	CRD_KERNEL *pCrdKer;
	KernelPrm **pKernel;

	pCrdPrm = &crdprm[crd];
	pCrdKer = &crdkernel[crd];

	//检查坐标系是否依然成立							//在启动代码中添加
//	if(CrdWithal(crd) != RTN_SUCC)
//		return RTN_ERROR;

	//检查参数输入和设置指针
	if(crd == 0)
	{
		pKernel = pKernelC[0];
	}
	else if(crd == 1)
	{
		pKernel = pKernelC[1];
	}
	else
		return RTN_ERROR;

	//检查能不能启动
	if(fifo == 0)
	{
		//检查第一个fifo中有没有数据，能不能启动
		if(pCrdKer->nowfifo[0] == pCrdKer->fifo[0])								//检查FIFO中的数据
			return RTN_ERROR;

		//是否是从暂停中恢复运动
		if(pCrdKer->runstate == 2 || pCrdKer->runstate == 4 || pCrdKer->runstate == 5)					//检查是不是从暂停中恢复
		{
			//暂停时候的位置和当前的位置，不相等，无法继续运动
			for(count= 0; count < pCrdPrm->dimension; count++)
			{
				if(pCrdKer->pausePos[count]  != (*(pKernel+count))->nowPos)
					return RTN_ERROR;
			}
		}
		else if(pCrdKer->runstate == 0)												//检查是不是重新开始运行
		{
			pCrdKer->runstate = 1;
		}
		else
			return RTN_ERROR;
		pCrdKer->runstate = 1;						//状态，设置为1

	}
	else if(fifo == 1)
	{
		//检查第二个fifo中有没有数据，能不能启动
		if(pCrdKer->nowfifo[1] == pCrdKer->fifo[1])								//检查FIFO中的数据
			return RTN_ERROR;

		//检查当前是不是在暂停中恢复												//检查是不是在FIFO中暂停
		if(pCrdKer->runstate == 2 || pCrdKer->runstate == 4)
		{
			pCrdKer->runstate = 3;
		}
		else
			return RTN_ERROR;

	}
	else
		return RTN_ERROR;

	for(count = 1; count < pCrdPrm->dimension; count++)
	{
		(*(pKernel+count))->kersta = MOTORS_STA_CRDAUX;
		//(*(pKernel+count))->aimPos = (*(pKernel+count))->nowPos;
		(*(pKernel+count))->step = 0;
		(*(pKernel+count))->flag = 0;
		crdindex[count] = crd;
	}

	(*pKernel)->kersta = MOTORS_STA_CRDMODE;
	//(*pKernel)->aimPos = (*pKernel)->nowPos;
	(*pKernel)->step = 0;
	(*pKernel)->flag = 1;		//需要做规划
	crdindex[0] = crd;
	return RTN_SUCC;
}

/*
 * 退出坐标系模式
 */
static void Quit_Crdmode(int crd, int fifo)
{
	int count;
	TCrdPrm *pCrdPrm;
	CRD_KERNEL *pCrdKer;
	KernelPrm **pKernel;

	pCrdPrm = &crdprm[crd];
	pCrdKer = &crdkernel[crd];

	if(crd == 0)
	{
		pKernel = pKernelC[0];
	}
	else
	{
		pKernel = pKernelC[1];
	}

	pCrdKer->nowfifo[fifo] = 0;
	pCrdKer->fifo[fifo] = 0;

	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		(*(pKernel+count))->kersta = MOTORS_STA_IDLE;
		(*(pKernel+count))->step = 0;
		(*(pKernel+count))->flag = 0;
		(*(pKernel+count))->count = 0;
	}

	return;
}

/*
 * 辅助检查参数和设置参数，直线和圆弧下使用
 */
static ERROR_CODE CheckPrmAux_LC(TCrdPrm** pCrdPrm, CRD_KERNEL** pCrdKer,CRD_FIFO_REG* tcf)
{
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			tcf->crd = 0;
			(*pCrdPrm) = &crdprm[0];		//用来修改最大合成速度等参数
			(*pCrdKer) = &crdkernel[0];		//用来判断
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)			//Crd 2
	{
		if(CrdWithal(1) == RTN_SUCC)	//坐标系依然成立
		{
			tcf->crd = 1;
			(*pCrdPrm) = &crdprm[1];
			(*pCrdKer) = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	if(cmd.prm[3] == 0)					//判断FIFO缓冲区的选择
	{
		tcf->fifo = 0;
	}
	else
	{
		tcf->fifo = 1;
		//fifo为1时，若主FIFO未暂停，则返回错误
		if((*pCrdKer)->runstate != 2)
			return RTN_ERROR;			//返回错误
	}
	//检查是否超出缓存区
	if((*pCrdKer)->fifo[tcf->fifo] >= CRDFIFOLEN)
		return RTN_ERROR;				//返回错误

	tcf->type = cmd.type & 0xFF;			//保存type
	tcf->ready = 0;

	return RTN_SUCC;
}

/*
 * 辅助检查参数和设置参数，Buf下使用
 */
static ERROR_CODE CheckPrmAux_Buf(CRD_KERNEL** pCrdKer,CRD_FIFO_REG* tcf)
{
	if(cmd.mark && 0x01)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			tcf->crd = 0;
			*pCrdKer = &crdkernel[0];	//用来判断
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark && 0x02)
	{
		if(CrdWithal(1) == RTN_SUCC)	//坐标系依然成立
		{
			tcf->crd = 1;
			*pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	if(cmd.prm[3] == 0)					//判断FIFO缓冲区的选择
	{
		tcf->fifo = 0;
	}
	else
	{
		tcf->fifo = 1;
		//fifo为1时，若主FIFO未暂停，则返回错误
		if((*pCrdKer)->runstate != 2)
			return RTN_ERROR;			//返回错误
	}
	//检查是否超出缓存区
	if((*pCrdKer)->fifo[tcf->fifo] >= CRDFIFOLEN)
		return RTN_ERROR;				//返回错误

	tcf->type = cmd.type & 0xFF;			//保存type
	tcf->ready = 0;

	return RTN_SUCC;
}

/*
 * 设置坐标系参数，建立坐标系
 */
static ERROR_CODE SetCrdPrm()
{
	int crd;			//记录当前使用的坐标系
	int count;
	int index;
	TCrdPrm *pCrd;			//坐标系结构体
	TCrdPrm *pCrd_temp;		//坐标系结构体临时变量
	CRD_KERNEL *pCrdKer;	//坐标系内核

	pCrd_temp = (TCrdPrm*)(&cmd.prm[3]);

	if(cmd.mark == 0x01)
	{
		crd = 0;
	}
	else if(cmd.mark == 0x02)
	{
		crd = 1;
	}
	else
		return RTN_INVALID_PARAMETER;
	//设置一些辅助指针
	pCrd = &crdprm[crd];
	pCrdKer = &crdkernel[crd];

	if(pCrd->dimension != 0)		//判断当前轴是否为空
		return RTN_ERROR;			//不为空，说明里面有数据，返回错误

	for(count = 0; count < pCrd_temp->dimension; count++)
	{
		//设置的轴正在运动或者轴正在当前设置的轴已经处在坐标系模式下
		index = pCrd_temp->axisIndex[count];
		if(kernel[index].kersta != MOTORS_STA_IDLE || kernel[index].axsta == MOTORS_STA_CRDMODE)
			return RTN_ERROR;		//返回错误
	}
	//执行到此处，说明输入指令没有问题，继续执行
	for(count = 0; count < pCrd_temp->dimension; count++)
	{
		index = pCrd_temp->axisIndex[count];
		pKernelC[crd][count] = &kernel[index];
		pKernelC[crd][count]->axsta = MOTORS_STA_CRDMODE;
	}
	//复制数据进去
	memcpy(pCrd, pCrd_temp, sizeof(TCrdPrm));
	/*
	 * 插入初始化坐标系的代码
	 */
	memset(pCrdKer, 0, sizeof(CRD_KERNEL)-sizeof(double)*3);					//初始化内核参数，最后三个不参与初始化
	if(pCrd->setOriginFlag == 1)												//设置原点位置
	{
		for(count = 0; count < pCrd->dimension; count++)
		{

			pCrdKer->zeroPos[count] = pKernelC[crd][count]->nowPos + pCrd->originPos[count];
			pCrdKer->helpPos[count] = pKernelC[crd][count]->nowPos;
		}
	}
	else
	{
		for(count = 0; count < pCrd->dimension; count++)
		{
			if(cmd.mark == 0x01)
				pCrdKer->zeroPos[count] = pKernelC[0][count]->nowPos;
			else
				pCrdKer->zeroPos[count] = pKernelC[1][count]->nowPos;
			pCrdKer->helpPos[count] = pCrdKer->zeroPos[count];
		}
	}
	return RTN_SUCC;
}

/*
 * 查询坐标参数
 * 1.判断当时查询的坐标系是否已经建立
 * 2.如果已经建立，则返回坐标系参数
 * 3.没有建立，则返回ERROR
 */
static ERROR_CODE GetCrdPrm()
{
	TCrdPrm* pCrd;
	unsigned int *pb = cmd.buf;
	if(cmd.mark == 0x01)                        	//查询1号坐标系
	{
		if(RTN_SUCC != CrdWithal(0))				//坐标系不成立
			return RTN_ERROR;
		pCrd = &crdprm[0];
	}
	else if(cmd.mark == 0x02)						//查询2号坐标系
	{
		if(RTN_SUCC != CrdWithal(1))				//坐标系不成立
			return RTN_ERROR;
		pCrd = &crdprm[1];
	}
	else
		return RTN_ERROR;

	if(pCrd->dimension == 0)                		//当前坐标系为空
	{
		return RTN_ERROR;
	}
	//开始传输坐标系数据回去
	memcpy(pb, pCrd,sizeof(TCrdPrm));
	cmd.buflen += sizeof(TCrdPrm);
	return RTN_SUCC;
}

/*
 * 解除坐标系的设定
 */
/*
 * 此处的指针的指向有问题
 * 修复方法，不使用二级指针，直接使用一级指针指向
 */
static ERROR_CODE QuitCrd()
{
	TCrdPrm* pCrd;
	int count;
	if(cmd.mark == 0x01)	                    	//1号坐标系解除绑定
	{
		pCrd = &crdprm[0];
		if(pCrd->dimension == 0)		        	//判断1号坐标系是否有绑定
		{											//没有绑定，直接退出
			return RTN_SUCC;
		}
		memset(pCrd, 0, sizeof(TCrdPrm));			//有绑定
		for(count = 0; count < 4; count++)
		{
			if(pKernelC[0][count] != NULL && pKernelC[0][count]->axsta == MOTORS_STA_CRDMODE)
			{
				pKernelC[0][count]->axsta = MOTORS_STA_IDLE;		//清除轴的状态
				pKernelC[0][count] = NULL;							//清除指向
			}
		}
	}
	else if(cmd.mark == 0x02)                   	//2号坐标系解除绑定
	{
		pCrd = &crdprm[1];
		if(pCrd->dimension == 0)		        	//判断2号坐标系是否有绑定
		{
			return RTN_ERROR;						//没有绑定，直接退出
		}
		memset(pCrd, 0, sizeof(TCrdPrm));
		for(count = 0; count < 4; count++)
		{
			if(pKernelC[1][count] != NULL && pKernelC[1][count]->axsta == MOTORS_STA_CRDMODE)
			{
				pKernelC[1][count]->axsta = MOTORS_STA_IDLE;		//清除轴的状态
				pKernelC[1][count] = NULL;							//清除指向
			}
		}
	}
	/*
	 * Insert code
	 * 清除FIFO中的数据
	 */

	return RTN_SUCC;
}

/*
 * 向指令缓存区添加数据
 * 1.先判度使用的是哪一个坐标系，然后判断当前的坐标系是否依然成立
 * 2.判断当前的fifo中已经存储的指令长度的大小，有没有超过范围
 */

/*
 * 向插补缓冲区增加插补数据
 */
static ERROR_CODE CrdData()
{

	return RTN_SUCC;
}

/*
 * 缓存区指令，二维直线插补
 */
static ERROR_CODE LnXY()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	//设置指针指向，检查参数是否合理
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXY));
	pCmdCrd->prm.xy.velEnd = pCmdCrd->prm.xy.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xy.velEnd;
	pCmdCrd->prm.xy.synVel = pCmdCrd->prm.xy.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xy.synVel;
	pCmdCrd->prm.xy.synAcc = pCmdCrd->prm.xy.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xy.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xy.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xy.y;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，三维直线插补
 */
static ERROR_CODE LnXYZ()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;			//返回错误

	if(pCrdPrm->dimension < 3)
		return RTN_ERROR;				//当前的坐标系小于三个，但是该指令需要三轴的支持

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXYZ));

	pCmdCrd->prm.xyz.velEnd = pCmdCrd->prm.xyz.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyz.velEnd;
	pCmdCrd->prm.xyz.synVel = pCmdCrd->prm.xyz.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyz.synVel;
	pCmdCrd->prm.xyz.synAcc = pCmdCrd->prm.xyz.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xyz.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xyz.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xyz.y;
	pCrdKer->helpPos[2] = pCmdCrd->prm.xyz.z;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，四维直线插补
 */
static ERROR_CODE LnXYZA()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 4)
		return RTN_ERROR;				//当前的坐标系小于四个，但是该指令需要四轴的支持

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXYZA));

	pCmdCrd->prm.xyza.velEnd = pCmdCrd->prm.xyza.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyza.velEnd;
	pCmdCrd->prm.xyza.synVel = pCmdCrd->prm.xyza.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyza.synVel;
	pCmdCrd->prm.xyza.synAcc = pCmdCrd->prm.xyza.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xyza.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xyza.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xyza.y;
	pCrdKer->helpPos[2] = pCmdCrd->prm.xyza.z;
	pCrdKer->helpPos[3] = pCmdCrd->prm.xyza.a;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，二维直线插补（终点速度始终为0）
 */
static ERROR_CODE LnXYG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXY));

	pCmdCrd->prm.xy.synVel = pCmdCrd->prm.xy.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xy.synVel;
	pCmdCrd->prm.xy.synAcc = pCmdCrd->prm.xy.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xy.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xy.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xy.y;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，三维直线插补（终点速度始终为0）
 */
static ERROR_CODE LnXYZG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)
		return RTN_ERROR;				//当前的坐标系小于三个，但是该指令需要三轴的支持

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXYZ));

	pCmdCrd->prm.xyz.synVel = pCmdCrd->prm.xyz.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyz.synVel;
	pCmdCrd->prm.xyz.synAcc = pCmdCrd->prm.xyz.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xyz.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xyz.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xyz.y;
	pCrdKer->helpPos[2] = pCmdCrd->prm.xyz.z;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，四维直线插补（终点速度始终为0）
 */
static ERROR_CODE LnXYZAG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 4)
		return RTN_ERROR;				//当前的坐标系小于四个，但是该指令需要四轴的支持

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDXYZA));

	pCmdCrd->prm.xyza.synVel = pCmdCrd->prm.xyza.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.xyza.synVel;
	pCmdCrd->prm.xyza.synAcc = pCmdCrd->prm.xyza.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.xyza.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.xyza.x;
	pCrdKer->helpPos[1] = pCmdCrd->prm.xyza.y;
	pCrdKer->helpPos[2] = pCmdCrd->prm.xyza.z;
	pCrdKer->helpPos[3] = pCmdCrd->prm.xyza.a;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，XY平面圆弧插补（以终点位置和半径作为输入参数）
 */
static ERROR_CODE ArcXYR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABR));
	pCmdCrd->prm.abr.velEnd = pCmdCrd->prm.abr.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.velEnd;
	pCmdCrd->prm.abr.synVel = pCmdCrd->prm.abr.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.synVel;
	pCmdCrd->prm.abr.synAcc = pCmdCrd->prm.abr.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abr.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.abr.a;
	pCrdKer->helpPos[1] = pCmdCrd->prm.abr.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，XY平面圆弧插补（以终点位置和圆心位置作为输入参数）
 */
static ERROR_CODE ArcXYC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABC));
	pCmdCrd->prm.abc.velEnd = pCmdCrd->prm.abc.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.velEnd;
	pCmdCrd->prm.abc.synVel = pCmdCrd->prm.abc.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.synVel;
	pCmdCrd->prm.abc.synAcc = pCmdCrd->prm.abc.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abc.synAcc;
	pCrdKer->helpPos[0] = pCmdCrd->prm.abc.a;
	pCrdKer->helpPos[1] = pCmdCrd->prm.abc.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，YZ平面圆弧插补（以终点位置和半径作为输入参数）
 */
static ERROR_CODE ArcYZR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//当前的坐标系小于三个，但是该指令需要三轴的支持
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABR));
	pCmdCrd->prm.abr.velEnd = pCmdCrd->prm.abr.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.velEnd;
	pCmdCrd->prm.abr.synVel = pCmdCrd->prm.abr.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.synVel;
	pCmdCrd->prm.abr.synAcc = pCmdCrd->prm.abr.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abr.synAcc;
	pCrdKer->helpPos[1] = pCmdCrd->prm.abr.a;
	pCrdKer->helpPos[2] = pCmdCrd->prm.abr.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，YZ平面圆弧插补（以终点位置和圆心位置作为输入参数）
 */
static ERROR_CODE ArcYZC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//当前的坐标系小于三个，但是该指令需要三轴的支持
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABC));
	pCmdCrd->prm.abc.velEnd = pCmdCrd->prm.abc.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.velEnd;
	pCmdCrd->prm.abc.synVel = pCmdCrd->prm.abc.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.synVel;
	pCmdCrd->prm.abc.synAcc = pCmdCrd->prm.abc.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abc.synAcc;
	pCrdKer->helpPos[1] = pCmdCrd->prm.abc.a;
	pCrdKer->helpPos[2] = pCmdCrd->prm.abc.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，ZX平面圆弧插补（以终点位置和半径作为输入参数）
 */
static ERROR_CODE ArcZXR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//当前的坐标系小于三个，但是该指令需要三轴的支持
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABR));
	pCmdCrd->prm.abr.velEnd = pCmdCrd->prm.abr.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.velEnd;
	pCmdCrd->prm.abr.synVel = pCmdCrd->prm.abr.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abr.synVel;
	pCmdCrd->prm.abr.synAcc = pCmdCrd->prm.abr.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abr.synAcc;
	pCrdKer->helpPos[2] = pCmdCrd->prm.abr.a;
	pCrdKer->helpPos[0] = pCmdCrd->prm.abr.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，ZX平面圆弧插补（以终点位置和圆心位置作为输入参数）
 */
static ERROR_CODE ArcZXC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//当前的坐标系小于三个，但是该指令需要三轴的支持
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//旋转方向
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[5], sizeof(CMDABC));
	pCmdCrd->prm.abc.velEnd = pCmdCrd->prm.abc.velEnd > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.velEnd;
	pCmdCrd->prm.abc.synVel = pCmdCrd->prm.abc.synVel > pCrdPrm->synVelMax?\
							pCrdPrm->synVelMax : pCmdCrd->prm.abc.synVel;
	pCmdCrd->prm.abc.synAcc = pCmdCrd->prm.abc.synAcc > pCrdPrm->synAccMax?\
							pCrdPrm->synAccMax : pCmdCrd->prm.abc.synAcc;
	pCrdKer->helpPos[2] = pCmdCrd->prm.abc.a;
	pCrdKer->helpPos[0] = pCmdCrd->prm.abc.b;
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，缓存区内数字量IO输出设置指令
 */
static ERROR_CODE BufIO()
{
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_Buf(&pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDIO));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，缓存区内延时设置指令
 */
static ERROR_CODE BufDealy()
{
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_Buf(&pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDDELAY));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，缓存区内输出DA值
 */
static ERROR_CODE BufDA()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//检查当前设置的DA的轴是否在坐标系里面
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//当前设置的轴在坐标系内，退出
		}
		else
		{
			if(count == (pCrdPrm->dimension - 1))
				return RTN_ERROR;
		}
	}

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDDA));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，缓存区内有效/无效限位开关
 */
static ERROR_CODE BufLmts()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//检查当前设置的轴是否在坐标系里面
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//当前设置的轴在坐标系内，退出
		}
		else
		{
			if(count == (pCrdPrm->dimension - 1))
				return RTN_ERROR;
		}
	}

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDLMTS));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，缓存区内设置axis的停止IO信息
 */
static ERROR_CODE BufSetStopIo()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//检查当前设置的轴是否在坐标系里面
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//当前设置的轴在坐标系内，退出
		}
		else
		{
			if(count == (pCrdPrm->dimension - 1))
				return RTN_ERROR;
		}
	}

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDSETSTOPIO));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，实现刀头跟随功能，启动某各轴点位运动
 * 该轴不能在坐标系中
 */
static ERROR_CODE BufMove()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;
	KernelPrm *pKer;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	pKer = &kernel[cmd.prm[4]];
	//检查当前设置的轴是否在坐标系里	pKer = &kernel[cmd.prm[4]];
	if(pKer->axsta == MOTORS_STA_CRDMODE)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDBUFFMOVE));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 缓存区指令，实现刀向跟随功能，启动某各轴跟随运动
 * 该轴不能在坐标系内
 */
static ERROR_CODE BufGear()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//临时变量
	CMDCRD *pCmdCrd;
	KernelPrm *pKer;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//检查当前设置的轴是否在坐标系里面
	pKer = &kernel[cmd.prm[4]];
	if(pKer->axsta == MOTORS_STA_CRDMODE)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDBUFFGEAR));
	pCrdKer->fifo[tcf.fifo] ++;			//自加，指向下一个
	return RTN_SUCC;
}

/*
 * 查询插补缓冲区剩余空间
 */
static ERROR_CODE CrdSpace()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	long space;							//用于存储插补缓冲区剩余空间

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	space  = CRDFIFOLEN - pCrdKer->fifo[cmd.prm[3]];
	memcpy(p,&space,2);
	cmd.buflen += 2;

	return RTN_SUCC;
}

/*
 * 清除插补缓存区内的插补数据
 */
static ERROR_CODE CrdClear()
{
	CRD_KERNEL* pCrdKer;
	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	pCrdKer->fifo[cmd.prm[3]] = 0;

	return RTN_SUCC;
}

/*
 * 启动插补运动
 */
static ERROR_CODE CrdStart()
{
	int crd;
	unsigned int *p = &cmd.prm[3];

	for(crd = 0; crd < CRDNUM; crd++)
	{
		if((cmd.mark >> crd) & 0x01)		//启动当前坐标系
		{
			if(CrdWithal(crd) == RTN_SUCC)	//坐标系依然成立
			{
				if(*p & 0x01)
				{
					if(Open_Crdmode(crd, 0) != RTN_SUCC)
						return RTN_ERROR;
				}
				else if(*p & 0x02)
				{
					if(Open_Crdmode(crd, 1) != RTN_SUCC)
						return RTN_ERROR;
				}
				else
					return RTN_ERROR;
			}
			else
				return RTN_ERROR;			//坐标系不成立，返回错误
		}
	}

	return RTN_SUCC;
}

/*
 * 查询插补运动坐标系状态
 */
static ERROR_CODE CrdStatus()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	short Run;							//插补运动的状态
	long Segment;						//读取当前已经完成的插补段数
	int usefifo;						//当前正在使用的fifo

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	usefifo = cmd.prm[3];
	if(pCrdKer->runstate == 1 || pCrdKer->runstate == 3)
		Run = 1;
	else
		Run = 0;

	Segment = pCrdKer->nowfifo[usefifo];
	memcpy(p, &Run, 1);
	p++;
	memcpy(p, &Segment, 2);
	cmd.buflen += 3;
	return RTN_SUCC;
}

/*
 * 读取未完成的插补段段数
 */
static ERROR_CODE GetRemainderSegNum()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	long Segment;						//读取当前已经完成的插补段数
	int usefifo;						//当前正在使用的fifo

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	usefifo = cmd.prm[3];

	Segment = pCrdKer->fifo[usefifo] - pCrdKer->nowfifo[usefifo];
	memcpy(p, &Segment, 2);
	cmd.buflen += 2;
	return RTN_SUCC;
}

/*
 * 设置插补运动平滑停止、急停合成加速度
 */
static ERROR_CODE SetCrdStopDec()
{
	unsigned int *p = &cmd.prm[3];
	CRD_KERNEL* pCrdKer;

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	memcpy(&(pCrdKer->decSmoothStop), p, 2);
	p += 2;
	memcpy(&(pCrdKer->decAbruptStop), p, 2);
	p += 2;
	return RTN_SUCC;
}

/*
 * 查询插补运动平滑停止、急停合成加速度
 */
static ERROR_CODE GetCrdStopDec()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	memcpy(p, &(pCrdKer->decSmoothStop), 2);
	p += 2;
	memcpy(p, &(pCrdKer->decAbruptStop), 2);
	p += 2;
	cmd.buflen += 4;
	return RTN_SUCC;
}

/*
 * 查询该坐标系的当前坐标位置值
 */
static ERROR_CODE GetCrdPos()
{
	unsigned int *p = cmd.buf;
	TCrdPrm *pCrdPrm;
	KernelPrm **pKernel;

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdPrm = &crdprm[0];
			pKernel = pKernelC[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdPrm = &crdprm[1];
			pKernel = pKernelC[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	memcpy(p++, &(pCrdPrm->dimension), 1);		//传回坐标系的大小
	cmd.buflen++;

	int count;
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		memcpy(p, &(*pKernel)->realPos, 2);
		p += 2;
		pKernel++;
		cmd.buflen += 2;
	}

	return RTN_SUCC;
}

/*
 * 查询该坐标系的当前合成速度值
 */
static ERROR_CODE GetCrdVel()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;

	//先检查参数
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[0];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//坐标系依然成立
		{
			pCrdKer = &crdkernel[1];
		}
		else							//坐标系不成立，返回错误
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	memcpy(p, &(pCrdKer->nowVel), 2);
	p += 2;
	cmd.buflen += 2;
	return RTN_SUCC;
}

/*
 * 解码坐标系指令
 */
void Decouple_CRDmode()
{
	switch(cmd.type)
	{
		case CMD_SET_CRD_PRM:				cmd.rtn = SetCrdPrm();				break;
		case CMD_GET_CRD_PRM:				cmd.rtn = GetCrdPrm();				break;
		case CMD_QUIT_CRD:					cmd.rtn = QuitCrd();				break;
		case CMD_CRD_DATA:					cmd.rtn = CrdData();				break;
		case CMD_LN_XY:						cmd.rtn = LnXY();					break;		//指令开始
		case CMD_LN_XYZ:					cmd.rtn = LnXYZ();					break;
		case CMD_LN_XYZA:					cmd.rtn = LnXYZA();					break;
		case CMD_LN_XYG0:					cmd.rtn = LnXYG0();					break;
		case CMD_LN_XYZG0:					cmd.rtn = LnXYZG0();				break;
		case CMD_LN_XYZAG0:					cmd.rtn = LnXYZAG0();				break;
		case CMD_ARC_XYR:					cmd.rtn = ArcXYR();					break;
		case CMD_ARC_XYC:					cmd.rtn = ArcXYC();					break;
		case CMD_ARC_YZR:					cmd.rtn = ArcYZR();					break;
		case CMD_ARC_YZC:					cmd.rtn = ArcYZC();					break;
		case CMD_ARC_ZXR:					cmd.rtn = ArcZXR();					break;
		case CMD_ARC_ZXC:					cmd.rtn = ArcZXC();					break;
		case CMD_BUF_IO:					cmd.rtn = BufIO();					break;
		case CMD_BUF_DELAY:					cmd.rtn = BufDealy();				break;
		case CMD_BUF_DA:					cmd.rtn = BufDA();					break;
		case CMD_BUF_LMTS_ON:				cmd.rtn = BufLmts();				break;
		case CMD_BUF_LMTS_OFF:				cmd.rtn = BufLmts();				break;
		case CMD_BUF_SET_STOP_IO:			cmd.rtn = BufSetStopIo();			break;
		case CMD_BUF_MOVE:					cmd.rtn = BufMove();				break;
		case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//指令结束
		case CMD_CRD_SPACE:					cmd.rtn = CrdSpace();				break;
		case CMD_CRD_CLEAR:					cmd.rtn = CrdClear();				break;
		case CMD_CRD_START:					cmd.rtn = CrdStart();				break;
		case CMD_CRD_STATUS:				cmd.rtn = CrdStatus();				break;
		case CMD_GET_REMAINDERS_SEG_NUM:	cmd.rtn = GetRemainderSegNum();		break;
		case CMD_SET_CRD_STOP_DEC:			cmd.rtn = SetCrdStopDec();			break;
		case CMD_GET_CRD_STOP_DEC:			cmd.rtn = GetCrdStopDec();			break;
		case CMD_GET_CRD_POS:				cmd.rtn = GetCrdPos();				break;
		case CMD_GET_CRD_VEL:				cmd.rtn = GetCrdVel();				break;
		case CMD_INIT_LOOK_AHEAD:												break;
		default:							cmd.rtn = RTN_INVALID_COMMAND;	    break;
	}
}

/*
 * 预处理二维直线模式
 */
static ERROR_CODE Prep_LNXY(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	long deltaX,deltaY;
	long endX,endY;
	CMDXY *pXY;

	pXY = &(pCmdCrd->prm.xy);
	endX = pCrdKer->zeroPos[0] + pXY->x;
	endY = pCrdKer->zeroPos[1] + pXY->y;

	pCrdKer->startPos[0] = (*(ppKernel))->nowPos;
	pCrdKer->startPos[1] = (*(ppKernel+1))->nowPos;

	deltaX = endX - pCrdKer->startPos[0];
	deltaY = endY - pCrdKer->startPos[1];

	double deltaPos = sqrt(deltaX*deltaX + deltaY*deltaY);
	pCrdKer->conver[0] = deltaX / deltaPos;		//此处系数自己会计算出正负
	pCrdKer->conver[1] = deltaY / deltaPos;

	if(pCrdKer->conver[0] >= 0)
	{
		(*ppKernel)->dir = 1;
	}
	else
	{
		(*ppKernel)->dir = 0;
	}

	if(pCrdKer->conver[1] >= 0)
	{
		(*(ppKernel+1))->dir = 1;
	}
	else
	{
		(*(ppKernel+1))->dir = 0;
	}

	double t0,t1,t2;
	double staVel = pCrdKer->nowVel;			//设置起始速度，起始速度与上一段速度衔接起来
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//平滑时间

	t0 = (pXY->synVel - staVel) / pXY->synAcc;
	pCrdKer->endPos[0] = (staVel + pXY->synVel) * t0 / 2;				//此处的Pos用作临时变量

	t2 = (pXY->synVel - pXY->velEnd) / pXY->synAcc;
	pCrdKer->endPos[2] = (pXY->synVel + pXY->velEnd) *t2 / 2;

	pCrdKer->endPos[1] = deltaPos - pCrdKer->endPos[0] - pCrdKer->endPos[2];
	if(pCrdKer->endPos[1] >= pXY->synVel*evenTime)
	{
		t1 = pCrdKer->endPos[1] / pXY->synVel;
		pCrdKer->endPos[1] += pCrdKer->endPos[0];
		pCrdKer->endPos[2] = deltaPos;
	}
	else
	{
		//此处的t1表示最大速度
		double b,c;
		b = pXY->synAcc*evenTime;
		c = pXY->synAcc * deltaPos + (staVel * staVel + pXY->velEnd + pXY->velEnd)/2.0;
		t1 = (sqrt(b*b + 4*c) - b)/2.0;
		pXY->synVel = t1;
		//t1 = sqrt((staVel*staVel + pXY->velEnd*pXY->velEnd + 2*pXY->synAcc*deltaPos)/ 2.0);
		t0 = fabs((t1 - staVel)/pXY->synAcc);
		t2 = fabs((t1 - pXY->velEnd)/pXY->synAcc);

		pCrdKer->endPos[0] = pXY->synAcc*t0*t0/2.0 + t0*staVel;
		pCrdKer->endPos[1] = pCrdKer->endPos[0] + t1*evenTime;
		pCrdKer->endPos[2] = deltaPos;
		t1 = evenTime;
	}

	pCrdKer->time[0] = Approximate(t0 * 10);
	pCrdKer->time[1] = Approximate(t1 * 10);
	pCrdKer->time[2] = Approximate(t2 * 10);
	(*(ppKernel))->flag = 0;
	(*(ppKernel))->count = 0;
	(*(ppKernel))->step = 0;

	pCmdCrd->tcf.ready = 1;							//表示已经预处理完成当前指令
	return RTN_SUCC;
}

/*
 * 预处理三维直线模式
 */
static ERROR_CODE Prep_LNXYZ(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	long deltaX,deltaY,deltaZ;
	long endX,endY,endZ;
	CMDXYZ *pXYZ;

	pXYZ = &(pCmdCrd->prm.xyz);
	endX = pCrdKer->zeroPos[0] + pXYZ->x;
	endY = pCrdKer->zeroPos[1] + pXYZ->y;
	endZ = pCrdKer->zeroPos[2] + pXYZ->z;

	pCrdKer->startPos[0] = (*(ppKernel))->nowPos;
	pCrdKer->startPos[1] = (*(ppKernel+1))->nowPos;
	pCrdKer->startPos[2] = (*(ppKernel+2))->nowPos;

	deltaX = endX - pCrdKer->startPos[0];
	deltaY = endY - pCrdKer->startPos[1];
	deltaZ = endZ - pCrdKer->startPos[2];

	double deltaPos = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
	pCrdKer->conver[0] = deltaX / deltaPos;		//此处系数自己会计算出正负
	pCrdKer->conver[1] = deltaY / deltaPos;
	pCrdKer->conver[2] = deltaZ / deltaPos;

	if(pCrdKer->conver[0] >= 0)
	{
		(*ppKernel)->dir = 1;
	}
	else
	{
		(*ppKernel)->dir = 0;
	}

	if(pCrdKer->conver[1] >= 0)
	{
		(*(ppKernel+1))->dir = 1;
	}
	else
	{
		(*(ppKernel+1))->dir = 0;
	}

	if(pCrdKer->conver[2] >= 0)
	{
		(*(ppKernel+2))->dir = 1;
	}
	else
	{
		(*(ppKernel+2))->dir = 0;
	}

	double t0,t1,t2;
	double staVel = pCrdKer->nowVel;			//设置起始速度，起始速度与上一段速度衔接起来
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//平滑时间

	t0 = (pXYZ->synVel - staVel) / pXYZ->synAcc;
	pCrdKer->endPos[0] = (staVel + pXYZ->synVel) * t0 / 2;				//此处的Pos用作临时变量

	t2 = (pXYZ->synVel - pXYZ->velEnd) / pXYZ->synAcc;
	pCrdKer->endPos[2] = (pXYZ->synVel + pXYZ->velEnd) *t2 / 2;

	pCrdKer->endPos[1] = deltaPos - pCrdKer->endPos[0] - pCrdKer->endPos[2];
	if(pCrdKer->endPos[1] >= pXYZ->synVel*evenTime)
	{
		t1 = pCrdKer->endPos[1] / pXYZ->synVel;
		pCrdKer->endPos[1] += pCrdKer->endPos[0];
		pCrdKer->endPos[2] = deltaPos;
	}
	else
	{
		//此处的t1表示最大速度
		double b,c;
		b = pXYZ->synAcc*evenTime;
		c = pXYZ->synAcc * deltaPos + (staVel * staVel + pXYZ->velEnd + pXYZ->velEnd)/2.0;
		t1 = (sqrt(b*b + 4*c) - b)/2.0;
		pXYZ->synVel = t1;
		//t1 = sqrt((staVel*staVel + pXY->velEnd*pXY->velEnd + 2*pXY->synAcc*deltaPos)/ 2.0);
		t0 = fabs((t1 - staVel)/pXYZ->synAcc);
		t2 = fabs((t1 - pXYZ->velEnd)/pXYZ->synAcc);

		pCrdKer->endPos[0] = pXYZ->synAcc*t0*t0/2.0 + t0*staVel;
		pCrdKer->endPos[1] = pCrdKer->endPos[0] + t1*evenTime;
		pCrdKer->endPos[2] = deltaPos;
		t1 = evenTime;
	}

	pCrdKer->time[0] = Approximate(t0 * 10);
	pCrdKer->time[1] = Approximate(t1 * 10);
	pCrdKer->time[2] = Approximate(t2 * 10);
	(*(ppKernel))->flag = 0;
	(*(ppKernel))->count = 0;
	(*(ppKernel))->step = 0;

	pCmdCrd->tcf.ready = 1;							//表示已经预处理完成当前指令
	return RTN_SUCC;
}

/*
 * 预处理四维直线模式
 */
static ERROR_CODE Prep_LNXYZA(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	long deltaX,deltaY,deltaZ,deltaA;
	long endX,endY,endZ,endA;
	CMDXYZA *pXYZA;

	pXYZA = &(pCmdCrd->prm.xyza);
	endX = pCrdKer->zeroPos[0] + pXYZA->x;
	endY = pCrdKer->zeroPos[1] + pXYZA->y;
	endZ = pCrdKer->zeroPos[2] + pXYZA->z;
	endA = pCrdKer->zeroPos[3] + pXYZA->a;

	pCrdKer->startPos[0] = (*(ppKernel))->nowPos;
	pCrdKer->startPos[1] = (*(ppKernel+1))->nowPos;
	pCrdKer->startPos[2] = (*(ppKernel+2))->nowPos;
	pCrdKer->startPos[3] = (*(ppKernel+3))->nowPos;

	deltaX = endX - pCrdKer->startPos[0];
	deltaY = endY - pCrdKer->startPos[1];
	deltaZ = endZ - pCrdKer->startPos[2];
	deltaA = endA - pCrdKer->startPos[3];

	double deltaPos = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ + deltaA*deltaA);
	pCrdKer->conver[0] = deltaX / deltaPos;		//此处系数自己会计算出正负
	pCrdKer->conver[1] = deltaY / deltaPos;
	pCrdKer->conver[2] = deltaZ / deltaPos;
	pCrdKer->conver[3] = deltaA / deltaPos;

	if(pCrdKer->conver[0] >= 0)
	{
		(*ppKernel)->dir = 1;
	}
	else
	{
		(*ppKernel)->dir = 0;
	}

	if(pCrdKer->conver[1] >= 0)
	{
		(*(ppKernel+1))->dir = 1;
	}
	else
	{
		(*(ppKernel+1))->dir = 0;
	}

	if(pCrdKer->conver[2] >= 0)
	{
		(*(ppKernel+2))->dir = 1;
	}
	else
	{
		(*(ppKernel+2))->dir = 0;
	}

	if(pCrdKer->conver[3] >= 0)
	{
		(*(ppKernel+3))->dir = 1;
	}
	else
	{
		(*(ppKernel+3))->dir = 0;
	}

	double t0,t1,t2;
	double staVel = pCrdKer->nowVel;			//设置起始速度，起始速度与上一段速度衔接起来
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//平滑时间

	t0 = (pXYZA->synVel - staVel) / pXYZA->synAcc;
	pCrdKer->endPos[0] = (staVel + pXYZA->synVel) * t0 / 2;				//此处的Pos用作临时变量

	t2 = (pXYZA->synVel - pXYZA->velEnd) / pXYZA->synAcc;
	pCrdKer->endPos[2] = (pXYZA->synVel + pXYZA->velEnd) *t2 / 2;

	pCrdKer->endPos[1] = deltaPos - pCrdKer->endPos[0] - pCrdKer->endPos[2];
	if(pCrdKer->endPos[1] >= pXYZA->synVel*evenTime)
	{
		t1 = pCrdKer->endPos[1] / pXYZA->synVel;
		pCrdKer->endPos[1] += pCrdKer->endPos[0];
		pCrdKer->endPos[2] = deltaPos;
	}
	else
	{
		//此处的t1表示最大速度
		double b,c;
		b = pXYZA->synAcc*evenTime;
		c = pXYZA->synAcc * deltaPos + (staVel * staVel + pXYZA->velEnd + pXYZA->velEnd)/2.0;
		t1 = (sqrt(b*b + 4*c) - b)/2.0;
		pXYZA->synVel = t1;
		//t1 = sqrt((staVel*staVel + pXY->velEnd*pXY->velEnd + 2*pXY->synAcc*deltaPos)/ 2.0);
		t0 = fabs((t1 - staVel)/pXYZA->synAcc);
		t2 = fabs((t1 - pXYZA->velEnd)/pXYZA->synAcc);

		pCrdKer->endPos[0] = pXYZA->synAcc*t0*t0/2.0 + t0*staVel;
		pCrdKer->endPos[1] = pCrdKer->endPos[0] + t1*evenTime;
		pCrdKer->endPos[2] = deltaPos;
		t1 = evenTime;
	}

	pCrdKer->time[0] = Approximate(t0 * 10);
	pCrdKer->time[1] = Approximate(t1 * 10);
	pCrdKer->time[2] = Approximate(t2 * 10);
	(*(ppKernel))->flag = 0;
	(*(ppKernel))->count = 0;
	(*(ppKernel))->step = 0;

	pCmdCrd->tcf.ready = 1;							//表示已经预处理完成当前指令
	return RTN_SUCC;
}

/*
 * 获取与圆相关的Index
 * 返回0：表示当前的指令为R
 * 返回1：表示当前的指令为C
 */
static void GetCirIndex(int *IndexA, int *IndexB ,COMMAND_TYPE type)
{
	switch(type)
	{
	case CMD_ARC_XYR:
	case CMD_ARC_XYC:
	{
		*IndexA = 0;
		*IndexB = 1;
		break;
	}
	case CMD_ARC_YZR:
	case CMD_ARC_YZC:
	{
		*IndexA = 1;
		*IndexB = 2;
		break;
	}
	case CMD_ARC_ZXR:
	case CMD_ARC_ZXC:
	{
		*IndexA = 2;
		*IndexB = 0;
		break;
	}
	default:		break;
	}
	return;
}

/*
 * 获取圆弧指令的参数的输入类型
 * 0：R版本
 * 1：C版本
 */
static int GetCirMode(COMMAND_TYPE type)
{
	int mode = 0;
	switch(type)
	{
	case CMD_ARC_XYR:
	case CMD_ARC_YZR:
	case CMD_ARC_ZXR:
		mode = 0;		break;
	case CMD_ARC_XYC:
	case CMD_ARC_YZC:
	case CMD_ARC_ZXC:
		mode = 1;		break;
	default:		break;
	}
	return mode;
}

/*
 * 预处理平面圆弧插补（以终点位置和圆心为输入参数）
 */
static ERROR_CODE Prep_Circle(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel, COMMAND_TYPE type)
{
	CMDABR *pABR;
	CMDABC *pABC;
	int mode;
	double r;
	int IndexA,IndexB;
	double deltaA,deltaB;
	double deltaPos;

	pABR = &pCmdCrd->prm.abr;
	pABC = &pCmdCrd->prm.abc;
	GetCirIndex(&IndexA, &IndexB, type);
	mode = GetCirMode(type);

	pCrdKer->startPos[IndexA] = (*(ppKernel+IndexA))->nowPos;						//获取起始的坐标
	pCrdKer->startPos[IndexB] = (*(ppKernel+IndexB))->nowPos;

	if(mode == 1)			//C版本
	{
		pCrdKer->ac = pABC->ac + pCrdKer->zeroPos[IndexA];								//获取圆心位置
		pCrdKer->bc = pABC->bc + pCrdKer->zeroPos[IndexB];

		deltaPos = (pABC->a - pABC->ac) * (pABC->a - pABC->ac) + (pABC->b - pABC->bc) * (pABC->b - pABC->bc);
		r = sqrt(deltaPos);		//获取半径
	}
	else					//R版本
	{
		r = pABR->r;
		double x1, y1, x2, y2, A, B, C, c1, c2;
		double cx1, cy1, cx2, cy2;
		double delta;
		int dir = 0;

		x1 = pCrdKer->startPos[IndexA];
		y1 = pCrdKer->startPos[IndexB];
		x2 = pCrdKer->zeroPos[IndexA] + pABR->a;
		y2 = pCrdKer->zeroPos[IndexB] + pABR->b;

		if(x1 != x2)
		{
			c1 = (x2*x2 - x1*x1 + y2*y2 - y1*y1) / (2 *(x2 - x1));
			c2 = (y2 - y1) / (x2 - x1);
			A = (c2*c2 + 1);
			B = (2 * x1*c2 - 2 * c1*c2 - 2 * y1);
			C = x1*x1 - 2 * x1*c1 + c1*c1 + y1*y1 - r*r;
			delta = B*B - 4 * A*C;
			delta = sqrt(delta);
			//生成圆心位置，圆心位置由两个
			cy1 = (-B + delta) / (2 * A);
			cx1 = c1 - c2 * cy1;
			cy2 = (-B - delta) / (2 * A);
			cx2 = c1 - c2 * cy2;
		}
		else
		{
			y1 = (y1 + y2) * 0.5;
			delta = r*r - (y1 - y2)*(y1 - y2);									//顺序不能乱
			y2 = y1;
			delta = sqrt(delta);
			x1 -= delta;
			x2 += delta;
		}
		if(r > 0)							//小角度弧度
		{
			if(pCmdCrd->tcf.cdir == 1)		//需要逆时针转动
			{
				dir = 1;					//左侧
			}
			else							//需要顺时针转动
			{
				dir = 0;					//右侧
			}
		}
		else								//大角度弧度
		{
			if(pCmdCrd->tcf.cdir == 1)		//需要逆时针转动
			{
				dir = 0;			//右侧
			}
			else							//需要顺时针转动
			{
				dir = 1;			//左侧
			}
		}
		delta = (x1 - cx1) * (y2 - cy1) - (x2 - cx1) * (y1 - cy1);
		if(delta > 0)		//当前点在直线左侧
		{
			if(dir == 1)
			{
				pCrdKer->ac = cx1;
				pCrdKer->bc = cy1;
			}
			else
			{
				pCrdKer->ac = cx2;
				pCrdKer->bc = cy2;
			}
		}
		else if(delta < 0)	//当前点在直线右侧
		{
			if(dir == 0)
			{
				pCrdKer->ac = cx1;
				pCrdKer->bc = cy1;
			}
			else
			{
				pCrdKer->ac = cx2;
				pCrdKer->bc = cy2;
			}
		}
		else				//当前点在直线上，求出来的两个点重合，取任何一个都可以
		{
			pCrdKer->ac = cx1;
			pCrdKer->bc = cy1;
		}



	}
	r = fabs(r);
	pCrdKer->r = r;														//存储半径的大小，r大小有正有负


	deltaA = pCrdKer->startPos[IndexA] - pCrdKer->ac;								//起始
	deltaB = pCrdKer->startPos[IndexB] - pCrdKer->bc;
	pCrdKer->conver[0] = atan2(deltaB, deltaA);										//起始时候的角度值

	deltaA = pABC->a + pCrdKer->zeroPos[IndexA] - pCrdKer->ac;						//结束
	deltaB = pABC->b + pCrdKer->zeroPos[IndexB] - pCrdKer->bc;
	pCrdKer->conver[3] = atan2(deltaB, deltaA);										//结束时候的角度值

	if(pCmdCrd->tcf.cdir == 1)														//逆时针转动，角度应该增加
	{
		if(pCrdKer->conver[3] < pCrdKer->conver[0])
			pCrdKer->conver[3] += M_2PI;

		deltaPos = pCrdKer->conver[3] - pCrdKer->conver[0];
	}
	else
	{								//当前，也就是起始的角速度，负值
		if(pCrdKer->conver[3] > pCrdKer->conver[0])
			pCrdKer->conver[3] -= M_2PI;

		deltaPos = pCrdKer->conver[0] - pCrdKer->conver[3];
	}
	deltaPos = deltaPos * r;

	double t0,t1,t2;
	long evenTime;
	double staVel;

	double synVel;
	double synAcc;
	double velEnd;

	synVel = pABR->synVel;
	synAcc = pABR->synAcc;
	velEnd = pABR->velEnd;

	evenTime = pCrdKer->pCrdPrm->evenTime;		//平滑时间
	pCrdKer->staVel = pCrdKer->nowVel;												//设置起始速度，起始速度与上一段速度衔接起来
	staVel = pCrdKer->staVel;

	t0 = (synVel - staVel) / synAcc;
	pCrdKer->endPos[0] = (staVel + synVel) * t0 / 2;				//此处的Pos用作临时变量

	t2 = (synVel - velEnd) / synAcc;
	pCrdKer->endPos[2] = (synVel + velEnd) *t2 / 2;

	pCrdKer->endPos[1] = deltaPos - pCrdKer->endPos[0] - pCrdKer->endPos[2];
	if(pCrdKer->endPos[1] >= synVel*evenTime)
	{
		t1 = pCrdKer->endPos[1] / synVel;
		pCrdKer->endPos[1] += pCrdKer->endPos[0];
		pCrdKer->endPos[2] = deltaPos;
	}
	else
	{
		//此处的t1表示最大速度
		double b,c;
		b = synAcc*evenTime;
		c = synAcc * deltaPos + (staVel * staVel + velEnd + velEnd)/2.0;
		t1 = (sqrt(b*b + 4*c) - b)/2.0;

		pABC->synVel = t1;					//此处需要幅值，使用指针指向

		//t1 = sqrt((staVel*staVel + pXY->velEnd*pXY->velEnd + 2*pXY->synAcc*deltaPos)/ 2.0);
		t0 = fabs((t1 - staVel)/synAcc);
		t2 = fabs((t1 - velEnd)/synAcc);

		pCrdKer->endPos[0] = synAcc*t0*t0/2.0 + t0*staVel;
		pCrdKer->endPos[1] = pCrdKer->endPos[0] + t1*evenTime;
		pCrdKer->endPos[2] = deltaPos;
		t1 = evenTime;
	}

	pCrdKer->time[0] = Approximate(t0 * 10);
	pCrdKer->time[1] = Approximate(t1 * 10);
	pCrdKer->time[2] = Approximate(t2 * 10);
	(*(ppKernel))->flag = 0;
	(*(ppKernel))->count = 0;
	(*(ppKernel))->step = 0;

	pCmdCrd->tcf.ready = 1;							//表示已经预处理完成当前指令
	return RTN_SUCC;
}

/*
 * 预处理缓冲区内延时设置指令
 */
static ERROR_CODE Prep_BufDelay(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
//	pCrdKer->startPos[0] = (*(ppKernel))->nowPos;
//	pCrdKer->startPos[1] = (*(ppKernel+1))->nowPos;
//
//	(*ppKernel)->dir = 1;
//	(*ppKernel+1)->dir = 1;

	pCrdKer->time[0] = Approximate(pCmdCrd->prm.delay.delayTimel * 10);
	(*(ppKernel))->flag = 0;
	(*(ppKernel))->count = 0;
	(*(ppKernel))->step = 0;

	pCmdCrd->tcf.ready = 1;							//表示已经预处理完成当前指令
	return RTN_SUCC;
}

ERROR_CODE Prep_CRDmode(int axis)
{
	//判断使用哪一个坐标系		使用crdindec[axis]
	//判断使用的是哪一个fifo					既然能运动，一定是检查过了，当前的fifo中一定有数据，但是执行后，会减少数据，所以还需继续检查FIFO中的数据
	//判断当前执行的是哪一条指令
	int crd;				//坐标系
	int fifo;
	CRD_KERNEL *pCrdKer;					//内核指针
	CMDCRD *pCmdCrd;						//指令指针
	KernelPrm **pKernel;					//指向内核


	crd = crdindex[axis];
	pCrdKer = &crdkernel[crd];
	pKernel = pKernelC[crd];

	//检查是否需要重新规划
	if((*pKernel)->flag == 0)					//不需要，直接退出啊
		return RTN_SUCC;

	//判断状态
	if(pCrdKer->runstate == 1)
		fifo = 0;
	else if(pCrdKer->runstate == 3)
		fifo = 1;
	else
		return RTN_ERROR;
/******************************************************************************************************/
//	if(pCrdKer->fifo[fifo] == pCrdKer->nowfifo[fifo])					//这部分代码也可以交给中断来处理，暂时已经放在中断中了
//	{
//		//数据执行完成，世界退出FIFO
//		//Quit_Crdmode();
//	}
/******************************************************************************************************/
	pCmdCrd= &crdfifo[crd][fifo][pCrdKer->nowfifo[fifo]];
	pCrdKer->pCmdCrd = pCmdCrd;

	if(pCmdCrd->tcf.ready == 1)											//当前指令不可以准备，尝试准备下一条指令
	{
		return RTN_SUCC;					//当前指令预处理完成，且未开始执行，返回正确
	}
	else if(pCmdCrd->tcf.ready != 0)		//可以预处理下一条指令
	{
		return RTN_ERROR;
	}

//	//检查数据
//#ifdef CHECK
//	if(pCmdCrd->tcf.fifo != fifo)
//		asm( " ESTOP" );
//#endif
	//开始预处理数据
	COMMAND_TYPE type;
	type = (COMMAND_TYPE)(pCmdCrd->tcf.type + CMD_CRD_MODE);
	switch(type)
	{
		case CMD_LN_XY:						return Prep_LNXY(pCrdKer, pCmdCrd, pKernel);
		case CMD_LN_XYZ:					return Prep_LNXYZ(pCrdKer, pCmdCrd, pKernel);
		case CMD_LN_XYZA:					return Prep_LNXYZA(pCrdKer, pCmdCrd, pKernel);
		case CMD_LN_XYG0:					return Prep_LNXY(pCrdKer, pCmdCrd, pKernel);
		case CMD_LN_XYZG0:					return Prep_LNXYZ(pCrdKer, pCmdCrd, pKernel);
		case CMD_LN_XYZAG0:					return Prep_LNXYZA(pCrdKer, pCmdCrd, pKernel);
		case CMD_ARC_XYR:
		case CMD_ARC_XYC:
		case CMD_ARC_YZR:
		case CMD_ARC_YZC:
		case CMD_ARC_ZXR:
		case CMD_ARC_ZXC:					return Prep_Circle(pCrdKer, pCmdCrd, pKernel, type);
		case CMD_BUF_IO:					break;
		case CMD_BUF_DELAY:					return Prep_BufDelay(pCrdKer, pCmdCrd, pKernel);
		case CMD_BUF_DA:					break;
		case CMD_BUF_LMTS_ON:				break;
		case CMD_BUF_LMTS_OFF:				break;
		case CMD_BUF_SET_STOP_IO:			cmd.rtn = BufSetStopIo();			break;
		case CMD_BUF_MOVE:					cmd.rtn = BufMove();				break;
		case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//指令结束
		default:							return RTN_ERROR;
	}

	//对于某些不需要预处理的指令，执行以下程序
	pCmdCrd->tcf.type = 1;
	(*(pKernel))->flag = 0;
	return RTN_SUCC;
}

/*
 * 运行二维直线模式
 */
static int Run_LNXY(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over：运行结束标志位

	acc = pCmdCrd->prm.xy.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//最后一次
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xy.synVel;

				(*ppKernel)->count = 0;								//为转跳做准备
				(*ppKernel)->step = 1;
			}
			else													//非最后一次
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//转跳下一步
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xy.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//最后一次
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//非最后一次
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xy.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//转跳下一步
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//最后一步，转跳出去
		{
			over = 1;
			pos = pCrdKer->endPos[2];
			pCrdKer->nowVel = pCmdCrd->prm.xy.velEnd;
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 0;
		}
		else
		{
			pos = pCrdKer->endPos[1] - acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCmdCrd->prm.xy.synVel * 0.1;
			pCrdKer->nowVel = pCmdCrd->prm.xy.synVel - (*ppKernel)->count * pCmdCrd->prm.xy.velEnd * 0.1;
		}
		break;
	}
	default:		break;
	}

	int count;
	for(count = 0; count < 2; count++)
	{
		(*(ppKernel+count))->realPos = pos * pCrdKer->conver[count] + pCrdKer->startPos[count];
		(*(ppKernel+count))->nowVel = pCrdKer->nowVel * pCrdKer->conver[count];
		(*(ppKernel+count))->nowacc = pCrdKer->nowAcc * pCrdKer->conver[count];
	}
	if(over == 1)
	{
		(*(ppKernel))->realPos = pCrdKer->zeroPos[0] + pCmdCrd->prm.xy.x;
		(*(ppKernel+1))->realPos = pCrdKer->zeroPos[1] + pCmdCrd->prm.xy.y;
	}

	long Pos;
	for(count = 0; count < 2; count++)
	{
		Pos = Approximate((*(ppKernel+count))->realPos);
		pvat.aim_pos = Pos;
		pvat.start_acc = 0;
		pvat.start_vel = (Pos - (*(ppKernel+count))->nowPos)*10000;
		pvat.min_period = TIME;
		M_SetPvat((*(ppKernel+count))->axis, &pvat);
		(*(ppKernel+count))->nowPos = pvat.aim_pos;
	}

	return over;
}

/*
 * 运行三维直线模式
 */
static int Run_LNXYZ(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over：运行结束标志位

	acc = pCmdCrd->prm.xyz.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//最后一次
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xyz.synVel;

				(*ppKernel)->count = 0;								//为转跳做准备
				(*ppKernel)->step = 1;
			}
			else													//非最后一次
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//转跳下一步
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xyz.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//最后一次
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//非最后一次
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xyz.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//转跳下一步
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//最后一步，转跳出去
		{
			over = 1;
			pos = pCrdKer->endPos[2];
			pCrdKer->nowVel = pCmdCrd->prm.xyz.velEnd;
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 0;
		}
		else
		{
			pos = pCrdKer->endPos[1] - acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCmdCrd->prm.xyz.synVel * 0.1;
			pCrdKer->nowVel = pCmdCrd->prm.xyz.synVel - (*ppKernel)->count * pCmdCrd->prm.xyz.velEnd * 0.1;
		}
		break;
	}
	default:		break;
	}

	int count;
	for(count = 0; count < 3; count++)
	{
		(*(ppKernel+count))->realPos = pos * pCrdKer->conver[count] + pCrdKer->startPos[count];
		(*(ppKernel+count))->nowVel = pCrdKer->nowVel * pCrdKer->conver[count];
		(*(ppKernel+count))->nowacc = pCrdKer->nowAcc * pCrdKer->conver[count];
	}
	if(over == 1)
	{
		(*(ppKernel))->realPos = pCrdKer->zeroPos[0] + pCmdCrd->prm.xyz.x;
		(*(ppKernel+1))->realPos = pCrdKer->zeroPos[1] + pCmdCrd->prm.xyz.y;
		(*(ppKernel+2))->realPos = pCrdKer->zeroPos[2] + pCmdCrd->prm.xyz.z;
	}

	long Pos;
	for(count = 0; count < 3; count++)
	{
		Pos = Approximate((*(ppKernel+count))->realPos);
		pvat.aim_pos = Pos;
		pvat.start_acc = 0;
		pvat.start_vel = (Pos - (*(ppKernel+count))->nowPos)*10000;
		pvat.min_period = TIME;
		M_SetPvat((*(ppKernel+count))->axis, &pvat);
		(*(ppKernel+count))->nowPos = pvat.aim_pos;
	}
	return over;
}

/*
 * 运行四维直线模式
 */
static int Run_LNXYZA(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over：运行结束标志位

	acc = pCmdCrd->prm.xyza.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//最后一次
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xyza.synVel;

				(*ppKernel)->count = 0;								//为转跳做准备
				(*ppKernel)->step = 1;
			}
			else													//非最后一次
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//转跳下一步
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xyza.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//最后一次
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//非最后一次
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xyza.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//转跳下一步
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//最后一步，转跳出去
		{
			over = 1;
			pos = pCrdKer->endPos[2];
			pCrdKer->nowVel = pCmdCrd->prm.xyza.velEnd;
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 0;
		}
		else
		{
			pos = pCrdKer->endPos[1] - acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCmdCrd->prm.xyza.synVel * 0.1;
			pCrdKer->nowVel = pCmdCrd->prm.xyza.synVel - (*ppKernel)->count * pCmdCrd->prm.xyza.velEnd * 0.1;
		}
		break;
	}
	default:		break;
	}

	int count;
	for(count = 0; count < 4; count++)
	{
		(*(ppKernel+count))->realPos = pos * pCrdKer->conver[count] + pCrdKer->startPos[count];
		(*(ppKernel+count))->nowVel = pCrdKer->nowVel * pCrdKer->conver[count];
		(*(ppKernel+count))->nowacc = pCrdKer->nowAcc * pCrdKer->conver[count];
	}
	if(over == 1)
	{
		(*(ppKernel))->realPos = pCrdKer->zeroPos[0] + pCmdCrd->prm.xyza.x;
		(*(ppKernel+1))->realPos = pCrdKer->zeroPos[1] + pCmdCrd->prm.xyza.y;
		(*(ppKernel+2))->realPos = pCrdKer->zeroPos[2] + pCmdCrd->prm.xyza.z;
		(*(ppKernel+3))->realPos = pCrdKer->zeroPos[3] + pCmdCrd->prm.xyza.a;
	}

	long Pos;
	for(count = 0; count < 4; count++)
	{
		Pos = Approximate((*(ppKernel+count))->realPos);
		pvat.aim_pos = Pos;
		pvat.start_acc = 0;
		pvat.start_vel = (Pos - (*(ppKernel+count))->nowPos)*10000;
		pvat.min_period = TIME;
		M_SetPvat((*(ppKernel+count))->axis, &pvat);
		(*(ppKernel+count))->nowPos = pvat.aim_pos;
	}

	return over;
}

/*
 * 运行平面圆弧插补
 * 两种输入参数的圆弧一起运行
 * 1.以终点位置和半径为输入参数		R版本
 * 2.以终点位置和圆心为输入参数		C版本
 */
static int Run_Circle(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel, COMMAND_TYPE type)
{
	PVAT_S pvat;
	double pos;
	double acc;
	double synVel,velEnd;
	double endA,endB;
	int IndexA,IndexB;
	int over = 0;				//over：运行结束标志位

	GetCirIndex(&IndexA, &IndexB, type);

	//对于两种圆弧的运动来说，以下五个参数，在内存中的地址是一样的，可以互相代替
	synVel = pCmdCrd->prm.abc.synVel;
	acc = pCmdCrd->prm.abc.synAcc;
	velEnd = pCmdCrd->prm.abc.velEnd;
	endA = pCmdCrd->prm.abc.a;
	endB = pCmdCrd->prm.abc.b;


	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//最后一次
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = synVel;

				(*ppKernel)->count = 0;								//为转跳做准备
				(*ppKernel)->step = 1;
			}
			else													//非最后一次
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//转跳下一步
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//最后一次
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//非最后一次
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//转跳下一步
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//最后一步，转跳出去
		{
			over = 1;
			pos = pCrdKer->endPos[2];
			pCrdKer->nowVel = velEnd;
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 0;
		}
		else
		{
			pos = pCrdKer->endPos[1] - acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * synVel * 0.1;
			pCrdKer->nowVel = synVel - (*ppKernel)->count * velEnd * 0.1;
		}
		break;
	}
	default:		break;
	}

	double theta;			//角度
	double r;
	r = pCrdKer->r;
	theta = pos / r;		//角度
	if(pCrdKer->pCmdCrd->tcf.cdir == 1)
	{
		theta = pCrdKer->conver[0] + theta;
		(*(ppKernel+IndexA))->realPos = pCrdKer->ac + r*cos(theta);
		(*(ppKernel+IndexA))->nowVel = -pCrdKer->nowVel * sin(theta);
		(*(ppKernel+IndexA))->nowacc = -pCrdKer->nowAcc * sin(theta);
		(*(ppKernel+IndexB))->realPos = pCrdKer->bc + r*sin(theta);
		(*(ppKernel+IndexB))->nowVel = pCrdKer->nowVel * cos(theta);
		(*(ppKernel+IndexB))->nowacc = pCrdKer->nowAcc * cos(theta);
	}
	else
	{
		theta = pCrdKer->conver[0] - theta;
		(*(ppKernel+IndexA))->realPos = pCrdKer->ac + r*cos(theta);
		(*(ppKernel+IndexA))->nowVel = pCrdKer->nowVel * sin(theta);
		(*(ppKernel+IndexA))->nowacc = pCrdKer->nowAcc * sin(theta);
		(*(ppKernel+IndexB))->realPos = pCrdKer->bc + r*sin(theta);
		(*(ppKernel+IndexB))->nowVel = -pCrdKer->nowVel * cos(theta);
		(*(ppKernel+IndexB))->nowacc = -pCrdKer->nowAcc * cos(theta);
	}

	if(over == 1)
	{
		(*(ppKernel+IndexA))->realPos = pCrdKer->zeroPos[IndexA] + endA;
		(*(ppKernel+IndexB))->realPos = pCrdKer->zeroPos[IndexB] + endB;
	}

	long Pos;
	Pos = Approximate((*(ppKernel+IndexA))->realPos);
	pvat.aim_pos = Pos;
	pvat.start_acc = 0;
	pvat.start_vel = (Pos - (*(ppKernel+IndexA))->nowPos)*10000;
	pvat.min_period = TIME;
	M_SetPvat((*(ppKernel+IndexA))->axis, &pvat);
	(*(ppKernel+IndexA))->nowPos = pvat.aim_pos;

	Pos = Approximate((*(ppKernel+IndexB))->realPos);
	pvat.aim_pos = Pos;
	pvat.start_vel = (Pos - (*(ppKernel+IndexB))->nowPos)*10000;
	M_SetPvat((*(ppKernel+IndexB))->axis, &pvat);
	(*(ppKernel+IndexB))->nowPos = pvat.aim_pos;

	return over;
}

/*
 * 预处理缓冲区内延时设置指令
 */
static int Run_BufDelay(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	int over = 0;				//over：运行结束标志位

	pCrdKer->nowAcc = 0;
	pCrdKer->nowVel = 0;
	if(pCrdKer->time[0] != 0)
	{
		(*ppKernel)->count++;
		if((*ppKernel)->count >= pCrdKer->time[0])				//最后一次
		{
			over = 1;
			(*ppKernel)->count = 0;								//为转跳做准备
		}
	}
	else
	{
		(*ppKernel)->count = 0;
		over = 1;
	}
	return over;
}

/*
 * 运行内核，依据当前的运行的指针，调用内核
 * 1.先依据轴，找到对应的坐标系，进入坐标系后，表示坐标系已经被锁住了，无法直接修改当前轴的状态了
 * 2.找到当前需要运行的指令，判断指令能否运行
 * 3.指令能运行，则运行指令
 */
void Run_CRDmode(int axis)
{
	int crd;				//坐标系
	int fifo;
	int over;				//当前指令的结束标志位
	CRD_KERNEL *pCrdKer;					//内核指针
	CMDCRD *pCmdCrd;						//指令指针
	KernelPrm **pKernel;					//指向内核
	COMMAND_TYPE type;


	crd = crdindex[axis];					//找出当前使用的坐标系
	pCrdKer = &crdkernel[crd];
	pKernel = pKernelC[crd];

	if((*pKernel)->flag == 1)		//数据没有处理完成，直接跳出
		return;

	pCmdCrd = pCrdKer->pCmdCrd;
	if(pCmdCrd->tcf.ready != 1)											//当前指令不可以准备，尝试准备下一条指令
	{
		return;						//当前指令预处理完成，且未开始执行，返回
	}

	type = (COMMAND_TYPE)(pCmdCrd->tcf.type + CMD_CRD_MODE);

	switch(type)
	{
	case CMD_LN_XY:						over = Run_LNXY(pCrdKer, pCmdCrd, pKernel);			break;
	case CMD_LN_XYZ:					over = Run_LNXYZ(pCrdKer, pCmdCrd, pKernel);		break;
	case CMD_LN_XYZA:					over = Run_LNXYZA(pCrdKer, pCmdCrd, pKernel);		break;
	case CMD_LN_XYG0:					over = Run_LNXY(pCrdKer, pCmdCrd, pKernel);			break;
	case CMD_LN_XYZG0:					over = Run_LNXYZ(pCrdKer, pCmdCrd, pKernel);		break;
	case CMD_LN_XYZAG0:					over = Run_LNXYZA(pCrdKer, pCmdCrd, pKernel);		break;
	case CMD_ARC_XYR:
	case CMD_ARC_XYC:
	case CMD_ARC_YZR:
	case CMD_ARC_YZC:
	case CMD_ARC_ZXR:
	case CMD_ARC_ZXC:					over = Run_Circle(pCrdKer, pCmdCrd, pKernel, type);	break;
	case CMD_BUF_IO:					break;
	case CMD_BUF_DELAY:					over = Run_BufDelay(pCrdKer, pCmdCrd, pKernel);		break;
	case CMD_BUF_DA:					break;
	case CMD_BUF_LMTS_ON:				break;
	case CMD_BUF_LMTS_OFF:				break;
	case CMD_BUF_SET_STOP_IO:			cmd.rtn = BufSetStopIo();			break;
	case CMD_BUF_MOVE:					cmd.rtn = BufMove();				break;
	case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//指令结束
	default:							return;
	}

	if(over == 1)			//当前的指令执行完成，分析是需要执行下一条指令还是退出程序
	{
		pCmdCrd->tcf.ready = 2;			//当前指令处理结束

		if(pCrdKer->runstate == 1)
			fifo = 0;
		else if(pCrdKer->runstate == 3)
			fifo = 1;
		else
		{
			return;
		}
		pCrdKer->nowfifo[fifo] ++;
		if(pCrdKer->nowfifo[fifo] == pCrdKer->fifo[fifo])		//退出坐标系
		{
			pCrdKer->nowfifo[fifo] = 0;
			pCrdKer->fifo[fifo] = 0;
			if(pCrdKer->runstate == 1)
			{
				pCrdKer->runstate = 0;
			}
			else
			{
				pCrdKer->runstate = 2;
			}
			Quit_Crdmode(crd, fifo);
		}
		else
		{
			pCrdKer->pCmdCrd++;			//指向下一个
			(*pKernel)->flag = 1;
		}
	}

	return;
}

/******************************************************************************************************************
 ********************************************     以下代码与停止有关          *******************************************
 *****************************************************************************************************************/
/*
 * 如果是运动模式下，需要停止一步一步停止运动
 *
 * 如何预处理信息
 * 需要判断当前的运动牵扯了哪些轴心
 */
/*
 * 直线模式停止
 */
void Stop_LINE()
{
	int crd;				//坐标系
	int fifo;
	int over;				//当前指令的结束标志位
	CRD_KERNEL *pCrdKer;	//内核指针
	CMDCRD *pCmdCrd;		//指令指针
	KernelPrm **pKernel;	//指向内核
	COMMAND_TYPE type;


	return;
}

/*
 * 停止模式，使用
 */
void Stop_CRDmode(int axis)
{
	return;
}
