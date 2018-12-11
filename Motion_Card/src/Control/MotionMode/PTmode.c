/*
 * 		  File: PTmode.c
 *     Version:
 * Description: PTģʽ
 * 				PTģʽʹ��һϵ�С�λ�á�ʱ�䡱���ݵ������ٶȹ滮���û���Ҫ���ٶ����߷ָ�
 * 				�����ɶΡ�PTģʽ�����ݶ�Ҫ���û�����ÿ�������ʱ���λ�õ㡣
 * 				PTģʽ������ͨ�Σ����ٶΣ�ֹͣ��
 * 				��̬ģʽ�£�����FIFO1��FIFO2������FIFO��СΪ1024�����2048��С�ĳ���
 * 				FIFOʹ�á�
 * 				��̬ģʽ�£�����ѡ��ʹ�õ�FIFO����С�̶�Ϊ1024����֧��ѭ������
 *
 *				ע��
 * 				һ��������һ��������PT�˶�ʱ����һ�ε����λ�ú�ʱ�䱻�ٶ�Ϊ0��ѹ�����
 * 				��������Ϊλ�õ㣬������ڵ�1�������ľ���ֵ��������ÿ��λ�Ƴ��ȡ�λ��
 * 				�ĵ�λ�����壨pulse����ʱ�䵥λ�Ǻ��루ms����
 * 
 *  Created on: 2018��3��15��
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#include "system.h"
#include <Kernel/interpolate.h>
#include "taskComm.h"
#include "PTmode.h"

#define FIFO_CHANNEL_NUM	2			//FIFO����
#define FIFO_MAX_LENGTH		1024		//ÿ��FIFO���ĸ���

/*
 * ����FIFOʹ��ģʽ
 */
#define PT_MODE_STATIC		0		//��̬ģʽ
#define PT_MODE_DYNAMIC		1		//��̬ģʽ
/*
 * ���ݶ����ͺ궨��
 */
#define PT_SEGMENT_NORMAL	0		//��ͨ��
#define PT_SEGMENT_EVEN		1		//���ٶ�
#define PT_SEGMENT_STOP		2		//���ٶ�

typedef struct
{
	//����˳���ֹ�޸�
	long pos;			//λ��
	long time;			//ʱ��
	int type;			//���ݶ�����		��ͨ�� ���ٶ� ���ٶ�
}PT_FIFO;

//���ᣬÿ�����Ӧ����FIFO��ÿ��FIFO���֧��1024��
PT_FIFO ptfifo[AXISNUM][FIFO_CHANNEL_NUM][FIFO_MAX_LENGTH];
#pragma DATA_SECTION(ptfifo,"EXTRAM_DATA");

typedef struct
{
	int mode;			//��̬ģʽ���߶�̬ģʽ
	int fifo;			//��̬ģʽ�£���¼��ǰ����ʹ�õ�FIFO
	int nowpoint;		//��ǰFIFOʹ�õ�ָ��
	int fpoint1;		//FIFO1�е����ݸ���
	int fpoint2;		//FIFO2�е����ݸ���
	long loop;			//ѭ������
	long nowloop;		//��ǰ��ѭ������

	long zeropos;		//��ǰPTģʽ��ԭ���ʵ��λ��
	long startpos;		//��ʼʱ��λ��
	long objpos;		//Ŀ��λ��
	long runtime;		//����ʱ��
	double startVel;	//��ʼ�ٶ�
	double endVel;		//�����ٶ�
	double acc;			//���ٶ�
}PT_KERNEL;
static PT_KERNEL ptkernel[AXISNUM];

/*
 * ��ʼ��PT_KERNEL
 */
void Init_PT_Kernel()
{
	int axis;
	PT_KERNEL *pPt = ptkernel;
	for(axis = 0; axis < AXISNUM; axis++)
	{
		pPt->fifo = 0;
		pPt->nowpoint = 0;
		pPt->fpoint1 = 0;						//FIFO1����������
		pPt->fpoint2 = 0;						//FIFO2����������
		pPt->loop = 1;							//��ʼ��Ϊ1��Ĭ��ִֻ��1��
		pPt->nowloop = 0;
		pPt ++;
	}
}

/*
 * �趨ָ����ΪPT�˶�ģʽ
 * ͬʱ����Ҫ���þ�̬��̬ģʽ
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
			if(kernel[axis].kersta == MOTORS_STA_IDLE)		//��鵱ǰ���Ƿ�������
			{
				pPt = &ptkernel[axis];
				kernel[axis].axsta = MOTORS_STA_PTMODE;
				pPt->mode = *p;							//���þ�̬���߶�̬ģʽ
				if(pPt->mode == PT_MODE_STATIC)
				{
					/*
					 * Insert Code
					 * �����ʼ�������Ĵ��룬����Ŀǰ�����ò���
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
 * ��ѯPT�˶�ģʽָ��FIFO��ʣ��ռ�
 * ��̬ģʽ��ָ��FIFO��Ч�����ǿ��Բ�ѯʣ��Ŀռ�
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
			if(pPt->mode == PT_MODE_STATIC)				//��̬ģʽ�´���
			{
				if(*pp == 0)								//ͨ��0
				{
					space = FIFO_MAX_LENGTH - pPt->fpoint1;
				}
				else if(*pp == 1)						//ͨ��1
				{
					space = FIFO_MAX_LENGTH - pPt->fpoint2;
				}
				else
					return RTN_ERROR;
			}
			else if(pPt->mode == PT_MODE_DYNAMIC)		//��̬ģʽ�´���
			{
				if(pPt->fpoint1 >= pPt->nowpoint)		//�����Ŵ��ڵ�ǰʹ�õ����
				{
					space = 2*FIFO_MAX_LENGTH - pPt->fpoint1 + pPt->nowpoint - 1;
				}
				else									//��ǰʹ�õ���Ҫ���ڴ����Ҫ
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
 * ��PT�˶�ģʽָ��FIFO�е�����
 * ��̬ģʽ��ָ��FIFO��Ч
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
			if(pPt->mode == PT_MODE_STATIC)				//��̬ģʽ�´���
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
			else if(pPt->mode == PT_MODE_DYNAMIC)		//��̬ģʽ�´���
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
 * ���PT�˶�ģʽָ��FIFO�е�����
 * �˶�״̬�¸�ָ����Ч
 * �˶�ģʽ�¸�ָ����Ч
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

			if(pPt->mode == PT_MODE_STATIC)				//��̬ģʽ�´���
			{
				if(pKer->kersta == MOTORS_STA_IDLE)		//û��������
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
			else if(pPt->mode == PT_MODE_DYNAMIC)		//��̬ģʽ�´���
			{
				if(pKer->kersta == MOTORS_STA_IDLE)
				{
					pPt->fpoint1 = 0;					//��ָ������
					pPt->nowpoint = 0;					//��ǰָ������
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
 * ����PT�˶�ģʽѭ��ִ�еĴ���
 * ��̬ģʽ�¸�ָ����Ч
 * loopΪ�Ǹ����������Ҫ����ѭ��������Ϊ0
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
			//��̬ģʽ������ָ����Ч
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
 * ��ѯPT�˶�ģʽѭ��ִ�еĴ���
 * ��̬ģʽ�¸�ָ����Ч
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
			//��̬ģʽ������ָ����Ч
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
 * �滮PT
 * ��Ҫ����׼ȷ���ҵ���Ҫ�滮�Ĳ������ĸ�
 */
ERROR_CODE Prep_PTmode(int axis)
{
	KernelPrm *pKer = &kernel[axis];
	PT_KERNEL *pPt = &ptkernel[axis];
	PT_FIFO *pFifo;
	//����Ƿ���Ҫ���¹滮
	if(pKer->flag == 0)
		return RTN_SUCC;
	//ѭ���������Ｋ��
	if(pPt->nowloop >= pPt->loop && pPt->loop != 0) 		//���㣬һֱѭ����ȥ������û�е���ѭ��
		return RTN_ERROR;

	if(pPt->mode == PT_MODE_STATIC)		//��̬ģʽ�´�������
	{
		if(pPt->fifo == 0)
			pFifo = ptfifo[axis][0];
		else
			pFifo = ptfifo[axis][1];

	}
	else								//��̬ģʽ�´�������
	{
		pFifo = ptfifo[axis][0];
	}
	pFifo += pPt->nowpoint;
	pPt->runtime = pFifo->time;

	long deltaPos;
	double evenVel;
	pPt->objpos = pPt->zeropos + pFifo->pos;
	pPt->startpos = pKer->nowPos;			//��ǰ�ε���ʼλ��
	deltaPos = pPt->objpos - pPt->startpos;
	evenVel = deltaPos / (pPt->runtime+0.0);//ת���ɸ��������м���

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

	pPt->runtime = pPt->runtime * 10;							//ʱ��̶ȵı任
	pKer->flag = 0;
	pKer->flag = 1;
	pKer->flag = 0;
	//pKer->aimPos = pPt->objpos;
	return RTN_SUCC;
}

/*
 * ����PT�˶�
 */
static ERROR_CODE Open_PTmode(KernelPrm *pKer)
{
	pKer->kersta = MOTORS_STA_PTMODE;
	pKer->step = 0;
	pKer->count = 0;
	pKer->flag = 1;			//��־λ��1
	return RTN_SUCC;
}

/*
 * �ر�PT�˶�
 */
static ERROR_CODE Exit_PTmode(KernelPrm *pKer)
{
	pKer->flag = 0;
	pKer->kersta = MOTORS_STA_IDLE;
	return RTN_SUCC;
}

/*
 * ����PT�˶�
 * ��̬ģʽ��ָ��FIFO��Ч
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
		if((cmd.mark >> axis) & 0x01)			//ʹ��״̬
		{
			pPt = &ptkernel[axis];
			pKer = &kernel[axis];

			/*
			 * �ж��Ƿ���Խ���PTģʽ
			 * 1.�Ƿ�������PTģʽ
			 * 2.�������Ƿ�������
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
					if(pPt->fpoint1 == 0)			//û������
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

				pPt->nowloop = 0;				//��ǰȦ������
				pPt->nowpoint = 0;				//��ǰ��ʼָ����
				pPt->endVel = 0;				//����ʼ����ͨ�Σ�����ͨ��������õ�һ�γ��ٶ�
				pPt->zeropos = pKer->nowPos;	//����PTģʽ�ǳ�ʼ��λ��
			}
			else
				return RTN_ERROR;
		}
//		else									//�ر�״̬		��ʱ��ʹ��
//		{
//			if(pKer->state == MOTORS_STA_PTMODE)		//��������ʱ��״̬
//			{
//
//			}
//		}
	}
	return RTN_SUCC;
}

/*
 * ����ָ��
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
 * PT�˶�ģʽ���к���
 * �����ж�������
 * ��Ҫ�ж��Ƿ����PTģʽ
 * ��Ҫ����ѭ���������ۼӺͿ�ʼʱ���λ��
 */
int  PTCOUNT = 0;
void Run_PTmode(int axis)
{
	int overflag = 0;
	long pos;
	PVAT_S pvat;
	KernelPrm *pKer = &kernel[axis];
	PT_KERNEL *pPt = &ptkernel[axis];
	if(pKer->flag == 1)		//����û�д�����ɣ�ֱ������
	{
		PTCOUNT ++;			//����ʹ�ã������ж�DSP�Ĵ�������
		return;
	}
	else
	{
		pKer->count ++;
		if(pKer->count >= pPt->runtime)			//���һ��
		{
			pKer->realPos = pPt->objpos;
			pKer->nowVel = pPt->endVel;
			pKer->count = 0;
			pKer->flag = 1;						//����׼��
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
			 * �ж�ѭ�����˳�
			 * ���ֶ�̬ģʽ�;�̬ģʽ
			 */
			if(pPt->mode == PT_MODE_DYNAMIC)		//��̬ģʽ�´���
			{
				if(pPt->nowpoint >= 2*FIFO_MAX_LENGTH)
					pPt->nowpoint = 0;
				//�ж��Ƿ���Ҫ������̬ģʽ
				if(pPt->nowpoint == pPt->fpoint1)	//����ָ����ȣ��˳�
				{
					Exit_PTmode(&kernel[axis]);
					pPt->nowpoint = 0;				//ָ������
					pPt->fpoint1 = 0;				//ָ������
				}
			}
			else if(pPt->mode == PT_MODE_STATIC)	//��̬ģʽ�´���
			{
				/*
				 * ��������ѭ��������ѭ�������
				 */
				if(pPt->loop == 0)					//����ѭ�������
				{/**************************************************************/
					/*
					 * �ж�ʹ�õ���ʲô������
					 * �жϵ�ǰ�������Ƿ�����
					 * �������
					 * ���±궨ԭ��
					 * ָ������
					 * ������һ��ѭ��
					 */
					if(pPt->fifo == 0)
					{
						if(pPt->nowpoint >= pPt->fpoint1)	//�����������һ��ѭ��
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
				else								//����ѭ������
				{/**************************************************************/
					if(pPt->fifo == 0)
					{
						if(pPt->nowpoint >= pPt->fpoint1)	//�Ƿ������������һ��ѭ��
						{
							pPt->nowloop ++;
							pPt->nowpoint = 0;
							pPt->zeropos = pKer->nowPos;
							pKer->flag = 1;
						}
						if(pPt->nowloop >= pPt->loop)		//�Ƿ����ѭ��
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
