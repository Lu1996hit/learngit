/*
 *        File: Crdmode.c
 *     Version:
 * Description: �岹ģʽ
 * 				��1��ֱ�߲岹��Բ���岹
 * 				��2����������ϵͬʱ���в岹�˶�
 * 				��3��ÿ������ϵ��������������
 * 				��4��ÿ��������������ʱ������������ȹ���
 * 				��5������ǰհԤ������
 * 				��6����Խ���ӣ�����Bug
 *
 *  Created on: 2018��7��20��
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
 * ͳһһ�²������ݵĸ�ʽ����crd.prm[]�У�
 * crd.prm[0]:�洢����tpye			//ָ������
 * crd.prm[1]:�洢����mark			//����ϵ���
 * crd.prm[2]:�洢�������к�
 * crd.prm[3]:�洢����fifo��ֵ		//������FIFO�ı�ţ������ǰ��ָ���fifo�й�
 */

/*
 * ָ��������ϵ���������������ں�
 */
KernelPrm* pKernelC[CRDNUM][4];		//����ϵ2ʹ��

//ÿ�����Ӧ������ϵ
static int crdindex[AXISNUM];

/*
 * ��������ϵ
 */
TCrdPrm crdprm[CRDNUM];							//һ������������ϵ

/*
 * ���建����
 */
CMDCRD crdfifo[CRDNUM][CRDFIFONUM][CRDFIFOLEN];			//��������С
#pragma DATA_SECTION(crdfifo, "EXTRAM_DATA");

/*
 * �ýṹ�������ڼ�¼ÿ�����ָ���������Ϣ
 */
typedef struct
{
	int fifo[CRDFIFONUM];						//��������ϵ��ÿ������ϵ������FIFO
												//���ڼ�¼��ǰFIFO��ʹ�����
	int nowfifo[CRDFIFONUM];					//��ǰ��ʹ�õ�FIFO�е�ָ��ĸ���
	int runstate;								//��¼��FIFO��ǰ���˶����
												//0��δ��ʼ�˶�ʱ
												//1��ʹ��0��fifo��ʼ�˶�
												//2������FIFO����ͣ
												//3��ʹ��1��fifo��ʼ�˶�
	double staVel;								//��ʼ�ٶ�
	double nowVel;								//��ǰ����ϵ�ĺϳ��ٶ�
	double nowAcc;								//��ǰ����ϵ�ĺϳɼ��ٶ�
	long helpPos[4];							//������������¼��ǰָ��ѹջ�����λ�á�
	long zeroPos[4];							//ԭ���λ�ã���ԭʼ����ϵ�е�λ��
	long pausePos[4];							//��ͣʱ���λ��

	double r;									//Բ���뾶
	double ac,bc;								//Բ��Բ��

	long startPos[4];							//ÿ������ϵ����ʼ�ĵ�ǰָ���µ���ʼ��λ��
	long time[3];								//����ʱ�䣬���� ���� ���ٵ�ʱ��
	double endPos[3];							//����ʱ�䣬ÿ��ʱ���Ӧ�����յ�λ��
	double conver[4];							//����ֱ���е�λ�û���

	CMDCRD *pCmdCrd;							//ָ��ǰ���е�ָ���ָ��
	//��Щ�����������óɳ�ʱ����Ч
	//�����봴������ϵʱ�������
	double decSmoothStop;						//ƽ��ֹͣ���ٶ�
	double decAbruptStop;						//����ֹͣ���ٶ�
	TCrdPrm *pCrdPrm;							//ָ�룬ָ������ϵ����
}CRD_KERNEL;

static CRD_KERNEL crdkernel[CRDNUM];
/*
 * ��ʼ������
 */
void Init_Crd_Kernel()
{
	int count;
	for(count = 0; count < 4; count++)
	{
		pKernelC[0][count] = NULL;								//ָ�����ָ������
		pKernelC[1][count] = NULL;
	}
	memset(crdprm, 0, sizeof(TCrdPrm)*2);
	for(count = 0; count < CRDNUM; count++)
	{
		crdkernel[count].decSmoothStop = 0.01;
		crdkernel[count].decAbruptStop = 0.1;				//���ó�ʼֵ
		memset(&crdkernel[count], 0, sizeof(CRD_KERNEL)-sizeof(double)*3);
		crdkernel[count].pCrdPrm = &crdprm[count];
	}
}

/*
 * ����ϵ�Ƿ���Ȼ��������û�б��޸ĵ�
 * index����ʾ����ϵ
 */
static ERROR_CODE CrdWithal(int index)
{
	TCrdPrm* pCrd;
	KernelPrm* pKernel;
	if(index == 0)				//1������ϵ
	{
		pCrd = &crdprm[0];
		pKernel = pKernelC[0][0];
	}
	else if(index == 1)			//2������ϵ
	{
		pCrd = &crdprm[1];
		pKernel = pKernelC[1][0];
	}
	else						//�����index����
	{
		return RTN_ERROR;
	}

	if(pCrd == NULL)			//Ϊ�գ����ش���
	{
		return RTN_ERROR;
	}

	if(pCrd->dimension < 2)		//����ϵ��û����ֵ�����ش���
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
 * ��ʼ��������ϵģʽ
 */
static ERROR_CODE Open_Crdmode(int crd, int fifo)
{
	int count;
	TCrdPrm *pCrdPrm;
	CRD_KERNEL *pCrdKer;
	KernelPrm **pKernel;

	pCrdPrm = &crdprm[crd];
	pCrdKer = &crdkernel[crd];

	//�������ϵ�Ƿ���Ȼ����							//���������������
//	if(CrdWithal(crd) != RTN_SUCC)
//		return RTN_ERROR;

	//���������������ָ��
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

	//����ܲ�������
	if(fifo == 0)
	{
		//����һ��fifo����û�����ݣ��ܲ�������
		if(pCrdKer->nowfifo[0] == pCrdKer->fifo[0])								//���FIFO�е�����
			return RTN_ERROR;

		//�Ƿ��Ǵ���ͣ�лָ��˶�
		if(pCrdKer->runstate == 2 || pCrdKer->runstate == 4 || pCrdKer->runstate == 5)					//����ǲ��Ǵ���ͣ�лָ�
		{
			//��ͣʱ���λ�ú͵�ǰ��λ�ã�����ȣ��޷������˶�
			for(count= 0; count < pCrdPrm->dimension; count++)
			{
				if(pCrdKer->pausePos[count]  != (*(pKernel+count))->nowPos)
					return RTN_ERROR;
			}
		}
		else if(pCrdKer->runstate == 0)												//����ǲ������¿�ʼ����
		{
			pCrdKer->runstate = 1;
		}
		else
			return RTN_ERROR;
		pCrdKer->runstate = 1;						//״̬������Ϊ1

	}
	else if(fifo == 1)
	{
		//���ڶ���fifo����û�����ݣ��ܲ�������
		if(pCrdKer->nowfifo[1] == pCrdKer->fifo[1])								//���FIFO�е�����
			return RTN_ERROR;

		//��鵱ǰ�ǲ�������ͣ�лָ�												//����ǲ�����FIFO����ͣ
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
	(*pKernel)->flag = 1;		//��Ҫ���滮
	crdindex[0] = crd;
	return RTN_SUCC;
}

/*
 * �˳�����ϵģʽ
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
 * ���������������ò�����ֱ�ߺ�Բ����ʹ��
 */
static ERROR_CODE CheckPrmAux_LC(TCrdPrm** pCrdPrm, CRD_KERNEL** pCrdKer,CRD_FIFO_REG* tcf)
{
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			tcf->crd = 0;
			(*pCrdPrm) = &crdprm[0];		//�����޸����ϳ��ٶȵȲ���
			(*pCrdKer) = &crdkernel[0];		//�����ж�
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)			//Crd 2
	{
		if(CrdWithal(1) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			tcf->crd = 1;
			(*pCrdPrm) = &crdprm[1];
			(*pCrdKer) = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	if(cmd.prm[3] == 0)					//�ж�FIFO��������ѡ��
	{
		tcf->fifo = 0;
	}
	else
	{
		tcf->fifo = 1;
		//fifoΪ1ʱ������FIFOδ��ͣ���򷵻ش���
		if((*pCrdKer)->runstate != 2)
			return RTN_ERROR;			//���ش���
	}
	//����Ƿ񳬳�������
	if((*pCrdKer)->fifo[tcf->fifo] >= CRDFIFOLEN)
		return RTN_ERROR;				//���ش���

	tcf->type = cmd.type & 0xFF;			//����type
	tcf->ready = 0;

	return RTN_SUCC;
}

/*
 * ���������������ò�����Buf��ʹ��
 */
static ERROR_CODE CheckPrmAux_Buf(CRD_KERNEL** pCrdKer,CRD_FIFO_REG* tcf)
{
	if(cmd.mark && 0x01)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			tcf->crd = 0;
			*pCrdKer = &crdkernel[0];	//�����ж�
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark && 0x02)
	{
		if(CrdWithal(1) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			tcf->crd = 1;
			*pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	if(cmd.prm[3] == 0)					//�ж�FIFO��������ѡ��
	{
		tcf->fifo = 0;
	}
	else
	{
		tcf->fifo = 1;
		//fifoΪ1ʱ������FIFOδ��ͣ���򷵻ش���
		if((*pCrdKer)->runstate != 2)
			return RTN_ERROR;			//���ش���
	}
	//����Ƿ񳬳�������
	if((*pCrdKer)->fifo[tcf->fifo] >= CRDFIFOLEN)
		return RTN_ERROR;				//���ش���

	tcf->type = cmd.type & 0xFF;			//����type
	tcf->ready = 0;

	return RTN_SUCC;
}

/*
 * ��������ϵ��������������ϵ
 */
static ERROR_CODE SetCrdPrm()
{
	int crd;			//��¼��ǰʹ�õ�����ϵ
	int count;
	int index;
	TCrdPrm *pCrd;			//����ϵ�ṹ��
	TCrdPrm *pCrd_temp;		//����ϵ�ṹ����ʱ����
	CRD_KERNEL *pCrdKer;	//����ϵ�ں�

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
	//����һЩ����ָ��
	pCrd = &crdprm[crd];
	pCrdKer = &crdkernel[crd];

	if(pCrd->dimension != 0)		//�жϵ�ǰ���Ƿ�Ϊ��
		return RTN_ERROR;			//��Ϊ�գ�˵�����������ݣ����ش���

	for(count = 0; count < pCrd_temp->dimension; count++)
	{
		//���õ��������˶����������ڵ�ǰ���õ����Ѿ���������ϵģʽ��
		index = pCrd_temp->axisIndex[count];
		if(kernel[index].kersta != MOTORS_STA_IDLE || kernel[index].axsta == MOTORS_STA_CRDMODE)
			return RTN_ERROR;		//���ش���
	}
	//ִ�е��˴���˵������ָ��û�����⣬����ִ��
	for(count = 0; count < pCrd_temp->dimension; count++)
	{
		index = pCrd_temp->axisIndex[count];
		pKernelC[crd][count] = &kernel[index];
		pKernelC[crd][count]->axsta = MOTORS_STA_CRDMODE;
	}
	//�������ݽ�ȥ
	memcpy(pCrd, pCrd_temp, sizeof(TCrdPrm));
	/*
	 * �����ʼ������ϵ�Ĵ���
	 */
	memset(pCrdKer, 0, sizeof(CRD_KERNEL)-sizeof(double)*3);					//��ʼ���ں˲�������������������ʼ��
	if(pCrd->setOriginFlag == 1)												//����ԭ��λ��
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
 * ��ѯ�������
 * 1.�жϵ�ʱ��ѯ������ϵ�Ƿ��Ѿ�����
 * 2.����Ѿ��������򷵻�����ϵ����
 * 3.û�н������򷵻�ERROR
 */
static ERROR_CODE GetCrdPrm()
{
	TCrdPrm* pCrd;
	unsigned int *pb = cmd.buf;
	if(cmd.mark == 0x01)                        	//��ѯ1������ϵ
	{
		if(RTN_SUCC != CrdWithal(0))				//����ϵ������
			return RTN_ERROR;
		pCrd = &crdprm[0];
	}
	else if(cmd.mark == 0x02)						//��ѯ2������ϵ
	{
		if(RTN_SUCC != CrdWithal(1))				//����ϵ������
			return RTN_ERROR;
		pCrd = &crdprm[1];
	}
	else
		return RTN_ERROR;

	if(pCrd->dimension == 0)                		//��ǰ����ϵΪ��
	{
		return RTN_ERROR;
	}
	//��ʼ��������ϵ���ݻ�ȥ
	memcpy(pb, pCrd,sizeof(TCrdPrm));
	cmd.buflen += sizeof(TCrdPrm);
	return RTN_SUCC;
}

/*
 * �������ϵ���趨
 */
/*
 * �˴���ָ���ָ��������
 * �޸���������ʹ�ö���ָ�룬ֱ��ʹ��һ��ָ��ָ��
 */
static ERROR_CODE QuitCrd()
{
	TCrdPrm* pCrd;
	int count;
	if(cmd.mark == 0x01)	                    	//1������ϵ�����
	{
		pCrd = &crdprm[0];
		if(pCrd->dimension == 0)		        	//�ж�1������ϵ�Ƿ��а�
		{											//û�а󶨣�ֱ���˳�
			return RTN_SUCC;
		}
		memset(pCrd, 0, sizeof(TCrdPrm));			//�а�
		for(count = 0; count < 4; count++)
		{
			if(pKernelC[0][count] != NULL && pKernelC[0][count]->axsta == MOTORS_STA_CRDMODE)
			{
				pKernelC[0][count]->axsta = MOTORS_STA_IDLE;		//������״̬
				pKernelC[0][count] = NULL;							//���ָ��
			}
		}
	}
	else if(cmd.mark == 0x02)                   	//2������ϵ�����
	{
		pCrd = &crdprm[1];
		if(pCrd->dimension == 0)		        	//�ж�2������ϵ�Ƿ��а�
		{
			return RTN_ERROR;						//û�а󶨣�ֱ���˳�
		}
		memset(pCrd, 0, sizeof(TCrdPrm));
		for(count = 0; count < 4; count++)
		{
			if(pKernelC[1][count] != NULL && pKernelC[1][count]->axsta == MOTORS_STA_CRDMODE)
			{
				pKernelC[1][count]->axsta = MOTORS_STA_IDLE;		//������״̬
				pKernelC[1][count] = NULL;							//���ָ��
			}
		}
	}
	/*
	 * Insert code
	 * ���FIFO�е�����
	 */

	return RTN_SUCC;
}

/*
 * ��ָ������������
 * 1.���ж�ʹ�õ�����һ������ϵ��Ȼ���жϵ�ǰ������ϵ�Ƿ���Ȼ����
 * 2.�жϵ�ǰ��fifo���Ѿ��洢��ָ��ȵĴ�С����û�г�����Χ
 */

/*
 * ��岹���������Ӳ岹����
 */
static ERROR_CODE CrdData()
{

	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹
 */
static ERROR_CODE LnXY()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	//����ָ��ָ�򣬼������Ƿ����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹
 */
static ERROR_CODE LnXYZ()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;			//���ش���

	if(pCrdPrm->dimension < 3)
		return RTN_ERROR;				//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��

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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹
 */
static ERROR_CODE LnXYZA()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 4)
		return RTN_ERROR;				//��ǰ������ϵС���ĸ������Ǹ�ָ����Ҫ�����֧��

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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹���յ��ٶ�ʼ��Ϊ0��
 */
static ERROR_CODE LnXYG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹���յ��ٶ�ʼ��Ϊ0��
 */
static ERROR_CODE LnXYZG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)
		return RTN_ERROR;				//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��

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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���άֱ�߲岹���յ��ٶ�ʼ��Ϊ0��
 */
static ERROR_CODE LnXYZAG0()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 4)
		return RTN_ERROR;				//��ǰ������ϵС���ĸ������Ǹ�ָ����Ҫ�����֧��

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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�XYƽ��Բ���岹�����յ�λ�úͰ뾶��Ϊ���������
 */
static ERROR_CODE ArcXYR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�XYƽ��Բ���岹�����յ�λ�ú�Բ��λ����Ϊ���������
 */
static ERROR_CODE ArcXYC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�YZƽ��Բ���岹�����յ�λ�úͰ뾶��Ϊ���������
 */
static ERROR_CODE ArcYZR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�YZƽ��Բ���岹�����յ�λ�ú�Բ��λ����Ϊ���������
 */
static ERROR_CODE ArcYZC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�ZXƽ��Բ���岹�����յ�λ�úͰ뾶��Ϊ���������
 */
static ERROR_CODE ArcZXR()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�ZXƽ��Բ���岹�����յ�λ�ú�Բ��λ����Ϊ���������
 */
static ERROR_CODE ArcZXC()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	if(pCrdPrm->dimension < 3)			//��ǰ������ϵС�����������Ǹ�ָ����Ҫ�����֧��
		return RTN_ERROR;

	tcf.cdir = cmd.prm[4];				//��ת����
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ���������������IO�������ָ��
 */
static ERROR_CODE BufIO()
{
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_Buf(&pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDIO));
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�����������ʱ����ָ��
 */
static ERROR_CODE BufDealy()
{
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	if(CheckPrmAux_Buf(&pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDDELAY));
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ������������DAֵ
 */
static ERROR_CODE BufDA()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//��鵱ǰ���õ�DA�����Ƿ�������ϵ����
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//��ǰ���õ���������ϵ�ڣ��˳�
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�����������Ч/��Ч��λ����
 */
static ERROR_CODE BufLmts()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//��鵱ǰ���õ����Ƿ�������ϵ����
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//��ǰ���õ���������ϵ�ڣ��˳�
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�������������axis��ֹͣIO��Ϣ
 */
static ERROR_CODE BufSetStopIo()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;

	int count;
	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//��鵱ǰ���õ����Ƿ�������ϵ����
	for(count = 0; count < pCrdPrm->dimension; count++)
	{
		if(cmd.prm[4] == pCrdPrm->axisIndex[count])
		{
			break;						//��ǰ���õ���������ϵ�ڣ��˳�
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
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�ʵ�ֵ�ͷ���湦�ܣ�����ĳ�����λ�˶�
 * ���᲻��������ϵ��
 */
static ERROR_CODE BufMove()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;
	KernelPrm *pKer;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	pKer = &kernel[cmd.prm[4]];
	//��鵱ǰ���õ����Ƿ�������ϵ��	pKer = &kernel[cmd.prm[4]];
	if(pKer->axsta == MOTORS_STA_CRDMODE)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDBUFFMOVE));
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ������ָ�ʵ�ֵ�����湦�ܣ�����ĳ��������˶�
 * ���᲻��������ϵ��
 */
static ERROR_CODE BufGear()
{
	TCrdPrm* pCrdPrm;
	CRD_KERNEL* pCrdKer;
	CRD_FIFO_REG tcf;					//��ʱ����
	CMDCRD *pCmdCrd;
	KernelPrm *pKer;

	if(CheckPrmAux_LC(&pCrdPrm, &pCrdKer,&tcf) != RTN_SUCC)
		return RTN_ERROR;
	//��鵱ǰ���õ����Ƿ�������ϵ����
	pKer = &kernel[cmd.prm[4]];
	if(pKer->axsta == MOTORS_STA_CRDMODE)
		return RTN_ERROR;

	pCmdCrd = &crdfifo[tcf.crd][tcf.fifo][pCrdKer->fifo[tcf.fifo]];
	memcpy(&(pCmdCrd->tcf), &tcf, sizeof(CRD_FIFO_REG));
	memcpy(&(pCmdCrd->prm), &cmd.prm[4], sizeof(CMDBUFFGEAR));
	pCrdKer->fifo[tcf.fifo] ++;			//�Լӣ�ָ����һ��
	return RTN_SUCC;
}

/*
 * ��ѯ�岹������ʣ��ռ�
 */
static ERROR_CODE CrdSpace()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	long space;							//���ڴ洢�岹������ʣ��ռ�

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ����岹�������ڵĲ岹����
 */
static ERROR_CODE CrdClear()
{
	CRD_KERNEL* pCrdKer;
	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * �����岹�˶�
 */
static ERROR_CODE CrdStart()
{
	int crd;
	unsigned int *p = &cmd.prm[3];

	for(crd = 0; crd < CRDNUM; crd++)
	{
		if((cmd.mark >> crd) & 0x01)		//������ǰ����ϵ
		{
			if(CrdWithal(crd) == RTN_SUCC)	//����ϵ��Ȼ����
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
				return RTN_ERROR;			//����ϵ�����������ش���
		}
	}

	return RTN_SUCC;
}

/*
 * ��ѯ�岹�˶�����ϵ״̬
 */
static ERROR_CODE CrdStatus()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	short Run;							//�岹�˶���״̬
	long Segment;						//��ȡ��ǰ�Ѿ���ɵĲ岹����
	int usefifo;						//��ǰ����ʹ�õ�fifo

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ��ȡδ��ɵĲ岹�ζ���
 */
static ERROR_CODE GetRemainderSegNum()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;
	long Segment;						//��ȡ��ǰ�Ѿ���ɵĲ岹����
	int usefifo;						//��ǰ����ʹ�õ�fifo

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ���ò岹�˶�ƽ��ֹͣ����ͣ�ϳɼ��ٶ�
 */
static ERROR_CODE SetCrdStopDec()
{
	unsigned int *p = &cmd.prm[3];
	CRD_KERNEL* pCrdKer;

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ��ѯ�岹�˶�ƽ��ֹͣ����ͣ�ϳɼ��ٶ�
 */
static ERROR_CODE GetCrdStopDec()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ��ѯ������ϵ�ĵ�ǰ����λ��ֵ
 */
static ERROR_CODE GetCrdPos()
{
	unsigned int *p = cmd.buf;
	TCrdPrm *pCrdPrm;
	KernelPrm **pKernel;

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdPrm = &crdprm[0];
			pKernel = pKernelC[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdPrm = &crdprm[1];
			pKernel = pKernelC[1];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else
		return RTN_ERROR;

	memcpy(p++, &(pCrdPrm->dimension), 1);		//��������ϵ�Ĵ�С
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
 * ��ѯ������ϵ�ĵ�ǰ�ϳ��ٶ�ֵ
 */
static ERROR_CODE GetCrdVel()
{
	unsigned int *p = cmd.buf;
	CRD_KERNEL* pCrdKer;

	//�ȼ�����
	if(cmd.mark & 0x01)  				//Crd 1
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[0];
		}
		else							//����ϵ�����������ش���
		{
			return RTN_ERROR;
		}
	}
	else if(cmd.mark & 0x02)
	{
		if(CrdWithal(0) == RTN_SUCC)	//����ϵ��Ȼ����
		{
			pCrdKer = &crdkernel[1];
		}
		else							//����ϵ�����������ش���
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
 * ��������ϵָ��
 */
void Decouple_CRDmode()
{
	switch(cmd.type)
	{
		case CMD_SET_CRD_PRM:				cmd.rtn = SetCrdPrm();				break;
		case CMD_GET_CRD_PRM:				cmd.rtn = GetCrdPrm();				break;
		case CMD_QUIT_CRD:					cmd.rtn = QuitCrd();				break;
		case CMD_CRD_DATA:					cmd.rtn = CrdData();				break;
		case CMD_LN_XY:						cmd.rtn = LnXY();					break;		//ָ�ʼ
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
		case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//ָ�����
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
 * Ԥ�����άֱ��ģʽ
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
	pCrdKer->conver[0] = deltaX / deltaPos;		//�˴�ϵ���Լ�����������
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
	double staVel = pCrdKer->nowVel;			//������ʼ�ٶȣ���ʼ�ٶ�����һ���ٶ��ν�����
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//ƽ��ʱ��

	t0 = (pXY->synVel - staVel) / pXY->synAcc;
	pCrdKer->endPos[0] = (staVel + pXY->synVel) * t0 / 2;				//�˴���Pos������ʱ����

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
		//�˴���t1��ʾ����ٶ�
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

	pCmdCrd->tcf.ready = 1;							//��ʾ�Ѿ�Ԥ������ɵ�ǰָ��
	return RTN_SUCC;
}

/*
 * Ԥ������άֱ��ģʽ
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
	pCrdKer->conver[0] = deltaX / deltaPos;		//�˴�ϵ���Լ�����������
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
	double staVel = pCrdKer->nowVel;			//������ʼ�ٶȣ���ʼ�ٶ�����һ���ٶ��ν�����
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//ƽ��ʱ��

	t0 = (pXYZ->synVel - staVel) / pXYZ->synAcc;
	pCrdKer->endPos[0] = (staVel + pXYZ->synVel) * t0 / 2;				//�˴���Pos������ʱ����

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
		//�˴���t1��ʾ����ٶ�
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

	pCmdCrd->tcf.ready = 1;							//��ʾ�Ѿ�Ԥ������ɵ�ǰָ��
	return RTN_SUCC;
}

/*
 * Ԥ������άֱ��ģʽ
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
	pCrdKer->conver[0] = deltaX / deltaPos;		//�˴�ϵ���Լ�����������
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
	double staVel = pCrdKer->nowVel;			//������ʼ�ٶȣ���ʼ�ٶ�����һ���ٶ��ν�����
	long evenTime;

	pCrdKer->staVel = staVel;
	evenTime = pCrdKer->pCrdPrm->evenTime;		//ƽ��ʱ��

	t0 = (pXYZA->synVel - staVel) / pXYZA->synAcc;
	pCrdKer->endPos[0] = (staVel + pXYZA->synVel) * t0 / 2;				//�˴���Pos������ʱ����

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
		//�˴���t1��ʾ����ٶ�
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

	pCmdCrd->tcf.ready = 1;							//��ʾ�Ѿ�Ԥ������ɵ�ǰָ��
	return RTN_SUCC;
}

/*
 * ��ȡ��Բ��ص�Index
 * ����0����ʾ��ǰ��ָ��ΪR
 * ����1����ʾ��ǰ��ָ��ΪC
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
 * ��ȡԲ��ָ��Ĳ�������������
 * 0��R�汾
 * 1��C�汾
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
 * Ԥ����ƽ��Բ���岹�����յ�λ�ú�Բ��Ϊ���������
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

	pCrdKer->startPos[IndexA] = (*(ppKernel+IndexA))->nowPos;						//��ȡ��ʼ������
	pCrdKer->startPos[IndexB] = (*(ppKernel+IndexB))->nowPos;

	if(mode == 1)			//C�汾
	{
		pCrdKer->ac = pABC->ac + pCrdKer->zeroPos[IndexA];								//��ȡԲ��λ��
		pCrdKer->bc = pABC->bc + pCrdKer->zeroPos[IndexB];

		deltaPos = (pABC->a - pABC->ac) * (pABC->a - pABC->ac) + (pABC->b - pABC->bc) * (pABC->b - pABC->bc);
		r = sqrt(deltaPos);		//��ȡ�뾶
	}
	else					//R�汾
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
			//����Բ��λ�ã�Բ��λ��������
			cy1 = (-B + delta) / (2 * A);
			cx1 = c1 - c2 * cy1;
			cy2 = (-B - delta) / (2 * A);
			cx2 = c1 - c2 * cy2;
		}
		else
		{
			y1 = (y1 + y2) * 0.5;
			delta = r*r - (y1 - y2)*(y1 - y2);									//˳������
			y2 = y1;
			delta = sqrt(delta);
			x1 -= delta;
			x2 += delta;
		}
		if(r > 0)							//С�ǶȻ���
		{
			if(pCmdCrd->tcf.cdir == 1)		//��Ҫ��ʱ��ת��
			{
				dir = 1;					//���
			}
			else							//��Ҫ˳ʱ��ת��
			{
				dir = 0;					//�Ҳ�
			}
		}
		else								//��ǶȻ���
		{
			if(pCmdCrd->tcf.cdir == 1)		//��Ҫ��ʱ��ת��
			{
				dir = 0;			//�Ҳ�
			}
			else							//��Ҫ˳ʱ��ת��
			{
				dir = 1;			//���
			}
		}
		delta = (x1 - cx1) * (y2 - cy1) - (x2 - cx1) * (y1 - cy1);
		if(delta > 0)		//��ǰ����ֱ�����
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
		else if(delta < 0)	//��ǰ����ֱ���Ҳ�
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
		else				//��ǰ����ֱ���ϣ���������������غϣ�ȡ�κ�һ��������
		{
			pCrdKer->ac = cx1;
			pCrdKer->bc = cy1;
		}



	}
	r = fabs(r);
	pCrdKer->r = r;														//�洢�뾶�Ĵ�С��r��С�����и�


	deltaA = pCrdKer->startPos[IndexA] - pCrdKer->ac;								//��ʼ
	deltaB = pCrdKer->startPos[IndexB] - pCrdKer->bc;
	pCrdKer->conver[0] = atan2(deltaB, deltaA);										//��ʼʱ��ĽǶ�ֵ

	deltaA = pABC->a + pCrdKer->zeroPos[IndexA] - pCrdKer->ac;						//����
	deltaB = pABC->b + pCrdKer->zeroPos[IndexB] - pCrdKer->bc;
	pCrdKer->conver[3] = atan2(deltaB, deltaA);										//����ʱ��ĽǶ�ֵ

	if(pCmdCrd->tcf.cdir == 1)														//��ʱ��ת�����Ƕ�Ӧ������
	{
		if(pCrdKer->conver[3] < pCrdKer->conver[0])
			pCrdKer->conver[3] += M_2PI;

		deltaPos = pCrdKer->conver[3] - pCrdKer->conver[0];
	}
	else
	{								//��ǰ��Ҳ������ʼ�Ľ��ٶȣ���ֵ
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

	evenTime = pCrdKer->pCrdPrm->evenTime;		//ƽ��ʱ��
	pCrdKer->staVel = pCrdKer->nowVel;												//������ʼ�ٶȣ���ʼ�ٶ�����һ���ٶ��ν�����
	staVel = pCrdKer->staVel;

	t0 = (synVel - staVel) / synAcc;
	pCrdKer->endPos[0] = (staVel + synVel) * t0 / 2;				//�˴���Pos������ʱ����

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
		//�˴���t1��ʾ����ٶ�
		double b,c;
		b = synAcc*evenTime;
		c = synAcc * deltaPos + (staVel * staVel + velEnd + velEnd)/2.0;
		t1 = (sqrt(b*b + 4*c) - b)/2.0;

		pABC->synVel = t1;					//�˴���Ҫ��ֵ��ʹ��ָ��ָ��

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

	pCmdCrd->tcf.ready = 1;							//��ʾ�Ѿ�Ԥ������ɵ�ǰָ��
	return RTN_SUCC;
}

/*
 * Ԥ������������ʱ����ָ��
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

	pCmdCrd->tcf.ready = 1;							//��ʾ�Ѿ�Ԥ������ɵ�ǰָ��
	return RTN_SUCC;
}

ERROR_CODE Prep_CRDmode(int axis)
{
	//�ж�ʹ����һ������ϵ		ʹ��crdindec[axis]
	//�ж�ʹ�õ�����һ��fifo					��Ȼ���˶���һ���Ǽ����ˣ���ǰ��fifo��һ�������ݣ�����ִ�к󣬻�������ݣ����Ի���������FIFO�е�����
	//�жϵ�ǰִ�е�����һ��ָ��
	int crd;				//����ϵ
	int fifo;
	CRD_KERNEL *pCrdKer;					//�ں�ָ��
	CMDCRD *pCmdCrd;						//ָ��ָ��
	KernelPrm **pKernel;					//ָ���ں�


	crd = crdindex[axis];
	pCrdKer = &crdkernel[crd];
	pKernel = pKernelC[crd];

	//����Ƿ���Ҫ���¹滮
	if((*pKernel)->flag == 0)					//����Ҫ��ֱ���˳���
		return RTN_SUCC;

	//�ж�״̬
	if(pCrdKer->runstate == 1)
		fifo = 0;
	else if(pCrdKer->runstate == 3)
		fifo = 1;
	else
		return RTN_ERROR;
/******************************************************************************************************/
//	if(pCrdKer->fifo[fifo] == pCrdKer->nowfifo[fifo])					//�ⲿ�ִ���Ҳ���Խ����ж���������ʱ�Ѿ������ж�����
//	{
//		//����ִ����ɣ������˳�FIFO
//		//Quit_Crdmode();
//	}
/******************************************************************************************************/
	pCmdCrd= &crdfifo[crd][fifo][pCrdKer->nowfifo[fifo]];
	pCrdKer->pCmdCrd = pCmdCrd;

	if(pCmdCrd->tcf.ready == 1)											//��ǰָ�����׼��������׼����һ��ָ��
	{
		return RTN_SUCC;					//��ǰָ��Ԥ������ɣ���δ��ʼִ�У�������ȷ
	}
	else if(pCmdCrd->tcf.ready != 0)		//����Ԥ������һ��ָ��
	{
		return RTN_ERROR;
	}

//	//�������
//#ifdef CHECK
//	if(pCmdCrd->tcf.fifo != fifo)
//		asm( " ESTOP" );
//#endif
	//��ʼԤ��������
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
		case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//ָ�����
		default:							return RTN_ERROR;
	}

	//����ĳЩ����ҪԤ�����ָ�ִ�����³���
	pCmdCrd->tcf.type = 1;
	(*(pKernel))->flag = 0;
	return RTN_SUCC;
}

/*
 * ���ж�άֱ��ģʽ
 */
static int Run_LNXY(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over�����н�����־λ

	acc = pCmdCrd->prm.xy.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//���һ��
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xy.synVel;

				(*ppKernel)->count = 0;								//Ϊת����׼��
				(*ppKernel)->step = 1;
			}
			else													//�����һ��
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//ת����һ��
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xy.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//���һ��
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//�����һ��
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xy.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//ת����һ��
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//���һ����ת����ȥ
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
 * ������άֱ��ģʽ
 */
static int Run_LNXYZ(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over�����н�����־λ

	acc = pCmdCrd->prm.xyz.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//���һ��
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xyz.synVel;

				(*ppKernel)->count = 0;								//Ϊת����׼��
				(*ppKernel)->step = 1;
			}
			else													//�����һ��
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//ת����һ��
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xyz.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//���һ��
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//�����һ��
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xyz.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//ת����һ��
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//���һ����ת����ȥ
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
 * ������άֱ��ģʽ
 */
static int Run_LNXYZA(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	PVAT_S pvat;
	double pos;
	double acc;
	int over = 0;				//over�����н�����־λ

	acc = pCmdCrd->prm.xyza.synAcc;

	switch((*ppKernel)->step)
	{
	case 0:
	{
		if(pCrdKer->time[0] != 0)
		{
			(*ppKernel)->count++;
			pCrdKer->nowAcc = acc;
			if((*ppKernel)->count >= pCrdKer->time[0])				//���һ��
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = pCmdCrd->prm.xyza.synVel;

				(*ppKernel)->count = 0;								//Ϊת����׼��
				(*ppKernel)->step = 1;
			}
			else													//�����һ��
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//ת����һ��
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = pCmdCrd->prm.xyza.synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//���һ��
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//�����һ��
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * pCmdCrd->prm.xyza.synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//ת����һ��
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//���һ����ת����ȥ
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
 * ����ƽ��Բ���岹
 * �������������Բ��һ������
 * 1.���յ�λ�úͰ뾶Ϊ�������		R�汾
 * 2.���յ�λ�ú�Բ��Ϊ�������		C�汾
 */
static int Run_Circle(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel, COMMAND_TYPE type)
{
	PVAT_S pvat;
	double pos;
	double acc;
	double synVel,velEnd;
	double endA,endB;
	int IndexA,IndexB;
	int over = 0;				//over�����н�����־λ

	GetCirIndex(&IndexA, &IndexB, type);

	//��������Բ�����˶���˵������������������ڴ��еĵ�ַ��һ���ģ����Ի������
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
			if((*ppKernel)->count >= pCrdKer->time[0])				//���һ��
			{
				pos = pCrdKer->endPos[0];
				pCrdKer->nowVel = synVel;

				(*ppKernel)->count = 0;								//Ϊת����׼��
				(*ppKernel)->step = 1;
			}
			else													//�����һ��
			{
				pos = acc * (*ppKernel)->count * (*ppKernel)->count * 0.005 + (*ppKernel)->count * pCrdKer->staVel * 0.1;
				pCrdKer->nowVel = (*ppKernel)->count * acc * 0.1 + pCrdKer->staVel;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 1;			//ת����һ��
		}

	}
	case 1:
	{
		if(pCrdKer->time[1] != 0)
		{
			(*ppKernel)->count++;

			pCrdKer->nowVel = synVel;
			pCrdKer->nowAcc = 0;

			if((*ppKernel)->count >= pCrdKer->time[1])				//���һ��
			{
				pos = pCrdKer->endPos[1];
				(*ppKernel)->count = 0;
				(*ppKernel)->step = 2;
			}
			else													//�����һ��
			{
				pos = pCrdKer->endPos[0] + (*ppKernel)->count * synVel * 0.1;
			}
			break;
		}
		else
		{
			(*ppKernel)->count = 0;
			(*ppKernel)->step = 2;			//ת����һ��
		}
	}
	case 2:
	{
		(*ppKernel)->count++;
		pCrdKer->nowAcc = -acc;
		if((*ppKernel)->count >= pCrdKer->time[2])					//���һ����ת����ȥ
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

	double theta;			//�Ƕ�
	double r;
	r = pCrdKer->r;
	theta = pos / r;		//�Ƕ�
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
 * Ԥ������������ʱ����ָ��
 */
static int Run_BufDelay(CRD_KERNEL *pCrdKer, CMDCRD *pCmdCrd, KernelPrm **ppKernel)
{
	int over = 0;				//over�����н�����־λ

	pCrdKer->nowAcc = 0;
	pCrdKer->nowVel = 0;
	if(pCrdKer->time[0] != 0)
	{
		(*ppKernel)->count++;
		if((*ppKernel)->count >= pCrdKer->time[0])				//���һ��
		{
			over = 1;
			(*ppKernel)->count = 0;								//Ϊת����׼��
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
 * �����ںˣ����ݵ�ǰ�����е�ָ�룬�����ں�
 * 1.�������ᣬ�ҵ���Ӧ������ϵ����������ϵ�󣬱�ʾ����ϵ�Ѿ�����ס�ˣ��޷�ֱ���޸ĵ�ǰ���״̬��
 * 2.�ҵ���ǰ��Ҫ���е�ָ��ж�ָ���ܷ�����
 * 3.ָ�������У�������ָ��
 */
void Run_CRDmode(int axis)
{
	int crd;				//����ϵ
	int fifo;
	int over;				//��ǰָ��Ľ�����־λ
	CRD_KERNEL *pCrdKer;					//�ں�ָ��
	CMDCRD *pCmdCrd;						//ָ��ָ��
	KernelPrm **pKernel;					//ָ���ں�
	COMMAND_TYPE type;


	crd = crdindex[axis];					//�ҳ���ǰʹ�õ�����ϵ
	pCrdKer = &crdkernel[crd];
	pKernel = pKernelC[crd];

	if((*pKernel)->flag == 1)		//����û�д�����ɣ�ֱ������
		return;

	pCmdCrd = pCrdKer->pCmdCrd;
	if(pCmdCrd->tcf.ready != 1)											//��ǰָ�����׼��������׼����һ��ָ��
	{
		return;						//��ǰָ��Ԥ������ɣ���δ��ʼִ�У�����
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
	case CMD_BUF_GEAR:					cmd.rtn = BufGear();				break;		//ָ�����
	default:							return;
	}

	if(over == 1)			//��ǰ��ָ��ִ����ɣ���������Ҫִ����һ��ָ����˳�����
	{
		pCmdCrd->tcf.ready = 2;			//��ǰָ������

		if(pCrdKer->runstate == 1)
			fifo = 0;
		else if(pCrdKer->runstate == 3)
			fifo = 1;
		else
		{
			return;
		}
		pCrdKer->nowfifo[fifo] ++;
		if(pCrdKer->nowfifo[fifo] == pCrdKer->fifo[fifo])		//�˳�����ϵ
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
			pCrdKer->pCmdCrd++;			//ָ����һ��
			(*pKernel)->flag = 1;
		}
	}

	return;
}

/******************************************************************************************************************
 ********************************************     ���´�����ֹͣ�й�          *******************************************
 *****************************************************************************************************************/
/*
 * ������˶�ģʽ�£���Ҫֹͣһ��һ��ֹͣ�˶�
 *
 * ���Ԥ������Ϣ
 * ��Ҫ�жϵ�ǰ���˶�ǣ������Щ����
 */
/*
 * ֱ��ģʽֹͣ
 */
void Stop_LINE()
{
	int crd;				//����ϵ
	int fifo;
	int over;				//��ǰָ��Ľ�����־λ
	CRD_KERNEL *pCrdKer;	//�ں�ָ��
	CMDCRD *pCmdCrd;		//ָ��ָ��
	KernelPrm **pKernel;	//ָ���ں�
	COMMAND_TYPE type;


	return;
}

/*
 * ֹͣģʽ��ʹ��
 */
void Stop_CRDmode(int axis)
{
	return;
}
