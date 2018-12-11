/*
 *        File: taskAnalysis.c
 *     Version:
 * Description: ����һ�����������ģ�飬�����滮�����ִ��
 *
 *  Created on: 2018��3��18��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "my_project.h"

/*
 * ��ʼ��ָ��
 */
void Init_Command()
{
	cmd.count = 0;
	cmd.buflen = 1;				//������ʼ��ռ��һ���ֽ�
	cmd.lastSign = 0223;		//���дһ��������0���ɣ����鲻Ҫ�޸�
}

/*
 * ׼��ָ��
 */
ERROR_CODE prepCmd()
{
	ERROR_CODE rtn;
	KernelPrm *pKer;
	int axis;
	pKer = kernel;
	for(axis = 0;axis < AXISNUM;axis ++)
	{
		switch(pKer->axsta)
		{
			case MOTORS_STA_IDLE:											break;
			case MOTORS_STA_PPMODE:			rtn = Prep_PPmode(axis);		break;
			case MOTORS_STA_JOGMODE:		rtn = Prep_JOGmode(axis);		break;
			case MOTORS_STA_PTMODE:			rtn = Prep_PTmode(axis);		break;
			case MOTORS_STA_GEARMODE:										break;
			case MOTORS_STA_FOLLOWMODE:										break;
			case MOTORS_STA_CRDMODE:		rtn = Prep_CRDmode(axis);		break;
			case MOTORS_STA_STOPMODE:		rtn = Prep_STOPmode(axis);		break;
			case MOTORS_STA_ERRORMODE:										break;
			default:														break;
		}
		pKer ++;
	}
	return rtn;
}

/*
 * Ĭ�ϴ������
 * ������Чָ��
 */
static void Decouple_Default()
{
	cmd.buflen = 1;
	cmd.rtn = RTN_INVALID_COMMAND;
}

/*
 * ����ָ��
 */
ERROR_CODE runCmd()
{
	COMMAND_TYPE type;
	type = (COMMAND_TYPE)(cmd.type & 0xFF00);
	cmd.buflen = 1;					//��һ�е�λ�ò��ܸģ���ֵҲ���ܸģ�������ʼ��ռ��һ���ֽ�
	switch(type)
	{
		case CMD_AXIS:			Decouple_Axis();				break;
		case CMD_PP_MODE: 		Decouple_PPmode();				break;
		case CMD_JOG_MODE:		Decouple_JOGmode();				break;
		case CMD_PT_MODE:		Decouple_PTmode();		 		break;
		case CMD_CRD_MODE:		Decouple_CRDmode();				break;
		case CMD_STOP_MODE:		Decouple_STOPmode();			break;
		default:				Decouple_Default();				break;
	}
	senddata();
	//senddata(cmd.type,cmd.mark,&cmd.rtn,cmd.buflen);
	return cmd.rtn;
}
