/*
 *        File: taskAnalysis.c
 *     Version:
 * Description: 创建一个分析任务的模块，用来规划任务的执行
 *
 *  Created on: 2018年3月18日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "my_project.h"

/*
 * 初始化指令
 */
void Init_Command()
{
	cmd.count = 0;
	cmd.buflen = 1;				//错误码始终占据一个字节
	cmd.lastSign = 0223;		//随便写一个，不是0即可，建议不要修改
}

/*
 * 准备指令
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
 * 默认处理情况
 * 返回无效指令
 */
static void Decouple_Default()
{
	cmd.buflen = 1;
	cmd.rtn = RTN_INVALID_COMMAND;
}

/*
 * 运行指令
 */
ERROR_CODE runCmd()
{
	COMMAND_TYPE type;
	type = (COMMAND_TYPE)(cmd.type & 0xFF00);
	cmd.buflen = 1;					//这一行的位置不能改，数值也不能改，错误码始终占据一个字节
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
