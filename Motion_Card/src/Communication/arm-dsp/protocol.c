/*
 * 		  File: protocol.c
 *     Version:
 * Description: 八轴运动控制卡
 *
 *  Created on: 2017年12月17日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */
#include <string.h>
#include "procotol.h"
#include "senddata.h"
#include "sysDefines.h"
#include "taskComm.h"

static unsigned int cmd_buf[COMMUNICATION_MAX_LEN];
static unsigned int cmd_ptr = 0;
static unsigned int protocol_len = 0;

unsigned int check_prm(unsigned int* pdat, unsigned int len)
{
	unsigned int i;
	unsigned int rtn = 0xFFFF;
	for (i = 0; i < len; i++)
		rtn ^= pdat[i];
 	return rtn;
}

ERROR_CODE protocol(unsigned int chc, unsigned int *dat_buf, unsigned int *dat_len)
{
	if (cmd_ptr <= 0)
	{
		if (chc == MSG_HEAD)
		{
			cmd_buf[cmd_ptr++] = chc;
			return RTN_ERROR;
		}
	}
	else if (cmd_ptr == 1)
	{
		protocol_len = chc;
		cmd_buf[cmd_ptr++] = chc;
		return RTN_ERROR;
	}
	else
	{
		// 防止越界，或者接收出错
		if (cmd_ptr >= COMMUNICATION_MAX_LEN)
		{
			cmd_ptr = 0;
			return RTN_ERROR;
		}
		//
		cmd_buf[cmd_ptr++] = chc;				//存数据
		if (cmd_ptr == protocol_len + 3)		//当前接收到的数据长度 = 接收到的长度
		{
			if (chc == check_prm(cmd_buf+2, protocol_len))	//验证
			{
				memcpy(dat_buf, &cmd_buf[2], protocol_len);
				cmd_ptr = 0;
				*dat_len = protocol_len;
				return RTN_SUCC;
			}
			cmd_ptr = 0;
		}
	}
	return RTN_ERROR;
}

