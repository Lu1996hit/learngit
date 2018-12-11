/*
 *        File: taskComm.c
 *     Version:
 * Description: 通信处理
 *
 *  Created on: 2018年1月18日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "system.h"
#include "communication.h"

COMMAND cmd;

ERROR_CODE decoupleCommand(unsigned int cmdLen);

//define for test
int GCC = 0;
int GGCC = 0;
//define for test end

//指令缓冲区
static unsigned int cmdLen;

/*
 *  检查是否有新的指令
 */
ERROR_CODE checkNewCommand()
{
	unsigned int dat;
	ERROR_CODE rtn = RTN_ERROR;
	/*
	 * 退出条件：
	 * SPI接收到的数据取空，或者取空之前遇到一帧完整的指令。
	 */
	while( RTN_ERROR != cb_get(&Spia.cb_rx, &dat))	//先提取数据到dat中
	{
#if 0
		// 测试部分
		if( dat == 0x23)
		{
			for( cmdLen =0; cmdLen <11; cmdLen++)
				if(SpiaRegs.SPIFFTX.bit.TXFFST < 16)
//					SpiaRegs.SPITXBUF = cmdBuf[cmdLen];
//				else
					cb_append(&Spia.cb_tx, &cmdBuf[cmdLen]);
			SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;	// load
		}
#else
		GCC ++;
		//存入数据后，尝试从数据中提取指令。
		if (RTN_SUCC == protocol(dat, cmd.prm, &cmdLen))
		{
		    GGCC ++;
		    //成功提取到指令后，对指令进行解码操作。
			rtn = decoupleCommand(cmdLen);
			cmdLen = 0;
			return rtn;
		}
#endif
	}
	return rtn;
}

/*
 * 指令分析，提取指令
 * 把指令存到cmd变量里面去（核心步骤）
 */
ERROR_CODE decoupleCommand(unsigned int cmdLen)
{
	ERROR_CODE rtn = RTN_SUCC;
	if (cmdLen <= 0)
		return RTN_ERROR;
	if(cmd.prm[2] == cmd.lastSign)			//当前指令等于上一条指令，返回值发送错位，重新发送
	{
		senddata();
		return RTN_ERROR;
	}
	else
		cmd.lastSign = cmd.prm[2];			//保存上一条的标记值

	memcpy(&cmd.type, cmd.prm, 2);			//保存指令类型type以及mark的值
	cmd.count ++;							//计数器加1
	return rtn;
}
