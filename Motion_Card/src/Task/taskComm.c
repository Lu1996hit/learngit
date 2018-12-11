/*
 *        File: taskComm.c
 *     Version:
 * Description: ͨ�Ŵ���
 *
 *  Created on: 2018��1��18��
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

//ָ�����
static unsigned int cmdLen;

/*
 *  ����Ƿ����µ�ָ��
 */
ERROR_CODE checkNewCommand()
{
	unsigned int dat;
	ERROR_CODE rtn = RTN_ERROR;
	/*
	 * �˳�������
	 * SPI���յ�������ȡ�գ�����ȡ��֮ǰ����һ֡������ָ�
	 */
	while( RTN_ERROR != cb_get(&Spia.cb_rx, &dat))	//����ȡ���ݵ�dat��
	{
#if 0
		// ���Բ���
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
		//�������ݺ󣬳��Դ���������ȡָ�
		if (RTN_SUCC == protocol(dat, cmd.prm, &cmdLen))
		{
		    GGCC ++;
		    //�ɹ���ȡ��ָ��󣬶�ָ����н��������
			rtn = decoupleCommand(cmdLen);
			cmdLen = 0;
			return rtn;
		}
#endif
	}
	return rtn;
}

/*
 * ָ���������ȡָ��
 * ��ָ��浽cmd��������ȥ�����Ĳ��裩
 */
ERROR_CODE decoupleCommand(unsigned int cmdLen)
{
	ERROR_CODE rtn = RTN_SUCC;
	if (cmdLen <= 0)
		return RTN_ERROR;
	if(cmd.prm[2] == cmd.lastSign)			//��ǰָ�������һ��ָ�����ֵ���ʹ�λ�����·���
	{
		senddata();
		return RTN_ERROR;
	}
	else
		cmd.lastSign = cmd.prm[2];			//������һ���ı��ֵ

	memcpy(&cmd.type, cmd.prm, 2);			//����ָ������type�Լ�mark��ֵ
	cmd.count ++;							//��������1
	return rtn;
}
