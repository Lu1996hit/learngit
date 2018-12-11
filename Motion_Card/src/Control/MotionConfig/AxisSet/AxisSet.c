/*
 *        File: AxisSet.c
 *     Version:
 * Description: �������
 *
 *  Created on: 2018��12��4��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include <AxisSet/AxisSet.h>
#include "system.h"
#include "device.h"
#include "taskComm.h"


/*
 * ��������ʹ��
 */
static ERROR_CODE AxisOn()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis ++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			Motor_ServoOn(axis);
		}
	}
	return RTN_SUCC;
}

/*
 * �ر�������ʹ��
 */
static ERROR_CODE AxisOff()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis ++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			Motor_ServoOff(axis);
		}
	}
	return RTN_SUCC;
}

/*
 * ��λ������ʹ��
 */
static ERROR_CODE AxisReset()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis ++)
	{
		if((cmd.mark >> axis) & 0x01)
		{
			Motor_Reset(axis);
		}
	}
	return RTN_SUCC;
}

/*
 * ��DSP��LED��
 */
static ERROR_CODE DspLedOn()
{
	int led;
	for(led = 0; led < DSPLEDNUM; led ++)
	{
		if((cmd.mark >> led) & 0x01)
		{
			Open_Led(led);
		}
	}
	return RTN_SUCC;
}

/*
 * �ر�DSP��LED��
 */
static ERROR_CODE DspLedOff()
{
	int led;
	for(led = 0; led < DSPLEDNUM; led ++)
	{
		if((cmd.mark >> led) & 0x01)
		{
			Close_Led(led);
		}
	}
	return RTN_SUCC;
}



void Decouple_Axis()
{
	switch(cmd.type)
	{
		case CMD_AXIS_ON:		cmd.rtn = AxisOn();			break;
		case CMD_AXIS_OFF:		cmd.rtn = AxisOff();		break;
		case CMD_AXIS_RESET:	cmd.rtn = AxisReset();		break;
		case CMD_DSPLED_ON:		cmd.rtn = DspLedOn();		break;
		case CMD_DSPLED_OFF:	cmd.rtn = DspLedOff();		break;
		default:				break;
	}
}
