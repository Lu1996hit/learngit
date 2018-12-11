/*
*  Created on: 2016-12-18
*      Author: QUENTIN
*      E-mail: qiyuexin@yeah.net
*/
//###########################################################################
//
//  FILE:   mymotors.c
//
// TITLE:   FPGA�������
//
//###########################################################################


#include "DSP2833x_Device.h"
#include "sysTypes.h"
#include "mymotors.h"
#include <string.h>

#include "DSP2833x_Examples.h"

#include "my_project.h"

volatile struct MOTORS_REGS MotorRegs[AXISNUM];

#pragma DATA_SECTION(MotorRegs,"MotorRegsFiles");

/*
 * FastOpen
 */
void Init_Axis(int axis)
{
	Motor_Reset(axis);
	MotorRegs[axis].MCTL.bit.START = 1;
	MotorRegs[axis].AD_CONF.bit.AD_EN = 1;
	Motor_ServoOn(axis);
}

/*
 * ��ʼ�����
 */
void Init_Motor()
{
	int axis;
	for(axis = 0; axis < AXISNUM; axis ++)
	{
		Motor_Reset(axis);
		MotorRegs[axis].MCTL.bit.START = 1;
		MotorRegs[axis].AD_CONF.bit.AD_EN = 1;
	}

}

/*
 * ���ʹ��
 * �͵�ƽʹ��
 */
void Motor_ServoOn(int axis)
{
	MotorRegs[axis].MCTL.bit.ENA = 0;
}

/*
 * ���ʹ�ܹر�
 * �ߵ�ƽ�ر�ʹ��
 */
void Motor_ServoOff(int axis)
{
	MotorRegs[axis].MCTL.bit.ENA = 1;
}

/*
 * �����λ
 */
void Motor_Reset(int axis)
{
	MotorRegs[axis].MCTL.bit.RST = 1;			//�������½���Ҳ�����ǵ͵�ƽ��������Ҳ����
	MotorRegs[axis].MCTL.bit.RST = 0;			// ��λ
	DELAY_US(1);
	MotorRegs[axis].MCTL.bit.RST = 1;			// �ָ�
}

// ����pvat������
void M_SetPvat( int axis, PVAT_S *dda){
	MotorRegs[axis].INPOS =  dda->aim_pos;
	MotorRegs[axis].INVEL =  dda->start_vel;
	MotorRegs[axis].INACC =  dda->start_acc;
	MotorRegs[axis].INXX  =  dda->min_period;
	//DELAY_US(1);
	MotorRegs[axis].MSTA.bit.NMSG = 1;	// push into the fifo of DDA.
}

