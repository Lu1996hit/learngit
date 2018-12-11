/*
*  Created on: 2016-12-18
*      Author: QUENTIN
*      E-mail: qiyuexin@yeah.net
*/
//###########################################################################
//
//  FILE:   mymotors.c
//
// TITLE:   FPGA电机部分
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
 * 初始化电机
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
 * 电机使能
 * 低电平使能
 */
void Motor_ServoOn(int axis)
{
	MotorRegs[axis].MCTL.bit.ENA = 0;
}

/*
 * 电机使能关闭
 * 高电平关闭使能
 */
void Motor_ServoOff(int axis)
{
	MotorRegs[axis].MCTL.bit.ENA = 1;
}

/*
 * 电机复位
 */
void Motor_Reset(int axis)
{
	MotorRegs[axis].MCTL.bit.RST = 1;			//可能是下降沿也可能是低电平触发，我也忘了
	MotorRegs[axis].MCTL.bit.RST = 0;			// 复位
	DELAY_US(1);
	MotorRegs[axis].MCTL.bit.RST = 1;			// 恢复
}

// 输入pvat的数据
void M_SetPvat( int axis, PVAT_S *dda){
	MotorRegs[axis].INPOS =  dda->aim_pos;
	MotorRegs[axis].INVEL =  dda->start_vel;
	MotorRegs[axis].INACC =  dda->start_acc;
	MotorRegs[axis].INXX  =  dda->min_period;
	//DELAY_US(1);
	MotorRegs[axis].MSTA.bit.NMSG = 1;	// push into the fifo of DDA.
}

