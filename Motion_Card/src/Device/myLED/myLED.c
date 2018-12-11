/*
 *        File: myLED.c
 *     Version:
 * Description: LED1 GPIO 29
 * 				LED2 GPIO 0
 * 				LED3 GPIO 1
 * 				LED4 GPIO 20
 *
 *  Created on: 2018年12月3日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "DSP2833x_Device.h"     	// DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   	// DSP2833x Examples Include File



/*
 * 初始化LED灯
 */
void Init_LED()
{
	EALLOW;
	//配置通用IO输出
	GpioCtrlRegs.GPAMUX2.bit.GPIO29= 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
	//GPIO配置为输出
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
	//默认关闭LED灯
	GpioDataRegs.GPADAT.bit.GPIO29 = 1;
	GpioDataRegs.GPADAT.bit.GPIO0 = 1;
	GpioDataRegs.GPADAT.bit.GPIO1 = 1;
	GpioDataRegs.GPADAT.bit.GPIO20 = 1;
	EDIS;
	return;
}

/*
 * 打开LED灯
 */
void Open_Led(int index)
{
	EALLOW;
	switch(index)
	{
	case 1:		GpioDataRegs.GPADAT.bit.GPIO29 = 0;		break;
	case 2:		GpioDataRegs.GPADAT.bit.GPIO0 = 0;		break;
	case 3:		GpioDataRegs.GPADAT.bit.GPIO1 = 0;		break;
	case 4:		GpioDataRegs.GPADAT.bit.GPIO20 = 0;		break;
	default:	break;
	}
	EDIS;
}

/*
 * 关闭LED灯
 */
void Close_Led(int index)
{
	EALLOW;
	switch(index)
	{
	case 1:		GpioDataRegs.GPADAT.bit.GPIO29 = 1;		break;
	case 2:		GpioDataRegs.GPADAT.bit.GPIO0 = 1;		break;
	case 3:		GpioDataRegs.GPADAT.bit.GPIO1 = 1;		break;
	case 4:		GpioDataRegs.GPADAT.bit.GPIO20 = 1;		break;
	default:	break;
	}
	EDIS;
}




