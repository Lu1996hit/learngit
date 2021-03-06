﻿/*
*  Created on: 2016-12-1
*      Author: QUENTIN
*      E-mail: qiyuexin@yeah.net
*/
//###########################################################################
//
//  FILE:   myPeripherals.c
//
// TITLE:   外设的初始化以及功能实现
//
//###########################################################################
// Peripheral 1: Cpu Timer is ok. @16:14 2016/12/2
// Peripheral 2: SCI ( UART )
// "*" means to do, and "+" means done.
// * uart_tx
// * uart_rx
// * uart_config

#include "my_project.h"

// 运动卡的EXTRAM用的zone6，EXTFLASH用的zone7, FPGA用的zone0.
// 运动卡的EXTRAM用的zone6，
// 运动卡的FPGA用的zone0，

void InitPeripherals(void)
{
    //InitGpio();
	// ==============================================================================
	//          Step 1. Initialize System Control:
	// ==============================================================================
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP2833x_SysCtrl.c file.
	InitSysCtrl();

    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
    InitFlash();
	// ==============================================================================
	//          Step 2. Clear all interrupts and initialize PIE vector table.
	// ==============================================================================
	// 部分寄存器名字说明：
	// PIE: Peripheral Interrupt Expansion.
	// IER: Interrupt Enable Register
	// IFR: Interrupt Flag Register
	DINT;	// Disable CPU interrupts

	// 默认把PIEIER与PIEIFR全部清零。
	InitPieCtrl();

	// 默认把CPU核的IER与IFR全部清零。
	IER = 0x0000;
	IFR = 0x0000;

	// PieVectTable 结构体的定义在文件“DSP2833x_PieVect.h”里，去那里寻找对应的中断指针。
	// 重新定义中断向量表，如果你使用了中断，可以仿照以下格式定义自己中断服务函数。
	// 各外设的中断向量表重定义放到了Step3里。
	InitPieVectTable();	// 初始化中断向量表，这是必要的，即使你没有使用中断。这样做可以避免PIE引起的错误。

	// ==============================================================================
	//          Step 3. Initialize the Peripherals you used.
	// ==============================================================================
	InitGpio();     	// 把GPIO0~95设置为： 1，普通IO口，不使用特殊功能；2，输入方向；
//						// 3，CLK与SYSCLKOUT同步；4，使用上拉电阻；

	InitCpuTimers();   	// Peripheral 2: Cpu timer 初始化，if used.
	InitScis();			// Peripheral 3: SCI 初始化，if used.
	InitSpis();			// Peripheral 4: SPI 初始化，if used.
	InitXintf();		// initializes the External Interface the default reset state.
//	InitDmas();			// Peripheral 5: DMA 初始化，if used.

	//硬件底层初始化
	FlashSST39_Init();	// Flash芯片初始化
	Init_LED();			// 初始化LED灯
	Init_Motor();		// 初始化电机

//	StartDMACHx( &Dma.RegsAddr->CH1);
//	InitAdc();			// Initializes ADC to a known state.
//	InitECan();			// Initialize eCAN-A module
//	InitECap();			// Initialize the eCAP(s) to a known state.
//	InitEPwm();			// Initializes the ePWM(s) to a known state.
//	InitEQep();			// Initializes the eQEP(s) to a known state.
//	InitI2C();			// initializes the I2C to a known state.
//	InitMcbsp();		// initializes the McBSP to a known state.

// ==============================================================================
// 			Step 4. Enable interrupts:
// ==============================================================================
	//初始化一些参数
	Init_Kernel();
	Init_Command();

	//初始化各种运动模式
	Init_PP_JOG_Kernel();
	Init_PT_Kernel();
	Init_Crd_Kernel();
	Init_Stop_Kernel();

	// Enable global Interrupts and higher priority real-time debug events:
	EnableInterrupts();
}
