/*
 *        File: myIsr.c
 *     Version:
 * Description: 中断服务函数
 *
 *  Created on: 2017年12月18日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#include "my_project.h"

//============================================================================
//      Peripheral 2. Cpu Timer
//============================================================================

#if( USE_CPU_TIMER0 )

interrupt void cpu_timer0_isr(void)			//0.1ms定时中断
{
//	static Uint32 preCount;
	Run_Kernel();

	PieCtrlRegs.PIEACK.bit.ACK1 = 1;			//此处需要清零		注意：写1清0
}
#endif

#if( USE_CPU_TIMER1 )
interrupt void cpu_timer1_isr(void)
{
	CpuTimer1.InterruptCount++;

#if( MY_TEST_DEMO == TEST_GPIO_TIMER_LED)
	//   Functions of TEST_GPIO_TIMER_LED
	if( timer_int_cnt++ >= 12) {
		timer_int_cnt = 0;
		LED3 = LED_OFF;
		CpuTimer0Regs.TCR.bit.TIE	= 0;	// 设置TIE = 0，关闭定时器0中断
		CpuTimer1Regs.TCR.bit.TIE  	= 0;	// 设置TIE = 0，开启定时器1中断
		CpuTimer2Regs.TCR.bit.TIE  	= 1; 	// 设置TIE = 1，关闭定时器2中断
	}

	LED3_TOG = 1;
#endif


	// The CPU acknowledges the interrupt.

}
#endif

#if( USE_CPU_TIMER2 )
interrupt void cpu_timer2_isr(void)
{
	CpuTimer2.InterruptCount++;

#if( MY_TEST_DEMO == TEST_GPIO_TIMER_LED)
	//   Functions of TEST_GPIO_TIMER_LED
	if( timer_int_cnt++ >= 12) {
		timer_int_cnt = 0;
		LED4 = LED_OFF;
		CpuTimer0Regs.TCR.bit.TIE	= 1;	// 设置TIE = 1，关闭定时器0中断
		CpuTimer1Regs.TCR.bit.TIE  	= 0;	// 设置TIE = 0，开启定时器1中断
		CpuTimer2Regs.TCR.bit.TIE  	= 0; 	// 设置TIE = 0，关闭定时器2中断
	}

	LED4_TOG = 1;
#endif


	// The CPU acknowledges the interrupt.

}
#endif

//============================================================================
//      Peripheral 3. Sci
//============================================================================
#if(USE_SCI_INT && USE_SCI_FIFO)

#if(USE_SCIA)
interrupt void scia_rx_isr(void)
{
	if(Scia.RegsAddr->SCIFFRX.bit.RXFFINT){
		//		RXFIFO 中断，我们设置的TXFFIL=16，所以当TXFFST=16时会触发中断。
		Sci_RxFifoFullHandler(&Scia);
	}
	if(Scia.RegsAddr->SCIFFRX.bit.RXFFOVF){
		//		overflow.
		Sci_RxFifoFullHandler(&Scia);
		Scia.RegsAddr->SCIFFRX.bit.RXFFOVFCLR = 1;
	}
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}
interrupt void scia_tx_isr(void)
{
	if( Scia.RegsAddr->SCIFFTX.bit.TXFFINT ) {
		//		TXFIFO 中断，我们设置的TXFFIL=0，所以当TXFFST=0时会触发中断。
		Sci_TxFifoFullHandler(&Scia);
	}
	// Acknowledge this interrupt to receive more interrupts from group 9
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}
#endif //(USE_SCIA)

#if(USE_SCIB)
interrupt void scib_rx_isr(void)
{
	if(Scib.RegsAddr->SCIFFRX.bit.RXFFINT){
		//		RXFIFO 中断，我们设置的TXFFIL=16，所以当TXFFST=16时会触发中断。
		Sci_RxFifoFullHandler(&Scib);
	}
	if(Scib.RegsAddr->SCIFFRX.bit.RXFFOVF){
		//		overflow.
		Sci_RxFifoFullHandler(&Scib);
		Scib.RegsAddr->SCIFFRX.bit.RXFFOVFCLR = 1;
	}
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}
interrupt void scib_tx_isr(void)
{
	if( Scib.RegsAddr->SCIFFTX.bit.TXFFINT ) {
		//		TXFIFO 中断，我们设置的TXFFIL=0，所以当TXFFST=0时会触发中断。
		Sci_TxFifoFullHandler(&Scib);
	}
	// Acknowledge this interrupt to receive more interrupts from group 9
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}
#endif // (USE_SCIB)

#if(USE_SCIC)
interrupt void scic_rx_isr(void)
{
	if(Scic.RegsAddr->SCIFFRX.bit.RXFFINT){
		//		RXFIFO 中断，我们设置的TXFFIL=16，所以当TXFFST=16时会触发中断。
		Sci_RxFifoFullHandler(&Scic);
	}
	if(Scic.RegsAddr->SCIFFRX.bit.RXFFOVF){
		//		overflow.
		Sci_RxFifoFullHandler(&Scic);
		Scic.RegsAddr->SCIFFRX.bit.RXFFOVFCLR = 1;
	}
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;
}
interrupt void scic_tx_isr(void)
{
	if( Scic.RegsAddr->SCIFFTX.bit.TXFFINT ) {
		//		TXFIFO 中断，我们设置的TXFFIL=0，所以当TXFFST=0时会触发中断。
		Sci_TxFifoFullHandler(&Scic);
	}
	// Acknowledge this interrupt to receive more interrupts from group 9
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;
}
#endif // (USE_SCIC)

#endif //(USE_SCI_INT && USE_SCI_FIFO)


//============================================================================
//      Peripheral 4. SPI
//============================================================================
#if(USE_SPIA)
long SPI_COUNT = 0;
interrupt void spia_rx_isr(void)
{
	unsigned int tmp;
	if(Spia.RegsAddr->SPISTS.bit.INT_FLAG)
	{
		if( RTN_ERROR != cb_get(&Spia.cb_tx, &tmp) )
		{
		}
		else
		{
			tmp = 0xAA55;
		}

		SpiaRegs.SPITXBUF = tmp;

		//接收内容
		tmp = SpiaRegs.SPIRXBUF;
		SPI_COUNT ++;
		cb_append(&Spia.cb_rx, &tmp);
	}
	if(Spia.RegsAddr->SPISTS.bit.OVERRUN_FLAG)
	{
		SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;
	}
	PieCtrlRegs.PIEACK.bit.ACK6 = 1;
}

/*
 * 发送中断程序，没有用
 */
interrupt void spia_tx_isr(void)
{
//	// Acknowledge this interrupt to receive more interrupts from group 6
	PieCtrlRegs.PIEACK.bit.ACK6 = 1;
}

#endif // (USE_SPIA)
