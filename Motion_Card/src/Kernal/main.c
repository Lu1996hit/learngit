/*
 * 		  File: main.c
 *     Version: V2.0
 * Description: 八轴运动控制卡
 *
 *  Created on: 2017年12月10日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

/*
 * 从上位机上过来的指令：SPI --> taskComm --> taskAnalysys  --> Execute
 * 目标：在Linux上开发出来静态链接库，主要是动态链接库太麻烦了
 */

#include "my_project.h"

void main()
{
	InitPeripherals();
	DELAY_US(10);

//	Init_Axis(0);

	while(1)
	{
		if(RTN_SUCC == checkNewCommand())		//接收成功？
		{
			runCmd();							//运行
		}
		prepCmd();								//预处理数据，处理数据
	}
}

/*
 * char		1 8
 * short 	1 16
 * int		1 16
 * long		2 32
 * float	2 32
 * double	2 32
 */
