/*
 * 		  File: main.c
 *     Version: V2.0
 * Description: �����˶����ƿ�
 *
 *  Created on: 2017��12��10��
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

/*
 * ����λ���Ϲ�����ָ�SPI --> taskComm --> taskAnalysys  --> Execute
 * Ŀ�꣺��Linux�Ͽ���������̬���ӿ⣬��Ҫ�Ƕ�̬���ӿ�̫�鷳��
 */

#include "my_project.h"

void main()
{
	InitPeripherals();
	DELAY_US(10);

//	Init_Axis(0);

	while(1)
	{
		if(RTN_SUCC == checkNewCommand())		//���ճɹ���
		{
			runCmd();							//����
		}
		prepCmd();								//Ԥ�������ݣ���������
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
