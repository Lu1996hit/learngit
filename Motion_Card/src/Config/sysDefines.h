/*
 *        File: sysDefines.h
 *     Version:
 * Description: ϵͳ�궨���ļ�
 *
 *  Created on: 2017��12��3��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SYSDEFINES_H_
#define SYSDEFINES_H_

#include "config.h"

/*
 *  1. ��������С����
 */

// ���Ի�������С
#ifndef COMMUNICATION_MAX_LEN
#define COMMUNICATION_MAX_LEN	256
#endif

// ѭ����������С
#ifndef CIRCLE_BUFFER_SIZE
#define CIRCLE_BUFFER_SIZE		256
#endif


//============================================================================
//      �ڲ���������
//============================================================================
#define USE_SCIA        		0   		// ʹ�� SCIA
// ���Զ˿ڣ�ok.
//#define USE_GPIO28_AS_SCIRXDA 	1			// RX - GPIO28
//#define USE_GPIO29_AS_SCITXDA 	1			// TX - GPIO29
#define SCIA_BAUD 				115200	 	    // SCIA BAUD = 115200

#define USE_SPIA 		1
#if( !TEST_BORD )
#define USE_GPIO19_AS_SPISTEA    1 			 // jzs �İ��Ӹ���ע�͵�
#else
#define USE_GPIO57_AS_SPISTEA    1
#endif
#define USE_GPIO54_AS_SPISIMOA   1      // func:2
#define USE_GPIO55_AS_SPISOMIA   1      // func:2
#define USE_GPIO56_AS_SPICLKA    1      // func:2

#define SPIA_PART_0             1
#define SPIA_PART_1             1

// �޸����Լ���FIFO����, ע�͵�ʹ��Ĭ�ϳ���16.
#define SPIA_SWFFTXDEEP CIRCLE_BUFFER_SIZE		// �������FIFO��TX�ռ��С
#define SPIA_SWFFRXDEEP CIRCLE_BUFFER_SIZE		// �������FIFO��RX�ռ��С


//ICS parameter define
#define MC_LIMIT_POSITIVE               0
#define MC_LIMIT_NEGATIVE               1
#define MC_ALARM                        2
#define MC_HOME                         3
#define MC_GPI                          4
#define MC_ARRIVE                       5

#define MC_ENABLE                       10
#define MC_CLEAR                        11
#define MC_GPO                          12

#define MC_DAC                          20
#define MC_STEP                         21
#define MC_PULSE                        22
#define MC_ENCODER                      23
#define MC_ADC                          24

#define MC_AXIS                         30
#define MC_PROFILE                      31
#define MC_CONTROL                      32

#endif /* SYSDEFINES_H_ */
