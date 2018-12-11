/*
 *        File: sysDefines.h
 *     Version:
 * Description: 系统宏定义文件
 *
 *  Created on: 2017年12月3日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SYSDEFINES_H_
#define SYSDEFINES_H_

#include "config.h"

/*
 *  1. 缓存区大小定义
 */

// 线性缓冲区大小
#ifndef COMMUNICATION_MAX_LEN
#define COMMUNICATION_MAX_LEN	256
#endif

// 循环缓冲区大小
#ifndef CIRCLE_BUFFER_SIZE
#define CIRCLE_BUFFER_SIZE		256
#endif


//============================================================================
//      内部外设配置
//============================================================================
#define USE_SCIA        		0   		// 使用 SCIA
// 测试端口，ok.
//#define USE_GPIO28_AS_SCIRXDA 	1			// RX - GPIO28
//#define USE_GPIO29_AS_SCITXDA 	1			// TX - GPIO29
#define SCIA_BAUD 				115200	 	    // SCIA BAUD = 115200

#define USE_SPIA 		1
#if( !TEST_BORD )
#define USE_GPIO19_AS_SPISTEA    1 			 // jzs 的板子该行注释掉
#else
#define USE_GPIO57_AS_SPISTEA    1
#endif
#define USE_GPIO54_AS_SPISIMOA   1      // func:2
#define USE_GPIO55_AS_SPISOMIA   1      // func:2
#define USE_GPIO56_AS_SPICLKA    1      // func:2

#define SPIA_PART_0             1
#define SPIA_PART_1             1

// 修改你自己的FIFO长度, 注释掉使用默认长度16.
#define SPIA_SWFFTXDEEP CIRCLE_BUFFER_SIZE		// 设置软件FIFO的TX空间大小
#define SPIA_SWFFRXDEEP CIRCLE_BUFFER_SIZE		// 设置软件FIFO的RX空间大小


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
