 /*
 *  Created on: 2016-12-2
 *      Author: QUENTIN
 *      E-mail: qiyuexin@yeah.net
 */
//###########################################################################
//
// FILE:   my_project.h
//
// TITLE:  DSP2833x Device Definitions.
//
//###########################################################################


#ifndef MY_PROJECT_H
#define MY_PROJECT_H

#include "../../src/Control/MotionConfig/AxisSet/AxisSet.h"
#include "DSP2833x_Device.h"     	// DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   	// DSP2833x Examples Include File

#include "math.h"
#include "IQmathLib.h"
#include "myPeripherals.h"		  	// Chen 定义的外设调用文件
#include "system.h"

//device
#include "device.h"

//device configuration
#include "AxisSet.h"


//Communication
#include "taskComm.h"
#include "taskAnalysis.h"


#include "interpolate.h"

//Motion mode
#include "PTmode.h"
#include "PPmode.h"
#include "JOGmode.h"
#include "Crdmode.h"
#include "Stopmode.h"


//Communication
#include "communication.h"

#endif // DSP2833x_PROJECT_H
