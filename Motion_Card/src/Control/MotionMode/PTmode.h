/*
 * 		  File: PTmode.c
 *     Version:
 * Description:
 *
 *  Created on: 2018Äê3ÔÂ15ÈÕ
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_PTMODE_H_
#define SRC_CONTROL_PTMODE_H_

extern void Init_PT_Kernel();
extern void Run_PTmode(int axis);
extern void Decouple_PTmode();
extern ERROR_CODE Prep_PTmode(int axis);

#endif /* SRC_CONTROL_PTMODE_H_ */
