/*
 * 		  File: interpolate.h
 *     Version:
 * Description: 
 * 
 *  Created on: 2018年3月15日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_INTERPOLATE_H_
#define SRC_CONTROL_INTERPOLATE_H_

#include "mymotors.h"


//插补控制周期
//不允许修改
//#define TIME 5000			//0.1ms
#define TIME 10000			//0.1ms  超频数据

/*
 * 内核状态枚举
 */
typedef enum {
	IDLE_STA = 0,		//空闲状态
	UP_STA ,			//直线插补的时候用
	STEADY_STA ,		//直线插补的时候用
	DOWN_STA , 			//直线插补的时候用
	CIRCLE_STA ,		//圆周插补的时候用
	UNCONTROL_STA ,		//用于某些不可控但是不可控时间是确定的状态
	GOHOME_STA,			//归零运动的状态

}INTERPOLATE_COMMAND_TYPE;

/*
 * 计算的时候取sin计算还是取cos计算
 */
typedef enum {
	COS_MODE = 0,
	SIN_MODE ,
}SIN_COS_COMMAND_TYPE;

/*
 * 插补内核结构体
 * 这其中有两个state
 * 第一个电机状态，表示把当前的轴设置为某种功能
 * 第二个电机状态，表示当前轴正在运行的状态（如果不是IDLE状态，则当前轴说明在运动）
 */
typedef struct
{
	MOTORS_STA axsta;			//轴的初始状态		预处理，运行，依据这个状态
	MOTORS_STA kersta;			//内核运行的状态

	int flag;                   // 是否需要重新规划的标志 1：需要重新规划 0：不需要重新规划 规划完成需要清0
    int step;                   // 状态标志位：将电机的速度曲线规划为若干段
    long count;                 // 程序执行计数器、每 0.1 ms 递增1

    int axis;                   // 当前轴的指向
    int dir;                    // 运行方向     1：正向        0：反向

    long nowPos;                // 当前位置
    //long aimPos;              // 当前指令下的目标位置
    double realPos;             // 下一步的真实位置

    //关于dir可以修改成，当轴在运动时，dir = 1表示正向运动，dir = 2表示反向运动，当轴静止时，dir = 0；
    //可以通过内核的状态来判断当前的轴是不是正在运动，不需要把运动的状态在记录到dir中

    double nowacc;              // 当前加速度
    double nowVel;              // 当前速度。单位：pulse/ms。
    double lastVel;             // 上一时刻速度
}KernelPrm;

extern KernelPrm kernel[AXISNUM];

long Approximate(double value);				//四舍五入的代码
extern void Init_Kernel();
extern void Run_Kernel();


#endif /* SRC_CONTROL_INTERPOLATE_H_ */
