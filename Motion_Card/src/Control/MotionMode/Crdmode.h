/*
 *        File: Crdmode.h
 *     Version:
 * Description:
 *
 *  Created on: 2018年7月20日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_CONTROL_MOTIONMODE_CRDMODE_H_
#define SRC_CONTROL_MOTIONMODE_CRDMODE_H_

typedef struct CrdPrm
{
	short dimension;					//1 * 1 = 1
	short axisIndex[4];					//1 * 4 = 4
	double synVelMax;					//2 * 1 = 2
	double synAccMax;					//2 * 1 = 2
	short evenTime;						//1 * 1 = 1
	short setOriginFlag;				//1 * 1 = 1
	long originPos[4];					//2 * 4 = 8
										//sum = 1 + 4 + 2 + 2 + 1 + 1 + 8 = 19 + 1 = 20
}TCrdPrm;

/*
 * dimension：坐标系的维数。取值范围：[2,4]。
 * axisIndex[4]：坐标系与规划器的映射关系。axisIndex[0..3]对应规划轴XYZA。取值范围[0,7]
 * synVelMax：该坐标系的最大合成速度。如果用户在输入插补段的时候所设置的目标速度大于了该速度，则将
 * 会被限制为该速度。取值范围：(0,32767)。单位：pulse/ms。
 * synAccMax：该坐标系的最大合成加速度。如果用户在输入插补段的时候所设置的加速度大于了该加速度，则
 * 将会被限制成该加速度。取值范围：(0,32767)。单位：pulse/ms^2。
 * evenTime：每个插补段的最小匀速段时间。取值范围：[0,32767)。单位：ms。
 * setOriginFlag：表示是否需要指定坐标系的原点坐标的规划位置，该参数可以方便用户建立区别于机床坐标
 * 系的加工坐标系。0：不需要指定原点坐标值，则坐标系的原点在当前规划位置上。1：需要指定原点坐标值，
 * 坐标系的原点在originPos指定的规划位置上。
 * originPos[4]：指定的坐标系原点的规划位置值。
 */

extern void Init_Crd_Kernel();
extern void Decouple_CRDmode();
extern void Run_CRDmode(int axis);
extern ERROR_CODE Prep_CRDmode(int axis);

#endif /* SRC_CONTROL_MOTIONMODE_CRDMODE_H_ */
