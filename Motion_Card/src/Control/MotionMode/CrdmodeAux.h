/*
 *        File: CrdmodeAux.h
 *     Version:
 * Description: 用于记录插补过程中的数据结构
 *
 *  Created on: 2018年8月3日
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */
#ifndef SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_
#define SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_

#define CRDNUM  	2		//两个坐标轴
#define CRDFIFONUM	2		//两个缓冲区
#define CRDFIFOLEN	1024	//FIFO的长度

/*
 * 坐标系和缓存区号的存储，实际使用的时候没有什么意义
 */
typedef struct
{
	unsigned int type:8;		//表示当前指令类型
	unsigned int crd:1;			//表示坐标系
	unsigned int fifo:1;		//表示缓存区号
	unsigned int cdir:1;		//圆弧的旋转方向，仅在圆弧指令中有效
								//0：顺时针圆弧		1：逆时针圆弧
	unsigned int ready:2;		//0：当前存储了数据
								//1：数据预处理完成了，可以运行
								//2：运行中
								//3：运行结束
}CRD_FIFO_REG;

/*
 * 二维直线指令
 */
typedef struct
{
	double synVel;
	double synAcc;
	double velEnd;
	long x;
	long y;
}CMDXY;

/*
 * 三维直线指令
 */
typedef struct
{
	double synVel;
	double synAcc;
	double velEnd;
	long x;
	long y;
	long z;
}CMDXYZ;

/*
 * 四维直线指令
 */
typedef struct
{
	double synVel;
	double synAcc;
	double velEnd;
	long x;
	long y;
	long z;
	long a;
}CMDXYZA;

/*
 * 平面圆弧插补（以终点位置和半径为输入参数）
 */
typedef struct
{
	double synVel;
	double synAcc;
	double velEnd;
	long a;
	long b;
	double r;
}CMDABR;

/*
 * 平面圆弧插补（以终点位置和圆心位置为输入参数）
 */
typedef struct
{
	double synVel;
	double synAcc;
	double velEnd;
	long a;
	long b;
	double ac;
	double bc;
}CMDABC;

/*
 * 缓存区内数字量IO输出设置指令
 * doType:
 * 数字量输出类型。
 * MC_ENABLE（该宏定义为10）：输出驱动器使能。
 * MC_CLEAR（该宏定义为11）：输出驱动器报警清除。
 * MC_GPO（该宏定义为12）：输出通用输出。
 * doMask：
 * 从bit0~bit15按位表示指定的数字量输出是否有操作。
 * 0：该路数字量输出无操作。
 * 1：该路数字量输出有操作。
 * doValue：
 * 从bit0~bit15按位表示指定的数字量输出的值。
 */
typedef struct
{
	unsigned short doType;			//不区分轴
	unsigned short doMask;
	unsigned short doValue;
}CMDIO;

/*
 * 缓存区内延时设置指令
 */
typedef struct
{									//不区分轴
	unsigned short delayTimel;		//延时时间
}CMDDELAY;

/*
 * 缓存区内输出DA的值
 */
typedef struct
{
	short chn;						//不区分轴
	short daValue;
}CMDDA;

/*
 * 缓存区内有效/无效限位开关指令
 */
typedef struct
{
	short axis;
	short limitType;
}CMDLMTS;

/*
 * 缓存区内设置axis的停止IO信息
 * inputType
 * 设置的数字量输入类型
 * MC_LIMIT_POSITIVE（该宏定义为0）：正限位
 * MC_LIMIT_NEGAITIVE（该宏定义为1）：负限位
 * MC_ALARM（该宏定义为2）：驱动报警
 * MC_HOME（该宏定义为3）：原点开关
 * MC_GPI（该宏定义为4）：通用输入
 * MC_ARRIVE（该宏定义为5）：电机到位信号
 * inputIndex
 */
typedef struct
{
	short axis;						//该轴在坐标系内
	short stopType;					//0：紧急停止类型		1：平滑停止类型
	short inputType;
	short inputIndex;
}CMDSETSTOPIO;

/*
 * 实现刀向跟随功能，启动某个轴点位运动
 */
typedef struct
{
	short moveAxis;					//该轴不在坐标系内（所有坐标系）
	long pos;
	double vel;
	double acc;
	short modal;
}CMDBUFFMOVE;

/*
 * 实现刀向跟随功能，启动某个轴跟随运动
 */
typedef struct
{
	short gearAxis;					//该轴不在坐标系内（所有坐标系）
	long pos;
}CMDBUFFGEAR;

/*
 * 命令枚举
 */
typedef union
{
	CMDXY 			xy;
	CMDXYZ 			xyz;
	CMDXYZA 		xyza;
	CMDABR			abr;
	CMDABC			abc;
//	CMDABR			yzr;
//	CMDABC			yzc;
//	CMDABR			zxr;
//	CMDABC			zxc;
	CMDIO			io;
	CMDDELAY		delay;
	CMDDA			da;
	CMDLMTS			lmt;
	CMDSETSTOPIO	stopio;
	CMDBUFFMOVE		move;
	CMDBUFFGEAR		gear;
}CMDUN;

/*
 * 指令组合
 */
typedef struct
{
	CRD_FIFO_REG tcf;			//结构体变量
	CMDUN prm;					//枚举变量
}CMDCRD;

#endif /* SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_ */
