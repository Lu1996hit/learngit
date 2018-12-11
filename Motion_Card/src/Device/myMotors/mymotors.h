/*
 * 		  File: mymotors.h
 *     Version:
 * Description: FPGA电机部分
 *
 *  Created on: 2017年12月15日
 *      Author: Joye
 *      E-mail: chenchenjoye@sina.com
 */

#ifndef MYMOTORS_H_
#define MYMOTORS_H_

//---------------------------------------------------
// Motor Status register bit definitions:
//
struct MSTA_BITS{
    uint16_t MBSY:1;    // 0,   Motor Busy, high active.
    uint16_t NMSG:1;    // 1,   Next Message
    uint16_t LMTN:1;    // 2,   limit- flag, high active.
    uint16_t LMTP:1;    // 3,   limit+ flag, high active.
    uint16_t ENCZ:1;    // 4,   encoder_z signal
    uint16_t ALM:1;     // 5,   Motor Alarm
    uint16_t HOME:1;	// 6,	Motor Home

    uint16_t rsvd:9;
  };

union MSTA_REG{
    uint16_t          all;
    struct MSTA_BITS  bit;
};

//---------------------------------------------------
// Motor Control register bit definitions:
//

struct MCTL_BITS{
    uint16_t RST:1;         // 0,   reset
    uint16_t ENA:1;         // 1,   enable
    uint16_t START:1;       // 2,   start
    uint16_t CLR_ALM:1;     // 3,   for pc only
    uint16_t ENC_EN:1;      // 4,   Encoder Enable
    uint16_t ENC_RST:1;     // 5,   Encoder Reset
    uint16_t LIMIT_RST:1;	// 6,	Limit Clear


    uint16_t rsvd2:9;
  };

union MCTL_REG{
    uint16_t          all;
    struct MCTL_BITS  bit;
};

//---------------------------------------------------
// Motor Configure register bit definitions:
//
struct MCONF_BITS{
    uint16_t LIMITNV:1;     // 0,   limit-'s active value
    uint16_t LIMITPV:1;     // 1,   limit+'s active value
    uint16_t LIMITDIS:1;    // 2,   1: limit function is disable
    enum MMODE_E{           // 3-4, Motor Output Mode,
        DIR_PUL_MODE = 0,
        CW_CCW_MODE,
        AD_DA_MODE} MMODE:2;
    uint16_t rsvd2:11;
  };

union MCONF_REG{
    uint16_t           all;
    struct MCONF_BITS  bit;
};

//---------------------------------------------------
// ADC Configure register bit definitions:
//
struct ADCONF_BITS{
    uint16_t AD_EN:1;     // 0,   ADC使能信号
    uint16_t rsvd2:11;
  };

union ADCONF_REG{
    uint16_t            all;
    struct ADCONF_BITS  bit;
};

//---------------------------------------------------
// ADC Configure register bit definitions:
//
struct DACONF_BITS{
    uint16_t EN:1;      // 0,   DAC使能信号
    uint16_t CHANNEL:3; // 1~3, DA通道选择
    uint16_t POWER:3;   // 4~6, DA输出范围选择
    uint16_t rsvd2:9;
  };

union DACONF_REG{
    uint16_t            all;
    struct DACONF_BITS  bit;
};


//---------------------------------------------------------------------------
// Motor Register File:
//
struct MOTORS_REGS{
    // 0x00~0x0f
    int32_t     INPOS;              // IN Position
    int32_t     INVEL;              // velocity
    int32_t     INACC;              // acceleration
    int32_t     INXX;  	            // jerk
    int32_t     NOWPOS;
    int32_t     NOWVEL;
    int32_t     NOWACC;
    int32_t     NOWXX;
    // 0x10~0x17
    int32_t     ENCP;	// Encode Position
    int32_t     ENCV;	// Encode Velocity

    //0x18~0x21
    int16_t             AD_RESULT;
    union ADCONF_REG    AD_CONF;
    uint16_t            AD_BIAS;
    uint16_t            DA_VALUE;
    union DACONF_REG    DA_CONF;
    int16_t             rsvdRegs1;

    union MSTA_REG      MSTA;         // Motor Status register
    union MCTL_REG      MCTL;         // Motor Control register
    union MCONF_REG     MCONF;        // Motor Configure register
    uint16_t            rsvdRegs2[2];
    uint16_t 			MTCNT;
};

/*
 * 定义运动模式
 */
typedef enum MOTORS_STA{
	MOTORS_STA_IDLE,				//空闲模式
	MOTORS_STA_PPMODE,				//点动模式
	MOTORS_STA_JOGMODE,				//Jog模式
	MOTORS_STA_PTMODE,				//PT模式
	MOTORS_STA_GEARMODE,			//电子齿轮模式
	MOTORS_STA_FOLLOWMODE,			//跟随模式
	MOTORS_STA_CRDMODE,				//插补模式
	MOTORS_STA_CRDAUX,				//坐标系模式下辅助模式，用在坐标系指令中的点位运动，跟随运动中
	MOTORS_STA_STOPMODE,			//停止状态

	MOTORS_STA_ERRORMODE			//错误模式
}MOTORS_STA;

extern volatile struct MOTORS_REGS MotorRegs[AXISNUM];


void Init_Motor();				//初始化电机
/* 基本操作 */
void Init_Axis(int axis);		//初始化电机
void Motor_ServoOn(int axis);	//电机使能
void Motor_ServoOff(int axis);	//电机使能关闭
void Motor_Reset(int axis);		//电机复位

/* PT 模式操作相关 */
void M_SetPvat( int axis, PVAT_S *dda);	// 输入pvat的数据


//void testMyDAC(void);

#endif /* MOTORS_H_ */
