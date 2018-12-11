/*
 * 		  File: mymotors.h
 *     Version:
 * Description: FPGA�������
 *
 *  Created on: 2017��12��15��
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
    uint16_t AD_EN:1;     // 0,   ADCʹ���ź�
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
    uint16_t EN:1;      // 0,   DACʹ���ź�
    uint16_t CHANNEL:3; // 1~3, DAͨ��ѡ��
    uint16_t POWER:3;   // 4~6, DA�����Χѡ��
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
 * �����˶�ģʽ
 */
typedef enum MOTORS_STA{
	MOTORS_STA_IDLE,				//����ģʽ
	MOTORS_STA_PPMODE,				//�㶯ģʽ
	MOTORS_STA_JOGMODE,				//Jogģʽ
	MOTORS_STA_PTMODE,				//PTģʽ
	MOTORS_STA_GEARMODE,			//���ӳ���ģʽ
	MOTORS_STA_FOLLOWMODE,			//����ģʽ
	MOTORS_STA_CRDMODE,				//�岹ģʽ
	MOTORS_STA_CRDAUX,				//����ϵģʽ�¸���ģʽ����������ϵָ���еĵ�λ�˶��������˶���
	MOTORS_STA_STOPMODE,			//ֹͣ״̬

	MOTORS_STA_ERRORMODE			//����ģʽ
}MOTORS_STA;

extern volatile struct MOTORS_REGS MotorRegs[AXISNUM];


void Init_Motor();				//��ʼ�����
/* �������� */
void Init_Axis(int axis);		//��ʼ�����
void Motor_ServoOn(int axis);	//���ʹ��
void Motor_ServoOff(int axis);	//���ʹ�ܹر�
void Motor_Reset(int axis);		//�����λ

/* PT ģʽ������� */
void M_SetPvat( int axis, PVAT_S *dda);	// ����pvat������


//void testMyDAC(void);

#endif /* MOTORS_H_ */
