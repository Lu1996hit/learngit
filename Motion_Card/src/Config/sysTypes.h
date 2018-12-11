#ifndef __MY_TYPES_H__
#define __MY_TYPES_H__

#include "config.h"
#include "sysDefines.h"			// ϵͳ�����ļ������û��Զ��塣

#ifndef MY_STD_TYPE
#define MY_STD_TYPE

typedef unsigned int    word;
typedef unsigned char   byte;
typedef unsigned char   uint8_t;
typedef 		 char   int8_t;
typedef unsigned int    uint16_t;
typedef          int    int16_t;
typedef unsigned long   uint32_t;
typedef          long   int32_t;

#endif

#define MAX_PRM_LENGTH	256
#define INTERRUPT_TIM 0.1
#define INTERRUPT_FRE 10   // ����������Ӧ����Ϊ����
/*
 * ״̬������
 * ���ص�ǰ�������е�״̬
 * ���Ƿ�����Ƿ��������е�
 */

typedef enum ERROR_CODE {
	RTN_SUCC = 0,
	RTN_ERROR = -1,
	RTN_INVALID_COMMAND = -2,
	RTN_INVALID_MEMORY = -3,
	RTN_DIVIDED_BY_ZERO = -4,
	RTN_NO_SPACE = -5,
	RTN_CMD_EXECUTING = -6,
	RTN_TRANS_STATE_INVALID = -7,
	RTN_CMD_AVAILABLE = -8,
	RTN_INVALID_PARAMETER = -9
//	RTN_ERROR = 1,
//	RTN_INVALID_COMMAND = 2,
//	RTN_INVALID_MEMORY = 3,
//	RTN_DIVIDED_BY_ZERO = 4,
//	RTN_NO_SPACE = 5,
//	RTN_CMD_EXECUTING = 6,
//	RTN_TRANS_STATE_INVALID = 7,
//	RTN_CMD_AVAILABLE = 8
}ERROR_CODE;

/*
 * ��λָ����
 */
typedef enum RESET_CODE {
    RUN_RESET = 0,
    CANCLE_RESET = 1

}RESET_CODE;

/*
 * ����״̬��
 */
typedef enum ONOFF_CODE {
    OFF_STATE = 0,
    ON_STATE = 1

}ONOFF_CODE;

/*
 * ָ������
 */
typedef enum COMMAND_TYPE {
	CMD_AXIS   = 0x0000,
	CMD_AXIS_ON ,
	CMD_AXIS_OFF,
	CMD_AXIS_RESET,
	CMD_DSPLED_ON,
	CMD_DSPLED_OFF,

	CMD_LINE_MODE,
	CMD_CIRCLE_MODE,
	CMD_GO_HOME,
	CMD_ENTER_JOG,
	CMD_ENTER_PT,
	CMD_SET_DDA,

	CMD_RD_DDA = 0x20,
	CMD_RD_MSTA,
	CMD_RD_MFIFO,
	CMD_RD_SRAM,

	CMD_PP_MODE = 0x8100,
	CMD_PP_SETPRM,
	CMD_PP_GETPRM,
	CMD_PP_SETPOS,
	CMD_PP_GETPOS,

	CMD_SETVEL,
	CMD_GETVEL,
	CMD_UPDATE,

	CMD_JOG_MODE = 0x8200,
	CMD_JOG_SETPRM,
	CMD_JOG_GETPRM,

	CMD_PT_MODE = 0x8300,
	CMD_PT_SPACE,
	CMD_PT_DATA,
	CMD_PT_CLEAR,
	CMD_PT_SET_LOOP,
	CMD_PT_GET_LOOP,
	CMD_PT_START,
	
	//��������ϵʱ�Զ������ģʽ,��������������ָ��ı����ͬ
	CMD_CRD_MODE = 0x8600,
	CMD_SET_CRD_PRM = 0x8600,
	CMD_GET_CRD_PRM,
	CMD_QUIT_CRD,
	CMD_CRD_DATA,
	CMD_LN_XY,
	CMD_LN_XYZ,
	CMD_LN_XYZA,
	CMD_LN_XYG0,
	CMD_LN_XYZG0,
	CMD_LN_XYZAG0,
	CMD_ARC_XYR,
	CMD_ARC_XYC,
	CMD_ARC_YZR,
	CMD_ARC_YZC,
	CMD_ARC_ZXR,
	CMD_ARC_ZXC,
	CMD_BUF_IO,
	CMD_BUF_DELAY,
	CMD_BUF_DA,
	CMD_BUF_LMTS_ON,
	CMD_BUF_LMTS_OFF,
	CMD_BUF_SET_STOP_IO,
	CMD_BUF_MOVE,
	CMD_BUF_GEAR,
	CMD_CRD_SPACE,
	CMD_CRD_CLEAR,
	CMD_CRD_START,
	CMD_CRD_STATUS,
	CMD_GET_REMAINDERS_SEG_NUM,
	CMD_SET_CRD_STOP_DEC,
	CMD_GET_CRD_STOP_DEC,
	CMD_GET_CRD_POS,
	CMD_GET_CRD_VEL,
	CMD_INIT_LOOK_AHEAD,

	CMD_STOP_MODE = 0x8F00,
	CMD_SET_STOP_DEC = 0x8F00,
	CMD_GET_STOP_DEC,
	CMD_STOP_MOVE,

}COMMAND_TYPE;


/* DDA ģʽ����
 * dda_pvt ģʽʹ�ñ���
 */
typedef struct {
	long aim_pos;			// Ŀ��λ��
	long start_vel;			// ��ʼ�ٶ�
	long start_acc;			// ��ʼ���ٶ�
	long min_period;		// ��С����ʱ��
}PVAT_S;

/*
 * ָ��ṹ��
 */
typedef struct
{
	//�����ĸ�ֵ��λ��˳���ܸı�
	COMMAND_TYPE type;   					//ָ������
	unsigned int mark;						//��ָ����ص���
	ERROR_CODE rtn;							//ָ���ֵ
	unsigned int buf[MAX_PRM_LENGTH];		//���ز���

	unsigned int lastSign;					//���ֵ������У��ָ��
	unsigned int buflen;					//���ز�������
	unsigned int prm[MAX_PRM_LENGTH];		//������Ĳ����洢λ��
	/*
	 * ����prm�����͹�����ʱ��prm[0]�洢����type��prm[1]�洢����mark��ֵ
	 * prm[2]�洢����ָ�����кţ�����ʵ���Ǵ�prm[3]��ʼ��
	 */
	unsigned long count;					//ָ�������
}COMMAND;

#endif // __MY_TYPES_H__
