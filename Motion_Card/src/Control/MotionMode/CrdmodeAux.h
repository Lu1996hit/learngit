/*
 *        File: CrdmodeAux.h
 *     Version:
 * Description: ���ڼ�¼�岹�����е����ݽṹ
 *
 *  Created on: 2018��8��3��
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */
#ifndef SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_
#define SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_

#define CRDNUM  	2		//����������
#define CRDFIFONUM	2		//����������
#define CRDFIFOLEN	1024	//FIFO�ĳ���

/*
 * ����ϵ�ͻ������ŵĴ洢��ʵ��ʹ�õ�ʱ��û��ʲô����
 */
typedef struct
{
	unsigned int type:8;		//��ʾ��ǰָ������
	unsigned int crd:1;			//��ʾ����ϵ
	unsigned int fifo:1;		//��ʾ��������
	unsigned int cdir:1;		//Բ������ת���򣬽���Բ��ָ������Ч
								//0��˳ʱ��Բ��		1����ʱ��Բ��
	unsigned int ready:2;		//0����ǰ�洢������
								//1������Ԥ��������ˣ���������
								//2��������
								//3�����н���
}CRD_FIFO_REG;

/*
 * ��άֱ��ָ��
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
 * ��άֱ��ָ��
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
 * ��άֱ��ָ��
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
 * ƽ��Բ���岹�����յ�λ�úͰ뾶Ϊ���������
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
 * ƽ��Բ���岹�����յ�λ�ú�Բ��λ��Ϊ���������
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
 * ��������������IO�������ָ��
 * doType:
 * ������������͡�
 * MC_ENABLE���ú궨��Ϊ10�������������ʹ�ܡ�
 * MC_CLEAR���ú궨��Ϊ11����������������������
 * MC_GPO���ú궨��Ϊ12�������ͨ�������
 * doMask��
 * ��bit0~bit15��λ��ʾָ��������������Ƿ��в�����
 * 0����·����������޲�����
 * 1����·����������в�����
 * doValue��
 * ��bit0~bit15��λ��ʾָ���������������ֵ��
 */
typedef struct
{
	unsigned short doType;			//��������
	unsigned short doMask;
	unsigned short doValue;
}CMDIO;

/*
 * ����������ʱ����ָ��
 */
typedef struct
{									//��������
	unsigned short delayTimel;		//��ʱʱ��
}CMDDELAY;

/*
 * �����������DA��ֵ
 */
typedef struct
{
	short chn;						//��������
	short daValue;
}CMDDA;

/*
 * ����������Ч/��Ч��λ����ָ��
 */
typedef struct
{
	short axis;
	short limitType;
}CMDLMTS;

/*
 * ������������axis��ֹͣIO��Ϣ
 * inputType
 * ���õ���������������
 * MC_LIMIT_POSITIVE���ú궨��Ϊ0��������λ
 * MC_LIMIT_NEGAITIVE���ú궨��Ϊ1��������λ
 * MC_ALARM���ú궨��Ϊ2������������
 * MC_HOME���ú궨��Ϊ3����ԭ�㿪��
 * MC_GPI���ú궨��Ϊ4����ͨ������
 * MC_ARRIVE���ú궨��Ϊ5���������λ�ź�
 * inputIndex
 */
typedef struct
{
	short axis;						//����������ϵ��
	short stopType;					//0������ֹͣ����		1��ƽ��ֹͣ����
	short inputType;
	short inputIndex;
}CMDSETSTOPIO;

/*
 * ʵ�ֵ�����湦�ܣ�����ĳ�����λ�˶�
 */
typedef struct
{
	short moveAxis;					//���᲻������ϵ�ڣ���������ϵ��
	long pos;
	double vel;
	double acc;
	short modal;
}CMDBUFFMOVE;

/*
 * ʵ�ֵ�����湦�ܣ�����ĳ��������˶�
 */
typedef struct
{
	short gearAxis;					//���᲻������ϵ�ڣ���������ϵ��
	long pos;
}CMDBUFFGEAR;

/*
 * ����ö��
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
 * ָ�����
 */
typedef struct
{
	CRD_FIFO_REG tcf;			//�ṹ�����
	CMDUN prm;					//ö�ٱ���
}CMDCRD;

#endif /* SRC_CONTROL_MOTIONMODE_CRDMODEAUX_H_ */
