#ifndef _PROCOTOL_H_
#define _PROCOTOL_H_


#include "sysTypes.h"

/*
 *  2. ��ARMЭ�鲿��
 */

// ���ĵ�ͷ
#define MSG_HEAD  '#'

extern ERROR_CODE protocol(unsigned int chc, unsigned int *dat_buf, unsigned int *dat_len);
extern unsigned int check_prm(unsigned int* pdat, unsigned int len);

#endif
