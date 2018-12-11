/*
 *        File: EXIO.h
 *     Version:
 * Description:
 *
 *  Created on: 2018Äê8ÔÂ25ÈÕ
 *      Author: Joye
 *      E-main: chenchenjoye@sina.com
 */

#ifndef SRC_DEVICE_MYEXIO_EXIO_H_
#define SRC_DEVICE_MYEXIO_EXIO_H_

struct EXIO_BITS{
    uint16_t EX1:1;
    uint16_t EX2:1;
    uint16_t EX3:1;
    uint16_t EX4:1;
    uint16_t EX5:1;
    uint16_t EX6:1;
    uint16_t EX7:1;
    uint16_t EX8:1;
    uint16_t EX9:1;
    uint16_t EX10:1;
    uint16_t EX11:1;
    uint16_t EX12:1;
    uint16_t EX13:1;
    uint16_t EX14:1;
    uint16_t EX15:1;
    uint16_t EX16:1;
};

union EXIO_IN_OUT_REG{
    uint16_t			all;
    struct EXIO_BITS	bit;
};

struct EXIO_REGS{
    union EXIO_IN_OUT_REG IN;
    union EXIO_IN_OUT_REG OUT;
};

extern volatile struct EXIO_REGS  ExioRegs;

#endif /* SRC_DEVICE_MYEXIO_EXIO_H_ */
