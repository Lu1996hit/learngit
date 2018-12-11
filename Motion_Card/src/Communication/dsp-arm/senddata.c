#include <string.h>
#include "senddata.h"
#include "sysDefines.h"

#define SEND_CH(d)  ch = d; cb_append(&Spia.cb_tx, &ch)

// 弃用占用大片内存方式
ERROR_CODE senddata()
{
	unsigned int ch, checksum = 0xFFFF;
	unsigned int len;
	unsigned int *p;

	len = cmd.buflen;
	if (len <= 0 || len > COMMUNICATION_MAX_LEN)
		return RTN_INVALID_COMMAND;

	SEND_CH(0x23);
	len += 2;
	SEND_CH(len);

	p = &cmd.type;
	checksum = check_prm(p, len);

	while (len--)
	{
		SEND_CH(*p++);
	}

	SEND_CH(checksum);
	return RTN_SUCC;
}
