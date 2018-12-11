/*
 * CircleBuffer.c
 *
 *  Created on: 2017-2-16
 *      Author: Administrator
 */
#include "sysTypes.h"
#include "CircleBuffer.h"
#include "stdlib.h"
#include "string.h"

/*
 * 创建块
 */
int cb_create(CIRCLE_BUFFER_S *buf, int block_size, int block_number)
{
	//cb_release(buf);
	memset(buf, 0, sizeof(CIRCLE_BUFFER_S));			//内存块清零
	buf->dat =(int*) malloc(block_size * block_number);	//申请内存
	buf->block_number = block_number;					//总的数据的个数
	buf->block_size = block_size;						//每个数据占用的内存的大小
	return buf->block_number;
}

/*
 * 释放块
 */
int cb_release(CIRCLE_BUFFER_S *buf)
{
	if (buf->dat == 0)
		return 0;
	memset(buf, 0, sizeof(CIRCLE_BUFFER_S));
	free(buf->dat);
	return 0;
}

/*
 * 向块中存数据
 */
int cb_append(CIRCLE_BUFFER_S *buf, void* block_dat)
{
	int tail = (buf->tail + 1) % buf->block_number;
	if (tail == buf->head)
		return RTN_ERROR;
	memcpy((unsigned int *)(buf->dat) + tail * buf->block_size, (unsigned int *)block_dat, buf->block_size);
	buf->tail = tail;
	return buf->tail;
}

/*
 * 在块中读数据
 */
int cb_get(CIRCLE_BUFFER_S *buf, void* block_dat)
{
	int head = (buf->head + 1) % buf->block_number;
	if (buf->head == buf->tail)
		return RTN_ERROR;
	memcpy((unsigned int *)block_dat, (unsigned int *)(buf->dat) + head * buf->block_size, buf->block_size);
	buf->head = head;
	return buf->head;
}

/*
 * 返回块中可用的空间
 */
int cb_usedSpace(CIRCLE_BUFFER_S *buf)
{
	int tail = buf->tail;
	if (tail < buf->head)
		tail += buf->block_number;
	return (buf->block_number - (tail - buf->head));
}

/*
 * 清除块中的数据
 */
int cb_clear(CIRCLE_BUFFER_S *buf)
{
	buf->head = buf->tail;
	return buf->head;
}

/*
 * memset是计算机中C/C++语言函数。
 * 将s所指向的某一块内存中的后n个 字节的内容全部设置为ch指定的ASCII值，
 * 第一个值为指定的内存地址，块的大小由第三个参数指定，这个函数通常为新申请的内存做初始化工作， 其返回值为s。
 * 表达式：memset(void *s,int ch,size_t n);
 */

/*
 * memcpy指的是c和c++使用的内存拷贝函数，memcpy函数的功能是从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中。
 * void *memcpy(void *dest, const void *src, size_t n);
 */
