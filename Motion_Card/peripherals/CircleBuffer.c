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
 * ������
 */
int cb_create(CIRCLE_BUFFER_S *buf, int block_size, int block_number)
{
	//cb_release(buf);
	memset(buf, 0, sizeof(CIRCLE_BUFFER_S));			//�ڴ������
	buf->dat =(int*) malloc(block_size * block_number);	//�����ڴ�
	buf->block_number = block_number;					//�ܵ����ݵĸ���
	buf->block_size = block_size;						//ÿ������ռ�õ��ڴ�Ĵ�С
	return buf->block_number;
}

/*
 * �ͷſ�
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
 * ����д�����
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
 * �ڿ��ж�����
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
 * ���ؿ��п��õĿռ�
 */
int cb_usedSpace(CIRCLE_BUFFER_S *buf)
{
	int tail = buf->tail;
	if (tail < buf->head)
		tail += buf->block_number;
	return (buf->block_number - (tail - buf->head));
}

/*
 * ������е�����
 */
int cb_clear(CIRCLE_BUFFER_S *buf)
{
	buf->head = buf->tail;
	return buf->head;
}

/*
 * memset�Ǽ������C/C++���Ժ�����
 * ��s��ָ���ĳһ���ڴ��еĺ�n�� �ֽڵ�����ȫ������Ϊchָ����ASCIIֵ��
 * ��һ��ֵΪָ�����ڴ��ַ����Ĵ�С�ɵ���������ָ�����������ͨ��Ϊ��������ڴ�����ʼ�������� �䷵��ֵΪs��
 * ���ʽ��memset(void *s,int ch,size_t n);
 */

/*
 * memcpyָ����c��c++ʹ�õ��ڴ濽��������memcpy�����Ĺ����Ǵ�Դsrc��ָ���ڴ��ַ����ʼλ�ÿ�ʼ����n���ֽڵ�Ŀ��dest��ָ���ڴ��ַ����ʼλ���С�
 * void *memcpy(void *dest, const void *src, size_t n);
 */
