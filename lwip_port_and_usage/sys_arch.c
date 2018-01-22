/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *         Simon Goldschmidt
 *
 */

#include <stdlib.h>
#include <stdio.h> /* sprintf() for task names */
#include <lwip/opt.h>
#include <lwip/arch.h>
#include <lwip/stats.h>
#include <lwip/debug.h>
#include <lwip/sys.h>
#include <raw_api.h>

#include <memary.h>

/******************************************************************************/
/*
 * port_malloc port_free是为了自己管理内存所用函数，非LwIP移植必须
 */
/******************************************************************************/
#define LWIP_BYTE_POOL_SIZE (1*1024*1024)
static RAW_BYTE_POOL_STRUCT lwip_byte_pool;
static RAW_U8 __attribute__((aligned(4))) __lwip_byte_pool[LWIP_BYTE_POOL_SIZE];

void *port_malloc(RAW_U32 size)
{
	void *ret;

	raw_byte_allocate(&lwip_byte_pool, &ret, size);

	return ret;
}

void port_free(void *free_ptr)
{
	raw_byte_release(&lwip_byte_pool, free_ptr);
}
/******************************************************************************/

#ifdef LWIP_RAM_HEAP_POINTER
//使用自己的lwip堆, 配置于lwipots.h
unsigned char __attribute__((aligned(4))) __lwip_heap[MEM_SIZE];
#endif

static int msg_fake_null;

u32_t sys_now()
{
	return raw_tick_count;
}

sys_prot_t sys_arch_protect(void)
{
	raw_disable_sche();
	return 0;
}

void sys_arch_unprotect(sys_prot_t pval)
{
	raw_enable_sche();
}

void sys_init()
{
	raw_byte_pool_create(&lwip_byte_pool,				/* byte内存池对象 */
						 (RAW_U8 *)"lwip_byte_pool",	/* byte内存池名称 */
						 __lwip_byte_pool,				/* 内存池起始地址 */
						 LWIP_BYTE_POOL_SIZE);			/* 内存池大小     */
}


err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	RAW_SEMAPHORE *semaphore_ptr = 0;

	if (sem == NULL)
	{
		RAW_ASSERT(0);
	}

	semaphore_ptr = port_malloc(sizeof(RAW_SEMAPHORE));
	if  (semaphore_ptr == 0)
	{
		RAW_ASSERT(0);
	}

	raw_semaphore_create(semaphore_ptr, (RAW_U8 *)"name_ptr", count);

	sem->sem = semaphore_ptr;

	return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{

	if ((sem == NULL) || (sem->sem == NULL))
	{
		RAW_ASSERT(0);
	}

	raw_semaphore_delete(sem->sem);
	raw_memset(sem->sem, sizeof( RAW_SEMAPHORE), 0);
	port_free(sem->sem);

	sem->sem = NULL;
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	RAW_U16  ret;
	RAW_U32  starttime, endtime;
	RAW_U32 actual_tick;

	if ((sem == NULL) || (sem->sem == NULL))
	{
		RAW_ASSERT(0);
	}

	starttime = sys_now();

	if(timeout == 0)
	{
		/* wait infinite */
		ret = raw_semaphore_get(sem->sem, RAW_WAIT_FOREVER);
		if (ret != RAW_SUCCESS)
		{
			RAW_ASSERT(0);
		}
	}
	else
	{
		if (timeout == 0xffffffff)
		{
			timeout = 0xfffffffe;
		}

		actual_tick = ( timeout / (1000 / RAW_TICKS_PER_SECOND));
		if (actual_tick == 0)
		{
			actual_tick = 1;
		}

		ret = raw_semaphore_get(sem->sem, actual_tick);
		if (ret == RAW_BLOCK_TIMEOUT)
		{
			return SYS_ARCH_TIMEOUT;	/* timeout */
		}
		else if (ret == RAW_SUCCESS)
		{
		}
		else
		{
			RAW_ASSERT(0);
		}
	}

	endtime = sys_now();

	/* return the time we waited for the sem */
	return ( (endtime - starttime)  * (1000 / RAW_TICKS_PER_SECOND) );
}

void sys_sem_signal(sys_sem_t *sem)
{
	RAW_U16  ret;

	if ((sem == NULL) || (sem->sem == NULL))
	{
		RAW_ASSERT(0);
	}

	ret = raw_semaphore_put(sem->sem);
	if (ret != RAW_SUCCESS)
	{
		RAW_ASSERT(0);
	}
}


sys_thread_t sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stacksize, int prio)
{
	RAW_U16 ret_task;

	RAW_TASK_OBJ  *task_obj_ptr = NULL;
	PORT_STACK *task_stack = NULL;

	if (stacksize == 0)
	{
		RAW_ASSERT(0);
	}

	task_obj_ptr = port_malloc(sizeof(RAW_TASK_OBJ));
	if (task_obj_ptr == NULL)
	{
		RAW_ASSERT(0);
	}

	task_stack =  port_malloc(stacksize * sizeof(PORT_STACK));
	if (task_stack == NULL)
	{
		RAW_ASSERT(0);
	}


	ret_task = raw_task_create(task_obj_ptr,			/* 任务控制块         */
							   (RAW_U8  *)name,			/* 任务名称           */
							   arg,						/* 任务参数           */
							   prio,					/* 任务优先级         */
							   0,						/* 任务时间片         */
							   task_stack,				/* 任务栈首地址       */
							   stacksize,				/* 任务栈大小         */
							   function,				/* 任务入口地址       */
							   RAW_TASK_AUTO_START);	/* 任务创建后自动运行 */

	if (ret_task != RAW_SUCCESS)
	{
		RAW_ASSERT(0);
	}

	return  (RAW_U32)task_obj_ptr;
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	RAW_QUEUE *mbox_ptr = 0;
	RAW_VOID  *mbox_mem;

	LWIP_ASSERT("mbox != NULL", mbox != NULL);
	LWIP_ASSERT("mboxsize = 0",  size != 0);

	if (mbox == NULL)
	{
		RAW_ASSERT(0);
	}

	if (size == 0)
	{
		RAW_ASSERT(0);
	}

	mbox_ptr = port_malloc(sizeof(RAW_QUEUE));
	if (mbox_ptr == NULL)
	{
		RAW_ASSERT(0);
	}

	mbox_mem = port_malloc(size * sizeof(void *));
	if  (mbox_mem ==NULL)
	{
		return ERR_MEM;
	}

	raw_queue_create(mbox_ptr, "queue1", (RAW_VOID **)mbox_mem, size);

	mbox->queue = mbox_ptr;

	return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{

	if ((mbox == NULL) || (mbox->queue == NULL))
	{
		RAW_ASSERT(0);
	}

	raw_queue_delete(mbox->queue);
	port_free(mbox->queue->msg_q.queue_start);
	port_free(mbox->queue);

	mbox->queue = NULL;
}


void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	RAW_U16 ret;

	if ((mbox == NULL) || (mbox->queue == NULL))
	{
		RAW_ASSERT(0);
	}

	if (msg == NULL)
	{
		msg =  &msg_fake_null;
	}

	ret = raw_queue_end_post(mbox->queue, msg);
	if (ret != RAW_SUCCESS)
	{
		LWIP_ASSERT("sys_mbox_post failed", 0);
	}
}


err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	RAW_U16 ret;

	if ((mbox == NULL) || (mbox->queue == NULL))
	{
		RAW_ASSERT(0);
	}

	if (msg == NULL)
	{
		msg =  &msg_fake_null;
	}

	ret = raw_queue_end_post(mbox->queue, msg);
	if (ret == RAW_MSG_MAX)
	{
		return ERR_MEM;
	}
	else if (ret == RAW_SUCCESS)
	{
	}
	else
	{
		RAW_ASSERT(0);
	}

	return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox ,void **msg, u32_t timeout)
{
	RAW_U16 ret;
	RAW_U32  starttime, endtime;
	RAW_U32 actual_tick;

	if ((mbox == NULL) || (mbox->queue == NULL))
	{
		RAW_ASSERT(0);
	}

	starttime = sys_now();

	if(timeout == 0)
	{
		/* wait infinite */
		ret =  raw_queue_receive (mbox->queue, RAW_WAIT_FOREVER, msg);
		if (ret != RAW_SUCCESS)
		{
			RAW_ASSERT(0);
		}
		else
		{
			if(*msg == &msg_fake_null)
			{
				*msg = NULL;
			}
		}
	}
	else
	{
		if (timeout == 0xffffffff)
		{
			timeout = 0xfffffffe;
		}

		actual_tick = ( timeout / (1000 / RAW_TICKS_PER_SECOND));
		if (actual_tick == 0)
		{
			actual_tick = 1;
		}

		ret =  raw_queue_receive (mbox->queue, actual_tick, msg);
		if (ret == RAW_BLOCK_TIMEOUT)
		{
			/* timeout */
			return SYS_ARCH_TIMEOUT;
		}
		else if (ret == RAW_SUCCESS)
		{
			if(*msg == &msg_fake_null)
			{
				*msg = NULL;
			}
		}
		else
		{
			RAW_ASSERT(0);
		}
	}

	endtime = sys_now();

	return ( (endtime - starttime)  * (1000 / RAW_TICKS_PER_SECOND) );
}


u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{

	RAW_U16 ret;

	if ((mbox == NULL) || (mbox->queue == NULL))
	{
		RAW_ASSERT(0);
	}

	ret =  raw_queue_receive (mbox->queue, RAW_NO_WAIT, msg);
	if (ret == RAW_SUCCESS)
	{
		if(*msg == &msg_fake_null)
		{
			*msg = 0;
		}
	}
	else if (ret == RAW_NO_PEND_WAIT)
	{
		return SYS_MBOX_EMPTY;
	}
	else
	{
		RAW_ASSERT(0);
	}

	return 0;
}


/*
 * Prints an assertion messages and aborts execution.
 */
void sys_assert( const char *pcMessage )
{
	(void) pcMessage;

	for (;;)
	{
	}
}

