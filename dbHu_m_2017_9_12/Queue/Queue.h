/*
 * queue.h
 *
 *  Created on: Aug 17, 2015
 *      Author: loywong
 */


#include <stdlib.h>
#include <ti/sysbios/knl/Task.h>
#include <string.h>

#ifndef QUEUE_H_
#define QUEUE_H_

template<typename T>
class Queue
{
private:
	T *array;
	int alen;
	volatile int head;
	volatile int tail;
public:
	Queue(int alen)
	{
		this->array = (T *)malloc(sizeof(T) * alen);
		this->alen = alen;
		this->head = 0;
		this->tail = 0;
	}
	~Queue()
	{
	    free((void *)this->array);
	}
//	Queue()
//	{
//		this->array = malloc(sizeof(T) * 4096);
//		this->alen = 4096;
//		this->head = 0;
//		this->tail = 0;
//	}
	void Clear()
	{
	    unsigned int key = Task_disable();
	    this->head = 0;
	    this->tail = 0;
	    Task_restore(key);
	}
	int Len()
	{
		int rtn;
		unsigned int key = Task_disable();
		if(this->tail >= this->head)
		{
			rtn = this->tail - this->head;
		}
		else
		{
			rtn = this->alen + this->tail - this->head;
		}
		Task_restore(key);
		return rtn;
	}
	bool En(const T &c)
	{
	    unsigned int key = Task_disable();
	    if(Len() < alen - 1)
		{
			array[tail] = c;
			if(tail == alen - 1)
			{
				tail = 0;
			}
			else
			{
				tail ++;
			}
	        Task_restore(key);
	        return true;
		}
	    else
	    {
            Task_restore(key);
            return false;
	    }
	}

	bool De(T &c)
	{
		unsigned int key = Task_disable();
		if(Len() > 0)
		{
			c = array[head];
			if(head == alen - 1)
			{
				head = 0;
			}
			else
			{
				head++;
			}
	        Task_restore(key);
	        return true;
		}
		else
		{
		    Task_restore(key);
		    return false;
		}
	}
};

#endif /* QUEUE_H_ */
