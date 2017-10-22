///*
// * queue.c
// *
// *  Created on: Aug 17, 2015
// *      Author: loywong
// */
//
//#include "../common.h"
//#include "mb_interface.h"
//#include "Queue.h"
//#include <stdlib.h>
//#include <malloc.h>
//
//template<class T>
//Queue<T>::Queue(int alen)
//{
//	this->array = malloc(sizeof(T) * alen);
//	this->alen = alen;
//	this->head = 0;
//	this->tail = 0;
//}
//
//template<class T>
//Queue<T>::Queue()
//{
//	this->array = malloc(sizeof(T) * 4096);
//	this->alen = 4096;
//	this->head = 0;
//	this->tail = 0;
//}
//
//template<class T>
//int Queue<T>::Len()
//{
//	microblaze_disable_interrupts();
//	if(this->tail >= this->head)
//	{
//		return this->tail - this->head;
//	}
//	else
//	{
//		return this->alen + this->tail - this->head;
//	}
//	microblaze_enable_interrupts();
//}
//
//template<class T>
//void Queue<T>::En(T c)
//{
//	microblaze_disable_interrupts();
//	if(Len() < alen - 1)
//	{
//		array[tail] = c;
//		if(tail == alen - 1)
//		{
//			tail = 0;
//		}
//		else
//		{
//			tail ++;
//		}
//	}
//	microblaze_enable_interrupts();
//}
//
//template<class T>
//T Queue<T>::De()
//{
//	T rtn;
//	microblaze_disable_interrupts();
//	if(Len() > 0)
//	{
//		rtn = array[head];
//		if(head == alen - 1)
//		{
//			head = 0;
//		}
//		else
//		{
//			head ++;
//		}
//	}
//	microblaze_enable_interrupts();
//	return rtn;
//}
