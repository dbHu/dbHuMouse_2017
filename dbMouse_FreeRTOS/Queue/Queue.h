/*
 * queue.h
 *
 *  Created on: Aug 17, 2015
 *      Author: loywong
 */

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>

#ifndef QUEUE_H_
#define QUEUE_H_

template<typename T, bool threadSafe = false>
class Queue
{
private:
    SemaphoreHandle_t mtx;
    T *array;
    int alen;
    volatile int head;
    volatile int tail;
    int l()
    {
        int rtn;
        if(this->tail >= this->head)
        {
            rtn = this->tail - this->head;
        }
        else
        {
            rtn = this->alen + this->tail - this->head;
        }
        return rtn;
    }
public:
    Queue(int alen)
    {
        if(threadSafe)
        {
            this->mtx = xSemaphoreCreateMutex();
            configASSERT(this->mtx);
        }
        this->array = new T[alen];
        this->alen = alen;
        this->head = 0;
        this->tail = 0;
    }
    ~Queue()
    {
        delete this->array;
        if(threadSafe)
        {
            vSemaphoreDelete(mtx);
        }
    }
    void Clear()
    {
        int ret;
        if(threadSafe)
        {
            ret = xSemaphoreTake(mtx, portMAX_DELAY);
            configASSERT(ret == pdPASS);
        }
        this->head = 0;
        this->tail = 0;
        if(threadSafe)
        {
            ret = xSemaphoreGive(mtx);
            configASSERT(ret == pdPASS);
        }
    }
    int Len()
    {
        int ret;
        int rtn;
        if(threadSafe)
        {
            ret = xSemaphoreTake(mtx, portMAX_DELAY);
            configASSERT(ret == pdPASS);
        }
        if(this->tail >= this->head)
        {
            rtn = this->tail - this->head;
        }
        else
        {
            rtn = this->alen + this->tail - this->head;
        }
        if(threadSafe)
        {
            ret = xSemaphoreGive(mtx);
            configASSERT(ret == pdPASS);
        }
        return rtn;
    }
    bool En(const T &c)
    {
        int ret;
        bool rtn = false;
        if(threadSafe)
        {
            ret = xSemaphoreTake(mtx, portMAX_DELAY);
            configASSERT(ret == pdPASS);
        }
        if(l() < alen - 1)
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
            rtn = true;
        }
        if(threadSafe)
        {
            ret = xSemaphoreGive(mtx);
            configASSERT(ret == pdPASS);
        }
        return rtn;
    }
    bool De(T &c)
    {
        int ret;
        bool rtn = false;
        if(threadSafe)
        {
            ret = xSemaphoreTake(mtx, portMAX_DELAY);
            configASSERT(ret == pdPASS);
        }
        if(l() > 0)
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
            rtn = true;
        }
        if(threadSafe)
        {
            ret = xSemaphoreGive(mtx);
            configASSERT(ret == pdPASS);
        }
        return rtn;
    }
};

#endif /* QUEUE_H_ */

