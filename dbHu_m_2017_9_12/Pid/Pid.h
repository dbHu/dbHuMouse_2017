#ifndef __PID_H__
#define __PID_H__

class Pid
{
private:
    float p, i, d, n, ts;
    float accI, accD, accIMax, accIMin, accDMax, accDMin;
public:
    Pid(float p, float i, float d, float n, float ts, float outMin, float outMax);
    float Tick(float diff);
    void Reset();
};

#endif
