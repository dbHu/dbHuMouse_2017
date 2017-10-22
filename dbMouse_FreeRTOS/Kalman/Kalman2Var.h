/*
 * Kalman1Var.h
 *
 *  Created on: Aug 4, 2016
 *      Author: loywong
 */

#ifndef KALMAN_KALMAN1VAR_H_
#define KALMAN_KALMAN1VAR_H_

#include "Matrix.h"

class Kalman2Var
{
private:
    Matrix2x2 	A, H, Q, R;
    Vector2     B;
    Vector2   	xpri, xpos;
    Matrix2x2   K, Ppri , Ppos;
public:
    Kalman2Var(Matrix2x2 A, Vector2 B, Matrix2x2 H, Matrix2x2 Q, Matrix2x2 R, Vector2 x0, Matrix2x2 P0);
    void Predict(float u);
    Vector2 Correct(Vector2 z);
    virtual ~Kalman2Var();
    void Reset(void);
};

#endif /* KALMAN_KALMAN1VAR_H_ */
