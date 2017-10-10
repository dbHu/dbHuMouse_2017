/*
 * Kalman1Var.cpp
 *
 *  Created on: Aug 4, 2016
 *      Author: loywong
 */

#include <Kalman/Kalman2Var.h>
#include "Matrix.h"

Kalman2Var::Kalman2Var(Matrix2x2 A, Vector2 B, Matrix2x2 H, Matrix2x2 Q, Matrix2x2 R, Vector2 x0, Matrix2x2 P0)
{
    this->A = A;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->xpos = x0;
    this->Ppos = P0;
}

 Kalman2Var::~Kalman2Var()
 {
     // TODO Auto-generated destructor stub
 }

void Kalman2Var::Predict(float u)
{

//    x^ -k = A x^ k – 1 + B u k – 1
    xpri = A * xpos + B * u;
//    P -k = A P k – 1 A T + Q
    Ppri = A * Ppos * A.Tran() + Q;
}

Matrix2x2 I(1.f,0.f,0.f,1.f);

Vector2 Kalman2Var::Correct(Vector2 z)
{
//    K k = P -k H T ( H P -k H T + R ) – 1
    K = Ppri * H.Tran() * (H * Ppri * H.Tran() + R).Inv();
//    x^ k = x^ -k + K k ( z k – H x^ -k )
    xpos = xpri + K * (z - H * xpri);
//    P k = ( I – K k H ) P -k
    Ppos = (I - K * H) * Ppri;
    return xpos;
}
