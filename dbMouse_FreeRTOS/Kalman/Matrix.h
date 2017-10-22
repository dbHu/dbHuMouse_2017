#ifndef KALMAN_MATRIX_H_
#define KALMAN_MATRIX_H_

class Vector2
{
public:
    float elem[2];
    Vector2(float a, float b)
    {
        elem[0] = a;
        elem[1] = b;
    };
    Vector2()
    {
        elem[0] = 0.f;
        elem[1] = 0.f;
    };
    Vector2 operator+(const Vector2 &m) const
    {
        Vector2 rtn;
        rtn.elem[0] = elem[0] + m.elem[0];
        rtn.elem[1] = elem[1] + m.elem[1];
        return rtn;
    };
    Vector2 operator-(const Vector2 &m) const
    {
        Vector2 rtn;
        rtn.elem[0] = elem[0] - m.elem[0];
        rtn.elem[1] = elem[1] - m.elem[1];
        return rtn;
    };
    Vector2 operator*(const float x) const
    {
        Vector2 rtn;
        for(int r = 0; r < 2; r++)
        {
            rtn.elem[r] = elem[r] * x;
        }
        return rtn;
    };

};

class Matrix2x2
{
public:
    float elem[2][2];
    Matrix2x2(float a, float b, float c, float d)
    {
        elem[0][0] = a;
        elem[0][1] = b;
        elem[1][0] = c;
        elem[1][1] = d;
    };
    Matrix2x2()
    {
        elem[0][0] = 0;
        elem[0][1] = 0;
        elem[1][0] = 0;
        elem[1][1] = 0;
    };
    Matrix2x2 operator*(const Matrix2x2 &m) const
    {
        Matrix2x2 rtn;
        for(int r = 0; r < 2; r++)
            for(int c = 0; c < 2; c++)
            {
                rtn.elem[r][c] = 0;
                for(int i = 0; i < 2; i++)
                {
                    rtn.elem[r][c] += elem[r][i] * m.elem[i][c];
                }
            }
        return rtn;
    };
    Matrix2x2 operator*(const float x) const
    {
        Matrix2x2 rtn;
        for(int r = 0; r < 2; r++)
            for(int c = 0; c < 2; c++)
            {
                rtn.elem[r][c] = elem[r][c] * x;
            }
        return rtn;
    };
    Vector2 operator*(const Vector2 &v) const
    {
        Vector2 rtn;
        for(int r = 0; r < 2; r++)
        {
            rtn.elem[r] = 0;
            for(int i = 0; i < 2; i++)
            {
                rtn.elem[r] += elem[r][i] * v.elem[i];
            }
        }
        return rtn;
    };
    Matrix2x2 operator+(const Matrix2x2 &m) const
    {
        Matrix2x2 rtn;
        for(int r = 0; r < 2; r++)
            for(int c = 0; c < 2; c++)
            {
                rtn.elem[r][c] = elem[r][c] + m.elem[r][c];
            }
        return rtn;
    }
    Matrix2x2 operator-(const Matrix2x2 &m) const
    {
        Matrix2x2 rtn;
        for(int r = 0; r < 2; r++)
            for(int c = 0; c < 2; c++)
            {
                rtn.elem[r][c] = elem[r][c] - m.elem[r][c];
            }
        return rtn;
    };
    Matrix2x2 Inv()
    {
        Matrix2x2 m;
        float den = -elem[0][1]*elem[1][0] + elem[0][0]*elem[1][1];
        if(den == 0.0f) den = 1e-18f;
        m.elem[0][0] = elem[1][1] / den;
        m.elem[0][1] = elem[0][1] / den;
        m.elem[1][0] = elem[1][0] / den;
        m.elem[1][1] = elem[0][0] / den;
        return m;
    };

    Matrix2x2 Tran()
    {
        Matrix2x2 m;
        m.elem[0][0] = elem[0][0];
        m.elem[0][1] = elem[1][0];
        m.elem[1][0] = elem[0][1];
        m.elem[1][1] = elem[1][1];
        return m;
    };
};

#endif
