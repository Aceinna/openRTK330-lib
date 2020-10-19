
#ifndef _NAV_MATH_H
#define _NAV_MATH_H

#include <stdint.h>

void mat_vec_mult3d(const double* M, const double* v, double* mv);
void matmulvec3x1(const double M[3][3], const double *v, double *mv);

void ecef2pos(const double *r, double *pos);
void deg2dms(double deg, double *dms, int ndec);
void blh2C_en(const double *blh, double C_en[3][3]);
void ecef2ned(const double *xyz, double *ned);
void pos2ecef(const double *pos, double *r);

#endif /* _NAV_MATH_H */