#include <math.h>

#include "constants.h"
#include "nav_math.h"

#define matrix_at(A, x, y, rownum)    ((A)[(x) + (rownum) * (y)]) 

void mat_vec_mult3d(const double* M, const double* v, double* mv)
{
	for (int32_t i = 0; i < 3; i++) {
		mv[i] = 0.0;
		for (int32_t j = 0; j < 3; j++) {
			mv[i] += matrix_at(M, i, j, 3) * v[j];
		}
	}
}

void matmulvec3x1(const double M[3][3], const double *v, double *mv)
{
    mv[0] = M[0][0] * v[0] + M[1][0] * v[1] + M[2][0] * v[2];
	mv[1] = M[0][1] * v[0] + M[1][1] * v[1] + M[2][1] * v[2];
	mv[2] = M[0][2] * v[0] + M[1][2] * v[1] + M[2][2] * v[2];
}

void ecef2pos(const double *r, double *pos)
{
	double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = r[0] * r[0] + r[1] * r[1];
	double z, zk, v = RE_WGS84, sinp;

	for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;)
	{
		zk = z;
		sinp = z / sqrt(r2 + z * z);
		v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
		z = r[2] + v * e2 * sinp;
	}
	pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
	pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
	pos[2] = sqrt(r2 + z * z) - v;
}

void deg2dms(double deg, double *dms, int ndec)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a);
	a = (a - dms[0]) * 60.0;
	dms[1] = floor(a);
	a = (a - dms[1]) * 60.0;
	dms[2] = floor(a / unit + 0.5) * unit;
	if (dms[2] >= 60.0)
	{
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0)
		{
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}

void blh2C_en(const double *blh, double C_en[3][3])
{
	/* blh => C_en */
	double lat = blh[0], lon = blh[1]; /*, ht = blh[2];*/
	C_en[0][0] = -sin(lat) * cos(lon);
	C_en[1][0] = -sin(lat) * sin(lon);
	C_en[2][0] = cos(lat);
	C_en[0][1] = -sin(lon);
	C_en[1][1] = cos(lon);
	C_en[2][1] = 0.0;
	C_en[0][2] = -cos(lat) * cos(lon);
	C_en[1][2] = -cos(lat) * sin(lon);
	C_en[2][2] = -sin(lat);
	return;
}

void ecef2ned(const double *xyz, double *ned)
{
    double C_en[3][3];
    double blh[3] = {0.0};

    ecef2pos(xyz, blh);
    blh2C_en(blh, C_en);

	ned[0] = C_en[0][0] * xyz[0] + C_en[1][0] * xyz[1] + C_en[2][0] * xyz[2];
	ned[1] = C_en[0][1] * xyz[0] + C_en[1][1] * xyz[1] + C_en[2][1] * xyz[2];
	ned[2] = C_en[0][2] * xyz[0] + C_en[1][2] * xyz[1] + C_en[2][2] * xyz[2];
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void pos2ecef(const double *pos, double *r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]);
	double cosl = cos(pos[1]);
	double e2 = FE_WGS84 * (2.0 - FE_WGS84);
	double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

	r[0] = (v + pos[2]) * cosp * cosl;
	r[1] = (v + pos[2]) * cosp * sinl;
	r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}